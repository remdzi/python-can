import logging
from typing import Optional, Tuple

import usb
from gs_usb.constants import CAN_EFF_FLAG, CAN_ERR_FLAG, CAN_RTR_FLAG
from gs_usb.constants import GS_CAN_FLAG_FD, GS_CAN_FLAG_BRS, GS_CAN_FLAG_ESI
from gs_usb.gs_usb import GsUsb
from gs_usb.gs_usb_frame import GS_USB_NONE_ECHO_ID, GsUsbFrame

import can

from ..exceptions import CanInitializationError, CanOperationError

logger = logging.getLogger(__name__)


class GsUsbBus(can.BusABC):
    def __init__(
        self,
        channel,
        bitrate,
        data_bitrate=None,
        index=None,
        bus=None,
        address=None,
        can_filters=None,
        flags=None,
        **kwargs,
    ):
        """
        :param channel: usb device name
        :param index: device number if using automatic scan, starting from 0.
            If specified, bus/address shall not be provided.
        :param bus: number of the bus that the device is connected to
        :param address: address of the device on the bus it is connected to
        :param can_filters: not supported
        :param bitrate: CAN network bandwidth (bits/s)
        """
        if (index is not None) and ((bus or address) is not None):
            raise CanInitializationError(
                "index and bus/address cannot be used simultaneously"
            )

        if index is not None:
            devs = GsUsb.scan()
            if len(devs) <= index:
                raise CanInitializationError(
                    f"Cannot find device {index}. Devices found: {len(devs)}"
                )
            gs_usb = devs[index]
        else:
            gs_usb = GsUsb.find(bus=bus, address=address)
            if not gs_usb:
                raise CanInitializationError(f"Cannot find device {channel}")

        self.gs_usb = gs_usb
        self.channel_info = channel
        self._can_protocol = can.CanProtocol.CAN_20

        self.gs_usb.set_bitrate(bitrate=bitrate, data=False)
        if (data_bitrate != None):
            self.gs_usb.set_bitrate(bitrate=data_bitrate, data=True)

        self.gs_usb.start(flags=flags)

        super().__init__(
            channel=channel,
            can_filters=can_filters,
            **kwargs,
        )

    def send(self, msg: can.Message, timeout: Optional[float] = None):
        """Transmit a message to the CAN bus.

        :param Message msg: A message object.
        :param timeout: timeout is not supported.
            The function won't return until message is sent or exception is raised.

        :raises CanOperationError:
            if the message could not be sent
        """
        can_id = msg.arbitration_id

        if msg.is_extended_id:
            can_id = can_id | CAN_EFF_FLAG

        if msg.is_remote_frame:
            can_id = can_id | CAN_RTR_FLAG

        if msg.is_error_frame:
            can_id = can_id | CAN_ERR_FLAG

        frame = GsUsbFrame(can_id=can_id, is_fd=msg.is_fd, brs=msg.bitrate_switch,
                            esi=msg.error_state_indicator, data=list(msg.data))
        frame.timestamp_us = int(msg.timestamp * 1000000)

        try:
            self.gs_usb.send(frame)
        except usb.core.USBError as exc:
            raise CanOperationError("The message could not be sent") from exc

    def _recv_internal(
        self, timeout: Optional[float]
    ) -> Tuple[Optional[can.Message], bool]:
        """
        Read a message from the bus and tell whether it was filtered.
        This methods may be called by :meth:`~can.BusABC.recv`
        to read a message multiple times if the filters set by
        :meth:`~can.BusABC.set_filters` do not match and the call has
        not yet timed out.

        Never raises an error/exception.

        :param float timeout: seconds to wait for a message,
                              see :meth:`~can.BusABC.send`
                              0 and None will be converted to minimum value 1ms.

        :return:
            1.  a message that was read or None on timeout
            2.  a bool that is True if message filtering has already
                been done and else False. In this interface it is always False
                since filtering is not available
        """
        frame = GsUsbFrame()

        # Do not set timeout as None or zero here to avoid blocking
        timeout_ms = round(timeout * 1000) if timeout else 1
        if not self.gs_usb.read(frame=frame, timeout_ms=timeout_ms):
            return None, False

        msg = can.Message(
            timestamp=frame.timestamp,
            arbitration_id=frame.arbitration_id,
            is_extended_id=frame.is_extended_id,
            is_remote_frame=frame.is_remote_frame,
            is_error_frame=frame.is_error_frame,
            channel=self.channel_info,
            dlc=frame.length,
            data=bytearray(frame.data)[0 : frame.length],
            is_rx=frame.echo_id == GS_USB_NONE_ECHO_ID,
            is_fd=True if (frame.flags & GS_CAN_FLAG_FD) else False,
            bitrate_switch=True if (frame.flags & GS_CAN_FLAG_BRS) else False,
            error_state_indicator=True if (frame.flags & GS_CAN_FLAG_ESI) else False
        )

        return msg, False

    def shutdown(self):
        super().shutdown()
        self.gs_usb.stop()
