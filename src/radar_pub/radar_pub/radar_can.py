""" Wraps the controlcan api similar to the usbcan """
from __future__ import annotations

import os
import sys
import ctypes
import logging
import signal, atexit, weakref, sys
from ctypes import(
    c_ubyte as BYTE,
    c_char as CHAR,
    c_ushort as USHORT,
    c_uint as UINT,
    c_ulong as ULONG,
    c_void_p as PVOID,
    c_int as INT,
    c_uint32 as UINT32,
    c_bool as BOOL,
)
from typing import List, Tuple

# --- Logging (device types) --- #
log = logging.getLogger(__name__)

# --- Auto-cleanining of registry (device types) --- #
_OPEN_HANDLES: "weakref.WeakSet[USBCAN]" = weakref.WeakSet()
def _cleanup_all(signum: int | None = None, _frame=None):
    """Reset & close every still‑open adapter (safe to call multiple times)."""
    while _OPEN_HANDLES:
        handle = _OPEN_HANDLES.pop()
        try:
            handle.reset_can()
            handle.close()
        except Exception:
            pass  # suppress during shutdown
    if signum is not None:
        # Re‑raise default signal behaviour (exit with same code)
        signal.signal(signum, signal.SIG_DFL)
        os.kill(os.getpid(), signum)

# Register only once – at import time
for _sig in (signal.SIGINT, signal.SIGTERM, signal.SIGTSTP):
    try:
        signal.signal(_sig, _cleanup_all)
    except (ValueError, AttributeError):
        pass  # not available on this platform
atexit.register(_cleanup_all)


############################################################################################3
# --- Constants (device types) --- #
VCI_PCI5121 = 1
VCI_PCI9810 = 2
VCI_USBCAN1 = 3
VCI_USBCAN2 = 4
VCI_PCI9820 = 5
VCI_CAN232 = 6
VCI_PCI5110 = 7
VCI_CANLite = 8
VCI_ISA9620 = 9
VCI_ISA5420 = 10

# --- Error codes and status flags (subset) --- #
ERR_CAN_OVERFLOW = 0x0001
ERR_CAN_ERRALARM = 0x0002
ERR_CAN_PASSIVE = 0x0004
ERR_CAN_LOSE = 0x0008
ERR_CAN_BUSERR = 0x0010

STATUS_OK = 1
STATUS_ERR = 0

# --- Structs from control can.h --- #
class VCI_BOARD_INFO(ctypes.Structure):
    _fields_ = [
        ("hw_Version", USHORT),
        ("fw_Version", USHORT),
        ("dr_Version", USHORT),
        ("in_Version", USHORT),
        ("irq_Num", USHORT),
        ("can_Num", BYTE),
        ("str_Serial_Num", CHAR * 20),
        ("str_hw_Type", CHAR * 40),
        ("Reserved", USHORT * 4),
    ]


class VCI_CAN_OBJ(ctypes.Structure):
    _fields_ = [
        ("ID", UINT),
        ("TimeStamp", UINT),
        ("TimeFlag", BYTE),
        ("SendType", BYTE),
        ("RemoteFlag", BYTE),  # 1 = Remote frame
        ("ExternFlag", BYTE),   # 1 = Extended frame
        ("DataLen", BYTE),
        ("Data", BYTE * 8),
        ("Reserved", BYTE * 3),
    ]


class VCI_CAN_STATUS(ctypes.Structure):
    _fields_ = [
        ("ErrInterrupt", BYTE),
        ("regMode", BYTE),
        ("regStatus", BYTE),
        ("regALCapture", BYTE),
        ("regECCapture", BYTE),
        ("regEWLimit", BYTE),
        ("regRECounter", BYTE),
        ("regTECounter", BYTE),
        ("Reserved", UINT),
    ]


class VCI_ERR_INFO(ctypes.Structure):
    _fields_ = [
        ("ErrCode", UINT),
        ("Passive_ErrData", BYTE * 3),
        ("ArLost_ErrData", BYTE),
    ]


class VCI_INIT_CONFIG(ctypes.Structure):
    _fields_ = [
        ("AccCode", UINT),
        ("AccMask", UINT),
        ("Reserved", UINT),
        ("Filter", BYTE),
        ("Timing0", BYTE),
        ("Timing1", BYTE),
        ("Mode", BYTE),
    ]

# --- Helper – bitrate lookup tables (Timing0/Timing1) for 16 MHz CAN clock --- #
BITRATES_16MHZ = {
    "5K": (0xBF, 0xFF),
    "10K": (0x31, 0x1C),
    "20K": (0x18, 0x1C),
    "40K": (0x87, 0xFF),
    "50K": (0x09, 0x1C),
    "80K": (0x83, 0xFF),
    "100K": (0x04, 0x1C),
    "125K": (0x03, 0x1C),
    "250K": (0x01, 0x1C),
    "500K": (0x00, 0x1C),
    "800K": (0x00, 0x16),
    "1M": (0x00, 0x14),
}

# --- Loading the library --- #
def _load_dll()->ctypes.CDLL:
    " Find and load shared library file based on Os"
    base = os.path.dirname(os.path.abspath(__file__))
    names_win = ["controlcan.dll"]
    names_lin = ["libusbcan.so", "libcontrolcan.so", "controlcan.so"]
    libnames = names_win if os.name == "nt" else names_lin
    search = [os.path.join(base,"usbcan"), base,]

    for d in search:
        for n in libnames:
            p = os.path.join(d,n)
            if os.path.isfile(p):
                log.debug("Loading Control can shared library: %s", p)
                return ctypes.cdll.LoadLibrary(p)

    raise FileNotFoundError("ControlCAN shared library not found")

dll = _load_dll()

# --- Function prototypes ---
def _decl(func, restype, *argtypes):
    f = getattr(dll, func)
    f.restype = restype
    f.argtypes = list(argtypes)
    return f

VCI_OpenDevice = _decl("VCI_OpenDevice", UINT, UINT, UINT, UINT)
VCI_CloseDevice = _decl("VCI_CloseDevice", UINT, UINT)
VCI_InitCAN = _decl("VCI_InitCAN", UINT, UINT, UINT, UINT, ctypes.POINTER(VCI_INIT_CONFIG))
VCI_StartCAN = _decl("VCI_StartCAN", UINT, UINT, UINT)
VCI_ResetCAN = _decl("VCI_ResetCAN", UINT, UINT, UINT)
VCI_Transmit = _decl("VCI_Transmit", UINT, UINT, UINT, UINT, ctypes.POINTER(VCI_CAN_OBJ), UINT)
VCI_Receive = _decl("VCI_Receive", UINT, UINT, UINT, UINT, ctypes.POINTER(VCI_CAN_OBJ), UINT, INT)
VCI_ReadErrInfo = _decl("VCI_ReadErrInfo", UINT, UINT, UINT, UINT, ctypes.POINTER(VCI_ERR_INFO))
VCI_ReadBoardInfo = _decl("VCI_ReadBoardInfo", UINT, UINT, ctypes.POINTER(VCI_BOARD_INFO))

class CANError(RuntimeError):
    """Raised when a ControlCAN API call returns STATUS_ERR or other failure."""

class USBCAN:
    """Basic operational layer
    Parameters
    ----------
    device_type : int
        One of the VCI_* constants (default: `VCI_USBCAN2`).
    device_index : int
        Index of the adapter when several identical units are connected.
    """
    def __init__(self, device_type:int=VCI_USBCAN1, device_index: int=None):
        self.device_type = device_type
        self.device_index = device_index
        self._opened = False

    @staticmethod
    def _check(code:int, msg:str):
        if code != STATUS_OK:
            raise CANError(f"{msg} failed (ret={code})")

    @staticmethod
    def _auto_index(device_type: int) -> int | None:
        for idx in range(8):
            if VCI_OpenDevice(device_type, idx, 0) == STATUS_OK:
                VCI_CloseDevice(device_type, idx)
                return idx
        return None

    # ---------- open / close ------------
    def open(self)->None:
        if self._opened:
            return
        # -------------------------- #
        if self.device_index is None:
            found = self._auto_index(self.device_type)
            if found is None:
                raise CANError("No adapter of requested type found")
            self.device_index = found

        self._check(VCI_OpenDevice(self.device_type, self.device_index, 0), "VCI_OpenDevice")
        self._opened = True
        log.info("Device opened (type=%d index=%d)", self.device_type, self.device_index)
        _OPEN_HANDLES.add(self)

    def close(self)->None:
        if not self._opened:
            return
        try:
            self._check(VCI_CloseDevice(self.device_type, self.device_index), "VCI_CloseDevice")
        finally:
            self._opened = False
            log.info("Device closed")
            _OPEN_HANDLES.discard(self)

    #  -- Setting up channels and connections --
    def init_can(self, channel:int=0, *, bitrate:str = "500K", mode:int=0, acc_code:int=0,
                 acc_mask:int=0xFFFFFFFF)->None:
        if bitrate not in BITRATES_16MHZ:
            raise ValueError(f"Unsupported bitrate '{bitrate}' . Available: {sorted(BITRATES_16MHZ)}")

        timing0, timing1 = BITRATES_16MHZ[bitrate]
        cfg = VCI_INIT_CONFIG(
            AccCode=acc_code,
            AccMask=acc_mask,
            Reserved=0,
            Filter=1,  # Single 32‑bit filter
            Timing0=timing0,
            Timing1=timing1,
            Mode=mode,
        )
        self._check(VCI_InitCAN(self.device_type, self.device_index, channel, ctypes.byref(cfg)), "VCI_InitCAN")
        log.info("Channel %d initialised @ %s", channel, bitrate)

    def start_can(self, channel: int = 0) -> None:
        self._check(VCI_StartCAN(self.device_type, self.device_index, channel), "VCI_StartCAN")
        log.info("Channel %d started", channel)

    def reset_can(self, channel: int = 0) -> None:
        self._check(VCI_ResetCAN(self.device_type, self.device_index, channel), "VCI_ResetCAN")
        log.info("Channel %d reset", channel)

    # --- Context manager handler ---
    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        try:
            self.reset_can()
        finally:
            self.close()
        return False

    def __del__(self):
        try:
            if self._opened:
                self.reset_can()
                self.close()
        except Exception:
            pass

    # --- Input Output information --- #
    def send(
        self,
        can_id: int,
        data: bytes | bytearray,
        *,
        channel: int = 0,
        extended: bool = False,
        remote: bool = False,
    ) -> None:
        if len(data) > 8:
            raise ValueError("CAN payload must be ≤ 8 bytes")
        frame = VCI_CAN_OBJ()
        frame.ID = can_id
        frame.SendType = 0  # normal
        frame.RemoteFlag = 1 if remote else 0
        frame.ExternFlag = 1 if extended else 0
        frame.DataLen = len(data)
        frame.Data[: frame.DataLen] = list(data)
        count = VCI_Transmit(self.device_type, self.device_index, channel, ctypes.byref(frame), 1)
        if count == 0:
            # Query the adapter for *why* the driver refused the frame
            err = self.read_error_info(channel)
            diagnostic = (
                f"ErrCode=0x{err.ErrCode:08X} "
                f"Passive={list(err.Passive_ErrData)} "
                f"ArLost={err.ArLost_ErrData}"
            )
            raise CANError(f"VCI_Transmit returned 0 (no frame sent); {diagnostic}")

        log.debug("TX 0x%X (%d bytes)", can_id, len(data))

    def receive(
        self,
        max_frames: int = 1000,
        *,
        channel: int = 0,
        timeout_ms: int = 0,
    ) -> List[Tuple[int, bytes]]:
        buffer = (VCI_CAN_OBJ * max_frames)()
        buf_ptr = ctypes.cast(buffer, ctypes.POINTER(VCI_CAN_OBJ))

        n = VCI_Receive(self.device_type, self.device_index, channel, buf_ptr, max_frames, timeout_ms)
        if n < 0:
            raise CANError("VCI_Receive error (negative return value)")
        return [(buffer[i].ID, bytes(buffer[i].Data[: buffer[i].DataLen])) for i in range(n)]

    # --- Diagnostics --- #
    def read_board_info(self) -> VCI_BOARD_INFO:
        info = VCI_BOARD_INFO()
        self._check(VCI_ReadBoardInfo(self.device_type, self.device_index, ctypes.byref(info)), "VCI_ReadBoardInfo")
        return info

    def read_error_info(self, channel: int = 0) -> VCI_ERR_INFO:
        err = VCI_ERR_INFO()
        self._check(VCI_ReadErrInfo(self.device_type, self.device_index, channel, ctypes.byref(err)), "VCI_ReadErrInfo")
        return err

if __name__=="__main__":
    import argparse, time
    logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")

    parser = argparse.ArgumentParser(description="Simple send/recv sanity test for USBCAN adapter")
    parser.add_argument("--bitrate", default="500K", help="CAN bit‑rate (default: 500K)")
    parser.add_argument("--channel", type=int, default=0, help="Channel index (0/1)")
    args = parser.parse_args()

    with USBCAN(VCI_USBCAN1) as can:
        can.init_can(channel=args.channel, bitrate=args.bitrate)
        can.start_can(channel=args.channel)

        # Fire a test frame every second, print whatever we get back.
        for _ in range(10):
            can.send(0x100, b"\xDE\xAD\xBE\xEF", channel=args.channel)
            frames = can.receive(timeout_ms=100)
            for fid, payload in frames:
                print(f"RX 0x{fid:03X}: {payload.hex()}")
            time.sleep(1)

        print("Demo finished.")


