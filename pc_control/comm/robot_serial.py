import threading
import time
import serial

from .encoder import encode_move, decode_response


class RobotSerial:
    """
    Quản lý kết nối serial với STM32 và gửi lệnh điều khiển motor.

    Sử dụng:
        rs = RobotSerial(port="COM3")
        rs.open()
        rs.send_angles(curr_q)         # gửi không chặn
        rs.wait_done(seq, timeout=30)  # chờ $D
        rs.close()

    Hoặc dùng context manager:
        with RobotSerial("COM3") as rs:
            rs.send_and_wait(curr_q)
    """

    def __init__(self, port: str, baud: int = 115200, read_timeout: float = 1.0):
        self._port = port
        self._baud = baud
        self._read_timeout = read_timeout

        self._ser: serial.Serial | None = None
        self._seq = 0
        self._lock = threading.Lock()

        # seq → threading.Event, set khi nhận $D hoặc $E
        self._pending: dict[int, threading.Event] = {}
        # seq → dict phản hồi cuối cùng
        self._results: dict[int, dict] = {}

        self._reader_thread: threading.Thread | None = None
        self._stop_event = threading.Event()

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def open(self) -> None:
        """Mở cổng serial và khởi background reader thread."""
        self._ser = serial.Serial(
            port=self._port,
            baudrate=self._baud,
            timeout=self._read_timeout,
        )
        self._stop_event.clear()
        self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader_thread.start()
        print(f"[RobotSerial] Opened {self._port} @ {self._baud} baud")

    def close(self) -> None:
        """Dừng reader thread và đóng cổng serial."""
        self._stop_event.set()
        if self._reader_thread is not None:
            self._reader_thread.join(timeout=3.0)
        if self._ser and self._ser.is_open:
            self._ser.close()
        print("[RobotSerial] Closed")

    def is_open(self) -> bool:
        return self._ser is not None and self._ser.is_open

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def send_angles(self, angles_rad) -> int:
        """
        Encode và gửi lệnh $M xuống STM32.

        Trả về seq number để dùng với wait_done(). Không chờ phản hồi.
        """
        with self._lock:
            seq = self._seq
            self._seq = (self._seq + 1) & 0xFF
            event = threading.Event()
            self._pending[seq] = event

        line = encode_move(seq, angles_rad)
        self._send_raw(line)
        return seq

    def wait_done(self, seq: int, timeout: float = 30.0) -> bool:
        """
        Chờ cho đến khi nhận $D,{seq} hoặc $E,{seq} từ STM32.

        Trả về True nếu nhận DONE, False nếu ERR hoặc timeout.
        """
        event = self._pending.get(seq)
        if event is None:
            return False

        triggered = event.wait(timeout=timeout)
        if not triggered:
            print(f"[RobotSerial] Timeout waiting for seq={seq}")
            return False

        result = self._results.pop(seq, {})
        self._pending.pop(seq, None)
        return result.get("type") == "DONE"

    def send_and_wait(self, angles_rad, timeout: float = 30.0) -> bool:
        """Kết hợp send_angles() + wait_done(). API tiện lợi nhất."""
        seq = self.send_angles(angles_rad)
        return self.wait_done(seq, timeout)

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _send_raw(self, text: str) -> None:
        if self._ser and self._ser.is_open:
            self._ser.write(text.encode("ascii"))
            self._ser.flush()

    def _reader_loop(self) -> None:
        """Background thread: đọc từng dòng và phân phối event cho wait_done()."""
        buf = b""
        while not self._stop_event.is_set():
            if self._ser is None or not self._ser.is_open:
                time.sleep(0.05)
                continue

            try:
                chunk = self._ser.read(64)
            except serial.SerialException:
                break

            if not chunk:
                continue

            buf += chunk
            while b"\n" in buf:
                line_bytes, buf = buf.split(b"\n", 1)
                line = line_bytes.decode("ascii", errors="replace").strip()
                if not line:
                    continue

                resp = decode_response(line)
                if resp is None:
                    continue

                rtype = resp.get("type")
                seq = resp.get("seq")
                print(f"[RobotSerial] RX: {line}")

                if rtype in ("DONE", "ERR") and seq is not None:
                    with self._lock:
                        self._results[seq] = resp
                        event = self._pending.get(seq)
                    if event:
                        event.set()

    # ------------------------------------------------------------------
    # Context manager
    # ------------------------------------------------------------------

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *_):
        self.close()
