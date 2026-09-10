import queue
import threading
from typing import Callable

from .protocol import Response, parse_response
from .telemetry import parse_line


class SerialWorker:
    def __init__(self, port: str, baudrate: int, incoming: queue.Queue, serial_factory: Callable | None = None):
        self.port = port
        self.baudrate = baudrate
        self.incoming = incoming
        self._serial_factory = serial_factory or self._open_serial
        self._serial = None
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._write_lock = threading.Lock()

    @classmethod
    def from_serial_for_test(cls, serial_object, incoming: queue.Queue):
        worker = cls("TEST", 0, incoming, serial_factory=lambda *_args, **_kwargs: serial_object)
        worker._serial = serial_object
        return worker

    @staticmethod
    def _open_serial(port: str, baudrate: int):
        try:
            import serial
        except ModuleNotFoundError as exc:
            raise RuntimeError("缺少 pyserial，请先安装 requirements.txt") from exc
        return serial.Serial(port=port, baudrate=baudrate, timeout=0.1)

    @property
    def connected(self) -> bool:
        return self._serial is not None

    def start(self) -> None:
        if self._serial is None:
            self._serial = self._serial_factory(self.port, self.baudrate)
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, name="foc-serial-reader", daemon=True)
        self._thread.start()

    def send(self, data: bytes) -> None:
        if self._serial is None:
            raise RuntimeError("serial worker is not connected")
        with self._write_lock:
            self._serial.write(data)

    def _run(self) -> None:
        while not self._stop_event.is_set():
            try:
                raw = self._serial.readline()
            except Exception as exc:
                if not self._stop_event.is_set():
                    self.incoming.put(("error", str(exc)))
                break
            if not raw:
                continue
            line = raw.decode("utf-8", errors="replace").strip()
            sample = parse_line(line)
            if sample is not None:
                self.incoming.put(("telemetry", sample))
                continue
            response = parse_response(line)
            if response is not None:
                self.incoming.put(("response", response))
            else:
                self.incoming.put(("log", line))

    def stop(self) -> None:
        self._stop_event.set()
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:
                pass
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        self._thread = None
        self._serial = None