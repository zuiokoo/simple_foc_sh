import queue
import unittest

from current_loop_tuner.serial_worker import SerialWorker


class FakeSerial:
    def __init__(self):
        self.writes = []
        self.closed = False

    def write(self, data):
        self.writes.append(data)

    def close(self):
        self.closed = True


class SerialWorkerTest(unittest.TestCase):
    def test_worker_writes_command_to_injected_serial(self):
        fake = FakeSerial()
        worker = SerialWorker.from_serial_for_test(fake, queue.Queue())
        worker.send(b"FOC IQ 100\n")
        self.assertEqual(fake.writes, [b"FOC IQ 100\n"])


if __name__ == "__main__":
    unittest.main()