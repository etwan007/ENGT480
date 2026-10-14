"""Nonblocking arrow-key input for a Linux/macOS terminal (including SSH)."""

import os
import select
import sys
import termios
import tty


class LiveOffsetKeys:
    def __init__(self, stream=None):
        self.stream = sys.stdin if stream is None else stream
        self.fd = None
        self._saved_mode = None
        self._pending = b""

    def start(self):
        """Enable immediate keys while preserving Ctrl+C; return False without a TTY."""
        if not self.stream.isatty():
            return False
        self.fd = self.stream.fileno()
        self._saved_mode = termios.tcgetattr(self.fd)
        try:
            # TCSANOW avoids waiting for terminal output or flushing typed input.
            tty.setcbreak(self.fd, termios.TCSANOW)
        except BaseException:
            self.close()
            raise
        return True

    def poll_steps(self):
        """Return net arrow presses; never wait for a key or a complete sequence."""
        if self._saved_mode is None or not select.select([self.fd], [], [], 0)[0]:
            return 0
        # Bound work per control cycle, even when a key is held down.
        data = os.read(self.fd, 64)
        if not data:
            self.close()
            return 0
        return self._decode(data)

    def _decode(self, data):
        self._pending += data
        steps = 0
        while self._pending:
            if self._pending[0] != 27:
                self._pending = self._pending[1:]
                continue
            if len(self._pending) < 2:
                break
            if self._pending[1:2] not in (b"[", b"O"):
                self._pending = self._pending[1:]
                continue
            # CSI and application-cursor arrows, including modifier parameters.
            end = 2
            while end < len(self._pending) and 0x20 <= self._pending[end] <= 0x3F:
                end += 1
            if end == len(self._pending):
                if end > 16:  # discard malformed, indefinitely long sequences
                    self._pending = b""
                break
            final = self._pending[end:end + 1]
            if final in (b"A", b"C"):
                steps += 1
            elif final in (b"B", b"D"):
                steps -= 1
            self._pending = self._pending[end + 1:]
        return steps

    def close(self):
        if self._saved_mode is not None:
            termios.tcsetattr(self.fd, termios.TCSANOW, self._saved_mode)
            self._saved_mode = None
        self._pending = b""
