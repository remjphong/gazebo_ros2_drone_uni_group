"""The Gazebo lidar stream (one `gz topic -e` process, newest scan always available)."""

import asyncio
import shutil
import subprocess
import threading
import time

from lidar_scan import LidarError, parse_scan


# ----------------------------------------------------------------------
# Lidar stream (one long-running `gz topic -e`, newest scan always ready)
# ----------------------------------------------------------------------
class LidarStream:
    def __init__(self, topic):
        self.topic = topic
        self.proc = None
        self.thread = None
        self.latest = None  # (arrival time.monotonic(), parsed scan)
        self.seq = 0
        self.bad = 0
        self.error = None
        self._stopping = False

    def start(self):
        if shutil.which("gz") is None:
            raise LidarError(
                "Could not find the 'gz' command. Is Gazebo installed, and is "
                "this terminal set up like the one running the simulator?"
            )
        cmd = ["gz", "topic", "-e", "-t", self.topic]
        if shutil.which("stdbuf"):  # ask for line-by-line output on a pipe
            cmd = ["stdbuf", "-oL"] + cmd
        self.proc = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
        )
        self.thread = threading.Thread(target=self._reader, daemon=True)
        self.thread.start()

    def _publish(self, lines):
        try:
            scan = parse_scan("\n".join(lines))
        except LidarError:
            self.bad += 1
            return
        self.latest = (time.monotonic(), scan)
        self.seq += 1

    def _reader(self):
        """Split the endless text output into one message per scan."""
        buf = []
        has_ranges = False
        try:
            for raw in self.proc.stdout:
                line = raw.rstrip("\n")
                blank = not line.strip()
                if has_ranges:
                    key = line.split(":", 1)[0].split("{", 1)[0].strip()
                    new_message = blank or (
                        not line[0].isspace()
                        and key not in ("ranges", "intensities")
                    )
                    if new_message:
                        self._publish(buf)
                        buf, has_ranges = [], False
                if blank:
                    continue
                buf.append(line)
                if line.lstrip().startswith("ranges:"):
                    has_ranges = True
            if has_ranges:
                self._publish(buf)
        except Exception as error:
            if not self._stopping:
                self.error = error
            return

        if not self._stopping:
            try:
                detail = self.proc.stderr.read().strip()
            except Exception:
                detail = ""
            self.error = LidarError(
                f"'gz topic -e -t {self.topic}' stopped unexpectedly. {detail}"
            )

    async def wait_first(self, timeout):
        deadline = time.monotonic() + timeout
        while self.seq == 0:
            if self.error is not None:
                raise self.error
            if time.monotonic() > deadline:
                raise LidarError(
                    f"No lidar scan arrived from {self.topic} within "
                    f"{timeout:g} s. Check what `gz topic -e -t {self.topic}` "
                    "prints in a terminal."
                )
            await asyncio.sleep(0.1)

    def stop(self):
        self._stopping = True
        if self.proc is not None and self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.proc.kill()
