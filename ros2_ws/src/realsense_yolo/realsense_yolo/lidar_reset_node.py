#!/usr/bin/env python3
"""
Reset Hokuyo LiDAR serial before urg_node2 starts.

After an unclean shutdown (kill -9, Ctrl+C), the Hokuyo stays in SCIP2
streaming mode. The next urg_node2 launch gets "invalid response" and
fails to configure. This node sends the SCIP2 'QT' (quit) command to
stop any ongoing scan, flushes the buffer, then exits cleanly so the
launch can proceed to start urg_node2.
"""

import os
import sys
import time
import termios


def main():
    port = '/dev/ttyACM0'
    if len(sys.argv) > 1:
        port = sys.argv[1]

    try:
        fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    except OSError as e:
        print(f'[lidar_reset] Cannot open {port}: {e}')
        return

    try:
        attrs = termios.tcgetattr(fd)
        attrs[4] = attrs[5] = termios.B115200
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        termios.tcflush(fd, termios.TCIOFLUSH)
        time.sleep(0.05)
        os.write(fd, b'QT\n')
        time.sleep(0.3)
        termios.tcflush(fd, termios.TCIOFLUSH)
        print(f'[lidar_reset] Sent QT to {port} — LiDAR streaming stopped')
    except Exception as e:
        print(f'[lidar_reset] Warning: {e}')
    finally:
        os.close(fd)


if __name__ == '__main__':
    main()
