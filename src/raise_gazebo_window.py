#!/usr/bin/env python3

import argparse
import ctypes
import ctypes.util
import time


def _load_x11():
    path = ctypes.util.find_library("X11")
    if not path:
        raise RuntimeError("libX11 is not available")
    x11 = ctypes.cdll.LoadLibrary(path)
    x11.XOpenDisplay.restype = ctypes.c_void_p
    x11.XDefaultRootWindow.restype = ctypes.c_ulong
    x11.XFetchName.restype = ctypes.c_int
    return x11


def _walk_windows(x11, display, window):
    yield window
    root_return = ctypes.c_ulong()
    parent_return = ctypes.c_ulong()
    children = ctypes.POINTER(ctypes.c_ulong)()
    child_count = ctypes.c_uint()
    if not x11.XQueryTree(
        display,
        window,
        ctypes.byref(root_return),
        ctypes.byref(parent_return),
        ctypes.byref(children),
        ctypes.byref(child_count),
    ):
        return
    for index in range(child_count.value):
        yield from _walk_windows(x11, display, children[index])


def _window_name(x11, display, window):
    name = ctypes.c_char_p()
    if x11.XFetchName(display, window, ctypes.byref(name)) and name.value:
        return name.value.decode(errors="replace")
    return ""


def move_window(title, x, y, width, height, timeout, interval):
    x11 = _load_x11()
    display = x11.XOpenDisplay(None)
    if not display:
        raise RuntimeError("cannot open X display")
    root = x11.XDefaultRootWindow(display)
    deadline = time.monotonic() + timeout

    while time.monotonic() <= deadline:
        for window in _walk_windows(x11, display, root):
            if title in _window_name(x11, display, window):
                # Why: WSLg may map Gazebo on a far-right virtual coordinate;
                # forcing a known visible rectangle avoids a "GUI is running but invisible" state.
                x11.XMoveResizeWindow(display, window, x, y, width, height)
                x11.XMapRaised(display, window)
                x11.XSetInputFocus(display, window, 2, 0)
                x11.XFlush(display)
                print(f"[raise_gazebo_window] moved and focused '{title}' to {width}x{height}+{x}+{y}")
                return 0
        time.sleep(interval)

    print(f"[raise_gazebo_window] window '{title}' not found within {timeout:.1f}s")
    return 1


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--title", default="Gazebo Sim")
    parser.add_argument("--x", type=int, default=80)
    parser.add_argument("--y", type=int, default=40)
    parser.add_argument("--width", type=int, default=1200)
    parser.add_argument("--height", type=int, default=850)
    parser.add_argument("--timeout", type=float, default=20.0)
    parser.add_argument("--interval", type=float, default=0.5)
    args = parser.parse_args()
    return move_window(
        args.title,
        args.x,
        args.y,
        args.width,
        args.height,
        args.timeout,
        args.interval,
    )


if __name__ == "__main__":
    raise SystemExit(main())
