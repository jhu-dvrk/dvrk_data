"""
dvrk_gst_socket — Abstract Unix socket naming utilities for dvrk GStreamer pipelines.

Convention
----------
Every inter-process video socket follows the pattern::

    @dvrk:<role>:<name>

The leading ``@`` is our notation for a Linux abstract-namespace socket.
In GStreamer pipeline strings the ``@`` is stripped::

    unixfdsink socket-path=dvrk:<role>:<name> socket-type=abstract
    unixfdsrc  socket-path=dvrk:<role>:<name> socket-type=abstract

Abstract sockets are not visible in the file system; they are automatically
released by the kernel when the last file descriptor that references them is
closed.  They appear in ``/proc/net/unix`` with their name prefixed by ``@``.

Fixed roles
-----------
``stereo_source``
    Camera capture (produces ``left`` / ``right`` frames).
``stereo_alignment``
    Alignment / compositing (consumes ``left`` + ``right``, produces ``stereo``).
``stereo_display``
    Display app (produces ``stereo`` / ``overlay`` for downstream consumers).

Socket values in JSON
---------------------
JSON configs use ``"gst_input"`` and ``"gst_output"`` fields. Socket values
must use the canonical ``"@dvrk:role:name"`` form.

Discovery
---------
:func:`list_sockets`  — returns all ``@dvrk:*`` sockets currently in the kernel.
:func:`print_sockets` — pretty-prints the list.
:func:`check_socket`  — verifies a socket exists; prints alternatives on failure.
"""

from __future__ import annotations

import sys
from pathlib import Path

# ── Fixed role names ──────────────────────────────────────────────────────────

ROLE_STEREO_SOURCE    = "stereo_source"
ROLE_STEREO_ALIGNMENT = "stereo_alignment"
ROLE_STEREO_DISPLAY   = "stereo_display"

PREFIX = "@dvrk:"

# ── Name construction ─────────────────────────────────────────────────────────

def make(role: str, name: str) -> str:
    """Return a fully-qualified abstract socket name ``@dvrk:<role>:<name>``."""
    return f"{PREFIX}{role}:{name}"


# ── Resolution ────────────────────────────────────────────────────────────────

def is_socket_reference(value: str) -> bool:
    return bool(value) and value.startswith(PREFIX)


def resolve(socket_field: str, default_role: str = "") -> str:
    """Resolve a canonical ``@dvrk:role:name`` socket reference.

    Parameters
    ----------
    socket_field:
        Raw value from a JSON ``gst_input``/``gst_output`` key.
    default_role:
        Retained in the call signature for application-level role context.
    """
    if not socket_field:
        return ""
    if not is_socket_reference(socket_field):
        return ""
    return socket_field


# ── GStreamer integration ─────────────────────────────────────────────────────

def to_gst_path(abstract_name: str) -> str:
    """Strip the leading ``@`` to get the value for GStreamer's ``socket-path=``."""
    return abstract_name[1:] if abstract_name.startswith("@") else abstract_name


def build_sink(abstract_name: str, sync: bool = False) -> str:
    """Return a GStreamer ``unixfdsink`` pipeline fragment.

    Parameters
    ----------
        abstract_name:
        Fully-qualified name, e.g. ``@dvrk:stereo_source:left``.
    sync:
        Value for GStreamer's ``sync=`` property.
    """
    sync_str = "true" if sync else "false"
    return (
        f"unixfdsink socket-path={to_gst_path(abstract_name)}"
        f" socket-type=abstract sync={sync_str} async=false"
    )


def build_src(abstract_name: str, width: int = 0, height: int = 0,
              fmt: str = "I420") -> str:
    """Return a GStreamer ``unixfdsrc`` pipeline fragment.

    Parameters
    ----------
    abstract_name:
        Fully-qualified name, e.g. ``@dvrk:stereo_source:left``.
    width:
        Caps filter width.  No caps filter added when ``width <= 0``.
    height:
        Caps filter height.  No caps filter added when ``height <= 0``.
    fmt:
        Pixel format for the caps filter.
    """
    s = (
        f"unixfdsrc socket-path={to_gst_path(abstract_name)}"
        f" socket-type=abstract do-timestamp=true"
    )
    if width > 0 and height > 0:
        s += f" ! video/x-raw,format={fmt},width={width},height={height}"
    return s


def build_input(gst_input: str, default_role: str, width: int = 0,
                height: int = 0, fmt: str = "I420") -> str:
    """Return a pipeline unchanged, or expand an @ socket reference to unixfdsrc."""
    if not is_socket_reference(gst_input):
        return gst_input
    socket_name = resolve(gst_input, default_role)
    if not socket_name:
        return ""
    return build_src(socket_name, width, height, fmt)


# ── Discovery ─────────────────────────────────────────────────────────────────

def list_sockets() -> list[str]:
    """Return all ``@dvrk:*`` abstract socket names currently registered in
    the Linux kernel, by parsing ``/proc/net/unix``.

    Abstract sockets appear with their name prefixed by ``@`` in
    ``/proc/net/unix``.  Each returned string is in the form
    ``@dvrk:<role>:<name>``.
    """
    result: list[str] = []
    try:
        with open("/proc/net/unix") as f:
            next(f)  # skip header
            for line in f:
                # The Path column is the last whitespace-separated token.
                parts = line.rstrip().rsplit(None, 1)
                if len(parts) < 2:
                    continue
                path = parts[-1]
                if path.startswith(PREFIX):
                    result.append(path)
    except OSError:
        pass

    seen: set[str] = set()
    unique: list[str] = []
    for s in sorted(result):
        if s not in seen:
            seen.add(s)
            unique.append(s)
    return unique


def print_sockets(file=None) -> None:
    """Print all existing ``@dvrk`` sockets.

    Parameters
    ----------
    file:
        Output stream.  Defaults to ``sys.stdout``.
    """
    if file is None:
        file = sys.stdout
    sockets = list_sockets()
    if not sockets:
        print("No @dvrk abstract sockets found.", file=file)
        return
    print("Available @dvrk sockets:", file=file)
    for s in sockets:
        print(f"  {s}", file=file)


def check_socket(abstract_name: str, file=None) -> bool:
    """Check whether a socket exists.

    Returns ``True`` if *abstract_name* is found in ``/proc/net/unix``.
    On failure, prints the expected name and lists alternatives.

    Parameters
    ----------
    abstract_name:
        Fully-qualified name to look for, e.g. ``@dvrk:stereo_source:left``.
    file:
        Error output stream.  Defaults to ``sys.stderr``.
    """
    if file is None:
        file = sys.stderr
    sockets = list_sockets()
    if abstract_name in sockets:
        return True
    print(f"Socket not found: {abstract_name}", file=file)
    if not sockets:
        print("No @dvrk sockets are currently active.", file=file)
    else:
        print("Available @dvrk sockets:", file=file)
        for s in sockets:
            print(f"  {s}", file=file)
    return False
