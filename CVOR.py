#!/usr/bin/env python3
"""
VOR / ASRACS / SAAF Airport Monitoring System v6.0.

This module provides a complete, self-contained implementation of a regional
airport monitoring application focused on VOR navigation, SAAF air base
oversight, ASRACS-style ground movement simulation, and approach guidance.

The implementation is designed to run in two modes:

* Full desktop mode when PyQt5, PyQtChart, and optional OpenGL are installed.
* Headless validation mode when only the Python standard library is present.

Version 6.0 highlights:
- Deadlock-safe simulation engines using ``threading.RLock``.
- Event-driven acquisition thread shutdown with no busy looping.
- Expanded African VOR station library.
- SAAF-to-ASRACS workflow integration.
- Terrain-enhanced approach guidance visuals.
- Dark themed desktop UI.
- Headless ``--self-test`` support.
"""

import csv
import logging
import math
import os
import random
import re
import socket
import struct
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from threading import Event, RLock, Thread
from typing import Any, Callable, Deque, Dict, Iterable, List, Optional, Sequence, Tuple

try:
    import numpy as np
    HAS_NUMPY = True
except ImportError:
    np = None
    HAS_NUMPY = False

try:
    import serial
    import serial.tools.list_ports
    HAS_SERIAL = True
except ImportError:
    serial = None
    HAS_SERIAL = False

try:
    import yaml
    HAS_YAML = True
except ImportError:
    yaml = None
    HAS_YAML = False

try:
    from PyQt5.QtChart import QChart, QChartView, QLineSeries
    from PyQt5.QtCore import QPoint, QPointF, QRectF, Qt, QThread, QTimer, pyqtSignal
    from PyQt5.QtGui import (
        QBrush,
        QColor,
        QFont,
        QPainter,
        QPalette,
        QPen,
        QPolygon,
        QPolygonF,
        QStandardItem,
    )
    from PyQt5.QtWidgets import (
        QAction,
        QApplication,
        QComboBox,
        QDialog,
        QFileDialog,
        QGridLayout,
        QGroupBox,
        QHBoxLayout,
        QLabel,
        QLineEdit,
        QListWidget,
        QListWidgetItem,
        QMainWindow,
        QMessageBox,
        QPushButton,
        QScrollArea,
        QSplitter,
        QTabWidget,
        QTableWidget,
        QTableWidgetItem,
        QTextEdit,
        QToolBar,
        QVBoxLayout,
        QWidget,
        QHeaderView,
        QOpenGLWidget,
    )
    HAS_QT = True
except ImportError:
    HAS_QT = False

    class _Signal:
        """Simple signal implementation used in headless mode."""

        def __init__(self) -> None:
            self._subscribers: List[Callable[..., None]] = []

        def connect(self, callback: Callable[..., None]) -> None:
            """Register a callback."""
            self._subscribers.append(callback)

        def emit(self, *args: Any, **kwargs: Any) -> None:
            """Emit the signal to all subscribers."""
            for callback in list(self._subscribers):
                callback(*args, **kwargs)

    def pyqtSignal(*_args: Any, **_kwargs: Any) -> _Signal:
        """Return a headless stand-in signal."""
        return _Signal()

    class Qt:
        """Small subset of Qt enum values used for type-safe fallbacks."""

        AlignCenter = 0
        AlignLeft = 0
        AlignTop = 0
        AlignVCenter = 0
        Horizontal = 0
        Vertical = 1
        black = 0
        white = 1
        red = 2
        yellow = 3
        green = 4
        lightGray = 5
        gray = 6
        NoBrush = 7

    class QWidget:
        """Headless QWidget stub."""

        def __init__(self, *_args: Any, **_kwargs: Any) -> None:
            self._minimum_size = (0, 0)

        def update(self) -> None:
            """Request a repaint in GUI mode."""

        def setMinimumSize(self, width: int, height: int) -> None:
            """Store a minimum size hint."""
            self._minimum_size = (width, height)

        def width(self) -> int:
            """Return a nominal width."""
            return self._minimum_size[0] or 640

        def height(self) -> int:
            """Return a nominal height."""
            return self._minimum_size[1] or 480

    class QMainWindow(QWidget):
        """Headless QMainWindow stub."""

    class QApplication:
        """Headless QApplication stub."""

        def __init__(self, _argv: Sequence[str]) -> None:
            self._argv = list(_argv)

        def setPalette(self, _palette: Any) -> None:
            """No-op in headless mode."""

        def setStyleSheet(self, _stylesheet: str) -> None:
            """No-op in headless mode."""

        def exec_(self) -> int:
            """Return an immediate success code."""
            return 0

    class QThread:
        """Very small QThread-compatible stub."""

        def __init__(self, *_args: Any, **_kwargs: Any) -> None:
            self._thread: Optional[Thread] = None

        def start(self) -> None:
            """Start the worker thread."""
            self._thread = Thread(target=self.run, daemon=True)
            self._thread.start()

        def run(self) -> None:
            """Override point for subclasses."""

        def wait(self, timeout_msecs: Optional[int] = None) -> bool:
            """Wait for the worker thread to complete."""
            if self._thread is None:
                return True
            self._thread.join(None if timeout_msecs is None else timeout_msecs / 1000.0)
            return True

    class QTimer:
        """Minimal timer object for headless mode."""

        def __init__(self, *_args: Any, **_kwargs: Any) -> None:
            self.timeout = _Signal()
            self._active = False
            self.interval_msec = 0

        def start(self, interval_msec: int) -> None:
            """Record timer start state."""
            self.interval_msec = interval_msec
            self._active = True

        def stop(self) -> None:
            """Stop the timer."""
            self._active = False

    class _Dummy:
        """Generic placeholder used for optional Qt types."""

        def __init__(self, *_args: Any, **_kwargs: Any) -> None:
            pass

        def __call__(self, *_args: Any, **_kwargs: Any) -> "_Dummy":
            return self

        def __getattr__(self, _name: str) -> Any:
            return self

        def __iter__(self) -> Iterable[Any]:
            return iter(())

        def count(self) -> int:
            return 0

        def append(self, *_args: Any, **_kwargs: Any) -> None:
            pass

        def remove(self, *_args: Any, **_kwargs: Any) -> None:
            pass

        def removePoints(self, *_args: Any, **_kwargs: Any) -> None:
            pass

    QChart = QChartView = QLineSeries = QAction = QComboBox = QDialog = QFileDialog = _Dummy
    QGridLayout = QGroupBox = QHBoxLayout = QLabel = QLineEdit = QListWidget = QListWidgetItem = _Dummy
    QMessageBox = QPushButton = QScrollArea = QSplitter = QTabWidget = QTableWidget = _Dummy
    QTableWidgetItem = QTextEdit = QToolBar = QVBoxLayout = QHeaderView = QOpenGLWidget = _Dummy
    QPoint = QPointF = QRectF = QBrush = QColor = QFont = QPainter = QPalette = QPen = _Dummy
    QPolygon = QPolygonF = QStandardItem = _Dummy

try:
    from OpenGL.GL import (
        GL_COLOR_BUFFER_BIT,
        GL_DEPTH_BUFFER_BIT,
        GL_LINE_STRIP,
        GL_QUADS,
        glBegin,
        glClear,
        glColor3f,
        glEnd,
        glLoadIdentity,
        glVertex3f,
    )
    from OpenGL.GLU import gluLookAt
    HAS_OPENGL = True
except ImportError:
    HAS_OPENGL = False

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    handlers=[logging.FileHandler("vor_monitor.log"), logging.StreamHandler()],
)
logger = logging.getLogger(__name__)

EARTH_RADIUS_NM = 3440.065
FEET_PER_NM = 6076.12
CONFIG_PATH = Path("vor_config.yaml")
SYSTEM_OPERATIONS_REFERENCE = (
    "CVOR merged operations profile: VOR monitoring, ASRACS surface surveillance, "
    "SAAF readiness overview, and resilient connection failover procedures."
)


MORSE_TABLE: Dict[str, str] = {
    "A": ".-",
    "B": "-...",
    "C": "-.-.",
    "D": "-..",
    "E": ".",
    "F": "..-.",
    "G": "--.",
    "H": "....",
    "I": "..",
    "J": ".---",
    "K": "-.-",
    "L": ".-..",
    "M": "--",
    "N": "-.",
    "O": "---",
    "P": ".--.",
    "Q": "--.-",
    "R": ".-.",
    "S": "...",
    "T": "-",
    "U": "..-",
    "V": "...-",
    "W": ".--",
    "X": "-..-",
    "Y": "-.--",
    "Z": "--..",
    "0": "-----",
    "1": ".----",
    "2": "..---",
    "3": "...--",
    "4": "....-",
    "5": ".....",
    "6": "-....",
    "7": "--...",
    "8": "---..",
    "9": "----.",
}


def morse_encode(text: str) -> str:
    """Return the ITU Morse code string for alphanumeric text."""
    encoded: List[str] = []
    for char in text.upper():
        if char == " ":
            encoded.append("/")
        elif char in MORSE_TABLE:
            encoded.append(MORSE_TABLE[char])
    return " ".join(encoded)


def clamp(value: float, minimum: float, maximum: float) -> float:
    """Clamp a floating-point value to the supplied bounds."""
    return max(minimum, min(maximum, value))


def haversine_nm(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Return great-circle distance between two coordinates in nautical miles."""
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2.0) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2.0) ** 2
    c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))
    return EARTH_RADIUS_NM * c


def great_circle_distance_nm(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Compatibility wrapper around :func:`haversine_nm`."""
    return haversine_nm(lat1, lon1, lat2, lon2)


def initial_bearing(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Return the initial great-circle bearing in degrees."""
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dlambda = math.radians(lon2 - lon1)
    x = math.sin(dlambda) * math.cos(phi2)
    y = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlambda)
    return (math.degrees(math.atan2(x, y)) + 360.0) % 360.0


def destination_point(lat: float, lon: float, bearing_deg: float, distance_nm: float) -> Tuple[float, float]:
    """Project a destination point from start, bearing and distance."""
    d_ratio = max(0.0, distance_nm) / EARTH_RADIUS_NM
    theta = math.radians(bearing_deg)
    phi1 = math.radians(lat)
    lam1 = math.radians(lon)
    phi2 = math.asin(math.sin(phi1) * math.cos(d_ratio) + math.cos(phi1) * math.sin(d_ratio) * math.cos(theta))
    lam2 = lam1 + math.atan2(
        math.sin(theta) * math.sin(d_ratio) * math.cos(phi1),
        math.cos(d_ratio) - math.sin(phi1) * math.sin(phi2),
    )
    lon2 = (math.degrees(lam2) + 540.0) % 360.0 - 180.0
    return math.degrees(phi2), lon2


def parse_runway_heading(runway: str) -> float:
    """Extract a magnetic runway heading in degrees from a runway designator."""
    token = "".join(ch for ch in runway if ch.isdigit())
    if not token:
        return 0.0
    heading = int(token[:2]) * 10.0
    return 360.0 if heading == 0 else heading


def polyline_length(points: Sequence[Tuple[float, float]]) -> float:
    """Return cumulative Euclidean length for a polyline."""
    if len(points) < 2:
        return 0.0
    total = 0.0
    for idx in range(1, len(points)):
        total += math.hypot(points[idx][0] - points[idx - 1][0], points[idx][1] - points[idx - 1][1])
    return total


def interpolate_polyline(points: Sequence[Tuple[float, float]], distance: float) -> Tuple[float, float]:
    """Interpolate a position along a polyline at a specified distance."""
    if not points:
        return 0.0, 0.0
    if len(points) == 1:
        return points[0]
    remain = max(0.0, distance)
    for idx in range(1, len(points)):
        x0, y0 = points[idx - 1]
        x1, y1 = points[idx]
        seg = math.hypot(x1 - x0, y1 - y0)
        if seg <= 0.0:
            continue
        if remain <= seg:
            ratio = remain / seg
            return x0 + (x1 - x0) * ratio, y0 + (y1 - y0) * ratio
        remain -= seg
    return points[-1]


def point_line_distance(point: Tuple[float, float], line_start: Tuple[float, float], line_end: Tuple[float, float]) -> float:
    """Return shortest distance from a point to a line segment."""
    px, py = point
    x1, y1 = line_start
    x2, y2 = line_end
    dx = x2 - x1
    dy = y2 - y1
    if dx == 0.0 and dy == 0.0:
        return math.hypot(px - x1, py - y1)
    t = ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)
    t = clamp(t, 0.0, 1.0)
    proj_x = x1 + t * dx
    proj_y = y1 + t * dy
    return math.hypot(px - proj_x, py - proj_y)


VOR_SENTENCE_RE = re.compile(
    r"^(?P<station>[A-Z0-9_]+),(?P<ident>[A-Z0-9]{2,4}),(?P<freq>\d{3}\.\d{2}),(?P<bearing>-?\d+(?:\.\d+)?),(?P<deviation>-?\d+(?:\.\d+)?),(?P<signal>\d+(?:\.\d+)?),(?P<distance>\d+(?:\.\d+)?)$"
)


def _station(
    icao: str,
    name: str,
    country: str,
    lat: float,
    lon: float,
    elevation_ft: int,
    vor_ident: str,
    vor_freq: float,
    vor_type: str,
    ils_runways: Sequence[Dict[str, Any]],
    ndb_freq: Optional[float],
    ndb_ident: str,
    region: str,
) -> Dict[str, Any]:
    """Build a station metadata dictionary."""
    return {
        "icao": icao,
        "name": name,
        "country": country,
        "lat": lat,
        "lon": lon,
        "elevation_ft": elevation_ft,
        "vor_ident": vor_ident,
        "vor_freq": float(vor_freq),
        "vor_type": vor_type,
        "ils_available": bool(ils_runways),
        "ils_runways": list(ils_runways),
        "ndb_freq": ndb_freq,
        "ndb_ident": ndb_ident,
        "region": region,
    }


def _runway(runway: str, freq: float, course: float, length_m: int, width_m: int = 45) -> Dict[str, Any]:
    """Return a standard runway metadata record."""
    return {
        "runway": runway,
        "frequency": float(freq),
        "course_deg": float(course),
        "glide_slope_deg": 3.0,
        "length_m": int(length_m),
        "width_m": int(width_m),
    }


VOR_STATIONS: Dict[str, Dict[str, Any]] = {
    "JNB": _station(
        "JNB",
        "OR Tambo International",
        "South Africa",
        -26.1337,
        28.2420,
        5558,
        "JNB",
        114.90,
        "VOR/DME",
        [_runway("03L", 110.30, 33.0, 4418)],
        347.0,
        "JS",
        "SA-Civil",
    ),
    "CPT": _station(
        "CPT",
        "Cape Town International",
        "South Africa",
        -33.9696,
        18.5972,
        151,
        "CTV",
        115.70,
        "VORTAC",
        [_runway("01", 110.30, 11.0, 3201), _runway("19", 108.90, 191.0, 3201)],
        382.0,
        "CT",
        "SA-Civil",
    ),
    "DUR": _station(
        "DUR",
        "King Shaka International",
        "South Africa",
        -29.6144,
        31.1197,
        295,
        "DNV",
        112.50,
        "VOR/DME",
        [_runway("06", 109.70, 61.0, 3700), _runway("24", 110.90, 241.0, 3700)],
        393.0,
        "DU",
        "SA-Civil",
    ),
}

SAAF_BASES: Dict[str, Dict[str, Any]] = {
    "FAWK": _station("FAWK", "AFB Waterkloof", "South Africa", -25.8294, 28.2242, 4940, "WKV", 116.90, "VORTAC", [_runway("01", 111.50, 10.0, 4900), _runway("19", 109.30, 190.0, 4900)], 315.0, "WK", "SA-SAAF"),
    "FALM": _station("FALM", "AFB Makhado", "South Africa", -23.1599, 29.6965, 3400, "LTV", 115.00, "VOR/DME", [_runway("10", 110.10, 101.0, 4020), _runway("28", 111.30, 281.0, 4020)], 457.0, "MK", "SA-SAAF"),
    "FAHS": _station("FAHS", "AFB Hoedspruit", "South Africa", -24.3686, 31.0487, 1743, "HSV", 114.00, "VOR/DME", [_runway("09", 109.50, 92.0, 3991), _runway("27", 110.70, 272.0, 3991)], 265.0, "HA", "SA-SAAF"),
    "FALW": _station("FALW", "AFB Langebaanweg", "South Africa", -32.9689, 18.1603, 108, "LWV", 117.00, "VORTAC", [], 345.0, "LW", "SA-SAAF"),
    "FAOB": _station("FAOB", "AFB Overberg", "South Africa", -34.5549, 20.2507, 52, "OBV", 115.40, "VOR/DME", [_runway("35", 110.50, 353.0, 3115), _runway("17", 108.70, 173.0, 3115)], 428.0, "OB", "SA-SAAF"),
    "FASK": _station("FASK", "AFB Swartkop", "South Africa", -25.8097, 28.1648, 4950, "WKV", 116.90, "VORTAC", [], 390.0, "SK", "SA-SAAF"),
    "FABL": _station("FABL", "AFB Bloemspruit", "South Africa", -29.0927, 26.3024, 4458, "BLV", 114.10, "VOR/DME", [_runway("20", 109.90, 201.0, 2559), _runway("02", 108.30, 21.0, 2559)], 380.0, "BL", "SA-SAAF"),
    "FAYP": _station("FAYP", "AFB Ysterplaat", "South Africa", -33.9000, 18.4980, 52, "CTV", 115.70, "VORTAC", [], 301.0, "YP", "SA-SAAF"),
    "FADN": _station("FADN", "AFB Durban", "South Africa", -29.9700, 30.9500, 303, "DNV", 112.50, "VOR/DME", [_runway("06", 109.70, 61.0, 2438), _runway("24", 111.10, 241.0, 2438)], 393.0, "DU", "SA-SAAF"),
    "FAPE": _station("FAPE", "AFS Port Elizabeth", "South Africa", -33.9849, 25.6173, 226, "PEV", 113.40, "VOR/DME", [_runway("08", 110.70, 82.0, 1980), _runway("26", 109.50, 262.0, 1980)], 263.0, "PE", "SA-SAAF"),
}


DEFAULT_SAAF_BASES: Dict[str, Dict[str, Any]] = {
    "FAWK": {"squadrons": ["21 Squadron"], "twr_freq": "118.70", "app_freq": "120.20", "atis_freq": "127.80", "gnd_freq": "121.90", "description": "SAAF strategic transport and VIP operations hub."},
    "FALM": {"squadrons": ["2 Squadron"], "twr_freq": "122.10", "app_freq": "120.90", "atis_freq": "127.00", "gnd_freq": "121.70", "description": "Primary SAAF fighter operations base."},
    "FAHS": {"squadrons": ["17 Squadron"], "twr_freq": "118.30", "app_freq": "120.60", "atis_freq": "126.60", "gnd_freq": "121.80", "description": "Rotary-wing and tactical support base."},
    "FALW": {"squadrons": ["SAAF Training Wing"], "twr_freq": "119.20", "app_freq": "120.40", "atis_freq": "126.20", "gnd_freq": "121.65", "description": "Flight training and maritime patrol support."},
    "FAOB": {"squadrons": ["Test Flight and Development Centre"], "twr_freq": "119.50", "app_freq": "121.20", "atis_freq": "126.90", "gnd_freq": "121.95", "description": "Weapons test and evaluation operations."},
    "FASK": {"squadrons": ["Air Force Museum"], "twr_freq": "118.10", "app_freq": "120.10", "atis_freq": "126.10", "gnd_freq": "121.75", "description": "Historic military field supporting ceremonial flights."},
    "FABL": {"squadrons": ["16 Squadron"], "twr_freq": "118.60", "app_freq": "120.00", "atis_freq": "127.10", "gnd_freq": "121.85", "description": "Central tactical airlift and helicopter support."},
    "FAYP": {"squadrons": ["22 Squadron Detachment"], "twr_freq": "118.40", "app_freq": "120.30", "atis_freq": "126.70", "gnd_freq": "121.60", "description": "Cape Town military support and utility operations."},
    "FADN": {"squadrons": ["Durban Support Unit"], "twr_freq": "118.90", "app_freq": "120.50", "atis_freq": "126.80", "gnd_freq": "121.90", "description": "Eastern seaboard support and logistics node."},
    "FAPE": {"squadrons": ["6 Squadron Detachment"], "twr_freq": "119.00", "app_freq": "120.70", "atis_freq": "127.20", "gnd_freq": "121.95", "description": "Southern coastal maritime and utility support."},
}

for _base_code, _meta in DEFAULT_SAAF_BASES.items():
    if _base_code in SAAF_BASES:
        SAAF_BASES[_base_code].update(_meta)


AFRICAN_VOR_STATIONS: Dict[str, Dict[str, Any]] = {
    "HECA": _station("HECA", "Cairo International", "Egypt", 30.1219, 31.4056, 382, "CAI", 113.90, "VOR/DME", [_runway("05L", 110.10, 51.0, 4000)], None, "", "Africa-North"),
    "HELX": _station("HELX", "Luxor International", "Egypt", 25.6710, 32.7066, 294, "LXR", 115.50, "VOR/DME", [_runway("02", 109.90, 19.0, 3000)], 335.0, "LX", "Africa-North"),
    "HESH": _station("HESH", "Sharm el-Sheikh International", "Egypt", 27.9773, 34.3950, 143, "SSH", 114.40, "VOR/DME", [_runway("22R", 110.90, 224.0, 3081)], None, "", "Africa-North"),
    "HEAX": _station("HEAX", "Alexandria International", "Egypt", 31.1839, 29.9489, 7, "ALX", 113.00, "VOR", [_runway("03", 109.30, 30.0, 2200)], 395.0, "AX", "Africa-North"),
    "HKJK": _station("HKJK", "Jomo Kenyatta International", "Kenya", -1.3192, 36.9278, 5330, "NBI", 114.20, "VOR/DME", [_runway("06", 110.30, 62.0, 4117)], 278.0, "NA", "Africa-East"),
    "HKMO": _station("HKMO", "Moi International", "Kenya", -4.0348, 39.5943, 200, "MBA", 113.50, "VOR/DME", [_runway("21", 109.50, 206.0, 3350)], 399.0, "MB", "Africa-East"),
    "HKEL": _station("HKEL", "Eldoret International", "Kenya", 0.4045, 35.2389, 6942, "ELD", 112.70, "VOR/DME", [_runway("09", 108.90, 87.0, 3475)], None, "", "Africa-East"),
    "DNMM": _station("DNMM", "Murtala Muhammed International", "Nigeria", 6.5774, 3.3212, 135, "LOS", 112.30, "VOR/DME", [_runway("18R", 110.70, 184.0, 3900)], 368.0, "LA", "Africa-West"),
    "DNAA": _station("DNAA", "Nnamdi Azikiwe International", "Nigeria", 9.0068, 7.2632, 1123, "ABV", 114.50, "VOR/DME", [_runway("22", 111.10, 223.0, 3610)], 395.0, "AB", "Africa-West"),
    "DNKN": _station("DNKN", "Mallam Aminu Kano International", "Nigeria", 12.0476, 8.5246, 1562, "KAN", 115.20, "VOR/DME", [_runway("24", 110.50, 236.0, 3300)], 385.0, "KN", "Africa-West"),
    "DNPO": _station("DNPO", "Port Harcourt International", "Nigeria", 5.0155, 6.9496, 87, "PHC", 113.10, "VOR/DME", [_runway("03", 109.10, 26.0, 3000)], 342.0, "PH", "Africa-West"),
    "HAAB": _station("HAAB", "Addis Ababa Bole International", "Ethiopia", 8.9779, 38.7993, 7657, "ADD", 114.80, "VOR/DME", [_runway("07L", 110.50, 71.0, 3800)], 370.0, "AD", "Africa-East"),
    "HADR": _station("HADR", "Dire Dawa International", "Ethiopia", 9.6247, 41.8542, 3826, "DIR", 112.60, "VOR/DME", [_runway("08", 109.10, 79.0, 2679)], 382.0, "DD", "Africa-East"),
    "HTDA": _station("HTDA", "Julius Nyerere International", "Tanzania", -6.8781, 39.2026, 182, "DAR", 113.70, "VOR/DME", [_runway("23", 110.30, 229.0, 3000)], 410.0, "DR", "Africa-East"),
    "HTKJ": _station("HTKJ", "Kilimanjaro International", "Tanzania", -3.4294, 37.0745, 2932, "KIA", 115.10, "VOR/DME", [_runway("09", 108.70, 91.0, 3600)], None, "", "Africa-East"),
    "DGAA": _station("DGAA", "Kotoka International", "Ghana", 5.6052, -0.1668, 205, "ACC", 114.30, "VOR/DME", [_runway("03", 109.50, 33.0, 3403)], 385.0, "AC", "Africa-West"),
    "GMMN": _station("GMMN", "Mohammed V International", "Morocco", 33.3675, -7.5899, 656, "CAS", 114.10, "VOR/DME", [_runway("36", 110.10, 356.0, 3720)], 399.0, "CS", "Africa-North"),
    "GMMX": _station("GMMX", "Marrakech Menara", "Morocco", 31.6069, -8.0363, 1545, "RAK", 113.80, "VOR/DME", [_runway("10", 109.30, 100.0, 3100)], 332.0, "RK", "Africa-North"),
    "GMME": _station("GMME", "Rabat-Salé", "Morocco", 34.0515, -6.7515, 276, "RBA", 112.90, "VOR/DME", [_runway("35", 108.90, 350.0, 3500)], None, "", "Africa-North"),
    "DTTA": _station("DTTA", "Tunis-Carthage", "Tunisia", 36.8510, 10.2272, 22, "TUN", 115.50, "VOR/DME", [_runway("01", 110.30, 12.0, 3200)], 410.0, "TU", "Africa-North"),
    "FNLU": _station("FNLU", "Quatro de Fevereiro", "Angola", -8.8584, 13.2312, 243, "LAD", 114.70, "VOR/DME", [_runway("24", 110.90, 245.0, 3716)], 365.0, "LD", "Africa-South"),
    "FQMA": _station("FQMA", "Maputo International", "Mozambique", -25.9208, 32.5726, 145, "MPM", 113.60, "VOR/DME", [_runway("05", 109.10, 52.0, 3660)], 402.0, "MP", "Africa-South"),
    "FVHA": _station("FVHA", "Robert Gabriel Mugabe International", "Zimbabwe", -17.9318, 31.0928, 4887, "HRE", 115.30, "VOR/DME", [_runway("06", 110.50, 59.0, 4725)], 395.0, "HR", "Africa-South"),
    "FVBU": _station("FVBU", "Joshua Mqabuko Nkomo International", "Zimbabwe", -20.0174, 28.6179, 4359, "BUQ", 113.90, "VOR/DME", [_runway("07", 108.70, 74.0, 2588)], 420.0, "BQ", "Africa-South"),
    "FLLS": _station("FLLS", "Kenneth Kaunda International", "Zambia", -15.3308, 28.4526, 3779, "LUN", 114.50, "VOR/DME", [_runway("10", 110.10, 95.0, 3962)], 399.0, "LU", "Africa-South"),
    "FBSK": _station("FBSK", "Sir Seretse Khama International", "Botswana", -24.5552, 25.9182, 3299, "GBE", 113.20, "VOR/DME", [_runway("08", 109.30, 82.0, 4000)], None, "", "Africa-South"),
    "FYWH": _station("FYWH", "Windhoek Hosea Kutako International", "Namibia", -22.4799, 17.4709, 5640, "WDH", 114.60, "VOR/DME", [_runway("07", 110.30, 79.0, 4532)], 335.0, "WH", "Africa-South"),
    "GOOO": _station("GOOO", "Blaise Diagne International", "Senegal", 14.6708, -17.0733, 290, "DKR", 115.00, "VOR/DME", [_runway("03", 109.50, 33.0, 3500)], 410.0, "DK", "Africa-West"),
    "DIAP": _station("DIAP", "Félix-Houphouët-Boigny International", "Côte d'Ivoire", 5.2614, -3.9263, 21, "ABJ", 114.40, "VOR/DME", [_runway("03", 110.10, 29.0, 3000)], 345.0, "AB", "Africa-West"),
    "FKKD": _station("FKKD", "Douala International", "Cameroon", 4.0061, 9.7195, 33, "DLA", 113.80, "VOR/DME", [_runway("30", 109.30, 297.0, 2850)], 415.0, "DL", "Africa-Central"),
    "FKYS": _station("FKYS", "Yaoundé Nsimalen International", "Cameroon", 3.7226, 11.5533, 2278, "YAO", 114.10, "VOR/DME", [_runway("21", 108.70, 213.0, 3400)], None, "", "Africa-Central"),
    "FZAA": _station("FZAA", "N'djili International", "DRC", -4.3857, 15.4446, 1027, "FIH", 113.50, "VOR/DME", [_runway("24", 109.50, 241.0, 4700)], 372.0, "FI", "Africa-Central"),
    "FZQA": _station("FZQA", "Lubumbashi International", "DRC", -11.5913, 27.5309, 4295, "FBM", 115.20, "VOR/DME", [_runway("09", 110.30, 92.0, 3200)], 410.0, "FB", "Africa-Central"),
    "HSSS": _station("HSSS", "Khartoum International", "Sudan", 15.5895, 32.5532, 1265, "KRT", 116.00, "VOR/DME", [_runway("36", 110.50, 356.0, 4000)], 375.0, "KH", "Africa-North"),
    "FMMI": _station("FMMI", "Ivato International", "Madagascar", -18.7969, 47.4788, 4198, "TNR", 113.30, "VOR/DME", [_runway("12", 109.10, 119.0, 3100)], 335.0, "TN", "Africa-South"),
    "HRYR": _station("HRYR", "Kigali International", "Rwanda", -1.9686, 30.1395, 4917, "KGL", 114.90, "VOR/DME", [_runway("10", 110.10, 101.0, 3500)], 405.0, "KG", "Africa-East"),
    "HUEN": _station("HUEN", "Entebbe International", "Uganda", 0.0424, 32.4435, 3782, "EBB", 113.70, "VOR/DME", [_runway("17", 109.50, 173.0, 3658)], 330.0, "EB", "Africa-East"),
    "DAAG": _station("DAAG", "Houari Boumediene", "Algeria", 36.6910, 3.2154, 82, "ALG", 114.20, "VOR/DME", [_runway("05", 110.50, 50.0, 3500)], 400.0, "AG", "Africa-North"),
    "DAOO": _station("DAOO", "Ahmed Ben Bella", "Algeria", 35.6239, -0.6212, 295, "ORN", 112.80, "VOR/DME", [_runway("36", 109.10, 360.0, 3060)], 345.0, "OR", "Africa-North"),
    "HLLT": _station("HLLT", "Tripoli Mitiga", "Libya", 32.8947, 13.2760, 36, "TIP", 115.80, "VOR/DME", [_runway("09", 110.90, 93.0, 3600)], 410.0, "TP", "Africa-North"),
    "HLLB": _station("HLLB", "Benina International", "Libya", 32.0968, 20.2695, 433, "BEN", 113.40, "VOR/DME", [_runway("27", 109.30, 274.0, 3500)], None, "", "Africa-North"),
}


def _merge_station_libraries() -> Dict[str, Dict[str, Any]]:
    """Return the merged master station dictionary."""
    merged = dict(VOR_STATIONS)
    merged.update(SAAF_BASES)
    merged.update(AFRICAN_VOR_STATIONS)
    return merged


VOR_STATIONS = _merge_station_libraries()


REGION_GROUPS: Dict[str, List[str]] = {
    "SA-Civil": ["JNB", "CPT", "DUR"],
    "SA-SAAF": sorted(SAAF_BASES.keys()),
    "Africa-East": ["HKJK", "HKMO", "HKEL", "HAAB", "HADR", "HTDA", "HTKJ", "HRYR", "HUEN"],
    "Africa-North": ["HECA", "HELX", "HESH", "HEAX", "GMMN", "GMMX", "GMME", "DTTA", "HSSS", "DAAG", "DAOO", "HLLT", "HLLB"],
    "Africa-West": ["DNMM", "DNAA", "DNKN", "DNPO", "DGAA", "GOOO", "DIAP"],
    "Africa-Central": ["FKKD", "FKYS", "FZAA", "FZQA"],
    "Africa-South": ["FNLU", "FQMA", "FVHA", "FVBU", "FLLS", "FBSK", "FYWH", "FMMI"],
}

DEFAULT_AIRPORTS: List[str] = ["JNB", "CPT", "DUR", "FAWK", "FALM", "FAHS", "FAPE"]


def _layout(
    name: str,
    runways: Sequence[Dict[str, Any]],
    taxiways: Sequence[List[Tuple[int, int]]],
    aprons: Sequence[Tuple[int, int, int, int]],
    gates: Sequence[Tuple[str, int, int]],
    hotspots: Sequence[Tuple[str, int, int, str]],
    bounds: Tuple[int, int, int, int],
) -> Dict[str, Any]:
    """Build a simple airfield layout description."""
    return {
        "name": name,
        "runways": list(runways),
        "taxiways": list(taxiways),
        "aprons": list(aprons),
        "gates": list(gates),
        "hotspots": list(hotspots),
        "bounds": tuple(bounds),
    }


JNB_LAYOUT_DATA: Dict[str, Any] = _layout(
    "OR Tambo International",
    [{"name": "03L/21R", "start": (170, 60), "end": (340, 360), "width": 18}, {"name": "03R/21L", "start": (110, 50), "end": (280, 350), "width": 16}],
    [[(120, 180), (200, 180), (280, 160), (330, 120)], [(110, 250), (190, 250), (260, 240), (320, 220)], [(145, 320), (180, 280), (210, 250)]],
    [(60, 130, 80, 90), (55, 235, 90, 80)],
    [("A1", 75, 150), ("A2", 95, 185), ("B1", 85, 250), ("B2", 110, 280)],
    [("H1", 175, 180, "Runway crossing"), ("H2", 250, 235, "Parallel taxi conflict")],
    (0, 0, 420, 420),
)

AIRFIELD_LAYOUTS: Dict[str, Dict[str, Any]] = {
    "JNB": JNB_LAYOUT_DATA,
    "FAWK": _layout(
        "AFB Waterkloof",
        [{"name": "01/19", "start": (210, 30), "end": (230, 390), "width": 20}],
        [[(140, 90), (180, 120), (210, 160)], [(260, 110), (240, 150), (230, 210)], [(150, 300), (190, 280), (230, 260)]],
        [(90, 70, 80, 90), (250, 70, 80, 90), (90, 270, 90, 70)],
        [("HGR1", 100, 90), ("HGR2", 130, 110), ("MIL1", 280, 105), ("VIP", 120, 300)],
        [("HS1", 205, 150, "Rapid exit"), ("HS2", 225, 255, "Run-up queue")],
        (0, 0, 400, 420),
    ),
    "FALM": _layout(
        "AFB Makhado",
        [{"name": "10/28", "start": (40, 210), "end": (380, 170), "width": 22}],
        [[(100, 250), (150, 230), (220, 210)], [(110, 120), (160, 150), (230, 180)], [(250, 260), (280, 220), (320, 190)]],
        [(60, 250, 80, 70), (80, 90, 85, 60), (250, 250, 90, 80)],
        [("FGT1", 80, 275), ("FGT2", 120, 292), ("OPS1", 100, 115), ("TEST1", 285, 285)],
        [("HS1", 160, 225, "Crossing active runway"), ("HS2", 260, 205, "Intersection hold")],
        (0, 0, 420, 360),
    ),
    "FAHS": _layout(
        "AFB Hoedspruit",
        [{"name": "09/27", "start": (30, 200), "end": (390, 200), "width": 22}],
        [[(80, 120), (140, 150), (200, 180)], [(210, 230), (260, 230), (320, 220)], [(70, 280), (130, 250), (200, 220)]],
        [(65, 80, 90, 60), (260, 235, 80, 70), (65, 260, 70, 50)],
        [("RH1", 80, 105), ("RH2", 115, 120), ("OPS", 280, 260), ("SAR", 90, 280)],
        [("HS1", 190, 180, "Runway entry"), ("HS2", 245, 220, "Helicopter crossing")],
        (0, 0, 420, 340),
    ),
    "FALW": _layout(
        "AFB Langebaanweg",
        [{"name": "01/19", "start": (200, 40), "end": (225, 360), "width": 16}],
        [[(140, 120), (180, 150), (205, 180)], [(240, 130), (230, 180), (220, 230)], [(130, 280), (170, 250), (205, 230)]],
        [(80, 90, 85, 70), (250, 90, 70, 70), (90, 270, 75, 55)],
        [("TRG1", 100, 120), ("TRG2", 130, 135), ("SIM", 265, 110), ("FLY", 110, 290)],
        [("HS1", 205, 165, "Training circuit")],
        (0, 0, 380, 380),
    ),
    "FAOB": _layout(
        "AFB Overberg",
        [{"name": "17/35", "start": (210, 30), "end": (220, 380), "width": 18}],
        [[(130, 120), (170, 145), (210, 180)], [(240, 120), (230, 170), (220, 220)], [(120, 310), (170, 280), (220, 250)]],
        [(75, 80, 80, 60), (260, 85, 60, 60), (80, 290, 90, 55)],
        [("T&E1", 90, 110), ("T&E2", 120, 125), ("UAV", 275, 110), ("RNG", 110, 312)],
        [("HS1", 208, 180, "Weapons test crossing")],
        (0, 0, 380, 400),
    ),
    "FASK": _layout(
        "AFB Swartkop",
        [{"name": "02/20", "start": (180, 40), "end": (240, 360), "width": 16}],
        [[(110, 140), (150, 165), (195, 205)], [(245, 110), (235, 155), (220, 205)], [(110, 285), (160, 260), (210, 240)]],
        [(70, 110, 70, 55), (255, 75, 70, 60), (90, 280, 80, 50)],
        [("MUS1", 85, 130), ("MUS2", 115, 142), ("LEG1", 275, 100), ("APR3", 105, 300)],
        [("HS1", 195, 190, "Museum crossing")],
        (0, 0, 380, 390),
    ),
    "FABL": _layout(
        "AFB Bloemspruit",
        [{"name": "02/20", "start": (170, 50), "end": (250, 360), "width": 18}],
        [[(95, 150), (140, 175), (185, 205)], [(250, 125), (240, 175), (225, 215)], [(110, 285), (160, 265), (220, 250)]],
        [(60, 120, 80, 60), (260, 100, 70, 65), (90, 280, 90, 60)],
        [("TRN1", 78, 145), ("TRN2", 112, 160), ("FRT", 280, 120), ("CARGO", 110, 304)],
        [("HS1", 188, 200, "Civil-military merge")],
        (0, 0, 400, 390),
    ),
    "FAYP": _layout(
        "AFB Ysterplaat",
        [{"name": "02/20", "start": (160, 40), "end": (240, 330), "width": 16}],
        [[(90, 140), (130, 165), (170, 200)], [(250, 120), (235, 160), (220, 200)], [(95, 250), (150, 235), (205, 220)]],
        [(65, 100, 85, 55), (255, 95, 60, 55), (80, 245, 80, 45)],
        [("NAVY1", 85, 122), ("NAVY2", 110, 133), ("LINE", 270, 110), ("HEL1", 100, 260)],
        [("HS1", 175, 190, "Short-field crossing")],
        (0, 0, 360, 360),
    ),
    "FADN": _layout(
        "AFB Durban",
        [{"name": "06/24", "start": (40, 260), "end": (370, 120), "width": 18}],
        [[(100, 300), (150, 275), (210, 235)], [(95, 170), (150, 190), (220, 215)], [(250, 260), (290, 220), (330, 180)]],
        [(55, 290, 90, 55), (60, 140, 85, 50), (260, 250, 70, 60)],
        [("SEA1", 80, 310), ("SEA2", 110, 320), ("OPS", 90, 160), ("RAMP", 280, 275)],
        [("HS1", 180, 250, "Runway merge"), ("HS2", 255, 210, "Crosswind taxi")],
        (0, 0, 410, 360),
    ),
    "FAPE": _layout(
        "AFS Port Elizabeth",
        [{"name": "08/26", "start": (40, 220), "end": (370, 180), "width": 18}],
        [[(90, 270), (150, 250), (215, 225)], [(90, 120), (150, 150), (220, 175)], [(250, 260), (300, 225), (345, 195)]],
        [(55, 260, 80, 60), (55, 100, 85, 55), (260, 255, 70, 55)],
        [("MAR1", 78, 285), ("MAR2", 110, 295), ("CIV", 100, 123), ("RAMP", 280, 278)],
        [("HS1", 205, 220, "Parallel access")],
        (0, 0, 400, 350),
    ),
    "HKJK": _layout(
        "Jomo Kenyatta International",
        [{"name": "06/24", "start": (40, 260), "end": (380, 110), "width": 22}],
        [[(80, 300), (140, 270), (210, 240)], [(95, 160), (170, 190), (240, 215)], [(250, 280), (300, 240), (350, 185)]],
        [(45, 280, 90, 60), (65, 130, 95, 55), (260, 270, 80, 60)],
        [("G1", 72, 300), ("G2", 108, 312), ("T1", 95, 150), ("C1", 280, 290)],
        [("HS1", 185, 245, "Crossing at terminal"), ("HS2", 275, 220, "Cargo entrance")],
        (0, 0, 430, 360),
    ),
    "HECA": _layout(
        "Cairo International",
        [{"name": "05L/23R", "start": (40, 230), "end": (380, 140), "width": 22}, {"name": "05R/23L", "start": (50, 280), "end": (390, 190), "width": 20}],
        [[(85, 320), (150, 285), (220, 255)], [(90, 170), (165, 190), (235, 205)], [(250, 310), (300, 270), (350, 235)]],
        [(45, 300, 90, 60), (60, 150, 100, 55), (270, 300, 80, 60)],
        [("A", 70, 320), ("B", 102, 330), ("C", 105, 170), ("D", 290, 320)],
        [("HS1", 190, 262, "Runway crossing"), ("HS2", 245, 225, "Parallel runway hotspot")],
        (0, 0, 440, 380),
    ),
    "DNMM": _layout(
        "Murtala Muhammed International",
        [{"name": "18R/36L", "start": (190, 30), "end": (190, 390), "width": 18}, {"name": "18L/36R", "start": (240, 30), "end": (240, 390), "width": 18}],
        [[(120, 110), (160, 150), (190, 190)], [(270, 110), (255, 150), (240, 200)], [(120, 290), (165, 260), (215, 230)]],
        [(55, 80, 90, 70), (285, 80, 80, 70), (70, 280, 95, 60)],
        [("G10", 80, 105), ("G11", 110, 120), ("G20", 310, 110), ("CARGO", 92, 305)],
        [("HS1", 190, 170, "Close parallel runway"), ("HS2", 235, 215, "Midfield crossing")],
        (0, 0, 420, 420),
    ),
    "HAAB": _layout(
        "Addis Ababa Bole International",
        [{"name": "07L/25R", "start": (40, 280), "end": (390, 150), "width": 22}],
        [[(90, 330), (150, 300), (215, 270)], [(90, 175), (160, 200), (240, 225)], [(260, 300), (310, 260), (355, 210)]],
        [(50, 310, 90, 60), (60, 150, 95, 55), (270, 295, 75, 60)],
        [("A1", 75, 325), ("A2", 105, 335), ("B1", 100, 170), ("C1", 290, 310)],
        [("HS1", 185, 275, "High-altitude traffic merge")],
        (0, 0, 430, 390),
    ),
    "GMMN": _layout(
        "Mohammed V International",
        [{"name": "17L/35R", "start": (170, 30), "end": (210, 390), "width": 18}, {"name": "17R/35L", "start": (230, 30), "end": (270, 390), "width": 18}],
        [[(100, 100), (145, 145), (185, 190)], [(300, 100), (285, 145), (255, 190)], [(100, 320), (160, 290), (220, 250)]],
        [(50, 75, 85, 70), (305, 75, 70, 70), (70, 315, 90, 60)],
        [("T1", 72, 100), ("T2", 105, 115), ("T3", 325, 100), ("C1", 95, 338)],
        [("HS1", 190, 185, "Parallel runway crossing"), ("HS2", 240, 200, "Rapid exit merge")],
        (0, 0, 420, 430),
    ),
}


TERRAIN_LIBRARY: Dict[str, List[int]] = {
    "FAWK": [4860, 4872, 4890, 4910, 4925, 4940, 4955, 4960, 4952, 4946, 4932, 4920, 4915, 4900, 4888],
    "FALM": [3250, 3285, 3310, 3340, 3362, 3385, 3400, 3420, 3435, 3450, 3440, 3422, 3398, 3370, 3345],
    "FAHS": [1680, 1695, 1704, 1712, 1721, 1730, 1738, 1743, 1740, 1736, 1729, 1721, 1715, 1706, 1698],
    "FALW": [82, 84, 88, 91, 96, 100, 105, 108, 104, 101, 98, 94, 90, 87, 84],
    "FAOB": [25, 28, 32, 36, 40, 45, 49, 52, 56, 60, 58, 54, 49, 45, 38],
    "FASK": [4865, 4878, 4895, 4912, 4930, 4948, 4956, 4950, 4940, 4931, 4922, 4910, 4900, 4887, 4875],
    "FABL": [4400, 4410, 4425, 4438, 4449, 4458, 4465, 4470, 4468, 4460, 4452, 4444, 4433, 4420, 4410],
    "FAYP": [30, 34, 39, 44, 47, 50, 52, 50, 47, 44, 41, 38, 35, 33, 31],
    "FADN": [260, 268, 276, 285, 292, 300, 303, 306, 304, 301, 296, 290, 284, 275, 268],
    "FAPE": [190, 198, 205, 211, 218, 223, 226, 229, 227, 223, 218, 212, 206, 198, 192],
    "HKJK": [5205, 5230, 5260, 5285, 5302, 5320, 5330, 5340, 5350, 5342, 5328, 5310, 5290, 5268, 5240],
    "HECA": [345, 350, 356, 361, 368, 374, 382, 388, 392, 390, 385, 378, 370, 362, 355],
    "DNMM": [102, 108, 114, 120, 126, 131, 135, 138, 140, 139, 136, 131, 125, 118, 112],
    "HAAB": [7420, 7460, 7500, 7545, 7585, 7615, 7657, 7680, 7705, 7692, 7665, 7622, 7580, 7538, 7495],
    "GMMN": [598, 605, 614, 622, 631, 642, 656, 665, 671, 668, 661, 650, 639, 625, 612],
}


@dataclass
class VORDataRecord:
    """Container for a single VOR telemetry sample."""

    timestamp: datetime
    station_key: str
    ident: str
    frequency_mhz: float
    bearing_deg: float
    deviation_deg: float
    signal_percent: float
    distance_nm: float
    dme_nm: Optional[float] = None
    flags: Dict[str, Any] = field(default_factory=dict)
    raw_line: str = ""

    def is_signal_valid(self) -> bool:
        """Return ``True`` when the sample should be considered usable."""
        return self.signal_percent >= 25.0 and self.flags.get("status", "OK") != "FAIL"

    def to_csv_row(self) -> List[str]:
        """Return the record serialized as a CSV row."""
        return [
            self.timestamp.isoformat(),
            self.station_key,
            self.ident,
            f"{self.frequency_mhz:.2f}",
            f"{self.bearing_deg:.2f}",
            f"{self.deviation_deg:.2f}",
            f"{self.signal_percent:.2f}",
            f"{self.distance_nm:.2f}",
            "" if self.dme_nm is None else f"{self.dme_nm:.2f}",
            self.raw_line,
        ]

    @classmethod
    def from_sentence(cls, line: str) -> "VORDataRecord":
        """Parse a telemetry sentence into a :class:`VORDataRecord`."""
        match = VOR_SENTENCE_RE.match(line.strip())
        if not match:
            raise ValueError(f"Unsupported VOR sentence: {line!r}")
        return cls(
            timestamp=datetime.now(timezone.utc),
            station_key=match.group("station"),
            ident=match.group("ident"),
            frequency_mhz=float(match.group("freq")),
            bearing_deg=float(match.group("bearing")),
            deviation_deg=float(match.group("deviation")),
            signal_percent=float(match.group("signal")),
            distance_nm=float(match.group("distance")),
            dme_nm=float(match.group("distance")),
            raw_line=line.strip(),
        )


class GlideSlopeDetector:
    """Compute ideal ILS glide path and deviation information."""

    def __init__(self, runway_elevation_ft: float = 0.0, glide_slope_deg: float = 3.0, threshold_crossing_height_ft: float = 50.0) -> None:
        """Initialize the detector with runway and glide path parameters."""
        self.runway_elevation_ft = float(runway_elevation_ft)
        self.glide_slope_deg = float(glide_slope_deg)
        self.threshold_crossing_height_ft = float(threshold_crossing_height_ft)

    def target_altitude(self, distance_nm: float) -> float:
        """Return the target MSL altitude for a given final approach distance."""
        distance_ft = max(0.0, float(distance_nm)) * FEET_PER_NM
        return self.runway_elevation_ft + self.threshold_crossing_height_ft + math.tan(math.radians(self.glide_slope_deg)) * distance_ft

    def deviation_from_path(self, distance_nm: float, altitude_msl_ft: float) -> float:
        """Return the vertical deviation from the nominal glide path in feet."""
        return float(altitude_msl_ft) - self.target_altitude(distance_nm)

    def status_label(self, distance_nm: float, altitude_msl_ft: float, tolerance_ft: float = 75.0) -> str:
        """Return a textual status for the supplied approach geometry."""
        deviation = self.deviation_from_path(distance_nm, altitude_msl_ft)
        if deviation > tolerance_ft:
            return "HIGH"
        if deviation < -tolerance_ft:
            return "LOW"
        return "ON GS"

    def ideal_altitude_ft(self, distance_nm: float, threshold_elev_ft: float = 0.0) -> float:
        """Compatibility helper returning ideal glide altitude."""
        return self.target_altitude(distance_nm) + float(threshold_elev_ft)

    def is_on_glide_slope(self, distance_nm: float, altitude_msl_ft: float, tolerance_ft: float = 75.0) -> bool:
        """Return True when the aircraft is within the glide-path tolerance."""
        return abs(self.deviation_from_path(distance_nm, altitude_msl_ft)) <= float(tolerance_ft)

    def required_descent_rate_fpm(self, groundspeed_kts: float) -> float:
        """Estimate required descent rate in feet-per-minute."""
        return max(300.0, float(groundspeed_kts) * 5.0 * (self.glide_slope_deg / 3.0))

    def validate_profile(self, samples: Sequence[Tuple[float, float]], tolerance_ft: float = 150.0) -> bool:
        """Validate an approach profile of ``(distance_nm, altitude_ft)`` samples."""
        for distance_nm, altitude_ft in samples:
            if abs(self.deviation_from_path(distance_nm, altitude_ft)) > tolerance_ft:
                return False
        return True


class SurfaceSlopeAnalyzer:
    """Analyze terrain profiles for surface slope and gradient hazards."""

    CAUTION_THRESHOLD = 2.5
    CRITICAL_THRESHOLD = 4.0

    def __init__(self, profile: Optional[Sequence[float]] = None) -> None:
        """Create the analyzer with an optional terrain profile."""
        self.profile = list(profile or [])

    def set_profile(self, profile: Sequence[float]) -> None:
        """Replace the active terrain profile."""
        self.profile = list(profile)

    def average_slope_percent(self) -> float:
        """Return the average end-to-end slope percentage."""
        if len(self.profile) < 2:
            return 0.0
        rise = self.profile[-1] - self.profile[0]
        run = max(1.0, len(self.profile) - 1)
        return (rise / run) * 100.0 / 1000.0

    def classify(self) -> str:
        """Return a qualitative slope description."""
        slope = abs(self.average_slope_percent())
        if slope < 1.0:
            return "FLAT"
        if slope < self.CAUTION_THRESHOLD:
            return "MODERATE"
        if slope < self.CRITICAL_THRESHOLD:
            return "CAUTION"
        return "CRITICAL"

    def obstacles_above(self, reference_ft: float) -> List[float]:
        """Return terrain samples above a reference altitude."""
        return [sample for sample in self.profile if sample > reference_ft]

    def analyze_profile(self, profile: Optional[Sequence[float]] = None) -> Dict[str, Any]:
        """Return aggregate terrain profile analysis."""
        if profile is not None:
            self.set_profile(profile)
        return {
            "sample_count": len(self.profile),
            "average_slope_percent": self.average_slope_percent(),
            "classification": self.classify(),
            "highest_ft": max(self.profile) if self.profile else 0.0,
            "lowest_ft": min(self.profile) if self.profile else 0.0,
        }

    def assess_approach_safety(self, threshold_altitude_ft: float = 0.0) -> Dict[str, Any]:
        """Assess approach safety using slope and obstacle checks."""
        analysis = self.analyze_profile()
        obstacles = self.obstacles_above(threshold_altitude_ft)
        risk = "LOW"
        if analysis["classification"] in {"CAUTION", "CRITICAL"}:
            risk = "MEDIUM"
        if analysis["classification"] == "CRITICAL" or len(obstacles) > max(2, len(self.profile) // 4):
            risk = "HIGH"
        return {
            "risk": risk,
            "analysis": analysis,
            "obstacle_count": len(obstacles),
        }


class LDAFileParser:
    """Parse light-weight line data archives containing VOR samples."""

    def __init__(self, source: Optional[Path] = None) -> None:
        """Initialize the parser with an optional source path."""
        self.source = Path(source) if source else None

    def parse_lines(self, lines: Iterable[str]) -> List[VORDataRecord]:
        """Parse an iterable of archive lines into records."""
        records: List[VORDataRecord] = []
        for raw in lines:
            line = raw.strip()
            if not line or line.startswith("#"):
                continue
            try:
                if "," in line and VOR_SENTENCE_RE.match(line):
                    records.append(VORDataRecord.from_sentence(line))
                    continue
                parts = [field.strip() for field in re.split(r"[;,\t]", line)]
                if len(parts) >= 7:
                    records.append(
                        VORDataRecord(
                            timestamp=datetime.now(timezone.utc),
                            station_key=parts[0],
                            ident=parts[1],
                            frequency_mhz=float(parts[2]),
                            bearing_deg=float(parts[3]),
                            deviation_deg=float(parts[4]),
                            signal_percent=float(parts[5]),
                            distance_nm=float(parts[6]),
                            dme_nm=float(parts[6]),
                            raw_line=line,
                        )
                    )
            except (TypeError, ValueError) as exc:
                logger.warning("Skipping malformed LDA line %r: %s", line, exc)
        return records

    def parse_file(self, path: Optional[Path] = None) -> List[VORDataRecord]:
        """Parse the configured file or the supplied path."""
        target = Path(path) if path else self.source
        if target is None:
            return []
        try:
            with target.open("r", encoding="utf-8") as handle:
                return self.parse_lines(handle)
        except OSError as exc:
            logger.error("Failed to parse LDA file %s: %s", target, exc)
            return []


class LDABinaryFileParser:
    """Binary+text LDA parser retained for compatibility with coded exports."""

    def __init__(self, source: Optional[Path] = None) -> None:
        self.source = Path(source) if source else None

    def parse_file(self, path: Optional[Path] = None) -> List[VORDataRecord]:
        """Parse LDA payload from text lines or compact binary records."""
        target = Path(path) if path else self.source
        if target is None:
            return []
        try:
            payload = target.read_bytes()
        except OSError as exc:
            logger.error("Failed to read LDA file %s: %s", target, exc)
            return []

        if b"\n" in payload or payload.startswith(b"#") or payload.startswith(b"$VOR"):
            text = payload.decode("utf-8", errors="ignore")
            return LDAFileParser().parse_lines(text.splitlines())

        records: List[VORDataRecord] = []
        stride = struct.calcsize("<4sfffff")
        if stride <= 0:
            return records
        for offset in range(0, len(payload) - stride + 1, stride):
            ident_raw, freq, bearing, deviation, signal, distance = struct.unpack("<4sfffff", payload[offset : offset + stride])
            ident = ident_raw.decode("ascii", errors="ignore").strip("\x00 ") or "LDA"
            records.append(
                VORDataRecord(
                    timestamp=datetime.now(timezone.utc),
                    station_key=ident,
                    ident=ident,
                    frequency_mhz=float(freq),
                    bearing_deg=float(bearing),
                    deviation_deg=float(deviation),
                    signal_percent=clamp(float(signal), 0.0, 100.0),
                    distance_nm=max(0.0, float(distance)),
                    dme_nm=max(0.0, float(distance)),
                    raw_line=f"{ident},{freq:.2f},{bearing:.2f},{deviation:.3f},{signal:.1f},{distance:.2f}",
                )
            )
        return records


class VORConnectionHandler:
    """Manage serial, TCP, or synthetic VOR data acquisition sources."""

    def __init__(self) -> None:
        """Create a disconnected handler."""
        self._lock = RLock()
        self._serial_conn: Any = None
        self._tcp_conn: Optional[socket.socket] = None
        self._mock_source: Optional[Callable[[], Optional[str]]] = None
        self._buffer: Deque[str] = deque(maxlen=128)

    def available_serial_ports(self) -> List[str]:
        """Return the names of available serial ports."""
        if not HAS_SERIAL:
            return []
        try:
            return [port.device for port in serial.tools.list_ports.comports()]
        except (OSError, serial.SerialException) as exc:  # type: ignore[attr-defined]
            logger.warning("Failed to enumerate serial ports: %s", exc)
            return []

    def connect_serial(self, port: str, baudrate: int = 9600, timeout: float = 0.2) -> bool:
        """Open a serial connection."""
        if not HAS_SERIAL:
            logger.error("pyserial is not installed")
            return False
        with self._lock:
            self.disconnect()
            try:
                self._serial_conn = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
                return True
            except (OSError, ValueError, serial.SerialException) as exc:  # type: ignore[attr-defined]
                logger.error("Unable to open serial port %s: %s", port, exc)
                self._serial_conn = None
                return False

    def connect_tcp(self, host: str, port: int, timeout: float = 0.5) -> bool:
        """Open a TCP connection."""
        with self._lock:
            self.disconnect()
            try:
                self._tcp_conn = socket.create_connection((host, int(port)), timeout=timeout)
                self._tcp_conn.settimeout(timeout)
                return True
            except OSError as exc:
                logger.error("Unable to connect to %s:%s: %s", host, port, exc)
                self._tcp_conn = None
                return False

    def set_mock_source(self, callback: Optional[Callable[[], Optional[str]]]) -> None:
        """Set a callback used to generate synthetic text lines."""
        with self._lock:
            self._mock_source = callback

    def write_command(self, command: str) -> bool:
        """Write a command to the active physical link."""
        payload = (command.strip() + "\n").encode("utf-8")
        with self._lock:
            try:
                if self._serial_conn is not None:
                    self._serial_conn.write(payload)
                    return True
                if self._tcp_conn is not None:
                    self._tcp_conn.sendall(payload)
                    return True
            except (OSError, AttributeError) as exc:
                logger.error("Failed to write command %s: %s", command, exc)
        return False

    def readline(self) -> Optional[str]:
        """Return a line of text from the active source or ``None`` when idle."""
        with self._lock:
            if self._mock_source is not None:
                try:
                    return self._mock_source()
                except (TypeError, ValueError) as exc:
                    logger.warning("Mock source failed: %s", exc)
                    return None

            if self._serial_conn is not None:
                try:
                    data = self._serial_conn.readline()
                    if not data:
                        return None
                    return data.decode("utf-8", errors="ignore").strip()
                except (OSError, AttributeError, UnicodeDecodeError) as exc:
                    logger.error("Serial read failed: %s", exc)
                    return None

            if self._tcp_conn is not None:
                try:
                    data = self._tcp_conn.recv(4096)
                    if not data:
                        return None
                    return data.decode("utf-8", errors="ignore").strip().splitlines()[0]
                except socket.timeout:
                    return None
                except OSError as exc:
                    logger.error("TCP read failed: %s", exc)
                    return None
        return None

    def disconnect(self) -> None:
        """Close all active connections."""
        with self._lock:
            if self._serial_conn is not None:
                try:
                    self._serial_conn.close()
                except (OSError, AttributeError):
                    pass
                self._serial_conn = None
            if self._tcp_conn is not None:
                try:
                    self._tcp_conn.close()
                except OSError:
                    pass
                self._tcp_conn = None


class DataAcquisitionThread(QThread if HAS_QT else QThread):
    """Continuously convert raw connection data into structured VOR records."""

    record_received = pyqtSignal(object)
    status_changed = pyqtSignal(str)
    error_occurred = pyqtSignal(str)

    def __init__(self, conn: VORConnectionHandler, poll_interval: float = 0.05) -> None:
        """Create the worker thread."""
        super().__init__()
        self._conn = conn
        self._poll_interval = poll_interval
        self._stop_event = Event()
        self._active = True

    def run(self) -> None:
        """Poll the connection until stopped and emit parsed records."""
        self.status_changed.emit("RUNNING")
        while self._active and not self._stop_event.is_set():
            line = self._conn.readline()
            if line is None:
                if not self._active:
                    break
                self._stop_event.wait(timeout=self._poll_interval)
                continue
            stripped = line.strip()
            if not stripped:
                self._stop_event.wait(timeout=self._poll_interval)
                continue
            try:
                record = VORDataRecord.from_sentence(stripped)
                self.record_received.emit(record)
            except ValueError as exc:
                self.error_occurred.emit(str(exc))
                logger.warning("Discarding unsupported acquisition line: %s", stripped)
            self._stop_event.wait(timeout=self._poll_interval)
        self.status_changed.emit("STOPPED")

    def stop(self) -> None:
        """Request clean thread shutdown."""
        self._active = False
        self._stop_event.set()


class MockVORTCPServer:
    """Very small TCP server that emits synthetic VOR telemetry."""

    def __init__(self, host: str = "127.0.0.1", port: int = 5000, station_key: str = "JNB") -> None:
        """Create the mock server."""
        self.host = host
        self.port = port
        self.station_key = station_key
        self._lock = RLock()
        self._running = False
        self._server_sock: Optional[socket.socket] = None
        self._thread: Optional[Thread] = None
        self._sim = SimulationEngine(station_key=station_key)

    def start(self) -> bool:
        """Start the listening thread."""
        with self._lock:
            if self._running:
                return True
            self._running = True
            self._thread = Thread(target=self._serve, daemon=True)
            self._thread.start()
            return True

    def _serve(self) -> None:
        """Accept client connections and send synthetic data."""
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
                server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                server.bind((self.host, self.port))
                server.listen(2)
                server.settimeout(0.2)
                with self._lock:
                    self._server_sock = server
                logger.info("Mock VOR server listening on %s:%s", self.host, self.port)
                while True:
                    with self._lock:
                        if not self._running:
                            break
                    try:
                        client, _addr = server.accept()
                    except socket.timeout:
                        continue
                    except OSError:
                        break
                    with client:
                        client.settimeout(0.2)
                        while True:
                            with self._lock:
                                if not self._running:
                                    break
                            try:
                                command = client.recv(1024)
                                if not command:
                                    break
                            except socket.timeout:
                                pass
                            except OSError:
                                break
                            record = self._sim.step()
                            line = (
                                f"{record.station_key},{record.ident},{record.frequency_mhz:.2f},"
                                f"{record.bearing_deg:.1f},{record.deviation_deg:.2f},"
                                f"{record.signal_percent:.1f},{record.distance_nm:.2f}\n"
                            )
                            try:
                                client.sendall(line.encode("utf-8"))
                            except OSError:
                                break
                            time.sleep(0.2)
        except OSError as exc:
            logger.error("Mock VOR server failed: %s", exc)
        finally:
            with self._lock:
                self._server_sock = None
                self._running = False

    def stop(self) -> None:
        """Stop the mock server and close the listening socket."""
        with self._lock:
            self._running = False
            if self._server_sock:
                try:
                    self._server_sock.close()
                except OSError:
                    pass
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None


@dataclass
class VORAirportConfig:
    """Persistent airport monitor configuration model."""

    station_key: str = "JNB"
    airport_code: str = "JNB"
    host: str = "127.0.0.1"
    port: int = 5000
    serial_port: str = ""
    serial_baud: int = 9600


@dataclass
class GroundTarget:
    """Rich ASRACS ground target model."""

    target_id: str
    x: float
    y: float
    target_type: str = "aircraft"
    speed: float = 0.0
    heading: float = 0.0
    callsign: str = ""
    track_history: Deque[Tuple[float, float]] = field(default_factory=lambda: deque(maxlen=120))
    metadata: Dict[str, Any] = field(default_factory=dict)
    route_points: List[Tuple[float, float]] = field(default_factory=list)
    progress_m: float = 0.0


@dataclass
class IncursionAlert:
    """Runway incursion or hotspot alert."""

    target_id: str
    severity: str
    message: str
    timestamp: float = field(default_factory=time.time)
    acknowledged: bool = False


@dataclass
class SurfaceTrack:
    """Representation of a moving ASRACS surface track."""

    callsign: str
    kind: str
    x: float
    y: float
    speed: float
    heading_deg: float
    route: List[Tuple[int, int]]
    route_index: int = 0
    alert: str = "NORMAL"

    def position(self) -> Tuple[float, float]:
        """Return the current map position."""
        return self.x, self.y


class SimulationEngine:
    """Generate synthetic airborne VOR approach telemetry."""

    def __init__(self, station_key: str = "JNB") -> None:
        """Create the simulation engine."""
        self._lock = threading.RLock()
        self.station_key = station_key if station_key in VOR_STATIONS else "JNB"
        self._phase = 0.0
        self._running = True
        self._glide = GlideSlopeDetector(VOR_STATIONS[self.station_key]["elevation_ft"])

    def set_station(self, station_key: str) -> None:
        """Select the active station."""
        with self._lock:
            if station_key in VOR_STATIONS:
                self.station_key = station_key
                self._phase = 0.0
                self._glide = GlideSlopeDetector(VOR_STATIONS[station_key]["elevation_ft"])

    def start(self) -> None:
        """Enable simulation output."""
        with self._lock:
            self._running = True

    def stop(self) -> None:
        """Pause simulation output."""
        with self._lock:
            self._running = False

    def step(self) -> VORDataRecord:
        """Advance the simulation and return the next record."""
        with self._lock:
            station = VOR_STATIONS[self.station_key]
            if not self._running:
                return VORDataRecord(
                    timestamp=datetime.now(timezone.utc),
                    station_key=self.station_key,
                    ident=station["vor_ident"],
                    frequency_mhz=station["vor_freq"],
                    bearing_deg=0.0,
                    deviation_deg=0.0,
                    signal_percent=0.0,
                    distance_nm=0.5,
                    dme_nm=0.5,
                    flags={"status": "STOPPED"},
                    raw_line="",
                )
            self._phase += 1.0
            bearing = (self._phase * 7.5) % 360.0
            deviation = math.sin(self._phase / 5.0) * 2.5
            signal = clamp(72.0 + math.sin(self._phase / 4.0) * 24.0 + random.uniform(-4.0, 4.0), 5.0, 100.0)
            dist = max(0.5, 15.0 - (self._phase % 30.0) * 0.5)
            altitude = self._glide.target_altitude(dist) + random.uniform(-120.0, 120.0)
            return VORDataRecord(
                timestamp=datetime.now(timezone.utc),
                station_key=self.station_key,
                ident=station["vor_ident"],
                frequency_mhz=station["vor_freq"],
                bearing_deg=bearing,
                deviation_deg=deviation,
                signal_percent=signal,
                distance_nm=dist,
                dme_nm=dist,
                flags={
                    "altitude_ft": altitude,
                    "glide_status": self._glide.status_label(dist, altitude),
                    "status": "OK",
                },
                raw_line="",
            )

    def as_sentence(self) -> str:
        """Return the current simulation sample as a wire sentence."""
        record = self.step()
        return (
            f"{record.station_key},{record.ident},{record.frequency_mhz:.2f},"
            f"{record.bearing_deg:.1f},{record.deviation_deg:.2f},"
            f"{record.signal_percent:.1f},{record.distance_nm:.2f}"
        )


class EnRouteSimEngine:
    """Long-range en-route simulation producing :class:`VORDataRecord` samples."""

    def __init__(self, route: Optional[Sequence[Tuple[float, float]]] = None) -> None:
        self._lock = threading.RLock()
        self._route = list(route or [(-26.1337, 28.2420), (-29.6144, 31.1197), (-33.9696, 18.5972)])
        self._leg = 0
        self._progress = 0.0

    def step(self) -> VORDataRecord:
        with self._lock:
            if len(self._route) < 2:
                self._route = [(-26.1337, 28.2420), (-29.6144, 31.1197)]
            p0 = self._route[self._leg % len(self._route)]
            p1 = self._route[(self._leg + 1) % len(self._route)]
            leg_nm = max(1.0, great_circle_distance_nm(*p0, *p1))
            self._progress += 12.0
            if self._progress >= leg_nm:
                self._leg = (self._leg + 1) % len(self._route)
                self._progress = 0.0
                p0 = self._route[self._leg % len(self._route)]
                p1 = self._route[(self._leg + 1) % len(self._route)]
                leg_nm = max(1.0, great_circle_distance_nm(*p0, *p1))
            brg = initial_bearing(*p0, *p1)
            lat, lon = destination_point(p0[0], p0[1], brg, self._progress)
            nearest = min(VOR_STATIONS.items(), key=lambda item: haversine_nm(lat, lon, item[1]["lat"], item[1]["lon"]))
            code, station = nearest
            dist = haversine_nm(lat, lon, station["lat"], station["lon"])
            signal = clamp(100.0 - dist * 6.0, 5.0, 100.0)
            return VORDataRecord(
                timestamp=datetime.now(timezone.utc),
                station_key=code,
                ident=station["vor_ident"],
                frequency_mhz=station["vor_freq"],
                bearing_deg=brg,
                deviation_deg=math.sin(self._progress / 3.0) * 1.5,
                signal_percent=signal,
                distance_nm=dist,
                dme_nm=dist,
                flags={"mode": "ENROUTE", "lat": lat, "lon": lon},
            )


class ASRACSSimEngine:
    """Generate synthetic airside surface traffic for the selected airport."""

    def __init__(self, airport_code: str = "JNB") -> None:
        """Create the ground movement simulation engine."""
        self._lock = threading.RLock()
        self.airport_code = airport_code if airport_code in AIRFIELD_LAYOUTS else "JNB"
        self._tracks: List[SurfaceTrack] = []
        self._alerts: List[IncursionAlert] = []
        self._tick = 0
        self._initialize_tracks()

    def _initialize_tracks(self) -> None:
        """Create seed ground tracks for the active airport."""
        layout = AIRFIELD_LAYOUTS[self.airport_code]
        taxiways = layout["taxiways"]
        self._tracks = []
        for index, route in enumerate(taxiways[:3], start=1):
            start_x, start_y = route[0]
            self._tracks.append(
                SurfaceTrack(
                    callsign=f"GND{index}",
                    kind="AIRCRAFT" if index < 3 else "VEHICLE",
                    x=float(start_x),
                    y=float(start_y),
                    speed=4.0 + index,
                    heading_deg=0.0,
                    route=list(route),
                )
            )
        if taxiways:
            route = list(taxiways[-1])
            vx, vy = route[0]
            self._tracks.append(
                SurfaceTrack(callsign="OPS1", kind="VEHICLE", x=float(vx), y=float(vy), speed=3.5, heading_deg=0.0, route=route)
            )

    def set_airport(self, airport_code: str) -> None:
        """Switch the simulation to a new airport layout."""
        with self._lock:
            if airport_code in AIRFIELD_LAYOUTS:
                self.airport_code = airport_code
                self._tick = 0
                self._initialize_tracks()

    def tracks(self) -> List[SurfaceTrack]:
        """Return a snapshot of current ground tracks."""
        with self._lock:
            return [SurfaceTrack(**track.__dict__) for track in self._tracks]

    def alerts(self) -> List[IncursionAlert]:
        """Return current active incursion alerts."""
        with self._lock:
            return [IncursionAlert(**alert.__dict__) for alert in self._alerts]

    def _update_incursion_alerts(self) -> None:
        self._alerts = []
        hotspots = AIRFIELD_LAYOUTS.get(self.airport_code, {}).get("hotspots", [])
        for track in self._tracks:
            for hs_name, hx, hy, hs_msg in hotspots:
                if math.hypot(track.x - hx, track.y - hy) <= 25.0:
                    sev = "WARNING" if track.kind == "VEHICLE" else "CAUTION"
                    self._alerts.append(IncursionAlert(track.callsign, sev, f"{hs_name}: {hs_msg}"))
        for idx, trk_a in enumerate(self._tracks):
            for trk_b in self._tracks[idx + 1 :]:
                sep = math.hypot(trk_a.x - trk_b.x, trk_a.y - trk_b.y)
                if sep < 18.0:
                    self._alerts.append(IncursionAlert(f"{trk_a.callsign}/{trk_b.callsign}", "CRITICAL", "Potential runway/taxiway incursion"))

    def step(self) -> List[SurfaceTrack]:
        """Advance the simulation one step and return track snapshots."""
        with self._lock:
            self._tick += 1
            for track in self._tracks:
                if len(track.route) < 2:
                    continue
                next_index = (track.route_index + 1) % len(track.route)
                tx, ty = track.route[next_index]
                dx = tx - track.x
                dy = ty - track.y
                distance = math.hypot(dx, dy)
                if distance < track.speed:
                    track.x = float(tx)
                    track.y = float(ty)
                    track.route_index = next_index
                else:
                    track.x += dx / distance * track.speed
                    track.y += dy / distance * track.speed
                track.heading_deg = (math.degrees(math.atan2(dx, -dy)) + 360.0) % 360.0
                track.alert = "CAUTION" if track.route_index == 1 else "NORMAL"
                if self._tick % 40 == 0 and track.kind == "VEHICLE":
                    track.alert = "WARNING"
            self._update_incursion_alerts()
            return self.tracks()


class VORDataProcessor:
    """Normalize and smooth incoming VOR data records."""

    def __init__(self, history_size: int = 180) -> None:
        self._lock = threading.RLock()
        self._history: Deque[VORDataRecord] = deque(maxlen=history_size)

    def add_record(self, record: VORDataRecord) -> VORDataRecord:
        with self._lock:
            self._history.append(record)
            return record

    def latest(self) -> Optional[VORDataRecord]:
        with self._lock:
            return self._history[-1] if self._history else None

    def smoothed_deviation(self, window: int = 5) -> float:
        with self._lock:
            if not self._history:
                return 0.0
            samples = list(self._history)[-max(1, window) :]
            return sum(r.deviation_deg for r in samples) / len(samples)


class AircraftTracker:
    """Tracks aircraft by ident and station with history."""

    def __init__(self, max_history: int = 60) -> None:
        self._lock = threading.RLock()
        self._tracks: Dict[str, Deque[VORDataRecord]] = {}
        self._max_history = max(5, max_history)

    def update(self, record: VORDataRecord) -> None:
        key = f"{record.station_key}:{record.ident}"
        with self._lock:
            bucket = self._tracks.setdefault(key, deque(maxlen=self._max_history))
            bucket.append(record)

    def get(self, key: str) -> List[VORDataRecord]:
        with self._lock:
            return list(self._tracks.get(key, ()))

    def all_keys(self) -> List[str]:
        with self._lock:
            return sorted(self._tracks.keys())


class JNBAirportLayout:
    """Convenience accessors for OR Tambo normalized layout references."""

    RWY_03L_THR = (0.30, 0.80)
    RWY_21R_THR = (0.70, 0.20)
    RWY_03R_THR = (0.40, 0.80)
    RWY_21L_THR = (0.80, 0.20)
    TAXIWAY_A = ((0.20, 0.50), (0.80, 0.50))
    TAXIWAY_B = ((0.50, 0.20), (0.50, 0.80))
    APRON_CENTRE = (0.50, 0.55)
    TERMINAL = (0.45, 0.60)


if HAS_QT:
    class CDIWidget(QWidget):
        """Classic lateral course deviation indicator display."""

        def __init__(self, parent: Optional[QWidget] = None) -> None:
            """Initialize the widget."""
            super().__init__(parent)
            self._deviation = 0.0
            self._bearing = 0.0
            self.setMinimumSize(260, 220)

        def set_data(self, deviation_deg: float, bearing_deg: float) -> None:
            """Update the displayed course deviation data."""
            self._deviation = float(deviation_deg)
            self._bearing = float(bearing_deg)
            self.update()

        def paintEvent(self, _event: Any) -> None:
            """Render the indicator."""
            painter = QPainter(self)
            painter.setRenderHint(QPainter.Antialiasing)
            painter.fillRect(self.rect(), QColor(18, 18, 22))
            w = self.width()
            h = self.height()
            cx = w // 2
            cy = h // 2
            radius = min(w, h) // 2 - 20
            painter.setPen(QPen(QColor(220, 220, 220), 2))
            painter.drawEllipse(cx - radius, cy - radius, radius * 2, radius * 2)
            for dot in range(-2, 3):
                dx = int(dot * radius * 0.22)
                painter.setBrush(QBrush(QColor(200, 200, 200)))
                painter.drawEllipse(cx + dx - 4, cy - 4, 8, 8)
            painter.setPen(QPen(QColor(160, 160, 160), 2))
            painter.drawLine(cx - radius + 20, cy, cx + radius - 20, cy)
            needle_x = int(cx + clamp(self._deviation / 2.5, -1.0, 1.0) * radius * 0.45)
            painter.setPen(QPen(QColor(255, 210, 0), 5))
            painter.drawLine(needle_x, cy - radius + 25, needle_x, cy + radius - 25)
            painter.setPen(QPen(QColor(0, 200, 255), 3))
            angle = math.radians(self._bearing - 90.0)
            painter.drawLine(cx, cy, int(cx + math.cos(angle) * radius * 0.75), int(cy + math.sin(angle) * radius * 0.75))
            painter.setPen(QColor(220, 220, 220))
            painter.drawText(self.rect(), Qt.AlignBottom | Qt.AlignHCenter, f"DEV {self._deviation:+.2f}°   BRG {self._bearing:03.0f}°")


    class VORSignalPanel(QWidget):
        """Compact VOR signal widget with compass, status, and Morse ident."""

        def __init__(self, parent: Optional[QWidget] = None) -> None:
            """Initialize the panel."""
            super().__init__(parent)
            self._bearing = 0.0
            self._signal_pct = 0.0
            self._signal_ok = False
            self._freq = "000.00"
            self._ident = "---"
            self._morse_visible = True
            self._morse_timer = QTimer(self)
            self._morse_timer.timeout.connect(self._toggle_morse)
            self._morse_timer.start(600)
            self.setMinimumSize(500, 200)

        def _toggle_morse(self) -> None:
            """Toggle Morse visibility to mimic station identification."""
            self._morse_visible = not self._morse_visible
            self.update()

        def set_data(self, bearing: float, signal_pct: float, signal_ok: bool, freq: str, ident: str) -> None:
            """Update the displayed VOR information."""
            self._bearing = float(bearing)
            self._signal_pct = clamp(float(signal_pct), 0.0, 100.0)
            self._signal_ok = bool(signal_ok)
            self._freq = str(freq)
            self._ident = str(ident)
            self.update()

        def paintEvent(self, _event: Any) -> None:
            """Render the custom panel."""
            painter = QPainter(self)
            painter.setRenderHint(QPainter.Antialiasing)
            painter.fillRect(self.rect(), QColor(20, 22, 28))
            w, h = self.width(), self.height()
            third = w // 3

            cx, cy = third // 2, h // 2
            radius = min(third, h) // 2 - 14
            painter.setPen(QPen(QColor(190, 190, 190), 2))
            painter.drawEllipse(cx - radius, cy - radius, radius * 2, radius * 2)
            for tick in range(36):
                angle = math.radians(tick * 10 - 90)
                outer_x = cx + math.cos(angle) * radius
                outer_y = cy + math.sin(angle) * radius
                inner = radius - (14 if tick % 3 == 0 else 8)
                inner_x = cx + math.cos(angle) * inner
                inner_y = cy + math.sin(angle) * inner
                painter.drawLine(int(inner_x), int(inner_y), int(outer_x), int(outer_y))
            painter.setPen(QColor(220, 220, 220))
            painter.drawText(cx - 8, cy - radius + 20, "N")
            painter.drawText(cx + radius - 18, cy + 5, "E")
            painter.drawText(cx - 6, cy + radius - 8, "S")
            painter.drawText(cx - radius + 10, cy + 5, "W")
            needle_angle = math.radians(self._bearing - 90.0)
            nx = cx + math.cos(needle_angle) * (radius - 20)
            ny = cy + math.sin(needle_angle) * (radius - 20)
            painter.setPen(QPen(QColor(255, 210, 0), 4))
            painter.drawLine(cx, cy, int(nx), int(ny))
            painter.setBrush(QBrush(QColor(255, 210, 0)))
            painter.drawEllipse(cx - 5, cy - 5, 10, 10)

            bar_x = third + 20
            bar_y = 20
            bar_w = third - 40
            bar_h = h - 40
            painter.setPen(QPen(QColor(120, 120, 120), 2))
            painter.drawRect(bar_x, bar_y, bar_w, bar_h)
            fill_h = int(bar_h * (self._signal_pct / 100.0))
            fill_color = QColor(20, 180, 60) if self._signal_pct >= 70 else QColor(240, 180, 30) if self._signal_pct >= 40 else QColor(200, 50, 50)
            painter.fillRect(bar_x + 4, bar_y + bar_h - fill_h + 4, bar_w - 8, max(0, fill_h - 8), fill_color)
            painter.setPen(QColor(220, 220, 220))
            painter.drawText(QRectF(bar_x, bar_y, bar_w, bar_h), Qt.AlignCenter, f"{self._signal_pct:.0f}%")

            info_x = third * 2
            painter.setPen(QPen(QColor(160, 160, 160), 1))
            painter.drawRoundedRect(info_x + 10, 20, third - 20, h - 40, 8, 8)
            painter.setPen(QColor(220, 220, 220))
            painter.setFont(QFont("Arial", 13, QFont.Bold))
            painter.drawText(info_x + 28, 56, f"{self._freq} MHz")
            painter.setFont(QFont("Arial", 20, QFont.Bold))
            painter.drawText(info_x + 28, 95, self._ident)
            painter.setFont(QFont("Courier New", 12))
            morse_text = morse_encode(self._ident) if self._morse_visible else ""
            painter.drawText(info_x + 28, 132, morse_text)
            painter.setFont(QFont("Arial", 11))
            painter.drawText(info_x + 28, 165, f"Bearing {self._bearing:03.0f}°")

            status_x = w - 20
            status_y = 20
            color = QColor(0, 200, 0) if self._signal_ok else QColor(200, 0, 0)
            painter.setBrush(QBrush(color))
            painter.setPen(QPen(QColor(30, 30, 30), 1))
            painter.drawEllipse(status_x - 10, status_y - 10, 20, 20)


    class RadarDisplay(QWidget):
        """En-route radar plan view used for synthetic traffic awareness."""

        def __init__(self, parent: Optional[QWidget] = None) -> None:
            """Initialize the radar display."""
            super().__init__(parent)
            self._records: Deque[Tuple[float, float]] = deque(maxlen=30)
            self._sweep = 0.0
            self.setMinimumSize(360, 320)

        def set_record(self, record: VORDataRecord) -> None:
            """Push a VOR sample into the radar trail."""
            angle = math.radians(record.bearing_deg - 90.0)
            radius = clamp(record.distance_nm / 15.0, 0.05, 1.0)
            self._records.append((math.cos(angle) * radius, math.sin(angle) * radius))
            self._sweep = record.bearing_deg
            self.update()

        def paintEvent(self, _event: Any) -> None:
            """Render the radar display."""
            painter = QPainter(self)
            painter.setRenderHint(QPainter.Antialiasing)
            painter.fillRect(self.rect(), QColor(8, 24, 12))
            w, h = self.width(), self.height()
            cx = w // 2
            cy = h // 2
            radius = min(w, h) // 2 - 20
            painter.setPen(QPen(QColor(20, 110, 40), 1))
            for ring in range(1, 5):
                rr = int(radius * ring / 4)
                painter.drawEllipse(cx - rr, cy - rr, rr * 2, rr * 2)
            painter.drawLine(cx - radius, cy, cx + radius, cy)
            painter.drawLine(cx, cy - radius, cx, cy + radius)
            sweep_angle = math.radians(self._sweep - 90.0)
            painter.setPen(QPen(QColor(100, 255, 140), 2))
            painter.drawLine(cx, cy, int(cx + math.cos(sweep_angle) * radius), int(cy + math.sin(sweep_angle) * radius))
            for idx, point in enumerate(self._records):
                x = int(cx + point[0] * radius)
                y = int(cy + point[1] * radius)
                alpha = int(80 + idx / max(1, len(self._records)) * 175)
                painter.setBrush(QBrush(QColor(80, 255, 120, alpha)))
                painter.setPen(Qt.NoPen)
                painter.drawEllipse(x - 4, y - 4, 8, 8)


    class ApproachGuidanceDisplay(QWidget):
        """Approach guidance panel with glide path and terrain profile."""

        def __init__(self, parent: Optional[QWidget] = None) -> None:
            """Initialize the widget."""
            super().__init__(parent)
            self._record: Optional[VORDataRecord] = None
            self._terrain: List[int] = TERRAIN_LIBRARY.get("JNB", [0] * 15)
            self._glide = GlideSlopeDetector(VOR_STATIONS["JNB"]["elevation_ft"])
            self.setMinimumSize(420, 280)

        def set_station(self, station_key: str) -> None:
            """Switch the widget to a new station context."""
            station = VOR_STATIONS.get(station_key, VOR_STATIONS["JNB"])
            self._glide = GlideSlopeDetector(station["elevation_ft"])
            self._terrain = TERRAIN_LIBRARY.get(station_key, self._terrain)
            self.update()

        def set_record(self, record: VORDataRecord) -> None:
            """Update the active approach sample."""
            self._record = record
            self.update()

        def paintEvent(self, _event: Any) -> None:
            """Render terrain, ideal glide path, and aircraft position."""
            painter = QPainter(self)
            painter.setRenderHint(QPainter.Antialiasing)
            painter.fillRect(self.rect(), QColor(16, 18, 24))
            w = self.width()
            h = self.height()
            margin = 30
            plot = QRectF(margin, margin, w - 2 * margin, h - 2 * margin)
            painter.setPen(QPen(QColor(150, 150, 150), 1))
            painter.drawRect(plot)
            painter.setPen(QPen(QColor(90, 140, 200), 2))
            painter.drawLine(int(plot.left()), int(plot.bottom() - 10), int(plot.right()), int(plot.bottom() - 10))

            terrain = self._terrain or [0] * 15
            polygon = QPolygonF()
            polygon.append(QPointF(plot.left(), plot.bottom()))
            tmin = min(terrain)
            tmax = max(terrain)
            span = max(1.0, float(tmax - tmin))
            for index, sample in enumerate(terrain):
                x = plot.left() + index / max(1, len(terrain) - 1) * plot.width()
                y = plot.bottom() - ((sample - tmin) / span) * (plot.height() * 0.4)
                polygon.append(QPointF(x, y))
            polygon.append(QPointF(plot.right(), plot.bottom()))
            painter.setPen(QPen(QColor(110, 72, 36), 1))
            painter.setBrush(QBrush(QColor(120, 84, 45, 160)))
            painter.drawPolygon(polygon)

            painter.setPen(QPen(QColor(255, 215, 0), 2))
            glide_points = QPolygonF([
                QPointF(plot.left(), plot.bottom() - 15),
                QPointF(plot.right(), plot.top() + plot.height() * 0.25),
            ])
            painter.drawPolyline(glide_points)

            if self._record is not None:
                dist = max(0.5, self._record.distance_nm)
                target_alt = self._glide.target_altitude(dist)
                actual_alt = float(self._record.flags.get("altitude_ft", target_alt))
                x = plot.right() - (dist / 15.0) * plot.width()
                y = plot.bottom() - clamp((actual_alt - self._glide.runway_elevation_ft) / 3500.0, 0.0, 1.0) * plot.height()
                painter.setBrush(QBrush(QColor(0, 220, 120)))
                painter.setPen(QPen(QColor(0, 220, 120), 2))
                painter.drawEllipse(QPointF(x, y), 6, 6)
                painter.setPen(QColor(230, 230, 230))
                painter.drawText(int(plot.left()) + 10, int(plot.top()) + 20, f"Dist {dist:.1f} NM")
                painter.drawText(int(plot.left()) + 10, int(plot.top()) + 40, f"GS {self._glide.status_label(dist, actual_alt)}")
                painter.drawText(int(plot.left()) + 10, int(plot.top()) + 60, f"Alt {actual_alt:.0f} ft")


    class Terrain3DWidget(QOpenGLWidget):
        """OpenGL terrain profile widget."""

        def __init__(self, parent: Optional[QWidget] = None) -> None:
            """Initialize the widget."""
            super().__init__(parent)
            self._terrain: Optional[List[int]] = TERRAIN_LIBRARY.get("JNB")
            self.setMinimumSize(360, 280)

        def set_terrain(self, terrain: Optional[Sequence[int]]) -> None:
            """Replace the active terrain profile."""
            self._terrain = list(terrain) if terrain is not None else None
            self.update()

        def initializeGL(self) -> None:
            """Initialize the OpenGL state when available."""
            if not HAS_OPENGL:
                return

        def paintGL(self) -> None:
            """Render the terrain profile."""
            if self._terrain is None:
                return
            if not HAS_OPENGL:
                painter = QPainter(self)
                painter.fillRect(self.rect(), QColor(18, 20, 26))
                painter.setPen(QPen(QColor(130, 180, 80), 2))
                points = []
                for idx, value in enumerate(self._terrain):
                    x = int(idx / max(1, len(self._terrain) - 1) * self.width())
                    y = int(self.height() - (value - min(self._terrain)) / max(1, max(self._terrain) - min(self._terrain)) * self.height() * 0.8 - 20)
                    points.append(QPoint(x, y))
                if points:
                    painter.drawPolyline(QPolygon(points))
                painter.end()
                return
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
            glLoadIdentity()
            gluLookAt(1.5, 1.3, 3.0, 0.0, 0.2, 0.0, 0.0, 1.0, 0.0)
            glColor3f(0.15, 0.18, 0.22)
            glBegin(GL_QUADS)
            glVertex3f(-2.0, -0.6, -2.0)
            glVertex3f(2.0, -0.6, -2.0)
            glVertex3f(2.0, -0.6, 2.0)
            glVertex3f(-2.0, -0.6, 2.0)
            glEnd()
            glColor3f(0.45, 0.32, 0.18)
            glBegin(GL_LINE_STRIP)
            minimum = min(self._terrain)
            maximum = max(self._terrain)
            span = max(1.0, float(maximum - minimum))
            for index, value in enumerate(self._terrain):
                x = -1.5 + 3.0 * index / max(1, len(self._terrain) - 1)
                y = -0.5 + ((value - minimum) / span) * 1.2
                glVertex3f(x, y, 0.0)
            glEnd()


    class ASRACSDisplay(QWidget):
        """Render the currently selected airfield layout and ground tracks."""

        def __init__(self, parent: Optional[QWidget] = None) -> None:
            """Initialize the display."""
            super().__init__(parent)
            self._layout = AIRFIELD_LAYOUTS["JNB"]
            self._tracks: List[SurfaceTrack] = []
            self.setMinimumSize(420, 320)

        def set_layout(self, airport_code: str) -> None:
            """Load a new airport layout."""
            if airport_code in AIRFIELD_LAYOUTS:
                self._layout = AIRFIELD_LAYOUTS[airport_code]
                self.update()

        def set_tracks(self, tracks: Sequence[SurfaceTrack]) -> None:
            """Update the visible ASRACS tracks."""
            self._tracks = list(tracks)
            self.update()

        def paintEvent(self, _event: Any) -> None:
            """Render airport geometry and simulated traffic."""
            painter = QPainter(self)
            painter.setRenderHint(QPainter.Antialiasing)
            painter.fillRect(self.rect(), QColor(18, 18, 20))
            bounds = self._layout["bounds"]
            sx = self.width() / max(1, bounds[2] - bounds[0])
            sy = self.height() / max(1, bounds[3] - bounds[1])

            def map_pt(pt: Tuple[int, int]) -> Tuple[int, int]:
                return int((pt[0] - bounds[0]) * sx), int((pt[1] - bounds[1]) * sy)

            painter.setBrush(QBrush(QColor(65, 65, 70)))
            painter.setPen(QPen(QColor(120, 120, 120), 1))
            for runway in self._layout["runways"]:
                x1, y1 = map_pt(runway["start"])
                x2, y2 = map_pt(runway["end"])
                painter.setPen(QPen(QColor(170, 170, 170), max(6, int(runway["width"] * sx / 5))))
                painter.drawLine(x1, y1, x2, y2)
                painter.setPen(QColor(230, 230, 230))
                painter.drawText((x1 + x2) // 2, (y1 + y2) // 2, runway["name"])
            painter.setPen(QPen(QColor(255, 180, 60), 3))
            for taxiway in self._layout["taxiways"]:
                poly = QPolygon([QPoint(*map_pt(point)) for point in taxiway])
                painter.drawPolyline(poly)
            painter.setBrush(QBrush(QColor(60, 80, 110)))
            painter.setPen(QPen(QColor(110, 140, 180), 1))
            for apron in self._layout["aprons"]:
                x, y = map_pt((apron[0], apron[1]))
                w = int(apron[2] * sx)
                h = int(apron[3] * sy)
                painter.drawRect(x, y, w, h)
            painter.setPen(QColor(255, 100, 100))
            painter.setBrush(QBrush(QColor(255, 100, 100, 70)))
            for hotspot in self._layout["hotspots"]:
                hx, hy = map_pt((hotspot[1], hotspot[2]))
                painter.drawEllipse(hx - 8, hy - 8, 16, 16)
                painter.drawText(hx + 10, hy, hotspot[0])
            for track in self._tracks:
                tx, ty = map_pt((int(track.x), int(track.y)))
                color = QColor(0, 220, 120) if track.alert == "NORMAL" else QColor(255, 190, 40) if track.alert == "CAUTION" else QColor(255, 70, 70)
                painter.setBrush(QBrush(color))
                painter.setPen(QPen(color, 2))
                painter.drawEllipse(tx - 6, ty - 6, 12, 12)
                painter.drawText(tx + 8, ty - 8, track.callsign)


    class ASRACSAlertPanel(QWidget):
        """Panel displaying active ASRACS incursion alerts."""

        def __init__(self, parent: Optional[QWidget] = None) -> None:
            super().__init__(parent)
            layout = QVBoxLayout(self)
            title = QLabel("⚠ Incursion Alerts")
            layout.addWidget(title)
            self._table = QTableWidget(0, 3)
            self._table.setHorizontalHeaderLabels(["Target", "Severity", "Message"])
            if hasattr(self._table, "horizontalHeader"):
                self._table.horizontalHeader().setSectionResizeMode(2, QHeaderView.Stretch)
            layout.addWidget(self._table)

        def update_alerts(self, alerts: Sequence[IncursionAlert]) -> None:
            self._table.setRowCount(len(alerts))
            for row, alert in enumerate(alerts):
                self._table.setItem(row, 0, QTableWidgetItem(alert.target_id))
                self._table.setItem(row, 1, QTableWidgetItem(alert.severity))
                self._table.setItem(row, 2, QTableWidgetItem(alert.message))


    class SAAFBaseMapWidget(QWidget):
        """SAAF base runway sketch widget."""

        def __init__(self, parent: Optional[QWidget] = None) -> None:
            super().__init__(parent)
            self._base_info: Optional[Dict[str, Any]] = None
            self.setMinimumSize(220, 180)

        def set_base(self, info: Optional[Dict[str, Any]]) -> None:
            self._base_info = info
            self.update()

        def paintEvent(self, _event: Any) -> None:
            painter = QPainter(self)
            painter.setRenderHint(QPainter.Antialiasing)
            painter.fillRect(self.rect(), QColor(12, 28, 18))
            if not self._base_info:
                painter.setPen(QColor(180, 180, 180))
                painter.drawText(self.rect(), Qt.AlignCenter, "Select a base")
                return
            runway = str(self._base_info.get("runway", "00/18"))
            parts = runway.split("/")
            heading = parse_runway_heading(parts[0])
            cx = self.width() // 2
            cy = self.height() // 2
            rlen = min(self.width(), self.height()) // 3
            dx = int(math.sin(math.radians(heading)) * rlen)
            dy = int(math.cos(math.radians(heading)) * rlen)
            painter.setPen(QPen(QColor(170, 170, 170), 8))
            painter.drawLine(cx - dx, cy + dy, cx + dx, cy - dy)
            painter.setPen(QColor(220, 220, 220))
            painter.drawText(8, 18, str(self._base_info.get("name", "")))


    class SAAFBaseInfoWidget(QWidget):
        """SAAF base textual detail panel."""

        def __init__(self, parent: Optional[QWidget] = None) -> None:
            super().__init__(parent)
            layout = QVBoxLayout(self)
            self._label = QLabel("Select a base on the map")
            self._label.setWordWrap(True)
            layout.addWidget(self._label)

        def set_base(self, icao: str, info: Optional[Dict[str, Any]]) -> None:
            if not info:
                self._label.setText("No information available")
                return
            self._label.setText(
                f"<b>{info.get('name','')} ({icao})</b><br>"
                f"Runway: {info.get('runway','')}<br>"
                f"VOR: {info.get('vor_ident','')} {float(info.get('vor_freq',0.0)):.2f} MHz<br>"
                f"Squadrons: {', '.join(info.get('squadrons', [])) if isinstance(info.get('squadrons'), list) else info.get('squadron','')}<br>"
                f"ATIS: {info.get('atis_freq','N/A')} | APP: {info.get('app_freq','N/A')} | TWR: {info.get('twr_freq','N/A')}"
            )


    class SAAFOverviewMap(QWidget):
        """Clickable overview map showing monitored SAAF air bases."""

        baseSelected = pyqtSignal(str)

        def __init__(self, parent: Optional[QWidget] = None) -> None:
            """Initialize the map widget."""
            super().__init__(parent)
            self.setMinimumSize(420, 320)

        def _project(self, lat: float, lon: float) -> Tuple[int, int]:
            """Project a geographic coordinate onto the widget surface."""
            x = int((lon - 15.0) / (40.0 - 15.0) * self.width())
            y = int((0.0 - lat) / (35.0 - 0.0) * self.height())
            return x, y

        def paintEvent(self, _event: Any) -> None:
            """Render the overview map with integer painter coordinates."""
            painter = QPainter(self)
            painter.setRenderHint(QPainter.Antialiasing)
            painter.fillRect(self.rect(), QColor(10, 16, 26))
            outline = [(-34.8, 18.4), (-31.0, 17.5), (-28.0, 16.0), (-25.0, 17.0), (-22.0, 19.0), (-20.0, 21.0), (-18.0, 24.0), (-16.0, 27.0), (-18.0, 31.0), (-22.0, 32.0), (-25.0, 31.0), (-29.0, 29.0), (-32.0, 26.0), (-34.8, 18.4)]
            painter.setPen(QPen(QColor(60, 90, 120), 2))
            painter.setBrush(QBrush(QColor(30, 45, 70)))
            poly = QPolygon([QPoint(int(px), int(py)) for lat, lon in outline for px, py in [self._project(lat, lon)]])
            painter.drawPolygon(poly)
            for code, base in SAAF_BASES.items():
                x, y = self._project(base["lat"], base["lon"])
                painter.setBrush(QBrush(QColor(255, 180, 0)))
                painter.setPen(QPen(QColor(20, 20, 20), 1))
                painter.drawEllipse(int(x - 5), int(y - 5), int(10), int(10))
                painter.setPen(QColor(230, 230, 230))
                painter.drawText(int(x + 8), int(y - 6), code)

        def mousePressEvent(self, event: Any) -> None:
            """Emit the nearest base when the map is clicked."""
            click_x = int(event.x())
            click_y = int(event.y())
            nearest_code = None
            nearest_distance = 999999.0
            for code, base in SAAF_BASES.items():
                x, y = self._project(base["lat"], base["lon"])
                distance = math.hypot(click_x - x, click_y - y)
                if distance < nearest_distance:
                    nearest_distance = distance
                    nearest_code = code
            if nearest_code and nearest_distance <= 30.0:
                self.baseSelected.emit(nearest_code)


    class VORAirportMonitorApp(QMainWindow):
        """Primary desktop application window for the monitoring system."""

        def __init__(self) -> None:
            """Build the application UI and start background timers."""
            super().__init__()
            self.setWindowTitle("VOR / ASRACS / SAAF Airport Monitoring System v6.0")
            self.resize(1320, 860)
            self._sim_engine = SimulationEngine()
            self._enroute_engine = EnRouteSimEngine()
            self._asracs_engine = ASRACSSimEngine("JNB")
            self._connection = VORConnectionHandler()
            self._processor = VORDataProcessor()
            self._tracker = AircraftTracker()
            self._config = VORAirportConfig()
            self._mock_server: Optional[MockVORTCPServer] = None
            self._acq_thread: Optional[DataAcquisitionThread] = None
            self._chart_x = 0
            self._current_record: Optional[VORDataRecord] = None
            self._build_ui()
            self._load_config()
            self._populate_vor_combo()
            self._set_airport_layout(self._config.airport_code)
            self._ui_timer = QTimer(self)
            self._ui_timer.timeout.connect(self._update_vor_synthetic)
            self._ui_timer.start(900)
            self._fast_timer = QTimer(self)
            self._fast_timer.timeout.connect(self._update_asracs_synthetic)
            self._fast_timer.start(250)

        def _build_ui(self) -> None:
            """Create all application tabs and toolbars."""
            self._tabs = QTabWidget()
            self.setCentralWidget(self._tabs)
            self._tab_vor()
            self._tab_radar()
            self._tab_approach()
            self._tab_terrain()
            self._tab_asracs()
            self._tab_saaf()
            self._tab_connection()
            self._tab_diagnostics()

        def _tab_vor(self) -> None:
            """Create the VOR monitoring tab."""
            page = QWidget()
            root = QVBoxLayout(page)
            top = QHBoxLayout()
            self.vor_combo = QComboBox()
            self.vor_combo.currentIndexChanged.connect(self._on_vor_index_changed)
            top.addWidget(QLabel("VOR Station:"))
            top.addWidget(self.vor_combo, 1)
            self.station_label = QLabel("---")
            top.addWidget(self.station_label)
            root.addLayout(top)
            self.signal_panel = VORSignalPanel()
            root.addWidget(self.signal_panel)
            middle = QHBoxLayout()
            self.cdi = CDIWidget()
            middle.addWidget(self.cdi, 1)
            chart_group = QGroupBox("Signal Trend")
            chart_layout = QVBoxLayout(chart_group)
            self._vor_series = QLineSeries()
            self._vor_chart = QChart()
            self._vor_chart.legend().hide()
            self._vor_chart.addSeries(self._vor_series)
            self._vor_chart.createDefaultAxes()
            self._vor_chart.setBackgroundVisible(False)
            chart_layout.addWidget(QChartView(self._vor_chart))
            middle.addWidget(chart_group, 1)
            root.addLayout(middle, 1)
            self.vor_status = QLabel("Awaiting data")
            root.addWidget(self.vor_status)
            self._tabs.addTab(page, "VOR Monitor")

        def _tab_radar(self) -> None:
            """Create the radar tab."""
            page = QWidget()
            layout = QVBoxLayout(page)
            self.radar = RadarDisplay()
            layout.addWidget(self.radar)
            self._tabs.addTab(page, "Radar")

        def _tab_approach(self) -> None:
            """Create the approach guidance tab."""
            page = QWidget()
            layout = QVBoxLayout(page)
            self.approach = ApproachGuidanceDisplay()
            layout.addWidget(self.approach)
            self._tabs.addTab(page, "Approach")

        def _tab_terrain(self) -> None:
            """Create the terrain tab."""
            page = QWidget()
            layout = QVBoxLayout(page)
            self.terrain = Terrain3DWidget()
            layout.addWidget(self.terrain)
            self._tabs.addTab(page, "Terrain 3D")

        def _tab_asracs(self) -> None:
            """Create the ASRACS tab with airport selection."""
            page = QWidget()
            root = QVBoxLayout(page)
            controls = QHBoxLayout()
            controls.addWidget(QLabel("Airport Layout:"))
            self.airport_combo = QComboBox()
            for code in sorted(AIRFIELD_LAYOUTS.keys()):
                self.airport_combo.addItem(f"{code} - {AIRFIELD_LAYOUTS[code]['name']}", code)
            self.airport_combo.currentIndexChanged.connect(self._on_airport_layout_changed)
            controls.addWidget(self.airport_combo, 1)
            root.addLayout(controls)
            self.asracs = ASRACSDisplay()
            root.addWidget(self.asracs, 1)
            self.alert_panel = ASRACSAlertPanel()
            root.addWidget(self.alert_panel)
            self._tabs.addTab(page, "ASRACS")

        def _tab_saaf(self) -> None:
            """Create the SAAF overview tab."""
            page = QWidget()
            root = QHBoxLayout(page)
            self.saaf_map = SAAFOverviewMap()
            self.saaf_map.baseSelected.connect(self._on_base_selected)
            root.addWidget(self.saaf_map, 2)
            side = QVBoxLayout()
            side.addWidget(QLabel("SAAF Bases"))
            self.base_list = QListWidget()
            for code, data in sorted(SAAF_BASES.items()):
                item = QListWidgetItem(f"{code} - {data['name']}")
                item.setData(Qt.UserRole, code)
                self.base_list.addItem(item)
            self.base_list.itemClicked.connect(self._on_base_item_clicked)
            side.addWidget(self.base_list, 1)
            self.base_map_widget = SAAFBaseMapWidget()
            self.base_info_widget = SAAFBaseInfoWidget()
            side.addWidget(self.base_map_widget)
            side.addWidget(self.base_info_widget)
            root.addLayout(side, 1)
            self._tabs.addTab(page, "SAAF")

        def _tab_connection(self) -> None:
            """Create the connection tab."""
            page = QWidget()
            layout = QGridLayout(page)
            layout.addWidget(QLabel("Host"), 0, 0)
            self.host_edit = QLineEdit("127.0.0.1")
            layout.addWidget(self.host_edit, 0, 1)
            layout.addWidget(QLabel("Port"), 1, 0)
            self.port_edit = QLineEdit("5000")
            layout.addWidget(self.port_edit, 1, 1)
            layout.addWidget(QLabel("Serial Port"), 2, 0)
            self.serial_combo = QComboBox()
            if HAS_SERIAL and hasattr(serial, "tools"):
                for port in serial.tools.list_ports.comports():
                    self.serial_combo.addItem(port.device)
            layout.addWidget(self.serial_combo, 2, 1)
            self.mock_button = QPushButton("Start Mock Server")
            self.mock_button.clicked.connect(self._toggle_mock_server)
            layout.addWidget(self.mock_button, 3, 0, 1, 2)
            self.connect_button = QPushButton("Connect TCP")
            self.connect_button.clicked.connect(self._connect_tcp)
            layout.addWidget(self.connect_button, 4, 0, 1, 2)
            self.serial_button = QPushButton("Connect Serial")
            self.serial_button.clicked.connect(self._connect_serial)
            layout.addWidget(self.serial_button, 5, 0, 1, 2)
            self.disconnect_button = QPushButton("Disconnect")
            self.disconnect_button.clicked.connect(self._disconnect)
            layout.addWidget(self.disconnect_button, 6, 0, 1, 2)
            self.connection_status = QLabel("Disconnected")
            layout.addWidget(self.connection_status, 7, 0, 1, 2)
            self._tabs.addTab(page, "Connection")

        def _tab_diagnostics(self) -> None:
            """Create the diagnostics tab."""
            page = QWidget()
            layout = QVBoxLayout(page)
            self.diag_text = QTextEdit()
            self.diag_text.setReadOnly(True)
            self.diag_text.append("Diagnostics ready")
            layout.addWidget(self.diag_text)
            self._tabs.addTab(page, "Diagnostics")

        def _populate_vor_combo(self) -> None:
            """Populate the grouped VOR station combo box."""
            self.vor_combo.blockSignals(True)
            self.vor_combo.clear()
            for group_name in ["SA-Civil", "SA-SAAF", "Africa-East", "Africa-North", "Africa-West", "Africa-Central", "Africa-South"]:
                self.vor_combo.addItem(f"--- {group_name} ---", None)
                index = self.vor_combo.count() - 1
                model = self.vor_combo.model()
                item = model.item(index) if hasattr(model, "item") else None
                if item is not None:
                    item.setEnabled(False)
                    item.setForeground(QColor(120, 150, 190))
                for code in REGION_GROUPS[group_name]:
                    station = VOR_STATIONS[code]
                    self.vor_combo.addItem(f"{code} - {station['name']} ({station['vor_freq']:.2f} MHz)", code)
            self.vor_combo.blockSignals(False)
            for idx in range(self.vor_combo.count()):
                if self.vor_combo.itemData(idx) == "JNB":
                    self.vor_combo.setCurrentIndex(idx)
                    break

        def _on_vor_index_changed(self, index: int) -> None:
            """React to station selection changes."""
            station_key = self.vor_combo.itemData(index)
            if not station_key:
                return
            station = VOR_STATIONS[station_key]
            self.station_label.setText(f"{station['vor_ident']} / {station['vor_freq']:.2f} MHz")
            self._sim_engine.set_station(station_key)
            if hasattr(self, "approach"):
                self.approach.set_station(station_key)
            if hasattr(self, "terrain"):
                self.terrain.set_terrain(TERRAIN_LIBRARY.get(station_key))
            if hasattr(self, "diag_text"):
                self.diag_text.append(f"Selected station {station_key}")

        def _on_airport_layout_changed(self, index: int) -> None:
            """Load the chosen airfield layout."""
            airport_code = self.airport_combo.itemData(index)
            if airport_code:
                self._set_airport_layout(airport_code)

        def _set_airport_layout(self, airport_code: str) -> None:
            """Apply a selected airport layout to the ASRACS display."""
            self.asracs.set_layout(airport_code)
            self._asracs_engine.set_airport(airport_code)

        def _on_base_selected(self, base_code: str) -> None:
            """Jump from the SAAF tab to the corresponding ASRACS layout."""
            if base_code in AIRFIELD_LAYOUTS:
                self._tabs.setCurrentWidget(self._tabs.widget(4))
                if hasattr(self, "airport_combo"):
                    idx = self.airport_combo.findData(base_code)
                    if idx >= 0:
                        self.airport_combo.setCurrentIndex(idx)
                self._set_airport_layout(base_code)
                if hasattr(self, "base_map_widget"):
                    self.base_map_widget.set_base(SAAF_BASES.get(base_code))
                if hasattr(self, "base_info_widget"):
                    self.base_info_widget.set_base(base_code, SAAF_BASES.get(base_code))

        def _on_base_item_clicked(self, item: QListWidgetItem) -> None:
            """Handle list-based SAAF base selection."""
            base_code = item.data(Qt.UserRole)
            if isinstance(base_code, str):
                self._on_base_selected(base_code)

        def _update_vor_synthetic(self) -> None:
            """Advance the airborne simulation and refresh displays."""
            record = self._sim_engine.step()
            self._current_record = record
            self._handle_acquired_data(record)
            self._update_radar_display(record)
            self._update_approach_display(record)
            self.vor_status.setText(
                f"{record.station_key}  BRG {record.bearing_deg:03.0f}°  DEV {record.deviation_deg:+.2f}°  SIG {record.signal_percent:.0f}%  GS {record.flags.get('glide_status', '---')}"
            )
            if self._vor_series.count() > 120:
                if self._vor_series.count() > 0:
                    try:
                        self._vor_series.remove(0)
                    except (TypeError, AttributeError):
                        self._vor_series.removePoints(0, 1)
            self._vor_series.append(self._chart_x, record.signal_percent)
            self._chart_x += 1
            self._vor_chart.createDefaultAxes()

        def _update_asracs_synthetic(self) -> None:
            """Advance the ASRACS simulation."""
            tracks = self._asracs_engine.step()
            self.asracs.set_tracks(tracks)
            if hasattr(self, "alert_panel"):
                self.alert_panel.update_alerts(self._asracs_engine.alerts())

        def _toggle_mock_server(self) -> None:
            """Start or stop the built-in mock TCP server."""
            if self._mock_server is None:
                try:
                    port = int(self.port_edit.text().strip() or "5000")
                except ValueError:
                    self.connection_status.setText("Invalid port number")
                    return
                self._mock_server = MockVORTCPServer(host=self.host_edit.text().strip(), port=port, station_key="JNB")
                self._mock_server.start()
                self.connection_status.setText("Mock server running")
                self.mock_button.setText("Stop Mock Server")
                return
            self._mock_server.stop()
            self._mock_server = None
            self.connection_status.setText("Mock server stopped")
            self.mock_button.setText("Start Mock Server")

        def _import_lda_file(self) -> None:
            """Import an LDA file and display its first valid record."""
            path, _ = QFileDialog.getOpenFileName(self, "Import LDA file", "", "LDA Files (*.lda);;All Files (*)")
            if not path:
                return
            try:
                records = LDABinaryFileParser(Path(path)).parse_file()
            except Exception as exc:
                self.diag_text.append(f"LDA import failed: {exc}")
                return
            if records:
                self._handle_acquired_data(records[0])
            self.diag_text.append(f"LDA import complete: {len(records)} record(s)")

        def _update_approach_display(self, record: Optional[VORDataRecord] = None) -> None:
            """Update approach display from the supplied record."""
            active = record or self._processor.latest()
            if active is not None:
                self.approach.set_record(active)

        def _update_radar_display(self, record: Optional[VORDataRecord] = None) -> None:
            """Update radar display from the supplied record."""
            active = record or self._processor.latest()
            if active is not None:
                self.radar.set_record(active)

        def _update_diagnostics(self) -> None:
            """Append diagnostic summary for current tracking state."""
            latest = self._processor.latest()
            if latest is None:
                return
            tracker_count = len(self._tracker.all_keys())
            self.diag_text.append(
                f"DIAG {latest.station_key} BRG={latest.bearing_deg:03.0f} DEV={latest.deviation_deg:+.2f} "
                f"SIG={latest.signal_percent:.1f}% TRACKS={tracker_count}"
            )

        def _connect_serial(self) -> None:
            """Connect to selected serial source."""
            port = self.serial_combo.currentText().strip() if hasattr(self, "serial_combo") else ""
            if not port:
                self.connection_status.setText("No serial port selected")
                return
            if not self._connection.connect_serial(port, self._config.serial_baud):
                self.connection_status.setText("Serial connect failed")
                return
            self.connection_status.setText(f"Serial connected: {port}")
            self._acq_thread = DataAcquisitionThread(self._connection)
            self._acq_thread.record_received.connect(self._handle_acquired_data)
            self._acq_thread.error_occurred.connect(self._handle_connection_lost)
            self._acq_thread.start()

        def _connect_tcp(self) -> None:
            """Connect to the configured TCP endpoint and start acquisition."""
            host = self.host_edit.text().strip()
            try:
                port = int(self.port_edit.text().strip() or "5000")
            except ValueError:
                self.connection_status.setText("Invalid port number")
                return
            ok = self._connection.connect_tcp(host, port)
            if not ok:
                self.connection_status.setText("TCP connect failed")
                return
            self._connection.write_command("READ")
            self.connection_status.setText("TCP connected")
            self._acq_thread = DataAcquisitionThread(self._connection)
            self._acq_thread.record_received.connect(self._handle_acquired_data)
            self._acq_thread.error_occurred.connect(self._handle_connection_lost)
            self._acq_thread.start()

        def _disconnect(self) -> None:
            """Disconnect active source and stop acquisition thread."""
            if self._acq_thread is not None:
                self._acq_thread.stop()
                self._acq_thread.wait(1000)
                self._acq_thread = None
            self._connection.disconnect()
            self.connection_status.setText("Disconnected")

        def _handle_acquired_data(self, record: VORDataRecord) -> None:
            """Handle an acquired VOR data record."""
            self._processor.add_record(record)
            self._tracker.update(record)
            self.signal_panel.set_data(record.bearing_deg, record.signal_percent, record.is_signal_valid(), f"{record.frequency_mhz:.2f}", record.ident)
            self.cdi.set_data(record.deviation_deg, record.bearing_deg)
            self._update_radar_display(record)
            self._update_approach_display(record)
            self.vor_status.setText(f"External feed: {record.station_key} {record.signal_percent:.0f}%")
            self._update_diagnostics()

        def _handle_connection_lost(self, message: str = "Connection lost") -> None:
            """Handle data source disconnection or thread errors."""
            self.connection_status.setText(message)
            self.diag_text.append(message)

        def _handle_external_record(self, record: VORDataRecord) -> None:
            """Compatibility handler for externally received records."""
            self._handle_acquired_data(record)

        def _load_config(self) -> None:
            """Load monitor configuration from YAML when available."""
            data: Dict[str, Any] = {}
            if HAS_YAML and CONFIG_PATH.exists():
                try:
                    loaded = yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8"))
                    if isinstance(loaded, dict):
                        data = loaded
                except Exception as exc:
                    logger.warning("Config load failed: %s", exc)
            self._config = VORAirportConfig(
                station_key=str(data.get("station_key", self._config.station_key)),
                airport_code=str(data.get("airport_code", self._config.airport_code)),
                host=str(data.get("host", self._config.host)),
                port=int(data.get("port", self._config.port)),
                serial_port=str(data.get("serial_port", self._config.serial_port)),
                serial_baud=int(data.get("serial_baud", self._config.serial_baud)),
            )
            self.host_edit.setText(self._config.host)
            self.port_edit.setText(str(self._config.port))

        def _save_config(self) -> None:
            """Persist monitor configuration as YAML."""
            self._config.host = self.host_edit.text().strip() or self._config.host
            try:
                self._config.port = int(self.port_edit.text().strip() or self._config.port)
            except ValueError:
                pass
            if hasattr(self, "serial_combo"):
                self._config.serial_port = self.serial_combo.currentText().strip()
            data = {
                "station_key": self._config.station_key,
                "airport_code": self._config.airport_code,
                "host": self._config.host,
                "port": self._config.port,
                "serial_port": self._config.serial_port,
                "serial_baud": self._config.serial_baud,
            }
            if HAS_YAML:
                try:
                    CONFIG_PATH.write_text(yaml.safe_dump(data, sort_keys=True), encoding="utf-8")
                except Exception as exc:
                    logger.warning("Config save failed: %s", exc)

        def closeEvent(self, event: Any) -> None:
            """Perform orderly shutdown for background activity."""
            if hasattr(self, "_ui_timer"):
                self._ui_timer.stop()
            if hasattr(self, "_fast_timer"):
                self._fast_timer.stop()
            if self._acq_thread is not None:
                self._acq_thread.stop()
                self._acq_thread.wait(1000)
            self._connection.disconnect()
            if self._mock_server is not None:
                self._mock_server.stop()
            self._save_config()
            event.accept()
else:
    class CDIWidget(QWidget):
        """Headless CDI widget placeholder."""

    class VORSignalPanel(QWidget):
        """Headless VOR signal panel placeholder."""

    class RadarDisplay(QWidget):
        """Headless radar placeholder."""

    class ApproachGuidanceDisplay(QWidget):
        """Headless approach guidance placeholder."""

    class Terrain3DWidget(QWidget):
        """Headless terrain placeholder."""

    class ASRACSDisplay(QWidget):
        """Headless ASRACS placeholder."""

    class ASRACSAlertPanel(QWidget):
        """Headless ASRACS alert panel placeholder."""

    class SAAFBaseMapWidget(QWidget):
        """Headless SAAF base map placeholder."""

    class SAAFBaseInfoWidget(QWidget):
        """Headless SAAF base info placeholder."""

    class SAAFOverviewMap(QWidget):
        """Headless SAAF overview placeholder."""

    class VORAirportMonitorApp(QMainWindow):
        """Headless placeholder for environments without Qt."""

        def __init__(self) -> None:
            """Create a placeholder application object."""
            super().__init__()


def main() -> int:
    """Launch the desktop application."""
    if not HAS_QT:
        logger.error("PyQt5 and PyQtChart are required to run the GUI application.")
        return 1
    app = QApplication(sys.argv)
    palette = QPalette()
    palette.setColor(QPalette.Window, QColor(28, 30, 36))
    palette.setColor(QPalette.WindowText, QColor(230, 230, 230))
    palette.setColor(QPalette.Base, QColor(22, 24, 28))
    palette.setColor(QPalette.AlternateBase, QColor(34, 36, 42))
    palette.setColor(QPalette.Text, QColor(230, 230, 230))
    palette.setColor(QPalette.Button, QColor(48, 52, 60))
    palette.setColor(QPalette.ButtonText, QColor(230, 230, 230))
    palette.setColor(QPalette.Highlight, QColor(58, 120, 190))
    palette.setColor(QPalette.HighlightedText, QColor(255, 255, 255))
    palette.setColor(QPalette.ToolTipBase, QColor(48, 52, 60))
    palette.setColor(QPalette.ToolTipText, QColor(245, 245, 245))
    palette.setColor(QPalette.Link, QColor(90, 170, 255))
    app.setPalette(palette)
    app.setStyleSheet(
        """
        QTabWidget::pane { border: 1px solid #424750; background: #1f2329; }
        QTabBar::tab { background: #323844; color: #dfe7f2; padding: 8px 14px; margin: 2px; }
        QTabBar::tab:selected { background: #43658b; }
        QGroupBox { border: 1px solid #505865; margin-top: 8px; padding-top: 12px; }
        QGroupBox::title { color: #dfe7f2; left: 10px; padding: 0 4px; }
        QLabel { color: #e6e6e6; }
        QPushButton { background: #3b4552; color: #f0f0f0; border: 1px solid #5f6a78; padding: 6px 12px; }
        QPushButton:hover { background: #4c5b6d; }
        QComboBox, QLineEdit, QTextEdit, QListWidget { background: #20242a; color: #f0f0f0; border: 1px solid #566170; }
        """
    )
    window = VORAirportMonitorApp()
    window.show()
    return app.exec_()


def _self_test() -> bool:
    """Run a headless validation suite for key logic and data structures."""
    tests: List[Tuple[str, bool]] = []
    tests.append(("morse-sos", morse_encode("SOS") == "... --- ..."))
    tests.append(("african-count", len(AFRICAN_VOR_STATIONS) >= 30))
    tests.append(("master-count", len(VOR_STATIONS) >= 40))
    tests.append(("layout-count", len(AIRFIELD_LAYOUTS) >= 10))
    tests.append(("terrain-count", len(TERRAIN_LIBRARY) >= 10))
    glide = GlideSlopeDetector(runway_elevation_ft=1000.0, glide_slope_deg=3.0, threshold_crossing_height_ft=50.0)
    expected_alt = 1000.0 + 50.0 + math.tan(math.radians(3.0)) * FEET_PER_NM
    tests.append(("glide-target", abs(glide.target_altitude(1.0) - expected_alt) < 0.01))
    tests.append(("lda-parser-init", isinstance(LDAFileParser(), LDAFileParser)))
    tests.append(("sim-engine-init", isinstance(SimulationEngine(), SimulationEngine)))
    tests.append(("asracs-init", isinstance(ASRACSSimEngine(), ASRACSSimEngine)))
    record = VORDataRecord(datetime.now(timezone.utc), "JNB", "JNB", 114.9, 180.0, 0.1, 75.0, 5.0)
    tests.append(("record-init", isinstance(record, VORDataRecord)))
    tests.append(("morse-numbers", morse_encode("12") == ".---- ..---"))
    saaf_required = {"icao", "name", "lat", "lon", "elevation_ft", "vor_ident", "vor_freq", "vor_type", "ils_available", "ils_runways", "ndb_freq", "ndb_ident", "region"}
    tests.append(("saaf-fields", all(saaf_required.issubset(base.keys()) for base in SAAF_BASES.values())))
    africa_required = {"icao", "vor_freq", "ils_available"}
    tests.append(("africa-fields", all(africa_required.issubset(station.keys()) for station in AFRICAN_VOR_STATIONS.values())))
    layout_required = {"runways", "taxiways", "bounds"}
    tests.append(("layout-fields", all(layout_required.issubset(layout.keys()) for layout in AIRFIELD_LAYOUTS.values())))
    tests.append(("terrain-samples", all(len(profile) == 15 for profile in TERRAIN_LIBRARY.values())))
    parser = LDAFileParser()
    parsed = parser.parse_lines(["JNB,JNB,114.90,180.0,0.00,75.0,5.0"])
    tests.append(("parser-parse", len(parsed) == 1 and parsed[0].ident == "JNB"))
    sim_sentence = SimulationEngine("HECA").as_sentence()
    tests.append(("sim-sentence", bool(VOR_SENTENCE_RE.match(sim_sentence))))
    results_ok = all(result for _name, result in tests)
    for name, result in tests:
        logger.info("SELF-TEST %-18s %s", name, "PASS" if result else "FAIL")
    return results_ok


if __name__ == "__main__":
    if "--self-test" in sys.argv:
        sys.exit(0 if _self_test() else 1)
    sys.exit(main())
