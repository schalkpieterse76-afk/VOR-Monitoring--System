#!/usr/bin/env python3
"""
VOR / ASRACS / SAAF Airport Monitoring System  v5.3
=====================================================
v5.3 Enhancements:
  - LDA file parser (Thales/Rohde & Schwarz ILS config)
  - ILS waveform extraction from LDA data

v5.2 Enhancements:
  - SAAF VOR / ILS metadata for all 10 monitored bases
  - Glide slope detection for approach guidance
  - Surface slope analysis for terrain-aware approaches

v5.1 Fixes:
  - FIXED  AttributeError: 'VORAirportMonitorApp' has no attribute 'radar'
           Root cause: _populate_vor_combo() was called inside _tab_vor()
           before _tab_radar() (and other tabs) had been built.
  Fix 1:   _on_vor_index_changed() guards every cross-tab widget with hasattr()
  Fix 2:   _populate_vor_combo() call removed from _tab_vor()
  Fix 3:   _populate_vor_combo() called at end of _build_ui() after ALL tabs built

Dependencies:
  pip install PyQt5 PyQtChart PyOpenGL PyOpenGL_accelerate pyserial numpy PyYAML
"""

from __future__ import annotations

import argparse
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
import traceback
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Deque, Dict, Iterable, List, Optional, Sequence, Tuple

try:
    import yaml
except ImportError:
    yaml = None

try:
    import numpy as np
except ImportError:
    np = None

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    serial = None

PYQT_AVAILABLE = True
CHART_AVAILABLE = True
OPENGL_AVAILABLE = True

try:
    from PyQt5.QtCore import QPoint, QPointF, QRect, QRectF, Qt, QThread, QTimer, pyqtSignal
    from PyQt5.QtGui import (
        QBrush,
        QColor,
        QFont,
        QPalette,
        QPainter,
        QPen,
        QPolygon,
        QPolygonF,
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
        QHeaderView,
        QLabel,
        QLineEdit,
        QListWidget,
        QListWidgetItem,
        QMainWindow,
        QMessageBox,
        QPushButton,
        QScrollArea,
        QSplitter,
        QTableWidget,
        QTableWidgetItem,
        QTabWidget,
        QTextEdit,
        QToolBar,
        QVBoxLayout,
        QWidget,
        QOpenGLWidget,
    )
    try:
        from PyQt5.QtChart import QChart, QChartView, QLineSeries
    except ImportError:
        CHART_AVAILABLE = False
        QChart = QChartView = QLineSeries = None
except ImportError:
    PYQT_AVAILABLE = False
    CHART_AVAILABLE = False

    class _DummySignal:
        def __init__(self) -> None:
            self._slots: List[Any] = []

        def connect(self, slot: Any) -> None:
            self._slots.append(slot)

        def emit(self, *args: Any, **kwargs: Any) -> None:
            for slot in list(self._slots):
                try:
                    slot(*args, **kwargs)
                except Exception:
                    pass

    def pyqtSignal(*_args: Any, **_kwargs: Any) -> _DummySignal:
        return _DummySignal()

    class _QtBase:
        def __init__(self, *args: Any, **kwargs: Any) -> None:
            self._dummy_signal = _DummySignal()

        def __getattr__(self, _name: str) -> Any:
            return lambda *args, **kwargs: None

        def setLayout(self, *_args: Any, **_kwargs: Any) -> None:
            pass

        def update(self) -> None:
            pass

        def repaint(self) -> None:
            pass

        def width(self) -> int:
            return 800

        def height(self) -> int:
            return 600

        def rect(self) -> Any:
            return None

    class Qt:
        AlignCenter = 0
        AlignLeft = 0
        AlignRight = 0
        AlignTop = 0
        AlignBottom = 0
        Horizontal = 0
        Vertical = 1
        NoBrush = 0
        SolidLine = 0
        DashLine = 1
        DotLine = 2
        StrongFocus = 0
        white = 0
        black = 0
        red = 0
        green = 0
        yellow = 0

    class QApplication(_QtBase):
        def setStyle(self, *_args: Any, **_kwargs: Any) -> None:
            pass

        def setPalette(self, *_args: Any, **_kwargs: Any) -> None:
            pass

        def exec_(self) -> int:
            return 0

    class QWidget(_QtBase):
        pass

    class QMainWindow(QWidget):
        pass

    class QDialog(QWidget):
        pass

    class QOpenGLWidget(QWidget):
        pass

    class QLabel(QWidget):
        def __init__(self, text: str = "", *args: Any, **kwargs: Any) -> None:
            super().__init__(*args, **kwargs)
            self._text = text

        def setText(self, text: str) -> None:
            self._text = text

        def text(self) -> str:
            return self._text

    class QTextEdit(QLabel):
        def append(self, text: str) -> None:
            self._text += ("\n" if self._text else "") + text

        def setReadOnly(self, *_args: Any) -> None:
            pass

        def setPlainText(self, text: str) -> None:
            self._text = text

    class QLineEdit(QLabel):
        def text(self) -> str:
            return self._text

    class QPushButton(QWidget):
        def __init__(self, text: str = "", *args: Any, **kwargs: Any) -> None:
            super().__init__(*args, **kwargs)
            self.clicked = _DummySignal()
            self._text = text

    class QListWidget(QWidget):
        def __init__(self, *args: Any, **kwargs: Any) -> None:
            super().__init__(*args, **kwargs)
            self._items: List[str] = []
            self.currentTextChanged = _DummySignal()

        def clear(self) -> None:
            self._items = []

        def addItem(self, text: str) -> None:
            self._items.append(text)

    class QListWidgetItem:
        def __init__(self, text: str = "") -> None:
            self._text = text

    class QComboBox(QWidget):
        def __init__(self, *args: Any, **kwargs: Any) -> None:
            super().__init__(*args, **kwargs)
            self.currentIndexChanged = _DummySignal()
            self.currentTextChanged = _DummySignal()
            self._items: List[Tuple[str, Any]] = []
            self._index = -1

        def clear(self) -> None:
            self._items = []
            self._index = -1

        def addItem(self, text: str, userData: Any = None) -> None:
            self._items.append((text, userData))
            if self._index < 0:
                self._index = 0

        def currentData(self) -> Any:
            if 0 <= self._index < len(self._items):
                return self._items[self._index][1]
            return None

        def currentText(self) -> str:
            if 0 <= self._index < len(self._items):
                return self._items[self._index][0]
            return ""

        def setCurrentIndex(self, index: int) -> None:
            self._index = index
            self.currentIndexChanged.emit(index)
            self.currentTextChanged.emit(self.currentText())

    class QTableWidget(QWidget):
        def __init__(self, *_args: Any, **_kwargs: Any) -> None:
            super().__init__()
            self._cols = 0
            self._rows = 0

        def setColumnCount(self, n: int) -> None:
            self._cols = n

        def setRowCount(self, n: int) -> None:
            self._rows = n

        def horizontalHeader(self) -> Any:
            return self

        def setHorizontalHeaderLabels(self, *_args: Any) -> None:
            pass

        def setSectionResizeMode(self, *_args: Any) -> None:
            pass

        def setItem(self, *_args: Any) -> None:
            pass

    class QTableWidgetItem:
        def __init__(self, text: str = "") -> None:
            self._text = text

    class QHeaderView:
        Stretch = 0

    class QVBoxLayout(_QtBase):
        def addWidget(self, *_args: Any, **_kwargs: Any) -> None:
            pass

        def addLayout(self, *_args: Any, **_kwargs: Any) -> None:
            pass

        def addStretch(self, *_args: Any) -> None:
            pass

    class QHBoxLayout(QVBoxLayout):
        pass

    class QGridLayout(QVBoxLayout):
        def addWidget(self, *_args: Any, **_kwargs: Any) -> None:
            pass

    class QGroupBox(QWidget):
        pass

    class QSplitter(QWidget):
        def addWidget(self, *_args: Any, **_kwargs: Any) -> None:
            pass

    class QScrollArea(QWidget):
        pass

    class QAction(QWidget):
        pass

    class QFileDialog:
        pass

    class QMessageBox:
        @staticmethod
        def warning(*_args: Any, **_kwargs: Any) -> None:
            pass

        @staticmethod
        def information(*_args: Any, **_kwargs: Any) -> None:
            pass

        @staticmethod
        def critical(*_args: Any, **_kwargs: Any) -> None:
            pass

    class QToolBar(QWidget):
        pass

    class QTabWidget(QWidget):
        def addTab(self, *_args: Any, **_kwargs: Any) -> None:
            pass

    class QTimer(_QtBase):
        def __init__(self, *args: Any, **kwargs: Any) -> None:
            super().__init__(*args, **kwargs)
            self.timeout = _DummySignal()

        def start(self, *_args: Any, **_kwargs: Any) -> None:
            pass

        def stop(self) -> None:
            pass

    class QThread(_QtBase):
        def __init__(self, *args: Any, **kwargs: Any) -> None:
            super().__init__(*args, **kwargs)
            self._running = False

        def start(self) -> None:
            self._running = True
            self.run()

        def quit(self) -> None:
            self._running = False

        def wait(self, *_args: Any, **_kwargs: Any) -> None:
            pass

        def isRunning(self) -> bool:
            return self._running

    class QColor:
        def __init__(self, *_args: Any, **_kwargs: Any) -> None:
            pass

    class QPalette:
        Window = 0
        WindowText = 1
        Base = 2
        AlternateBase = 3
        Text = 4
        Button = 5
        ButtonText = 6
        Highlight = 7
        HighlightedText = 8

        def setColor(self, *_args: Any, **_kwargs: Any) -> None:
            pass

    class QPainter(_QtBase):
        Antialiasing = 0

    class QPen:
        def __init__(self, *_args: Any, **_kwargs: Any) -> None:
            pass

    class QBrush:
        def __init__(self, *_args: Any, **_kwargs: Any) -> None:
            pass

    class QFont:
        def __init__(self, *_args: Any, **_kwargs: Any) -> None:
            pass

    class QRect:
        def __init__(self, *_args: Any, **_kwargs: Any) -> None:
            pass

    class QRectF(QRect):
        pass

    class QPoint:
        def __init__(self, x: int = 0, y: int = 0) -> None:
            self.x = x
            self.y = y

    class QPointF(QPoint):
        pass

    class QPolygon(list):
        pass

    class QPolygonF(list):
        pass

try:
    from OpenGL import GL, GLU
except ImportError:
    OPENGL_AVAILABLE = False
    GL = None
    GLU = None

LOG_PATH = Path(__file__).with_name("vor_monitor.log")
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    handlers=[logging.FileHandler(str(LOG_PATH)), logging.StreamHandler()],
)
logger = logging.getLogger(__name__)


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


def safe_int(value: float) -> int:
    return int(round(float(value)))


def haversine_nm(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    r_km = 6371.0
    p1 = math.radians(lat1)
    p2 = math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return (2 * r_km * math.atan2(math.sqrt(a), math.sqrt(1 - a))) * 0.539957


def initial_bearing_deg(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    p1 = math.radians(lat1)
    p2 = math.radians(lat2)
    dl = math.radians(lon2 - lon1)
    y = math.sin(dl) * math.cos(p2)
    x = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
    return (math.degrees(math.atan2(y, x)) + 360.0) % 360.0


def destination_point(lat: float, lon: float, bearing_deg_value: float, distance_nm: float) -> Tuple[float, float]:
    earth_radius_nm = 3440.065
    brg = math.radians(bearing_deg_value)
    ang = distance_nm / earth_radius_nm
    lat1 = math.radians(lat)
    lon1 = math.radians(lon)
    lat2 = math.asin(math.sin(lat1) * math.cos(ang) + math.cos(lat1) * math.sin(ang) * math.cos(brg))
    lon2 = lon1 + math.atan2(
        math.sin(brg) * math.sin(ang) * math.cos(lat1),
        math.cos(ang) - math.sin(lat1) * math.sin(lat2),
    )
    return math.degrees(lat2), ((math.degrees(lon2) + 540) % 360) - 180


def polar_to_screen(cx: int, cy: int, range_radius_px: int, bearing_deg_value: float, distance_nm: float, max_range_nm: float) -> Tuple[int, int]:
    scale = 0.0 if max_range_nm <= 0 else clamp(distance_nm / max_range_nm, 0.0, 1.0)
    r = range_radius_px * scale
    angle = math.radians(bearing_deg_value - 90.0)
    x = cx + math.cos(angle) * r
    y = cy + math.sin(angle) * r
    return safe_int(x), safe_int(y)


def rate_from_angle(speed_kts: float, angle_deg_value: float) -> float:
    groundspeed_fpm = speed_kts * 101.2686
    return math.tan(math.radians(angle_deg_value)) * groundspeed_fpm


@dataclass
class VORData:
    frequency: float = 0.0
    ident: str = ""
    bearing: float = 0.0
    deviation: float = 0.0
    signal_strength: float = 0.0
    raw: str = ""
    timestamp: float = 0.0


@dataclass
class AircraftData:
    callsign: str = ""
    lat: float = 0.0
    lon: float = 0.0
    altitude: float = 0.0
    speed: float = 0.0
    heading: float = 0.0
    track_history: list = field(default_factory=list)


@dataclass
class GroundTarget:
    x: float = 0.0
    y: float = 0.0
    speed: float = 0.0
    heading: float = 0.0
    target_type: str = "vehicle"
    callsign: str = ""
    on_runway: bool = False


@dataclass
class IncursionAlert:
    target: str = ""
    runway: str = ""
    severity: str = "CAUTION"
    message: str = ""
    timestamp: float = 0.0


class VORAirportConfig:
    """Loads SAAF and civilian airport/VOR configuration."""

    def __init__(self, config_path: Optional[Path] = None) -> None:
        self.config_path = config_path or Path(__file__).with_name("vor_config.yaml")
        self.data: Dict[str, Any] = {"airports": {}, "vor_stations": {}}
        self.load()

    def load(self) -> None:
        loaded = None
        if yaml is not None and self.config_path.exists():
            try:
                with self.config_path.open("r", encoding="utf-8") as handle:
                    loaded = yaml.safe_load(handle) or {}
                logger.info("Loaded configuration from %s", self.config_path)
            except Exception as exc:
                logger.warning("Failed to load YAML config: %s", exc)
        if not self._is_valid(loaded):
            loaded = self._fallback_data()
            logger.info("Using hardcoded fallback configuration")
        self.data = loaded

    @staticmethod
    def _is_valid(data: Any) -> bool:
        return isinstance(data, dict) and len(data.get("vor_stations", {})) >= 13 and len(data.get("airports", {})) >= 13

    def airports(self) -> Dict[str, Dict[str, Any]]:
        return dict(self.data.get("airports", {}))

    def vor_stations(self) -> Dict[str, Dict[str, Any]]:
        return dict(self.data.get("vor_stations", {}))

    def get_airport(self, code: str) -> Dict[str, Any]:
        return dict(self.data.get("airports", {}).get(code, {}))

    def get_station(self, key: str) -> Dict[str, Any]:
        return dict(self.data.get("vor_stations", {}).get(key, {}))

    def station_keys(self) -> List[str]:
        return list(self.data.get("vor_stations", {}).keys())

    def base_codes(self) -> List[str]:
        return [
            code
            for code in self.data.get("airports", {})
            if code.startswith("FA") or code == "JNB" or code == "CPT" or code == "DUR"
        ]

    @staticmethod
    def _fallback_data() -> Dict[str, Any]:
        return {
            "airports": {
                "JNB": {"name": "OR Tambo International", "latitude": -25.5967, "longitude": 28.2394, "elevation": 1623},
                "CPT": {"name": "Cape Town International", "latitude": -33.9648, "longitude": 18.6017, "elevation": 47},
                "DUR": {"name": "King Shaka International", "latitude": -29.6144, "longitude": 31.1197, "elevation": 110},
                "FAWK": {"name": "AFB Waterkloof", "latitude": -25.8300, "longitude": 28.2225, "elevation": 1506},
                "FALM": {"name": "AFB Makhado", "latitude": -23.1600, "longitude": 29.6967, "elevation": 1524},
                "FAHS": {"name": "AFB Hoedspruit", "latitude": -24.3547, "longitude": 31.0503, "elevation": 479},
                "FALW": {"name": "AFB Langebaanweg", "latitude": -32.9689, "longitude": 18.1653, "elevation": 46},
                "FAOB": {"name": "AFB Overberg", "latitude": -34.5547, "longitude": 20.2506, "elevation": 52},
                "FASK": {"name": "AFB Swartkop", "latitude": -25.8069, "longitude": 28.1644, "elevation": 1519},
                "FABL": {"name": "AFB Bloemspruit", "latitude": -29.0939, "longitude": 26.3039, "elevation": 1354},
                "FAYP": {"name": "AFB Ysterplaat", "latitude": -33.9011, "longitude": 18.4833, "elevation": 15},
                "FADN": {"name": "AFB Durban", "latitude": -29.9686, "longitude": 30.9478, "elevation": 89},
                "FAPE": {"name": "AFS Port Elizabeth", "latitude": -33.9850, "longitude": 25.6103, "elevation": 58},
            },
            "vor_stations": {
                "JNB_VOR": {
                    "airport": "JNB", "channel": 96, "frequency": 114.90, "ident": "JHB", "ils_available": True,
                    "ils_runways": [{"frequency": 110.30, "glide_slope_deg": 3.0, "ident": "IJB", "runway": "03L"}],
                    "latitude": -25.5967, "longitude": 28.2394, "name": "OR Tambo VOR/DME", "ndb_freq": None,
                    "ndb_ident": "", "remarks": "OR Tambo approach VOR. Primary Johannesburg area navaid.", "type": "VOR/DME", "vor_available": True,
                },
                "CPT_VOR": {
                    "airport": "CPT", "channel": 104, "frequency": 115.70, "ident": "CTV", "ils_available": True,
                    "ils_runways": [{"frequency": 110.90, "glide_slope_deg": 3.0, "ident": "ICT", "runway": "02"}],
                    "latitude": -33.9648, "longitude": 18.6017, "name": "Cape Town VORTAC", "ndb_freq": None,
                    "ndb_ident": "", "remarks": "Cape Town approach VOR. Also serves AFB Ysterplaat.", "type": "VORTAC", "vor_available": True,
                },
                "DUR_VOR": {
                    "airport": "DUR", "channel": 72, "frequency": 112.50, "ident": "DNV", "ils_available": True,
                    "ils_runways": [{"frequency": 109.70, "glide_slope_deg": 3.0, "ident": "IDN", "runway": "06"}],
                    "latitude": -29.6144, "longitude": 31.1197, "name": "Durban VOR/DME", "ndb_freq": 393,
                    "ndb_ident": "DU", "remarks": "King Shaka / Durban area VOR. Also serves AFB Durban.", "type": "VOR/DME", "vor_available": True,
                },
                "FAWK_VOR": {
                    "airport": "FAWK", "channel": 116, "frequency": 116.90, "ident": "WKV", "ils_available": True,
                    "ils_runways": [{"frequency": 111.50, "glide_slope_deg": 3.0, "ident": "IWK", "runway": "01"}],
                    "latitude": -25.8300, "longitude": 28.2225, "name": "Waterkloof VORTAC", "ndb_freq": 315,
                    "ndb_ident": "WK", "remarks": "Primary SAAF strategic base. Serves Waterkloof & Swartkop.", "type": "VORTAC", "vor_available": True,
                },
                "FALM_VOR": {
                    "airport": "FALM", "channel": 97, "frequency": 115.00, "ident": "LTV", "ils_available": True,
                    "ils_runways": [
                        {"frequency": 110.10, "glide_slope_deg": 3.0, "ident": "ILM", "runway": "10"},
                        {"frequency": 111.30, "glide_slope_deg": 3.0, "ident": "ILM2", "runway": "28"},
                    ],
                    "latitude": -23.1600, "longitude": 29.6967, "name": "Makhado VOR/DME", "ndb_freq": 457,
                    "ndb_ident": "MK", "remarks": "Fighter base. Gripen & Hawk.", "type": "VOR/DME", "vor_available": True,
                },
                "FAHS_VOR": {
                    "airport": "FAHS", "channel": 87, "frequency": 114.00, "ident": "HSV", "ils_available": True,
                    "ils_runways": [{"frequency": 109.50, "glide_slope_deg": 3.0, "ident": "IHS", "runway": "09"}],
                    "latitude": -24.3547, "longitude": 31.0503, "name": "Hoedspruit VOR/DME", "ndb_freq": 265,
                    "ndb_ident": "HA", "remarks": "Rooivalk attack helicopter base.", "type": "VOR/DME", "vor_available": True,
                },
                "FALW_VOR": {
                    "airport": "FALW", "channel": 117, "frequency": 117.00, "ident": "LWV", "ils_available": False,
                    "ils_runways": [], "latitude": -32.9689, "longitude": 18.1653, "name": "Langebaanweg VORTAC", "ndb_freq": 345,
                    "ndb_ident": "LW", "remarks": "Pilot training base.", "type": "VORTAC", "vor_available": True,
                },
                "FAOB_VOR": {
                    "airport": "FAOB", "channel": 101, "frequency": 115.40, "ident": "OBV", "ils_available": True,
                    "ils_runways": [{"frequency": 110.50, "glide_slope_deg": 3.0, "ident": "IOB", "runway": "35"}],
                    "latitude": -34.5547, "longitude": 20.2506, "name": "Overberg VOR/DME", "ndb_freq": 428,
                    "ndb_ident": "OB", "remarks": "Test & Eval base.", "type": "VOR/DME", "vor_available": True,
                },
                "FASK_VOR": {
                    "airport": "FASK", "channel": 116, "frequency": 116.90, "ident": "WKV", "ils_available": False,
                    "ils_runways": [], "latitude": -25.8069, "longitude": 28.1644, "name": "Swartkop NDB / WKV", "ndb_freq": 390,
                    "ndb_ident": "SK", "remarks": "Historic SAAF Museum base.", "type": "VORTAC", "vor_available": True,
                },
                "FABL_VOR": {
                    "airport": "FABL", "channel": 88, "frequency": 114.10, "ident": "BLV", "ils_available": True,
                    "ils_runways": [{"frequency": 109.90, "glide_slope_deg": 3.0, "ident": "IBL", "runway": "20"}],
                    "latitude": -29.0939, "longitude": 26.3039, "name": "Bloemfontein VOR/DME", "ndb_freq": 380,
                    "ndb_ident": "BL", "remarks": "Shared civil/military field.", "type": "VOR/DME", "vor_available": True,
                },
                "FAYP_VOR": {
                    "airport": "FAYP", "channel": 104, "frequency": 115.70, "ident": "CTV", "ils_available": False,
                    "ils_runways": [], "latitude": -33.9011, "longitude": 18.4833, "name": "Ysterplaat NDB / CTV", "ndb_freq": 284,
                    "ndb_ident": "YP", "remarks": "Maritime patrol & SAR.", "type": "VORTAC", "vor_available": True,
                },
                "FADN_VOR": {
                    "airport": "FADN", "channel": 72, "frequency": 112.50, "ident": "DNV", "ils_available": True,
                    "ils_runways": [{"frequency": 109.70, "glide_slope_deg": 3.0, "ident": "IDN", "runway": "06"}],
                    "latitude": -29.9686, "longitude": 30.9478, "name": "AFB Durban VOR/DME", "ndb_freq": 393,
                    "ndb_ident": "DU", "remarks": "Maritime patrol base KZN.", "type": "VOR/DME", "vor_available": True,
                },
                "FAPE_VOR": {
                    "airport": "FAPE", "channel": 81, "frequency": 113.40, "ident": "PEV", "ils_available": True,
                    "ils_runways": [{"frequency": 110.70, "glide_slope_deg": 3.0, "ident": "IPE", "runway": "08"}],
                    "latitude": -33.9850, "longitude": 25.6103, "name": "Port Elizabeth AFS VOR", "ndb_freq": 330,
                    "ndb_ident": "PE", "remarks": "AFS Port Elizabeth.", "type": "VOR/DME", "vor_available": True,
                },
            },
        }


class GlideSlopeDetector:
    """ICAO-standard glide slope calculator."""

    def __init__(self, standard_angle_deg: float = 3.0) -> None:
        self.standard_angle_deg = standard_angle_deg

    def required_altitude(self, distance_nm: float, threshold_elevation_ft: float = 0.0, angle_deg: Optional[float] = None) -> float:
        angle = self.standard_angle_deg if angle_deg is None else angle_deg
        if distance_nm <= 0:
            return threshold_elevation_ft
        distance_ft = distance_nm * 6076.12
        return threshold_elevation_ft + math.tan(math.radians(angle)) * distance_ft

    def approach_angle(self, distance_nm: float, altitude_ft: float, threshold_elevation_ft: float = 0.0) -> float:
        if distance_nm <= 0:
            return 90.0 if altitude_ft > threshold_elevation_ft else 0.0
        return math.degrees(math.atan2(altitude_ft - threshold_elevation_ft, distance_nm * 6076.12))

    def deviation_ft(self, distance_nm: float, altitude_ft: float, threshold_elevation_ft: float = 0.0, angle_deg: Optional[float] = None) -> float:
        return altitude_ft - self.required_altitude(distance_nm, threshold_elevation_ft, angle_deg)

    def deviation_deg(self, distance_nm: float, altitude_ft: float, threshold_elevation_ft: float = 0.0) -> float:
        return self.approach_angle(distance_nm, altitude_ft, threshold_elevation_ft) - self.standard_angle_deg

    def deviation_dots(self, distance_nm: float, altitude_ft: float, threshold_elevation_ft: float = 0.0) -> float:
        return clamp(self.deviation_deg(distance_nm, altitude_ft, threshold_elevation_ft) / 0.35, -2.5, 2.5)

    def classify(self, distance_nm: float, altitude_ft: float, threshold_elevation_ft: float = 0.0) -> str:
        dots = abs(self.deviation_dots(distance_nm, altitude_ft, threshold_elevation_ft))
        if dots < 0.5:
            return "ON_PROFILE"
        if dots < 1.5:
            return "MINOR_DEVIATION"
        return "UNSTABLE"

    def analyze(self, distance_nm: float, altitude_ft: float, speed_kts: float, threshold_elevation_ft: float = 0.0) -> Dict[str, float]:
        angle = self.approach_angle(distance_nm, altitude_ft, threshold_elevation_ft)
        required_alt = self.required_altitude(distance_nm, threshold_elevation_ft)
        deviation_ft_value = altitude_ft - required_alt
        return {
            "required_altitude_ft": required_alt,
            "current_angle_deg": angle,
            "deviation_ft": deviation_ft_value,
            "deviation_deg": angle - self.standard_angle_deg,
            "deviation_dots": self.deviation_dots(distance_nm, altitude_ft, threshold_elevation_ft),
            "required_descent_rate_fpm": abs(rate_from_angle(speed_kts, self.standard_angle_deg)),
        }


class SurfaceSlopeAnalyzer:
    """Terrain and surface gradient analysis."""

    def analyze_profile(self, profile: Sequence[Tuple[float, float]]) -> Dict[str, Any]:
        if len(profile) < 2:
            return {
                "segment_count": 0,
                "max_slope_percent": 0.0,
                "avg_slope_percent": 0.0,
                "min_elevation_ft": profile[0][1] if profile else 0.0,
                "max_elevation_ft": profile[0][1] if profile else 0.0,
                "terrain_clearance_ft": 0.0,
                "segments": [],
            }
        segments = []
        abs_slopes = []
        elevations = [pt[1] for pt in profile]
        for (d1, a1), (d2, a2) in zip(profile[:-1], profile[1:]):
            run_ft = max(abs(d2 - d1) * 6076.12, 1.0)
            rise_ft = a2 - a1
            slope_percent = (rise_ft / run_ft) * 100.0
            abs_slopes.append(abs(slope_percent))
            segments.append({
                "start_nm": d1,
                "end_nm": d2,
                "rise_ft": rise_ft,
                "run_ft": run_ft,
                "slope_percent": slope_percent,
            })
        return {
            "segment_count": len(segments),
            "max_slope_percent": max(abs_slopes) if abs_slopes else 0.0,
            "avg_slope_percent": sum(abs_slopes) / len(abs_slopes) if abs_slopes else 0.0,
            "min_elevation_ft": min(elevations),
            "max_elevation_ft": max(elevations),
            "terrain_clearance_ft": max(elevations) - min(elevations),
            "segments": segments,
        }

    def required_descent_rate(self, speed_kts: float, glide_angle_deg: float = 3.0) -> float:
        return abs(rate_from_angle(speed_kts, glide_angle_deg))

    def synthesize_approach_profile(self, threshold_elevation_ft: float, distance_nm: float, samples: int = 20) -> List[Tuple[float, float]]:
        detector = GlideSlopeDetector()
        profile = []
        for i in range(samples + 1):
            dist = distance_nm * (1.0 - (i / samples))
            baseline = detector.required_altitude(dist, threshold_elevation_ft)
            terrain = threshold_elevation_ft + 40 * math.sin(i / 3.0) + 25 * math.cos(i / 4.0)
            profile.append((dist, max(terrain, baseline - 150.0)))
        profile.sort(key=lambda item: item[0])
        return profile


class LDAFileParser:
    """Parses text and mixed-binary LDA configuration files."""

    _TEXT_PATTERN = re.compile(r"([A-Za-z0-9_./\-]+)\s*[:=]\s*(.+)")
    _FREQ_PATTERN = re.compile(r"(\d{2,3}\.\d{1,3})")
    _LIMIT_PATTERN = re.compile(r"([A-Za-z0-9_./\-]+)\s*(?:LOW|HI|HIGH|LIMIT)?\s*[:=]\s*([-+]?\d+(?:\.\d+)?)")

    def parse_file(self, path: Path) -> Dict[str, Any]:
        data = path.read_bytes()
        return self.parse_bytes(data, source_name=str(path))

    def parse_bytes(self, data: bytes, source_name: str = "<memory>") -> Dict[str, Any]:
        text_chunks = []
        try:
            text_chunks.append(data.decode("utf-8"))
        except UnicodeDecodeError:
            text_chunks.append(data.decode("latin-1", errors="ignore"))
        extracted = self._extract_binary_strings(data)
        text_chunks.extend(extracted)
        parsed: Dict[str, Any] = {
            "source": source_name,
            "vendor": "Unknown",
            "waveforms": [],
            "frequencies": [],
            "nominal_values": {},
            "alarm_limits": {},
            "ident": "",
            "runway": "",
            "raw_strings": extracted,
        }
        for chunk in text_chunks:
            self._parse_text_block(chunk, parsed)
        parsed["waveforms"] = sorted(set(parsed["waveforms"]))
        parsed["frequencies"] = sorted(set(parsed["frequencies"]))
        return parsed

    def _extract_binary_strings(self, data: bytes) -> List[str]:
        strings: List[str] = []
        current = bytearray()
        for byte in data:
            if 32 <= byte <= 126:
                current.append(byte)
            else:
                if len(current) >= 4:
                    strings.append(current.decode("ascii", errors="ignore"))
                current = bytearray()
        if len(current) >= 4:
            strings.append(current.decode("ascii", errors="ignore"))
        return strings

    def _parse_text_block(self, text: str, parsed: Dict[str, Any]) -> None:
        for raw_line in text.splitlines():
            line = raw_line.strip().strip("\x00")
            if not line:
                continue
            upper = line.upper()
            if "THALES" in upper:
                parsed["vendor"] = "Thales"
            elif "ROHDE" in upper or "SCHWARZ" in upper:
                parsed["vendor"] = "Rohde & Schwarz"
            if "WAVE" in upper or "CSB" in upper or "SBO" in upper:
                parsed["waveforms"].append(line)
            if "IDENT" in upper and not parsed["ident"]:
                match = self._TEXT_PATTERN.search(line)
                if match:
                    parsed["ident"] = match.group(2).strip()
            if "RUNWAY" in upper and not parsed["runway"]:
                match = self._TEXT_PATTERN.search(line)
                if match:
                    parsed["runway"] = match.group(2).strip()
            for freq in self._FREQ_PATTERN.findall(line):
                try:
                    value = float(freq)
                    if 75.0 <= value <= 120.0:
                        parsed["frequencies"].append(value)
                except ValueError:
                    pass
            match = self._TEXT_PATTERN.search(line)
            if not match:
                continue
            key = match.group(1).strip()
            value = match.group(2).strip()
            key_upper = key.upper()
            try:
                numeric = float(value.split()[0])
            except (ValueError, IndexError):
                numeric = None
            if numeric is not None:
                if any(token in key_upper for token in ["NOMINAL", "MOD", "LEVEL", "POWER"]):
                    parsed["nominal_values"][key] = numeric
                if any(token in key_upper for token in ["LIMIT", "ALARM", "LOW", "HIGH", "HI"]):
                    parsed["alarm_limits"][key] = numeric


class JNBAirportLayout:
    """Approximate OR Tambo ASRACS training layout in meters."""

    WIDTH_M = 4600.0
    HEIGHT_M = 3200.0
    RUNWAYS = {
        "03L/21R": {"x": 300.0, "y": 700.0, "w": 3800.0, "h": 80.0, "heading": 30.0},
        "03R/21L": {"x": 300.0, "y": 1600.0, "w": 3800.0, "h": 80.0, "heading": 30.0},
    }
    TAXIWAYS = [
        {"name": "A", "x": 250.0, "y": 1000.0, "w": 3900.0, "h": 28.0},
        {"name": "B", "x": 250.0, "y": 1900.0, "w": 3900.0, "h": 28.0},
        {"name": "C", "x": 650.0, "y": 730.0, "w": 24.0, "h": 1200.0},
        {"name": "D", "x": 1800.0, "y": 730.0, "w": 24.0, "h": 1200.0},
        {"name": "E", "x": 3100.0, "y": 730.0, "w": 24.0, "h": 1200.0},
    ]
    TERMINALS = [
        {"name": "Terminal A", "x": 900.0, "y": 2300.0, "w": 800.0, "h": 300.0},
        {"name": "Terminal B", "x": 1800.0, "y": 2300.0, "w": 900.0, "h": 320.0},
        {"name": "Cargo", "x": 3000.0, "y": 2250.0, "w": 700.0, "h": 260.0},
    ]

    @classmethod
    def world_to_screen(cls, x: float, y: float, width: int, height: int, margin: int = 20) -> Tuple[int, int]:
        usable_w = max(width - margin * 2, 1)
        usable_h = max(height - margin * 2, 1)
        sx = margin + (x / cls.WIDTH_M) * usable_w
        sy = margin + (y / cls.HEIGHT_M) * usable_h
        return safe_int(sx), safe_int(sy)

    @classmethod
    def rect_to_screen(cls, rect: Dict[str, float], width: int, height: int, margin: int = 20) -> Tuple[int, int, int, int]:
        x1, y1 = cls.world_to_screen(rect["x"], rect["y"], width, height, margin)
        x2, y2 = cls.world_to_screen(rect["x"] + rect["w"], rect["y"] + rect["h"], width, height, margin)
        return int(x1), int(y1), int(x2 - x1), int(y2 - y1)

    @classmethod
    def runway_name_at(cls, x: float, y: float) -> Optional[str]:
        for name, rect in cls.RUNWAYS.items():
            if rect["x"] <= x <= rect["x"] + rect["w"] and rect["y"] <= y <= rect["y"] + rect["h"]:
                return name
        return None


class ASRACSSimEngine:
    """Surface movement simulation for runway incursion scenarios."""

    def __init__(self) -> None:
        self.targets: List[GroundTarget] = []
        self.alerts: List[IncursionAlert] = []
        self.lock = threading.Lock()
        self._seed_targets()

    def _seed_targets(self) -> None:
        aircraft_spawns = [
            (600, 1010, 12, 90, "aircraft", "SAA241"),
            (1100, 1010, 14, 90, "aircraft", "BAW102"),
            (900, 1920, 16, 90, "aircraft", "LNX55"),
            (2500, 1010, 18, 270, "aircraft", "KLM591"),
            (3200, 1920, 15, 270, "aircraft", "SFR915"),
        ]
        vehicle_spawns = [
            (700, 760, 8, 0, "vehicle", "OPS1"),
            (1800, 760, 9, 180, "vehicle", "FIRE2"),
            (3100, 760, 7, 180, "vehicle", "MECH3"),
            (900, 2250, 11, 0, "vehicle", "BUS4"),
            (2900, 2250, 10, 180, "vehicle", "CARGO5"),
        ]
        self.targets = [GroundTarget(*spawn) for spawn in aircraft_spawns + vehicle_spawns]
        for target in self.targets:
            target.on_runway = JNBAirportLayout.runway_name_at(target.x, target.y) is not None

    def step(self, dt: float = 1.0) -> None:
        with self.lock:
            for target in self.targets:
                turn = random.uniform(-12.0, 12.0)
                if random.random() < 0.15:
                    target.heading = (target.heading + turn) % 360.0
                distance_m = target.speed * 0.514444 * dt
                rad = math.radians(target.heading)
                target.x += math.cos(rad) * distance_m
                target.y += math.sin(rad) * distance_m
                target.x = clamp(target.x, 100.0, JNBAirportLayout.WIDTH_M - 100.0)
                target.y = clamp(target.y, 100.0, JNBAirportLayout.HEIGHT_M - 100.0)
                runway = JNBAirportLayout.runway_name_at(target.x, target.y)
                target.on_runway = runway is not None
                if target.target_type == "vehicle" and random.random() < 0.08:
                    target.heading = random.choice([0, 90, 180, 270])
            self.alerts = self._detect_incursions()

    def _detect_incursions(self) -> List[IncursionAlert]:
        alerts: List[IncursionAlert] = []
        runway_map: Dict[str, List[GroundTarget]] = {}
        for target in self.targets:
            runway = JNBAirportLayout.runway_name_at(target.x, target.y)
            if runway:
                runway_map.setdefault(runway, []).append(target)
        for runway, occupants in runway_map.items():
            if len(occupants) <= 1:
                continue
            contains_aircraft = any(t.target_type == "aircraft" for t in occupants)
            contains_vehicle = any(t.target_type == "vehicle" for t in occupants)
            if contains_aircraft and contains_vehicle:
                severity = "CRITICAL"
                message = f"Runway incursion on {runway}: aircraft and vehicle conflict"
            elif len(occupants) >= 3:
                severity = "WARNING"
                message = f"Congestion on {runway}: multiple movements detected"
            else:
                severity = "CAUTION"
                message = f"Runway occupancy on {runway}: monitor separation"
            alerts.append(
                IncursionAlert(
                    target=", ".join(sorted(t.callsign for t in occupants)),
                    runway=runway,
                    severity=severity,
                    message=message,
                    timestamp=time.time(),
                )
            )
        return alerts

    def snapshot(self) -> Tuple[List[GroundTarget], List[IncursionAlert]]:
        with self.lock:
            return list(self.targets), list(self.alerts)


class MockVORTCPServer:
    """Simple TCP server that streams synthetic VOR data sentences."""

    def __init__(self, config: Optional[VORAirportConfig] = None, host: str = "127.0.0.1", port: int = 5000) -> None:
        self.config = config or VORAirportConfig()
        self.host = host
        self.port = port
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._socket: Optional[socket.socket] = None

    def is_running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def start(self) -> None:
        if self.is_running():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, name="MockVORTCPServer", daemon=True)
        self._thread.start()
        logger.info("Mock VOR TCP server started on %s:%s", self.host, self.port)

    def stop(self) -> None:
        self._stop_event.set()
        if self._socket:
            try:
                self._socket.close()
            except OSError:
                pass
            self._socket = None
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None
        logger.info("Mock VOR TCP server stopped")

    def _generate_sentence(self) -> str:
        station_key = random.choice(self.config.station_keys())
        station = self.config.get_station(station_key)
        bearing = random.uniform(0.0, 359.9)
        deviation = random.uniform(-10.0, 10.0)
        signal = random.uniform(60.0, 99.0)
        return (
            f"FREQ={station.get('frequency', 0.0):.2f},"
            f"IDENT={station.get('ident', '')},"
            f"BRG={bearing:.1f},"
            f"DEV={deviation:.2f},"
            f"SIG={signal:.1f}\n"
        )

    def _run(self) -> None:
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.settimeout(1.0)
        server.bind((self.host, self.port))
        server.listen(5)
        self._socket = server
        while not self._stop_event.is_set():
            try:
                conn, _addr = server.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            conn.settimeout(1.0)
            with conn:
                while not self._stop_event.is_set():
                    sentence = self._generate_sentence()
                    try:
                        conn.sendall(sentence.encode("utf-8"))
                    except OSError:
                        break
                    time.sleep(0.25)
        try:
            server.close()
        except OSError:
            pass


class VORConnectionHandler:
    """Handles TCP, serial and mock-server connectivity."""

    def __init__(self, config: Optional[VORAirportConfig] = None) -> None:
        self.config = config or VORAirportConfig()
        self.mode = "disconnected"
        self.sock: Optional[socket.socket] = None
        self.serial_port: Any = None
        self.mock_server = MockVORTCPServer(self.config)
        self._recv_buffer = b""

    def available_serial_ports(self) -> List[str]:
        if serial is None:
            return []
        try:
            return [p.device for p in serial.tools.list_ports.comports()]
        except Exception:
            return []

    def connect_tcp(self, host: str, port: int) -> bool:
        self.disconnect()
        try:
            self.sock = socket.create_connection((host, port), timeout=2.0)
            self.sock.settimeout(1.0)
            self.mode = "tcp"
            logger.info("Connected to TCP VOR source %s:%s", host, port)
            return True
        except Exception as exc:
            logger.error("TCP connect failed: %s", exc)
            self.sock = None
            return False

    def connect_serial(self, port: str, baudrate: int = 9600) -> bool:
        self.disconnect()
        if serial is None:
            logger.error("pyserial not available")
            return False
        try:
            self.serial_port = serial.Serial(port, baudrate=baudrate, timeout=1.0)
            self.mode = "serial"
            logger.info("Connected to serial VOR source %s @ %s", port, baudrate)
            return True
        except Exception as exc:
            logger.error("Serial connect failed: %s", exc)
            self.serial_port = None
            return False

    def start_mock_server(self) -> None:
        self.mock_server.start()

    def stop_mock_server(self) -> None:
        self.mock_server.stop()

    def disconnect(self) -> None:
        if self.sock:
            try:
                self.sock.close()
            except OSError:
                pass
        if self.serial_port:
            try:
                self.serial_port.close()
            except OSError:
                pass
        self.sock = None
        self.serial_port = None
        self._recv_buffer = b""
        self.mode = "disconnected"

    def is_connected(self) -> bool:
        return self.mode in {"tcp", "serial"} and (self.sock is not None or self.serial_port is not None)

    def read_line(self) -> Optional[str]:
        if self.mode == "tcp" and self.sock:
            try:
                while b"\n" not in self._recv_buffer:
                    chunk = self.sock.recv(4096)
                    if not chunk:
                        return None
                    self._recv_buffer += chunk
                line, self._recv_buffer = self._recv_buffer.split(b"\n", 1)
                return line.decode("utf-8", errors="ignore").strip()
            except socket.timeout:
                return None
            except Exception as exc:
                logger.error("TCP read failed: %s", exc)
                return None
        if self.mode == "serial" and self.serial_port:
            try:
                data = self.serial_port.readline()
                return data.decode("utf-8", errors="ignore").strip() if data else None
            except Exception as exc:
                logger.error("Serial read failed: %s", exc)
                return None
        return None


class VORDataProcessor:
    """Parses VOR receiver sentences into structured records."""

    def __init__(self, maxlen: int = 1000) -> None:
        self.buffer: Deque[VORData] = deque(maxlen=maxlen)
        self.last_data = VORData()

    def _safe_float(self, value: Any, default: float = 0.0) -> float:
        try:
            return float(value)
        except Exception:
            return default

    def parse_line(self, line: str) -> Optional[VORData]:
        if not line:
            return None
        line = line.strip()
        raw = line
        parsed = VORData(raw=raw, timestamp=time.time())
        if "=" in line:
            items = {}
            for part in line.split(","):
                if "=" not in part:
                    continue
                key, value = part.split("=", 1)
                items[key.strip().upper()] = value.strip()
            parsed.frequency = self._safe_float(items.get("FREQ", items.get("FREQUENCY", 0.0)))
            parsed.ident = str(items.get("IDENT", items.get("ID", ""))).strip().upper()
            parsed.bearing = self._safe_float(items.get("BRG", items.get("BEARING", 0.0))) % 360.0
            parsed.deviation = self._safe_float(items.get("DEV", items.get("DEVIATION", 0.0)))
            parsed.signal_strength = self._safe_float(items.get("SIG", items.get("SIGNAL", 0.0)))
        else:
            parts = [p.strip() for p in line.split(",")]
            if len(parts) >= 5:
                parsed.frequency = self._safe_float(parts[0])
                parsed.ident = parts[1].upper()
                parsed.bearing = self._safe_float(parts[2]) % 360.0
                parsed.deviation = self._safe_float(parts[3])
                parsed.signal_strength = self._safe_float(parts[4])
            else:
                return None
        self.last_data = parsed
        self.buffer.append(parsed)
        return parsed

    def signal_series(self, limit: int = 100) -> List[Tuple[float, float]]:
        data = list(self.buffer)[-limit:]
        return [(item.timestamp, item.signal_strength) for item in data]

    def deviation_series(self, limit: int = 100) -> List[Tuple[float, float]]:
        data = list(self.buffer)[-limit:]
        return [(item.timestamp, item.deviation) for item in data]

    def stats(self) -> Dict[str, float]:
        if not self.buffer:
            return {"count": 0, "avg_signal": 0.0, "avg_abs_deviation": 0.0}
        signals = [item.signal_strength for item in self.buffer]
        devs = [abs(item.deviation) for item in self.buffer]
        return {
            "count": float(len(self.buffer)),
            "avg_signal": sum(signals) / len(signals),
            "avg_abs_deviation": sum(devs) / len(devs),
        }


class AircraftTracker:
    """Tracks aircraft around the currently selected VOR."""

    def __init__(self, reference_lat: float = -25.5967, reference_lon: float = 28.2394) -> None:
        self.reference_lat = reference_lat
        self.reference_lon = reference_lon
        self.aircraft: Dict[str, AircraftData] = {}

    def set_reference(self, lat: float, lon: float) -> None:
        self.reference_lat = lat
        self.reference_lon = lon

    def update_track(self, callsign: str, lat: float, lon: float, altitude: float = 0.0, speed: float = 0.0, heading: float = 0.0) -> AircraftData:
        aircraft = self.aircraft.get(callsign, AircraftData(callsign=callsign))
        aircraft.lat = lat
        aircraft.lon = lon
        aircraft.altitude = altitude
        aircraft.speed = speed
        aircraft.heading = heading
        aircraft.track_history.append((time.time(), lat, lon))
        aircraft.track_history = aircraft.track_history[-50:]
        self.aircraft[callsign] = aircraft
        return aircraft

    def add_or_update_from_vor(self, callsign: str, bearing: float, distance_nm: float, altitude_ft: float = 0.0, speed_kts: float = 0.0, heading: Optional[float] = None) -> AircraftData:
        lat, lon = destination_point(self.reference_lat, self.reference_lon, bearing, distance_nm)
        return self.update_track(callsign, lat, lon, altitude_ft, speed_kts, bearing if heading is None else heading)

    def relative_position(self, lat: float, lon: float) -> Tuple[float, float]:
        bearing = initial_bearing_deg(self.reference_lat, self.reference_lon, lat, lon)
        distance = haversine_nm(self.reference_lat, self.reference_lon, lat, lon)
        return bearing, distance

    def snapshot(self) -> List[AircraftData]:
        return list(self.aircraft.values())


class SimulationEngine:
    """Generates en-route aircraft tracks for the radar display."""

    def __init__(self, tracker: AircraftTracker) -> None:
        self.tracker = tracker
        self._state: Dict[str, Dict[str, float]] = {}
        self._seed()

    def _seed(self) -> None:
        names = ["SAA101", "BAW54", "KLM771", "ETH812", "LNX402", "SFR990"]
        for idx, callsign in enumerate(names):
            bearing = (idx * 60.0 + 20.0) % 360.0
            distance = 15.0 + idx * 7.0
            altitude = 3000.0 + idx * 1400.0
            speed = 180.0 + idx * 20.0
            heading = (bearing + 180.0) % 360.0
            self._state[callsign] = {
                "bearing": bearing,
                "distance_nm": distance,
                "altitude": altitude,
                "speed": speed,
                "heading": heading,
            }
            self.tracker.add_or_update_from_vor(callsign, bearing, distance, altitude, speed, heading)

    def step(self, dt: float = 1.0) -> None:
        for callsign, state in self._state.items():
            groundspeed_nmps = state["speed"] / 3600.0
            state["distance_nm"] += groundspeed_nmps * dt * math.cos(math.radians((state["heading"] - state["bearing"]) % 360.0))
            if state["distance_nm"] < 3.0:
                state["heading"] = (state["heading"] + 180.0) % 360.0
                state["distance_nm"] = 3.0
            elif state["distance_nm"] > 90.0:
                state["heading"] = (state["heading"] + 180.0) % 360.0
                state["distance_nm"] = 90.0
            state["bearing"] = (state["bearing"] + random.uniform(-1.5, 1.5)) % 360.0
            if random.random() < 0.1:
                state["heading"] = (state["heading"] + random.uniform(-8, 8)) % 360.0
            state["altitude"] += random.uniform(-50.0, 50.0)
            self.tracker.add_or_update_from_vor(
                callsign,
                state["bearing"],
                state["distance_nm"],
                state["altitude"],
                state["speed"],
                state["heading"],
            )


class DataAcquisitionThread(QThread):
    data_received = pyqtSignal(object)
    status_changed = pyqtSignal(str)
    error_occurred = pyqtSignal(str)

    def __init__(self, handler: VORConnectionHandler, processor: VORDataProcessor) -> None:
        super().__init__()
        self.handler = handler
        self.processor = processor
        self._stop_flag = False

    def run(self) -> None:
        self._stop_flag = False
        self.status_changed.emit("running")
        while not self._stop_flag:
            line = self.handler.read_line()
            if line is None:
                time.sleep(0.05)
                continue
            try:
                parsed = self.processor.parse_line(line)
                if parsed:
                    self.data_received.emit(parsed)
            except Exception as exc:
                logger.exception("Acquisition failure")
                self.error_occurred.emit(str(exc))
                time.sleep(0.25)
        self.status_changed.emit("stopped")

    def stop(self) -> None:
        self._stop_flag = True
        self.wait(1000)


if PYQT_AVAILABLE:

    class ASRACSDisplay(QWidget):
        def __init__(self, engine: ASRACSSimEngine, parent: Optional[QWidget] = None) -> None:
            super().__init__(parent)
            self.engine = engine
            self.setMinimumSize(640, 420)
            self.timer = QTimer(self)
            self.timer.timeout.connect(self.update)
            self.timer.start(250)

        def paintEvent(self, _event: Any) -> None:
            painter = QPainter(self)
            painter.setRenderHint(QPainter.Antialiasing)
            painter.fillRect(self.rect(), QColor(17, 24, 39))
            width = self.width()
            height = self.height()
            painter.setPen(QPen(QColor(75, 85, 99), 1))
            for taxiway in JNBAirportLayout.TAXIWAYS:
                x, y, w, h = JNBAirportLayout.rect_to_screen(taxiway, width, height)
                painter.setBrush(QBrush(QColor(55, 65, 81)))
                painter.drawRect(x, y, w, h)
                painter.setPen(QPen(QColor(229, 231, 235), 1))
                painter.drawText(x, y - 14, max(w, 50), 14, Qt.AlignCenter, taxiway["name"])
                painter.setPen(QPen(QColor(75, 85, 99), 1))
            for terminal in JNBAirportLayout.TERMINALS:
                x, y, w, h = JNBAirportLayout.rect_to_screen(terminal, width, height)
                painter.setBrush(QBrush(QColor(30, 41, 59)))
                painter.drawRect(x, y, w, h)
                painter.setPen(QPen(QColor(148, 163, 184), 1))
                painter.drawText(x, y, w, h, Qt.AlignCenter, terminal["name"])
            for runway_name, runway in JNBAirportLayout.RUNWAYS.items():
                x, y, w, h = JNBAirportLayout.rect_to_screen(runway, width, height)
                painter.setBrush(QBrush(QColor(38, 38, 38)))
                painter.setPen(QPen(QColor(180, 180, 180), 2))
                painter.drawRect(x, y, w, h)
                center_y = int(y + h / 2)
                painter.setPen(QPen(QColor(250, 250, 250), 2, Qt.DashLine))
                painter.drawLine(int(x + 10), int(center_y), int(x + w - 10), int(center_y))
                painter.setPen(QPen(QColor(255, 255, 255), 1))
                painter.drawText(x, int(y - 18), w, 18, Qt.AlignCenter, runway_name)
            targets, alerts = self.engine.snapshot()
            for target in targets:
                sx, sy = JNBAirportLayout.world_to_screen(target.x, target.y, width, height)
                color = QColor(239, 68, 68) if target.on_runway else QColor(34, 197, 94)
                if target.target_type == "vehicle":
                    color = QColor(250, 204, 21) if target.on_runway else QColor(96, 165, 250)
                painter.setPen(QPen(color, 2))
                painter.setBrush(QBrush(color))
                size = 10 if target.target_type == "vehicle" else 14
                painter.drawEllipse(int(sx - size / 2), int(sy - size / 2), int(size), int(size))
                hx = sx + math.cos(math.radians(target.heading)) * 16
                hy = sy + math.sin(math.radians(target.heading)) * 16
                painter.drawLine(int(sx), int(sy), int(hx), int(hy))
                painter.drawText(int(sx + 8), int(sy - 18), int(90), int(18), Qt.AlignLeft, target.callsign)
            banner_y = 8
            for alert in alerts[:5]:
                bg = QColor(220, 38, 38) if alert.severity == "CRITICAL" else QColor(245, 158, 11) if alert.severity == "WARNING" else QColor(59, 130, 246)
                painter.setBrush(QBrush(bg))
                painter.setPen(QPen(bg, 1))
                painter.drawRect(int(8), int(banner_y), int(width - 16), int(22))
                painter.setPen(QPen(QColor(255, 255, 255), 1))
                painter.drawText(int(14), int(banner_y + 2), int(width - 28), int(18), Qt.AlignLeft, f"{alert.severity}: {alert.message}")
                banner_y += 26


    class ASRACSAlertPanel(QWidget):
        def __init__(self, parent: Optional[QWidget] = None) -> None:
            super().__init__(parent)
            layout = QVBoxLayout(self)
            self.list_widget = QListWidget()
            layout.addWidget(QLabel("Active Incursion Alerts"))
            layout.addWidget(self.list_widget)

        def refresh(self, alerts: Sequence[IncursionAlert]) -> None:
            self.list_widget.clear()
            for alert in alerts:
                self.list_widget.addItem(f"[{alert.severity}] {alert.runway} - {alert.message}")


    class SAAFOverviewMap(QWidget):
        def __init__(self, config: VORAirportConfig, parent: Optional[QWidget] = None) -> None:
            super().__init__(parent)
            self.config = config
            self.selected_code: Optional[str] = None
            self.setMinimumSize(500, 420)

        def set_selected_code(self, code: str) -> None:
            self.selected_code = code
            self.update()

        def _project(self, lat: float, lon: float) -> Tuple[int, int]:
            codes = self.config.airports()
            lats = [a["latitude"] for a in codes.values()]
            lons = [a["longitude"] for a in codes.values()]
            lat_min, lat_max = min(lats) - 1.0, max(lats) + 1.0
            lon_min, lon_max = min(lons) - 1.0, max(lons) + 1.0
            x = 30 + ((lon - lon_min) / max(lon_max - lon_min, 0.1)) * (self.width() - 60)
            y = 30 + ((lat_max - lat) / max(lat_max - lat_min, 0.1)) * (self.height() - 60)
            return safe_int(x), safe_int(y)

        def paintEvent(self, _event: Any) -> None:
            painter = QPainter(self)
            painter.setRenderHint(QPainter.Antialiasing)
            painter.fillRect(self.rect(), QColor(15, 23, 42))
            painter.setPen(QPen(QColor(51, 65, 85), 1))
            painter.setBrush(Qt.NoBrush)
            painter.drawRect(int(20), int(20), int(self.width() - 40), int(self.height() - 40))
            painter.setPen(QPen(QColor(100, 116, 139), 1, Qt.DashLine))
            for i in range(1, 5):
                x = int(20 + i * (self.width() - 40) / 5)
                painter.drawLine(int(x), int(20), int(x), int(self.height() - 20))
            for i in range(1, 4):
                y = int(20 + i * (self.height() - 40) / 4)
                painter.drawLine(int(20), int(y), int(self.width() - 20), int(y))
            for code, airport in self.config.airports().items():
                x, y = self._project(airport["latitude"], airport["longitude"])
                is_saaf = code.startswith("FA")
                color = QColor(59, 130, 246) if is_saaf else QColor(34, 197, 94)
                size = 12 if code == self.selected_code else 8
                if code == self.selected_code:
                    color = QColor(248, 113, 113)
                painter.setPen(QPen(color, 2))
                painter.setBrush(QBrush(color))
                painter.drawEllipse(int(x - size / 2), int(y - size / 2), int(size), int(size))
                painter.setPen(QPen(QColor(226, 232, 240), 1))
                painter.drawText(int(x + 6), int(y - 10), int(160), int(20), Qt.AlignLeft, f"{code} {airport['name']}")


    class SAAFBaseMapWidget(QWidget):
        def __init__(self, config: VORAirportConfig, parent: Optional[QWidget] = None) -> None:
            super().__init__(parent)
            self.config = config
            self.base_code = "FAWK"
            self.runway_headings = {
                "FAWK": [("01/19", 10.0)], "FALM": [("10/28", 100.0)], "FAHS": [("09/27", 90.0)],
                "FALW": [("01/19", 10.0)], "FAOB": [("17/35", 170.0)], "FASK": [("07/25", 70.0)],
                "FABL": [("02/20", 20.0)], "FAYP": [("02/20", 20.0)], "FADN": [("06/24", 60.0)],
                "FAPE": [("08/26", 80.0)], "JNB": [("03L/21R", 30.0), ("03R/21L", 30.0)],
                "CPT": [("01/19", 10.0)], "DUR": [("06/24", 60.0)],
            }
            self.setMinimumSize(420, 300)

        def set_base(self, code: str) -> None:
            self.base_code = code
            self.update()

        def paintEvent(self, _event: Any) -> None:
            painter = QPainter(self)
            painter.setRenderHint(QPainter.Antialiasing)
            painter.fillRect(self.rect(), QColor(10, 15, 26))
            painter.setPen(QPen(QColor(100, 116, 139), 1))
            painter.drawRect(int(10), int(10), int(self.width() - 20), int(self.height() - 20))
            center_x = int(self.width() / 2)
            center_y = int(self.height() / 2)
            painter.setBrush(QBrush(QColor(30, 41, 59)))
            painter.setPen(QPen(QColor(148, 163, 184), 1))
            painter.drawRect(int(center_x - 70), int(center_y + 60), int(140), int(45))
            painter.drawText(int(center_x - 70), int(center_y + 60), int(140), int(45), Qt.AlignCenter, "Apron")
            for idx, (name, heading) in enumerate(self.runway_headings.get(self.base_code, [("RWY", 0.0)])):
                length = min(self.width(), self.height()) * (0.55 if idx == 0 else 0.42)
                width = 24
                angle = math.radians(heading - 90.0)
                dx = math.cos(angle) * (length / 2)
                dy = math.sin(angle) * (length / 2)
                nx = math.cos(angle + math.pi / 2) * (width / 2)
                ny = math.sin(angle + math.pi / 2) * (width / 2)
                points = [
                    QPoint(int(center_x - dx - nx), int(center_y - dy - ny)),
                    QPoint(int(center_x - dx + nx), int(center_y - dy + ny)),
                    QPoint(int(center_x + dx + nx), int(center_y + dy + ny)),
                    QPoint(int(center_x + dx - nx), int(center_y + dy - ny)),
                ]
                painter.setBrush(QBrush(QColor(51, 65, 85)))
                painter.setPen(QPen(QColor(226, 232, 240), 2))
                painter.drawPolygon(QPolygon(points))
                painter.setPen(QPen(QColor(248, 250, 252), 1, Qt.DashLine))
                painter.drawLine(int(center_x - dx), int(center_y - dy), int(center_x + dx), int(center_y + dy))
                painter.setPen(QPen(QColor(248, 250, 252), 1))
                painter.drawText(int(center_x - 100), int(20 + idx * 22), int(200), int(20), Qt.AlignCenter, f"{self.base_code} {name}")


    class SAAFBaseInfoWidget(QWidget):
        def __init__(self, config: VORAirportConfig, parent: Optional[QWidget] = None) -> None:
            super().__init__(parent)
            self.config = config
            layout = QVBoxLayout(self)
            self.title = QLabel("Base Information")
            self.summary = QTextEdit()
            self.summary.setReadOnly(True)
            layout.addWidget(self.title)
            layout.addWidget(self.summary)
            self.set_base("FAWK")

        def set_base(self, code: str) -> None:
            airport = self.config.get_airport(code)
            station = next((s for s in self.config.vor_stations().values() if s.get("airport") == code), None)
            self.title.setText(f"{code} - {airport.get('name', 'Unknown')}")
            lines = [
                f"Latitude:  {airport.get('latitude', 'N/A')}",
                f"Longitude: {airport.get('longitude', 'N/A')}",
                f"Elevation: {airport.get('elevation', 'N/A')} ft",
            ]
            if station:
                lines.extend([
                    f"VOR: {station.get('name', 'N/A')} ({station.get('ident', '---')})",
                    f"Frequency: {station.get('frequency', 'N/A')} MHz",
                    f"Type: {station.get('type', 'N/A')}",
                    f"Remarks: {station.get('remarks', 'N/A')}",
                ])
            self.summary.setPlainText("\n".join(lines))


    class Terrain3DWidget(QOpenGLWidget):
        def __init__(self, parent: Optional[QWidget] = None) -> None:
            super().__init__(parent)
            self.rot_x = 35.0
            self.rot_y = -35.0
            self.zoom = -180.0
            self.pan_x = 0.0
            self.pan_y = -15.0
            self.last_mouse_pos = None
            self.terrain = self._generate_terrain()
            self.setMinimumSize(420, 320)

        def _generate_terrain(self) -> Any:
            size = 36
            if np is None:
                grid = []
                for y in range(size):
                    row = []
                    for x in range(size):
                        row.append(12.0 * math.sin(x / 4.2) + 15.0 * math.cos(y / 5.1) + 6.0 * math.sin((x + y) / 3.2))
                    grid.append(row)
                return grid
            xs = np.linspace(-3.5, 3.5, size)
            ys = np.linspace(-3.5, 3.5, size)
            grid = np.zeros((size, size), dtype=float)
            for yi, y in enumerate(ys):
                for xi, x in enumerate(xs):
                    grid[yi, xi] = 12.0 * math.sin(x * 1.6) + 15.0 * math.cos(y * 1.2) + 8.0 * math.sin((x + y) * 2.0)
            return grid

        def initializeGL(self) -> None:
            if not OPENGL_AVAILABLE:
                return
            GL.glClearColor(0.04, 0.08, 0.12, 1.0)
            GL.glEnable(GL.GL_DEPTH_TEST)
            GL.glShadeModel(GL.GL_SMOOTH)

        def resizeGL(self, width: int, height: int) -> None:
            if not OPENGL_AVAILABLE:
                return
            GL.glViewport(0, 0, int(width), int(max(height, 1)))
            GL.glMatrixMode(GL.GL_PROJECTION)
            GL.glLoadIdentity()
            aspect = width / max(height, 1)
            GLU.gluPerspective(45.0, aspect, 1.0, 1000.0)
            GL.glMatrixMode(GL.GL_MODELVIEW)

        def paintGL(self) -> None:
            if not OPENGL_AVAILABLE:
                painter = QPainter(self)
                painter.fillRect(self.rect(), QColor(17, 24, 39))
                painter.setPen(QPen(QColor(248, 250, 252), 1))
                painter.drawText(int(10), int(10), int(self.width() - 20), int(self.height() - 20), Qt.AlignCenter, "OpenGL unavailable")
                return
            GL.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT)
            GL.glLoadIdentity()
            GL.glTranslatef(float(self.pan_x), float(self.pan_y), float(self.zoom))
            GL.glRotatef(float(self.rot_x), 1.0, 0.0, 0.0)
            GL.glRotatef(float(self.rot_y), 0.0, 1.0, 0.0)
            size = len(self.terrain)
            scale = 4.0
            GL.glBegin(GL.GL_QUADS)
            for y in range(size - 1):
                for x in range(size - 1):
                    h1 = float(self.terrain[y][x])
                    h2 = float(self.terrain[y][x + 1])
                    h3 = float(self.terrain[y + 1][x + 1])
                    h4 = float(self.terrain[y + 1][x])
                    avg = (h1 + h2 + h3 + h4) / 4.0
                    c = clamp((avg + 30.0) / 60.0, 0.0, 1.0)
                    GL.glColor3f(0.08 + c * 0.30, 0.32 + c * 0.45, 0.12 + c * 0.20)
                    x0 = (x - size / 2) * scale
                    x1 = (x + 1 - size / 2) * scale
                    y0 = (y - size / 2) * scale
                    y1 = (y + 1 - size / 2) * scale
                    GL.glVertex3f(float(x0), float(h1), float(y0))
                    GL.glVertex3f(float(x1), float(h2), float(y0))
                    GL.glVertex3f(float(x1), float(h3), float(y1))
                    GL.glVertex3f(float(x0), float(h4), float(y1))
            GL.glEnd()
            GL.glColor3f(0.9, 0.9, 0.9)
            GL.glBegin(GL.GL_LINES)
            GL.glVertex3f(-80.0, 0.0, 0.0)
            GL.glVertex3f(80.0, 0.0, 0.0)
            GL.glVertex3f(0.0, 0.0, -80.0)
            GL.glVertex3f(0.0, 0.0, 80.0)
            GL.glEnd()

        def mousePressEvent(self, event: Any) -> None:
            self.last_mouse_pos = event.pos()

        def mouseMoveEvent(self, event: Any) -> None:
            if self.last_mouse_pos is None:
                return
            delta = event.pos() - self.last_mouse_pos
            if event.buttons() & 1:
                self.rot_x += delta.y() * 0.6
                self.rot_y += delta.x() * 0.6
            self.last_mouse_pos = event.pos()
            self.update()

        def wheelEvent(self, event: Any) -> None:
            self.zoom += event.angleDelta().y() * 0.02
            self.zoom = clamp(self.zoom, -400.0, -40.0)
            self.update()


    class RadarDisplay(QWidget):
        def __init__(self, tracker: AircraftTracker, parent: Optional[QWidget] = None) -> None:
            super().__init__(parent)
            self.tracker = tracker
            self.range_nm = 100.0
            self.sweep_angle = 0.0
            self.timer = QTimer(self)
            self.timer.timeout.connect(self._advance_sweep)
            self.timer.start(50)
            self.setMinimumSize(420, 420)

        def _advance_sweep(self) -> None:
            self.sweep_angle = (self.sweep_angle + 3.0) % 360.0
            self.update()

        def paintEvent(self, _event: Any) -> None:
            painter = QPainter(self)
            painter.setRenderHint(QPainter.Antialiasing)
            painter.fillRect(self.rect(), QColor(0, 18, 10))
            cx = int(self.width() / 2)
            cy = int(self.height() / 2)
            max_radius = int(min(self.width(), self.height()) / 2 - 20)
            painter.setPen(QPen(QColor(20, 120, 50), 1))
            for fraction in [0.25, 0.5, 0.75, 1.0]:
                r = int(max_radius * fraction)
                painter.drawEllipse(int(cx - r), int(cy - r), int(r * 2), int(r * 2))
            painter.drawLine(int(cx - max_radius), int(cy), int(cx + max_radius), int(cy))
            painter.drawLine(int(cx), int(cy - max_radius), int(cx), int(cy + max_radius))
            angle = math.radians(self.sweep_angle - 90.0)
            sx = cx + math.cos(angle) * max_radius
            sy = cy + math.sin(angle) * max_radius
            painter.setPen(QPen(QColor(120, 255, 120), 2))
            painter.drawLine(int(cx), int(cy), int(sx), int(sy))
            painter.setPen(QPen(QColor(120, 255, 120), 1))
            painter.drawText(int(8), int(8), int(180), int(20), Qt.AlignLeft, f"Range: {self.range_nm:.0f} NM")
            for aircraft in self.tracker.snapshot():
                bearing, distance = self.tracker.relative_position(aircraft.lat, aircraft.lon)
                x, y = polar_to_screen(cx, cy, max_radius, bearing, distance, self.range_nm)
                painter.setBrush(QBrush(QColor(86, 255, 170)))
                painter.setPen(QPen(QColor(86, 255, 170), 2))
                painter.drawEllipse(int(x - 4), int(y - 4), int(8), int(8))
                painter.drawText(int(x + 6), int(y - 10), int(110), int(20), Qt.AlignLeft, aircraft.callsign)
                painter.setPen(QPen(QColor(60, 160, 100), 1))
                if len(aircraft.track_history) >= 2:
                    prev_points = []
                    for _ts, lat, lon in aircraft.track_history[-8:]:
                        brg, dist = self.tracker.relative_position(lat, lon)
                        px, py = polar_to_screen(cx, cy, max_radius, brg, dist, self.range_nm)
                        prev_points.append(QPoint(int(px), int(py)))
                    for p1, p2 in zip(prev_points[:-1], prev_points[1:]):
                        painter.drawLine(int(p1.x()), int(p1.y()), int(p2.x()), int(p2.y()))


    class CDIDisplay(QWidget):
        def __init__(self, parent: Optional[QWidget] = None) -> None:
            super().__init__(parent)
            self.deviation = 0.0
            self.to_from = "TO"
            self.setMinimumSize(280, 280)

        def set_deviation(self, deviation: float) -> None:
            self.deviation = clamp(deviation, -10.0, 10.0)
            self.update()

        def paintEvent(self, _event: Any) -> None:
            painter = QPainter(self)
            painter.setRenderHint(QPainter.Antialiasing)
            painter.fillRect(self.rect(), QColor(20, 20, 30))
            cx = int(self.width() / 2)
            cy = int(self.height() / 2)
            radius = int(min(self.width(), self.height()) / 2 - 18)
            painter.setPen(QPen(QColor(220, 220, 220), 2))
            painter.drawEllipse(int(cx - radius), int(cy - radius), int(radius * 2), int(radius * 2))
            painter.drawLine(int(cx - radius), int(cy), int(cx + radius), int(cy))
            painter.drawLine(int(cx), int(cy - radius), int(cx), int(cy + radius))
            dot_spacing = radius / 5.0
            painter.setBrush(QBrush(QColor(220, 220, 220)))
            for i in range(-2, 3):
                if i == 0:
                    continue
                x = cx + i * dot_spacing
                painter.drawEllipse(int(x - 4), int(cy - 4), int(8), int(8))
            needle_offset = (self.deviation / 10.0) * (dot_spacing * 2.0)
            painter.setPen(QPen(QColor(248, 113, 113), 4))
            painter.drawLine(int(cx + needle_offset), int(cy - radius + 24), int(cx + needle_offset), int(cy + radius - 24))
            painter.setPen(QPen(QColor(226, 232, 240), 1))
            painter.drawText(int(cx - 40), int(12), int(80), int(20), Qt.AlignCenter, f"{self.to_from}")


    class ApproachGuidanceDisplay(QWidget):
        def __init__(self, parent: Optional[QWidget] = None) -> None:
            super().__init__(parent)
            self.localizer_dev = 0.0
            self.glide_dev = 0.0
            self.setMinimumSize(340, 280)

        def set_deviations(self, localizer: float, glide: float) -> None:
            self.localizer_dev = clamp(localizer, -2.5, 2.5)
            self.glide_dev = clamp(glide, -2.5, 2.5)
            self.update()

        def paintEvent(self, _event: Any) -> None:
            painter = QPainter(self)
            painter.setRenderHint(QPainter.Antialiasing)
            painter.fillRect(self.rect(), QColor(16, 24, 39))
            width = self.width()
            height = self.height()
            cx = int(width * 0.35)
            cy = int(height * 0.60)
            radius = int(min(width, height) * 0.22)
            painter.setPen(QPen(QColor(226, 232, 240), 2))
            painter.drawRect(int(cx - radius), int(cy - radius), int(radius * 2), int(radius * 2))
            painter.drawLine(int(cx - radius), int(cy), int(cx + radius), int(cy))
            painter.drawLine(int(cx), int(cy - radius), int(cx), int(cy + radius))
            for i in range(-2, 3):
                if i == 0:
                    continue
                x = cx + i * (radius / 2.5)
                y = cy + i * (radius / 2.5)
                painter.drawEllipse(int(x - 3), int(cy - 3), int(6), int(6))
                painter.drawEllipse(int(cx - 3), int(y - 3), int(6), int(6))
            loc_x = cx + (self.localizer_dev / 2.5) * radius
            gs_y = cy - (self.glide_dev / 2.5) * radius
            painter.setPen(QPen(QColor(248, 113, 113), 4))
            painter.drawLine(int(loc_x), int(cy - radius + 12), int(loc_x), int(cy + radius - 12))
            painter.drawLine(int(cx - radius + 12), int(gs_y), int(cx + radius - 12), int(gs_y))
            painter.setPen(QPen(QColor(226, 232, 240), 1))
            panel_x = int(width * 0.70)
            panel_y = int(height * 0.25)
            panel_w = int(width * 0.20)
            panel_h = int(height * 0.50)
            painter.drawRect(int(panel_x), int(panel_y), int(panel_w), int(panel_h))
            painter.drawText(int(panel_x), int(panel_y - 24), int(panel_w), int(20), Qt.AlignCenter, "GS")
            center_y = int(panel_y + panel_h / 2)
            painter.drawLine(int(panel_x + 8), int(center_y), int(panel_x + panel_w - 8), int(center_y))
            gs_indicator_y = center_y - (self.glide_dev / 2.5) * (panel_h / 2 - 12)
            painter.setBrush(QBrush(QColor(96, 165, 250)))
            painter.drawRect(int(panel_x + 10), int(gs_indicator_y - 6), int(panel_w - 20), int(12))


    class VORAirportMonitorApp(QMainWindow):
        def __init__(self) -> None:
            super().__init__()
            self.setWindowTitle("VOR / ASRACS / SAAF Airport Monitoring System v5.3")
            self.resize(1440, 920)
            self.config = VORAirportConfig()
            default_station = self.config.get_station(self.config.station_keys()[0]) if self.config.station_keys() else {}
            self.processor = VORDataProcessor()
            self.connection = VORConnectionHandler(self.config)
            self.tracker = AircraftTracker(default_station.get("latitude", -25.5967), default_station.get("longitude", 28.2394))
            self.simulation = SimulationEngine(self.tracker)
            self.asracs_engine = ASRACSSimEngine()
            self.glide_detector = GlideSlopeDetector()
            self.slope_analyzer = SurfaceSlopeAnalyzer()
            self.lda_parser = LDAFileParser()
            self.acquisition_thread: Optional[DataAcquisitionThread] = None
            self.current_station_key = self.config.station_keys()[0] if self.config.station_keys() else ""
            self._build_ui()
            self._bind_timers()
            self._populate_vor_combo()
            self._refresh_all_views()

        def _build_ui(self) -> None:
            self.tabs = QTabWidget()
            self.setCentralWidget(self.tabs)
            self.tabs.addTab(self._tab_vor(), "VOR Monitor")
            self.tabs.addTab(self._tab_radar(), "Aircraft Tracking")
            self.tabs.addTab(self._tab_approach(), "Approach Guidance")
            self.tabs.addTab(self._tab_terrain(), "Terrain 3D")
            self.tabs.addTab(self._tab_asracs(), "ASRACS Surface")
            self.tabs.addTab(self._tab_saaf(), "SAAF Bases")
            self.tabs.addTab(self._tab_connection(), "Connection")
            self.tabs.addTab(self._tab_diagnostics(), "Diagnostics")

        def _bind_timers(self) -> None:
            self.sim_timer = QTimer(self)
            self.sim_timer.timeout.connect(self._tick_simulation)
            self.sim_timer.start(1000)
            self.ui_timer = QTimer(self)
            self.ui_timer.timeout.connect(self._refresh_diagnostics)
            self.ui_timer.start(2000)

        def _tab_vor(self) -> QWidget:
            widget = QWidget()
            layout = QVBoxLayout(widget)
            top = QHBoxLayout()
            self.vor_combo = QComboBox()
            self.vor_combo.currentIndexChanged.connect(self._on_vor_index_changed)
            self.vor_freq_label = QLabel("Frequency: --")
            self.vor_ident_label = QLabel("Ident: ---")
            self.signal_label = QLabel("Signal: --")
            top.addWidget(QLabel("Station:"))
            top.addWidget(self.vor_combo)
            top.addWidget(self.vor_freq_label)
            top.addWidget(self.vor_ident_label)
            top.addWidget(self.signal_label)
            top.addStretch()
            layout.addLayout(top)
            center = QHBoxLayout()
            self.cdi = CDIDisplay()
            center.addWidget(self.cdi)
            if CHART_AVAILABLE:
                self.signal_series = QLineSeries()
                self.chart = QChart()
                self.chart.addSeries(self.signal_series)
                self.chart.createDefaultAxes()
                self.chart.legend().hide()
                self.chart.setTitle("Signal Strength")
                self.chart_view = QChartView(self.chart)
                center.addWidget(self.chart_view)
            else:
                self.chart_view = QTextEdit()
                self.chart_view.setReadOnly(True)
                center.addWidget(self.chart_view)
            layout.addLayout(center)
            return widget

        def _tab_radar(self) -> QWidget:
            widget = QWidget()
            layout = QHBoxLayout(widget)
            self.radar = RadarDisplay(self.tracker)
            layout.addWidget(self.radar, 2)
            right = QVBoxLayout()
            self.aircraft_table = QTableWidget(0, 6)
            self.aircraft_table.setHorizontalHeaderLabels(["Callsign", "Lat", "Lon", "Alt", "Spd", "Hdg"])
            self.aircraft_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
            right.addWidget(QLabel("Tracked Aircraft"))
            right.addWidget(self.aircraft_table)
            layout.addLayout(right, 1)
            return widget

        def _tab_approach(self) -> QWidget:
            widget = QWidget()
            layout = QVBoxLayout(widget)
            top = QHBoxLayout()
            self.runway_combo = QComboBox()
            top.addWidget(QLabel("Runway:"))
            top.addWidget(self.runway_combo)
            top.addStretch()
            layout.addLayout(top)
            mid = QHBoxLayout()
            self.approach_display = ApproachGuidanceDisplay()
            self.approach_summary = QTextEdit()
            self.approach_summary.setReadOnly(True)
            mid.addWidget(self.approach_display, 1)
            mid.addWidget(self.approach_summary, 1)
            layout.addLayout(mid)
            return widget

        def _tab_terrain(self) -> QWidget:
            widget = QWidget()
            layout = QVBoxLayout(widget)
            self.terrain_widget = Terrain3DWidget()
            layout.addWidget(self.terrain_widget)
            return widget

        def _tab_asracs(self) -> QWidget:
            widget = QWidget()
            layout = QHBoxLayout(widget)
            self.asracs_display = ASRACSDisplay(self.asracs_engine)
            self.asracs_alerts = ASRACSAlertPanel()
            layout.addWidget(self.asracs_display, 2)
            layout.addWidget(self.asracs_alerts, 1)
            return widget

        def _tab_saaf(self) -> QWidget:
            widget = QWidget()
            layout = QHBoxLayout(widget)
            self.saaf_overview = SAAFOverviewMap(self.config)
            right = QVBoxLayout()
            self.base_combo = QComboBox()
            self.base_combo.currentTextChanged.connect(self._on_base_changed)
            for code in self.config.airports().keys():
                self.base_combo.addItem(code)
            self.base_map = SAAFBaseMapWidget(self.config)
            self.base_info = SAAFBaseInfoWidget(self.config)
            right.addWidget(QLabel("Base/Airport"))
            right.addWidget(self.base_combo)
            right.addWidget(self.base_map)
            right.addWidget(self.base_info)
            layout.addWidget(self.saaf_overview, 2)
            layout.addLayout(right, 1)
            return widget

        def _tab_connection(self) -> QWidget:
            widget = QWidget()
            layout = QVBoxLayout(widget)
            tcp_group = QGroupBox("TCP Connection")
            tcp_layout = QGridLayout(tcp_group)
            self.host_edit = QLineEdit("127.0.0.1")
            self.port_edit = QLineEdit("5000")
            self.tcp_connect_btn = QPushButton("Connect TCP")
            self.tcp_connect_btn.clicked.connect(self._connect_tcp)
            self.mock_btn = QPushButton("Start Mock Server")
            self.mock_btn.clicked.connect(self._toggle_mock_server)
            tcp_layout.addWidget(QLabel("Host"), 0, 0)
            tcp_layout.addWidget(self.host_edit, 0, 1)
            tcp_layout.addWidget(QLabel("Port"), 1, 0)
            tcp_layout.addWidget(self.port_edit, 1, 1)
            tcp_layout.addWidget(self.tcp_connect_btn, 2, 0)
            tcp_layout.addWidget(self.mock_btn, 2, 1)
            serial_group = QGroupBox("Serial Connection")
            serial_layout = QGridLayout(serial_group)
            self.serial_combo = QComboBox()
            for port in self.connection.available_serial_ports():
                self.serial_combo.addItem(port)
            self.serial_connect_btn = QPushButton("Connect Serial")
            self.serial_connect_btn.clicked.connect(self._connect_serial)
            self.disconnect_btn = QPushButton("Disconnect")
            self.disconnect_btn.clicked.connect(self._disconnect_source)
            serial_layout.addWidget(QLabel("Port"), 0, 0)
            serial_layout.addWidget(self.serial_combo, 0, 1)
            serial_layout.addWidget(self.serial_connect_btn, 1, 0)
            serial_layout.addWidget(self.disconnect_btn, 1, 1)
            self.conn_status = QLabel("Status: disconnected")
            layout.addWidget(tcp_group)
            layout.addWidget(serial_group)
            layout.addWidget(self.conn_status)
            layout.addStretch()
            return widget

        def _tab_diagnostics(self) -> QWidget:
            widget = QWidget()
            layout = QVBoxLayout(widget)
            self.diag_text = QTextEdit()
            self.diag_text.setReadOnly(True)
            layout.addWidget(self.diag_text)
            return widget

        def _populate_vor_combo(self) -> None:
            self.vor_combo.clear()
            for key, station in self.config.vor_stations().items():
                self.vor_combo.addItem(f"{station.get('ident', '---')} - {station.get('name', key)}", key)
            if self.config.station_keys():
                self.vor_combo.setCurrentIndex(0)

        def _on_vor_index_changed(self, _index: int) -> None:
            key = self.vor_combo.currentData() if hasattr(self, "vor_combo") else None
            if not key:
                return
            self.current_station_key = key
            station = self.config.get_station(key)
            self.tracker.set_reference(station.get("latitude", -25.5967), station.get("longitude", 28.2394))
            self.vor_freq_label.setText(f"Frequency: {station.get('frequency', 0.0):.2f} MHz")
            self.vor_ident_label.setText(f"Ident: {station.get('ident', '---')}")
            if hasattr(self, "runway_combo"):
                self.runway_combo.clear()
                for ils in station.get("ils_runways", []):
                    self.runway_combo.addItem(f"RWY {ils.get('runway')} - {ils.get('frequency')} MHz", ils)
                if not station.get("ils_runways"):
                    self.runway_combo.addItem("No ILS runway", {})
            if hasattr(self, "saaf_overview"):
                self.saaf_overview.set_selected_code(station.get("airport", ""))
            if hasattr(self, "base_combo"):
                codes = [self.base_combo.itemText(i) for i in range(self.base_combo.count())]
                airport_code = station.get("airport", "")
                if airport_code in codes:
                    self.base_combo.setCurrentText(airport_code)
            if hasattr(self, "radar"):
                self.radar.update()
            if hasattr(self, "approach_display"):
                self._refresh_approach()

        def _on_base_changed(self, code: str) -> None:
            if hasattr(self, "saaf_overview"):
                self.saaf_overview.set_selected_code(code)
            if hasattr(self, "base_map"):
                self.base_map.set_base(code)
            if hasattr(self, "base_info"):
                self.base_info.set_base(code)

        def _connect_tcp(self) -> None:
            host = self.host_edit.text().strip() or "127.0.0.1"
            port = int(self.port_edit.text().strip() or "5000")
            if self.connection.connect_tcp(host, port):
                self._start_acquisition()
                self.conn_status.setText(f"Status: TCP connected to {host}:{port}")
            else:
                self.conn_status.setText("Status: TCP connection failed")

        def _connect_serial(self) -> None:
            port = self.serial_combo.currentText().strip()
            if port and self.connection.connect_serial(port):
                self._start_acquisition()
                self.conn_status.setText(f"Status: Serial connected to {port}")
            else:
                self.conn_status.setText("Status: Serial connection failed")

        def _disconnect_source(self) -> None:
            if self.acquisition_thread and self.acquisition_thread.isRunning():
                self.acquisition_thread.stop()
            self.connection.disconnect()
            self.conn_status.setText("Status: disconnected")

        def _toggle_mock_server(self) -> None:
            if self.connection.mock_server.is_running():
                self.connection.stop_mock_server()
                self.mock_btn.setText("Start Mock Server")
                self.conn_status.setText("Status: mock server stopped")
            else:
                self.connection.start_mock_server()
                self.mock_btn.setText("Stop Mock Server")
                self.conn_status.setText(f"Status: mock server listening on {self.connection.mock_server.host}:{self.connection.mock_server.port}")

        def _start_acquisition(self) -> None:
            if self.acquisition_thread and self.acquisition_thread.isRunning():
                self.acquisition_thread.stop()
            self.acquisition_thread = DataAcquisitionThread(self.connection, self.processor)
            self.acquisition_thread.data_received.connect(self._on_vor_data)
            self.acquisition_thread.error_occurred.connect(lambda text: self.diag_text.append(f"ERROR: {text}"))
            self.acquisition_thread.start()

        def _on_vor_data(self, data: VORData) -> None:
            self.signal_label.setText(f"Signal: {data.signal_strength:.1f}%")
            self.cdi.set_deviation(data.deviation)
            if CHART_AVAILABLE:
                values = self.processor.signal_series(60)
                self.signal_series.clear()
                base = values[0][0] if values else time.time()
                for ts, strength in values:
                    self.signal_series.append(ts - base, strength)
                self.chart.createDefaultAxes()
            else:
                self.chart_view.setPlainText(f"Bearing {data.bearing:.1f}\nDeviation {data.deviation:.2f}\nSignal {data.signal_strength:.1f}")
            self._refresh_approach(with_live_data=data)

        def _tick_simulation(self) -> None:
            self.simulation.step(1.0)
            self.asracs_engine.step(1.0)
            self._refresh_aircraft_table()
            self.asracs_alerts.refresh(self.asracs_engine.snapshot()[1])
            self._refresh_approach()
            self.radar.update()

        def _refresh_aircraft_table(self) -> None:
            aircraft = self.tracker.snapshot()
            self.aircraft_table.setRowCount(len(aircraft))
            for row, ac in enumerate(aircraft):
                items = [
                    ac.callsign,
                    f"{ac.lat:.4f}",
                    f"{ac.lon:.4f}",
                    f"{ac.altitude:.0f}",
                    f"{ac.speed:.0f}",
                    f"{ac.heading:.0f}",
                ]
                for col, value in enumerate(items):
                    self.aircraft_table.setItem(row, col, QTableWidgetItem(value))

        def _refresh_approach(self, with_live_data: Optional[VORData] = None) -> None:
            station = self.config.get_station(self.current_station_key)
            ils = self.runway_combo.currentData() if hasattr(self, "runway_combo") else {}
            airport = self.config.get_airport(station.get("airport", ""))
            distance_nm = 8.0
            altitude_ft = self.glide_detector.required_altitude(distance_nm, airport.get("elevation", 0.0)) + random.uniform(-180, 180)
            speed = 145.0
            if with_live_data is not None:
                distance_nm = 6.0 + abs(with_live_data.deviation) * 0.8
            analysis = self.glide_detector.analyze(distance_nm, altitude_ft, speed, airport.get("elevation", 0.0))
            localizer_dev = (with_live_data.deviation / 4.0) if with_live_data else random.uniform(-1.0, 1.0)
            glide_dev = analysis["deviation_dots"]
            self.approach_display.set_deviations(localizer_dev, glide_dev)
            profile = self.slope_analyzer.synthesize_approach_profile(airport.get("elevation", 0.0), 10.0)
            slope = self.slope_analyzer.analyze_profile(profile)
            lines = [
                f"Airport: {airport.get('name', 'Unknown')} ({station.get('airport', '---')})",
                f"Runway: {ils.get('runway', 'N/A')}",
                f"Glide slope frequency: {ils.get('frequency', 'N/A')} MHz",
                f"Required altitude: {analysis['required_altitude_ft']:.0f} ft",
                f"Current angle: {analysis['current_angle_deg']:.2f}°",
                f"Deviation: {analysis['deviation_ft']:.0f} ft / {analysis['deviation_dots']:.2f} dots",
                f"Descent rate: {analysis['required_descent_rate_fpm']:.0f} fpm",
                f"Terrain max slope: {slope['max_slope_percent']:.2f}%",
                f"Terrain average slope: {slope['avg_slope_percent']:.2f}%",
            ]
            self.approach_summary.setPlainText("\n".join(lines))

        def _refresh_diagnostics(self) -> None:
            stats = self.processor.stats()
            targets, alerts = self.asracs_engine.snapshot()
            lines = [
                f"Time: {time.strftime('%Y-%m-%d %H:%M:%S')}",
                f"PyQt available: {PYQT_AVAILABLE}",
                f"OpenGL available: {OPENGL_AVAILABLE}",
                f"Connection mode: {self.connection.mode}",
                f"Mock server running: {self.connection.mock_server.is_running()}",
                f"Buffered VOR frames: {int(stats['count'])}",
                f"Average signal: {stats['avg_signal']:.2f}",
                f"Average |deviation|: {stats['avg_abs_deviation']:.2f}",
                f"Tracked aircraft: {len(self.tracker.snapshot())}",
                f"Surface targets: {len(targets)}",
                f"Active incursions: {len(alerts)}",
            ]
            self.diag_text.setPlainText("\n".join(lines))

        def _refresh_all_views(self) -> None:
            self._refresh_aircraft_table()
            self._refresh_approach()
            self._refresh_diagnostics()
            self.asracs_alerts.refresh(self.asracs_engine.snapshot()[1])
            self._on_base_changed(self.base_combo.currentText())

        def closeEvent(self, event: Any) -> None:
            try:
                if self.acquisition_thread and self.acquisition_thread.isRunning():
                    self.acquisition_thread.stop()
                self.connection.disconnect()
                self.connection.stop_mock_server()
            finally:
                event.accept()

else:

    class ASRACSDisplay(QWidget):
        def __init__(self, *args: Any, **kwargs: Any) -> None:
            super().__init__(*args, **kwargs)

    class ASRACSAlertPanel(QWidget):
        def refresh(self, _alerts: Sequence[IncursionAlert]) -> None:
            pass

    class SAAFOverviewMap(QWidget):
        def set_selected_code(self, _code: str) -> None:
            pass

    class SAAFBaseMapWidget(QWidget):
        def set_base(self, _code: str) -> None:
            pass

    class SAAFBaseInfoWidget(QWidget):
        def set_base(self, _code: str) -> None:
            pass

    class Terrain3DWidget(QWidget):
        pass

    class RadarDisplay(QWidget):
        pass

    class CDIDisplay(QWidget):
        def set_deviation(self, _deviation: float) -> None:
            pass

    class ApproachGuidanceDisplay(QWidget):
        def set_deviations(self, _localizer: float, _glide: float) -> None:
            pass

    class VORAirportMonitorApp(QMainWindow):
        def __init__(self) -> None:
            super().__init__()
            raise RuntimeError("PyQt5 is required to launch the GUI application")


def _self_test() -> None:
    tests: List[Tuple[str, Any]] = []

    def run_test(name: str, fn: Any) -> None:
        try:
            fn()
            print(f"PASS: {name}")
            tests.append((name, True))
        except Exception as exc:
            print(f"FAIL: {name} - {exc}")
            traceback.print_exc()
            tests.append((name, False))

    def test_config() -> None:
        config = VORAirportConfig()
        assert len(config.vor_stations()) == 13, f"Expected 13 VOR stations, got {len(config.vor_stations())}"
        assert len(config.airports()) >= 13
        assert config.get_station("FAWK_VOR").get("ident") == "WKV"

    def test_glide_slope() -> None:
        gs = GlideSlopeDetector()
        required = gs.required_altitude(5.0, 500.0)
        assert 2000.0 < required < 2200.0, required
        angle = gs.approach_angle(5.0, required, 500.0)
        assert abs(angle - 3.0) < 0.05, angle
        analysis = gs.analyze(5.0, required + 100.0, 140.0, 500.0)
        assert analysis["deviation_ft"] > 90.0

    def test_surface_slope() -> None:
        analyzer = SurfaceSlopeAnalyzer()
        profile = [(0.0, 500.0), (1.0, 650.0), (2.0, 800.0)]
        result = analyzer.analyze_profile(profile)
        assert result["segment_count"] == 2
        assert result["max_slope_percent"] > 2.0
        assert analyzer.required_descent_rate(140.0, 3.0) > 700.0

    def test_processor() -> None:
        processor = VORDataProcessor()
        parsed = processor.parse_line("FREQ=114.90,IDENT=JHB,BRG=123.4,DEV=-1.25,SIG=88.0")
        assert parsed is not None
        assert parsed.ident == "JHB"
        assert abs(parsed.bearing - 123.4) < 0.01
        assert len(processor.buffer) == 1

    def test_tracker() -> None:
        tracker = AircraftTracker(-25.5967, 28.2394)
        ac = tracker.add_or_update_from_vor("TEST1", 90.0, 10.0, 4500.0, 150.0)
        assert ac.callsign == "TEST1"
        bearing, distance = tracker.relative_position(ac.lat, ac.lon)
        assert 80.0 < bearing < 100.0
        assert 9.5 < distance < 10.5

    def test_lda_parser() -> None:
        parser = LDAFileParser()
        sample = (
            b"THALES\x00RUNWAY=03L\nIDENT=IJB\nWAVEFORM=CSB_90HZ\n"
            b"FREQ=110.30\nNOMINAL_MOD=40.0\nALARM_HIGH=48.0\n"
        )
        result = parser.parse_bytes(sample, source_name="dummy.lda")
        assert result["vendor"] == "Thales"
        assert result["ident"] == "IJB"
        assert 110.30 in result["frequencies"]
        assert result["nominal_values"]
        assert result["alarm_limits"]

    run_test("Config load", test_config)
    run_test("Glide slope", test_glide_slope)
    run_test("Surface slope", test_surface_slope)
    run_test("VOR parser", test_processor)
    run_test("Aircraft tracker", test_tracker)
    run_test("LDA parser", test_lda_parser)

    failed = [name for name, ok in tests if not ok]
    print(f"Summary: {len(tests) - len(failed)}/{len(tests)} passed")
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    if "--self-test" in sys.argv:
        _self_test()
    else:
        if not PYQT_AVAILABLE:
            print("PyQt5 is required to launch the GUI application.", file=sys.stderr)
            sys.exit(2)
        app = QApplication(sys.argv)
        app.setStyle("Fusion")
        palette = QPalette()
        palette.setColor(QPalette.Window, QColor(30, 30, 46))
        palette.setColor(QPalette.WindowText, QColor(205, 214, 244))
        palette.setColor(QPalette.Base, QColor(24, 24, 37))
        palette.setColor(QPalette.AlternateBase, QColor(30, 30, 46))
        palette.setColor(QPalette.Text, QColor(205, 214, 244))
        palette.setColor(QPalette.Button, QColor(49, 50, 68))
        palette.setColor(QPalette.ButtonText, QColor(205, 214, 244))
        palette.setColor(QPalette.Highlight, QColor(137, 180, 250))
        palette.setColor(QPalette.HighlightedText, QColor(30, 30, 46))
        app.setPalette(palette)
        window = VORAirportMonitorApp()
        window.show()
        sys.exit(app.exec_())
