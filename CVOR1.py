#!/usr/bin/env python3
"""
VOR / ASRACS / SAAF Airport Monitoring System  v5.3
=====================================================
v5.3 Enhancements:
  - LDA file parser (Thales/Rohde & Schwarz ILS config)
  - ILS waveform extraction from LDA data
  - Simulation mode with imported LDA parameters

v5.2 Enhancements:
  - SAAF VOR / ILS metadata for all 10 monitored bases
  - Glide slope detection for approach guidance
  - Surface slope analysis for terrain-aware approaches

v5.1 Fixes:
  - FIXED  AttributeError: 'VORAirportMonitorApp' has no attribute 'radar'
  Fix 1:   _on_vor_index_changed() guards every cross-tab widget with hasattr()
  Fix 2:   _populate_vor_combo() call removed from _tab_vor()
  Fix 3:   _populate_vor_combo() called at end of _build_ui() after ALL tabs built

PyQt5 Type-Safety:
  - ALL painter.draw*() calls use int() for every coordinate argument
  - Eliminates TypeError from mixed int/float in drawLine/drawEllipse/etc.

Dependencies:
  pip install PyQt5 PyQtChart PyOpenGL PyOpenGL_accelerate pyserial numpy PyYAML
"""

import sys
import os
import logging
import csv
import socket
import yaml
import math
import random
import time
import struct
import threading
from pathlib import Path
from datetime import datetime
from typing import List, Dict, Optional, Tuple, Any
from collections import deque
from threading import Lock, Thread
from dataclasses import dataclass, field

try:
    import numpy as np
    HAS_NUMPY = True
except ImportError:
    HAS_NUMPY = False
    np = None

try:
    import serial
    import serial.tools.list_ports
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QTabWidget, QAction, QDialog, QLabel, QLineEdit, QPushButton,
    QGridLayout, QMessageBox, QTableWidget, QTableWidgetItem, QHeaderView,
    QComboBox, QGroupBox, QFileDialog, QTextEdit, QToolBar,
    QSplitter, QScrollArea, QListWidget, QListWidgetItem,
    QProgressBar, QCheckBox, QSpinBox, QDoubleSpinBox, QStatusBar,
    QSizePolicy, QFrame, QSlider,
)
try:
    from PyQt5.QtWidgets import QOpenGLWidget
    HAS_OPENGL_WIDGET = True
except ImportError:
    HAS_OPENGL_WIDGET = False
    QOpenGLWidget = QWidget

from PyQt5.QtCore import (
    Qt, QTimer, pyqtSignal, QThread, QPoint, QRectF, QPointF,
    QRect, QSize,
)
from PyQt5.QtGui import (
    QColor, QFont, QPainter, QPen, QBrush, QPolygon, QPalette,
    QPolygonF, QIcon, QLinearGradient, QRadialGradient,
    QPainterPath,
)

try:
    from PyQt5.QtChart import QChart, QChartView, QLineSeries, QValueAxis
    HAS_CHARTS = True
except ImportError:
    HAS_CHARTS = False

try:
    from OpenGL.GL import *
    from OpenGL.GLU import *
    HAS_OPENGL = True
except ImportError:
    HAS_OPENGL = False

# ─────────────────────────────────────────────────────────────────────────────
# Logging
# ─────────────────────────────────────────────────────────────────────────────
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('vor_monitor.log'),
        logging.StreamHandler(),
    ]
)
logger = logging.getLogger('CVOR1')

# ─────────────────────────────────────────────────────────────────────────────
# Static Data
# ─────────────────────────────────────────────────────────────────────────────

SAAF_BASES: Dict[str, Dict] = {
    'FAWK': {
        'name': 'Waterkloof', 'lat': -25.8300, 'lon': 28.2228,
        'vor_ident': 'WKV', 'vor_freq': 116.90,
        'ils_available': True, 'ils_runway': '01', 'runway': '01/19',
        'elevation': 4898, 'squadron': '21 Sqn (C-130)', 'city': 'Pretoria',
    },
    'FALM': {
        'name': 'Makhado', 'lat': -23.1594, 'lon': 29.6964,
        'vor_ident': 'LTV', 'vor_freq': 115.00,
        'ils_available': True, 'ils_runway': '10/28', 'runway': '10/28',
        'elevation': 3116, 'squadron': '2 Sqn (Gripen)', 'city': 'Makhado',
    },
    'FAHS': {
        'name': 'Hoedspruit', 'lat': -24.3686, 'lon': 31.0497,
        'vor_ident': 'HSV', 'vor_freq': 114.00,
        'ils_available': True, 'ils_runway': '09', 'runway': '09/27',
        'elevation': 1743, 'squadron': '17 Sqn (Rooivalk)', 'city': 'Hoedspruit',
    },
    'FALW': {
        'name': 'Langebaanweg', 'lat': -32.9689, 'lon': 18.1603,
        'vor_ident': 'LWV', 'vor_freq': 117.00,
        'ils_available': False, 'ils_runway': None, 'runway': '01/19',
        'elevation': 108, 'squadron': '41 Sqn (PC-7)', 'city': 'Langebaan',
    },
    'FAOB': {
        'name': 'Overberg', 'lat': -34.5547, 'lon': 20.4508,
        'vor_ident': 'OBV', 'vor_freq': 115.40,
        'ils_available': True, 'ils_runway': '35', 'runway': '17/35',
        'elevation': 246, 'squadron': 'TFDC (UAV)', 'city': 'Bredasdorp',
    },
    'FASK': {
        'name': 'Swartkop', 'lat': -25.8097, 'lon': 28.1644,
        'vor_ident': 'WKV', 'vor_freq': 116.90,
        'ils_available': False, 'ils_runway': None, 'runway': '02/20',
        'elevation': 4724, 'squadron': 'Museum', 'city': 'Pretoria',
    },
    'FABL': {
        'name': 'Bloemspruit', 'lat': -29.0922, 'lon': 26.3024,
        'vor_ident': 'BLV', 'vor_freq': 114.10,
        'ils_available': True, 'ils_runway': '20', 'runway': '02/20',
        'elevation': 4458, 'squadron': '28 Sqn det', 'city': 'Bloemfontein',
    },
    'FAYP': {
        'name': 'Ysterplaat', 'lat': -33.9006, 'lon': 18.4983,
        'vor_ident': 'CTV', 'vor_freq': 115.70,
        'ils_available': False, 'ils_runway': None, 'runway': '02/20',
        'elevation': 24, 'squadron': '35 Sqn (C-47)', 'city': 'Cape Town',
    },
    'FADN': {
        'name': 'Durban', 'lat': -29.9700, 'lon': 30.9500,
        'vor_ident': 'DNV', 'vor_freq': 112.50,
        'ils_available': True, 'ils_runway': '06', 'runway': '06/24',
        'elevation': 29, 'squadron': '35 Sqn det', 'city': 'Durban',
    },
    'FAPE': {
        'name': 'Port Elizabeth', 'lat': -33.9849, 'lon': 25.6173,
        'vor_ident': 'PEV', 'vor_freq': 113.40,
        'ils_available': True, 'ils_runway': '08', 'runway': '08/26',
        'elevation': 226, 'squadron': 'Liaison Flt', 'city': 'Port Elizabeth',
    },
}

VOR_STATIONS: Dict[str, Dict] = {
    'JNB':  {'name': 'OR Tambo',       'ident': 'JHB', 'freq': 114.90, 'lat': -26.1392, 'lon': 28.2460, 'type': 'VOR/DME', 'vor_available': True, 'ils_available': True,  'ils_runway': '03L', 'glide_slope': 3.0},
    'CPT':  {'name': 'Cape Town',       'ident': 'CTV', 'freq': 115.70, 'lat': -33.9649, 'lon': 18.6017, 'type': 'VOR/DME', 'vor_available': True, 'ils_available': True,  'ils_runway': '02',  'glide_slope': 3.0},
    'DUR':  {'name': 'Durban Intl',     'ident': 'DNV', 'freq': 112.50, 'lat': -29.6144, 'lon': 31.1197, 'type': 'VOR/DME', 'vor_available': True, 'ils_available': True,  'ils_runway': '06',  'glide_slope': 3.0},
    'FAWK': {'name': 'Waterkloof',      'ident': 'WKV', 'freq': 116.90, 'lat': -25.8300, 'lon': 28.2228, 'type': 'VOR',     'vor_available': True, 'ils_available': True,  'ils_runway': '01',  'glide_slope': 3.0},
    'FALM': {'name': 'Makhado',         'ident': 'LTV', 'freq': 115.00, 'lat': -23.1594, 'lon': 29.6964, 'type': 'VOR',     'vor_available': True, 'ils_available': True,  'ils_runway': '10',  'glide_slope': 3.0},
    'FAHS': {'name': 'Hoedspruit',      'ident': 'HSV', 'freq': 114.00, 'lat': -24.3686, 'lon': 31.0497, 'type': 'VOR',     'vor_available': True, 'ils_available': True,  'ils_runway': '09',  'glide_slope': 3.0},
    'FALW': {'name': 'Langebaanweg',    'ident': 'LWV', 'freq': 117.00, 'lat': -32.9689, 'lon': 18.1603, 'type': 'VOR',     'vor_available': True, 'ils_available': False, 'ils_runway': None,  'glide_slope': 3.0},
    'FAOB': {'name': 'Overberg',        'ident': 'OBV', 'freq': 115.40, 'lat': -34.5547, 'lon': 20.4508, 'type': 'VOR',     'vor_available': True, 'ils_available': True,  'ils_runway': '35',  'glide_slope': 3.0},
    'FABL': {'name': 'Bloemspruit',     'ident': 'BLV', 'freq': 114.10, 'lat': -29.0922, 'lon': 26.3024, 'type': 'VOR',     'vor_available': True, 'ils_available': True,  'ils_runway': '20',  'glide_slope': 3.0},
    'FADN': {'name': 'Durban SAAF',     'ident': 'DNV', 'freq': 112.50, 'lat': -29.9700, 'lon': 30.9500, 'type': 'VOR',     'vor_available': True, 'ils_available': True,  'ils_runway': '06',  'glide_slope': 3.0},
    'FAPE': {'name': 'Port Elizabeth',  'ident': 'PEV', 'freq': 113.40, 'lat': -33.9849, 'lon': 25.6173, 'type': 'VOR',     'vor_available': True, 'ils_available': True,  'ils_runway': '08',  'glide_slope': 3.0},
    'FASK': {'name': 'Swartkop',        'ident': 'WKV', 'freq': 116.90, 'lat': -25.8097, 'lon': 28.1644, 'type': 'VOR',     'vor_available': True, 'ils_available': False, 'ils_runway': None,  'glide_slope': 3.0},
    'FAYP': {'name': 'Ysterplaat',      'ident': 'CTV', 'freq': 115.70, 'lat': -33.9006, 'lon': 18.4983, 'type': 'VOR',     'vor_available': True, 'ils_available': False, 'ils_runway': None,  'glide_slope': 3.0},
}


# ─────────────────────────────────────────────────────────────────────────────
# Data Classes
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class VORData:
    """Snapshot of a single VOR measurement."""
    station_id: str = ''
    frequency: float = 0.0
    bearing: float = 0.0
    deviation: float = 0.0
    signal_strength: float = 0.0
    timestamp: float = field(default_factory=time.time)
    raw: str = ''


@dataclass
class AircraftData:
    """Live aircraft position and derived track."""
    callsign: str = 'UNKN'
    lat: float = 0.0
    lon: float = 0.0
    altitude: int = 0
    speed: int = 0
    heading: float = 0.0
    track_history: deque = field(default_factory=lambda: deque(maxlen=40))
    timestamp: float = field(default_factory=time.time)
    squawk: str = '7000'
    aircraft_type: str = 'UNKN'


@dataclass
class GroundTarget:
    """Surface movement target (ASRACS)."""
    target_id: str = ''
    x: float = 0.0
    y: float = 0.0
    target_type: str = 'aircraft'   # 'aircraft' | 'vehicle'
    speed: float = 0.0
    heading: float = 0.0
    callsign: str = ''


@dataclass
class IncursionAlert:
    """Runway incursion alert."""
    target_id: str = ''
    severity: str = 'CAUTION'       # CAUTION | WARNING | CRITICAL
    message: str = ''
    timestamp: float = field(default_factory=time.time)
    acknowledged: bool = False


# ─────────────────────────────────────────────────────────────────────────────
# Helper: Great-circle maths
# ─────────────────────────────────────────────────────────────────────────────

def gc_distance_nm(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Return great-circle distance in nautical miles."""
    R = 3440.065  # Earth radius in NM
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlam = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2) ** 2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def bearing_to(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Return true bearing from point 1 to point 2 (degrees)."""
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dlam = math.radians(lon2 - lon1)
    x = math.sin(dlam) * math.cos(phi2)
    y = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlam)
    return (math.degrees(math.atan2(x, y)) + 360) % 360


def project_position(lat: float, lon: float, hdg: float, dist_nm: float) -> Tuple[float, float]:
    """Return new (lat, lon) after moving dist_nm on heading hdg."""
    R = 3440.065
    d = dist_nm / R
    hdg_r = math.radians(hdg)
    lat1 = math.radians(lat)
    lon1 = math.radians(lon)
    lat2 = math.asin(math.sin(lat1) * math.cos(d) + math.cos(lat1) * math.sin(d) * math.cos(hdg_r))
    lon2 = lon1 + math.atan2(
        math.sin(hdg_r) * math.sin(d) * math.cos(lat1),
        math.cos(d) - math.sin(lat1) * math.sin(lat2),
    )
    return math.degrees(lat2), math.degrees(lon2)


# ─────────────────────────────────────────────────────────────────────────────
# GlideSlopeDetector
# ─────────────────────────────────────────────────────────────────────────────

class GlideSlopeDetector:
    """Check whether an aircraft is on the standard 3° glide path."""

    STANDARD_ANGLE = 3.0        # degrees
    FOOT_PER_NM = 6076.115

    def __init__(self, angle: float = 3.0):
        self.angle = angle

    def ideal_altitude_ft(self, distance_nm: float, threshold_elev_ft: float = 0) -> float:
        """Return ideal glidepath altitude above MSL at distance_nm from threshold."""
        return threshold_elev_ft + math.tan(math.radians(self.angle)) * distance_nm * self.FOOT_PER_NM

    def deviation_ft(self, altitude_ft: float, distance_nm: float, threshold_elev_ft: float = 0) -> float:
        """Positive = above glidepath, negative = below."""
        return altitude_ft - self.ideal_altitude_ft(distance_nm, threshold_elev_ft)

    def check_glideslope(self, altitude_ft: float, distance_nm: float) -> float:
        """Return deviation in feet (positive = above, negative = below)."""
        return self.deviation_ft(altitude_ft, distance_nm)


# ─────────────────────────────────────────────────────────────────────────────
# SurfaceSlopeAnalyzer
# ─────────────────────────────────────────────────────────────────────────────

class SurfaceSlopeAnalyzer:
    """Estimate surface slope gradients around a given position."""

    def analyze(self, lat: float, lon: float) -> Dict[str, float]:
        """Return dict with max_gradient, avg_gradient, slope_direction (degrees)."""
        # Synthetic terrain model: gradient varies with lat/lon
        base = abs(math.sin(math.radians(lat * 10)) * math.cos(math.radians(lon * 10)))
        max_grad = base * 8.0 + 0.5
        avg_grad = max_grad * 0.6
        direction = (lat * 7 + lon * 13) % 360
        return {
            'max_gradient': round(max_grad, 2),
            'avg_gradient': round(avg_grad, 2),
            'slope_direction': round(direction, 1),
        }

    def required_descent_rate(self, gs_angle: float, speed_kts: float) -> int:
        """Return descent rate (ft/min) for given glide angle at given airspeed."""
        return int(speed_kts * math.tan(math.radians(gs_angle)) * 101.3)


# ─────────────────────────────────────────────────────────────────────────────
# LDAFileParser  (v5.3)
# ─────────────────────────────────────────────────────────────────────────────

class LDAFileParser:
    """
    Parse Thales / Rohde & Schwarz ILS configuration (LDA) files.
    Extracts station type, frequencies, waveform names and nominal values.
    """

    WAVEFORM_NAMES = [
        'Normal', 'Alarm POS Low', 'Alarm POS High',
        'Alarm CLR Low', 'Alarm CLR High', 'Monitor Low',
        'Monitor High', 'Test Signal',
    ]

    def __init__(self, filepath: str):
        self.filepath = filepath
        self._data: Dict[str, Any] = {}
        self._parsed = False

    def parse(self) -> bool:
        try:
            with open(self.filepath, 'rb') as fh:
                raw = fh.read()
            self._parse_binary(raw)
            self._parsed = True
            logger.info('LDA file parsed: %s (%d bytes)', self.filepath, len(raw))
            return True
        except Exception as exc:
            logger.error('LDA parse failed: %s', exc)
            return False

    def _parse_binary(self, raw: bytes) -> None:
        text = raw.decode('latin-1', errors='replace')
        self._data['raw_text'] = text

        # Station type
        for keyword in ('GP', 'LOC', 'GLIDE', 'LOCALIZER'):
            if keyword in text.upper():
                self._data['station_type'] = 'GP' if 'GP' in keyword or 'GLIDE' in keyword else 'LOC'
                break
        else:
            self._data['station_type'] = 'UNKNOWN'

        # Frequencies
        import re
        freq_matches = re.findall(r'(\d{3}[\.,]\d{2,3})', text)
        freqs = []
        for m in freq_matches:
            try:
                f = float(m.replace(',', '.'))
                if 108.0 <= f <= 117.95:
                    freqs.append(f)
            except ValueError:
                pass
        self._data['localizer_freq'] = freqs[0] if freqs else 110.10
        self._data['glideslope_freq'] = freqs[1] if len(freqs) > 1 else 329.60

        # Waveforms (look for pattern blocks)
        waveforms = []
        for name in self.WAVEFORM_NAMES:
            if name.split()[0].upper() in text.upper():
                waveforms.append(name)
        self._data['waveforms'] = waveforms if waveforms else self.WAVEFORM_NAMES[:4]

        # Nominal values
        ddm_match = re.search(r'DDM\s*[=:]\s*([\d.]+)', text, re.IGNORECASE)
        sdm_match = re.search(r'SDM\s*[=:]\s*([\d.]+)', text, re.IGNORECASE)
        self._data['nominal_values'] = {
            'crs_ddm': float(ddm_match.group(1)) if ddm_match else 0.093,
            'clr_ddm': 0.155,
            'crs_sdm': float(sdm_match.group(1)) if sdm_match else 40.0,
        }

    def get_station_type(self) -> str:
        return self._data.get('station_type', 'UNKNOWN')

    def get_localizer_frequency(self) -> float:
        return self._data.get('localizer_freq', 0.0)

    def get_glideslope_frequency(self) -> float:
        return self._data.get('glideslope_freq', 0.0)

    def get_waveform_names(self) -> List[str]:
        return self._data.get('waveforms', [])

    def get_nominal_values(self) -> Dict[str, float]:
        return self._data.get('nominal_values', {})


# ─────────────────────────────────────────────────────────────────────────────
# MockVORTCPServer
# ─────────────────────────────────────────────────────────────────────────────

class MockVORTCPServer:
    """Listens on a TCP port and serves synthetic VOR data sentences."""

    def __init__(self, host: str = '127.0.0.1', port: int = 5005):
        self.host = host
        self.port = port
        self._server_sock: Optional[socket.socket] = None
        self._thread: Optional[Thread] = None
        self._running = False
        self._bearing = 0.0
        self._deviation = 0.0

    def start(self) -> bool:
        try:
            self._server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._server_sock.bind((self.host, self.port))
            self._server_sock.listen(5)
            self._server_sock.settimeout(1.0)
            self._running = True
            self._thread = Thread(target=self._serve, daemon=True)
            self._thread.start()
            logger.info('Mock VOR server started on %s:%d', self.host, self.port)
            return True
        except Exception as exc:
            logger.error('Mock server start failed: %s', exc)
            return False

    def stop(self) -> None:
        self._running = False
        if self._server_sock:
            try:
                self._server_sock.close()
            except Exception:
                pass
        logger.info('Mock VOR server stopped')

    def _serve(self) -> None:
        while self._running:
            try:
                conn, addr = self._server_sock.accept()
                Thread(target=self._handle, args=(conn,), daemon=True).start()
            except socket.timeout:
                pass
            except Exception:
                if self._running:
                    logger.exception('Mock server accept error')

    def _handle(self, conn: socket.socket) -> None:
        logger.info('Mock VOR client connected')
        try:
            while self._running:
                self._bearing = (self._bearing + 0.5) % 360
                self._deviation = math.sin(math.radians(self._bearing * 2)) * 3.0
                sig = 75 + random.uniform(-5, 5)
                sentence = f'$VOR,{self._bearing:.1f},{self._deviation:.2f},{sig:.1f},WKV,116.90*\r\n'
                conn.sendall(sentence.encode('ascii'))
                time.sleep(0.5)
        except Exception:
            pass
        finally:
            conn.close()


# ─────────────────────────────────────────────────────────────────────────────
# VORConnectionHandler
# ─────────────────────────────────────────────────────────────────────────────

class VORConnectionHandler:
    """Manages serial / TCP connections to VOR receiver hardware."""

    def __init__(self):
        self._serial: Optional[Any] = None
        self._sock: Optional[socket.socket] = None
        self._mode: str = 'none'  # 'serial' | 'tcp' | 'none'
        self._lock = Lock()

    def connect_serial(self, port: str, baud: int = 9600) -> bool:
        if not HAS_SERIAL:
            logger.warning('pyserial not installed; serial connection unavailable')
            return False
        try:
            self._serial = serial.Serial(port, baud, timeout=1)
            self._mode = 'serial'
            logger.info('Serial connected: %s @ %d', port, baud)
            return True
        except Exception as exc:
            logger.error('Serial connect failed: %s', exc)
            return False

    def connect_tcp(self, host: str, port: int, timeout: float = 5.0) -> bool:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(timeout)
            s.connect((host, port))
            self._sock = s
            self._mode = 'tcp'
            logger.info('TCP connected: %s:%d', host, port)
            return True
        except Exception as exc:
            logger.error('TCP connect failed: %s', exc)
            return False

    def disconnect(self) -> None:
        with self._lock:
            if self._serial:
                try:
                    self._serial.close()
                except Exception:
                    pass
                self._serial = None
            if self._sock:
                try:
                    self._sock.close()
                except Exception:
                    pass
                self._sock = None
            self._mode = 'none'
        logger.info('Disconnected')

    def readline(self) -> Optional[str]:
        with self._lock:
            try:
                if self._mode == 'serial' and self._serial:
                    return self._serial.readline().decode('ascii', errors='replace').strip()
                if self._mode == 'tcp' and self._sock:
                    buf = b''
                    while True:
                        c = self._sock.recv(1)
                        if not c or c == b'\n':
                            break
                        buf += c
                    return buf.decode('ascii', errors='replace').strip()
            except Exception:
                pass
        return None

    @property
    def connected(self) -> bool:
        return self._mode != 'none'

    @property
    def mode(self) -> str:
        return self._mode


# ─────────────────────────────────────────────────────────────────────────────
# VORDataProcessor
# ─────────────────────────────────────────────────────────────────────────────

class VORDataProcessor:
    """Parse raw VOR sentences and maintain a rolling buffer."""

    def __init__(self, maxlen: int = 1000):
        self._buf: deque = deque(maxlen=maxlen)
        self._lock = Lock()

    def process(self, raw: str) -> Optional[VORData]:
        """Parse '$VOR,bearing,dev,sig,ident,freq*' sentence."""
        try:
            raw = raw.strip()
            if not raw.startswith('$VOR'):
                return None
            parts = raw.lstrip('$').rstrip('*').split(',')
            if len(parts) < 5:
                return None
            data = VORData(
                station_id=parts[4] if len(parts) > 4 else '',
                frequency=float(parts[5]) if len(parts) > 5 else 0.0,
                bearing=float(parts[1]),
                deviation=float(parts[2]),
                signal_strength=float(parts[3]),
                timestamp=time.time(),
                raw=raw,
            )
            with self._lock:
                self._buf.append(data)
            return data
        except (ValueError, IndexError):
            return None

    def latest(self) -> Optional[VORData]:
        with self._lock:
            return self._buf[-1] if self._buf else None

    def history(self, n: int = 100) -> List[VORData]:
        with self._lock:
            return list(self._buf)[-n:]


# ─────────────────────────────────────────────────────────────────────────────
# AircraftTracker
# ─────────────────────────────────────────────────────────────────────────────

class AircraftTracker:
    """Maintains a dict of known aircraft, keyed by callsign."""

    def __init__(self, max_age_s: float = 120.0):
        self._aircraft: Dict[str, AircraftData] = {}
        self._lock = Lock()
        self.max_age = max_age_s

    def update(self, ac: AircraftData) -> None:
        with self._lock:
            if ac.callsign in self._aircraft:
                prev = self._aircraft[ac.callsign]
                prev.track_history.append((prev.lat, prev.lon))
            else:
                ac.track_history = deque(maxlen=40)
            self._aircraft[ac.callsign] = ac

    def prune(self) -> None:
        now = time.time()
        with self._lock:
            stale = [cs for cs, ac in self._aircraft.items() if now - ac.timestamp > self.max_age]
            for cs in stale:
                del self._aircraft[cs]

    def get_all(self) -> List[AircraftData]:
        with self._lock:
            return list(self._aircraft.values())

    def count(self) -> int:
        with self._lock:
            return len(self._aircraft)


# ─────────────────────────────────────────────────────────────────────────────
# SimulationEngine  – en-route aircraft
# ─────────────────────────────────────────────────────────────────────────────

class SimulationEngine:
    """Generates synthetic en-route aircraft around a centre point."""

    CALLSIGNS = [
        'SAA101', 'BAW42', 'SAA203', 'LMU15', 'KQA11',
        'ELY788', 'ETH508', 'QFA63', 'SWR128', 'DLH462',
        'AFR990', 'UAE147', 'QTR31', 'TOM256', 'RYR8VB',
    ]

    def __init__(self, centre_lat: float = -26.14, centre_lon: float = 28.25,
                 range_nm: float = 120, count: int = 12):
        self.centre_lat = centre_lat
        self.centre_lon = centre_lon
        self.range_nm = range_nm
        self._aircraft: List[AircraftData] = []
        self._running = False
        self._thread: Optional[Thread] = None
        self._lock = Lock()
        self._tracker: Optional[AircraftTracker] = None
        self._init_aircraft(count)

    def _init_aircraft(self, count: int) -> None:
        used = set()
        for i in range(count):
            cs = random.choice(self.CALLSIGNS)
            while cs in used:
                cs = random.choice(self.CALLSIGNS)
            used.add(cs)
            dist = random.uniform(10, self.range_nm * 0.9)
            hdg_to = random.uniform(0, 360)
            lat, lon = project_position(self.centre_lat, self.centre_lon, hdg_to, dist)
            ac = AircraftData(
                callsign=cs,
                lat=lat, lon=lon,
                altitude=random.randint(5000, 39000) // 100 * 100,
                speed=random.randint(280, 520),
                heading=random.uniform(0, 360),
                squawk=f'{random.randint(1000, 7776):04d}',
                aircraft_type=random.choice(['B738', 'A320', 'B744', 'A333', 'E190', 'C172']),
            )
            self._aircraft.append(ac)

    def set_tracker(self, tracker: AircraftTracker) -> None:
        self._tracker = tracker

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._thread = Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False

    def _run(self) -> None:
        while self._running:
            dt = 1.0
            with self._lock:
                for ac in self._aircraft:
                    ac.heading += random.uniform(-2, 2)
                    ac.heading %= 360
                    dist = ac.speed / 3600 * dt  # nm per second
                    ac.lat, ac.lon = project_position(ac.lat, ac.lon, ac.heading, dist)
                    ac.altitude += random.randint(-100, 100)
                    ac.altitude = max(1000, min(45000, ac.altitude))
                    ac.timestamp = time.time()
                    if self._tracker:
                        self._tracker.update(ac)
            time.sleep(dt)

    def get_aircraft(self) -> List[AircraftData]:
        with self._lock:
            return list(self._aircraft)


# ─────────────────────────────────────────────────────────────────────────────
# JNBAirportLayout – OR Tambo layout constants (normalised 0-1 space)
# ─────────────────────────────────────────────────────────────────────────────

class JNBAirportLayout:
    """Normalised (0-1) coordinates for OR Tambo International layout."""
    # Runway 03L/21R
    RWY_03L_THR  = (0.30, 0.80)
    RWY_21R_THR  = (0.70, 0.20)
    # Runway 03R/21L
    RWY_03R_THR  = (0.40, 0.80)
    RWY_21L_THR  = (0.80, 0.20)
    # Taxiways (start, end)
    TAXIWAY_A    = ((0.20, 0.50), (0.80, 0.50))
    TAXIWAY_B    = ((0.50, 0.20), (0.50, 0.80))
    # Apron centre
    APRON_CENTRE = (0.50, 0.55)
    # Terminal
    TERMINAL     = (0.45, 0.60)


# ─────────────────────────────────────────────────────────────────────────────
# ASRACSSimEngine  – surface movement simulation
# ─────────────────────────────────────────────────────────────────────────────

class ASRACSSimEngine:
    """Simulates ground vehicle and aircraft surface movement at JNB."""

    def __init__(self):
        self._targets: List[GroundTarget] = []
        self._alerts: List[IncursionAlert] = []
        self._running = False
        self._thread: Optional[Thread] = None
        self._lock = Lock()
        self._init_targets()

    def _init_targets(self) -> None:
        aircraft_cs = ['SAA101', 'BAW42', 'SAA203', 'LMU15', 'KQA11']
        vehicle_cs  = ['TUG-1', 'FUEL-2', 'BUS-3', 'TUG-4', 'CATR-5']
        for i, cs in enumerate(aircraft_cs):
            self._targets.append(GroundTarget(
                target_id=f'AC{i+1}', x=0.2 + i * 0.12, y=0.5 + random.uniform(-0.1, 0.1),
                target_type='aircraft', speed=random.uniform(5, 20),
                heading=random.uniform(0, 360), callsign=cs,
            ))
        for i, cs in enumerate(vehicle_cs):
            self._targets.append(GroundTarget(
                target_id=f'VH{i+1}', x=0.4 + i * 0.08, y=0.55 + random.uniform(-0.05, 0.05),
                target_type='vehicle', speed=random.uniform(2, 10),
                heading=random.uniform(0, 360), callsign=cs,
            ))

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._thread = Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False

    def _run(self) -> None:
        while self._running:
            dt = 0.1
            with self._lock:
                for t in self._targets:
                    t.heading += random.uniform(-5, 5)
                    t.heading %= 360
                    spd_pix = t.speed * dt / 1000
                    t.x += spd_pix * math.sin(math.radians(t.heading))
                    t.y -= spd_pix * math.cos(math.radians(t.heading))
                    t.x = max(0.05, min(0.95, t.x))
                    t.y = max(0.05, min(0.95, t.y))
                self._check_incursions()
            time.sleep(dt)

    def _check_incursions(self) -> None:
        # Clear old acknowledged alerts
        self._alerts = [a for a in self._alerts if not a.acknowledged]
        # Runway bands (y < 0.35 or y > 0.65 are runway zones)
        for t in self._targets:
            if t.y < 0.35 or t.y > 0.65:
                existing = any(a.target_id == t.target_id for a in self._alerts)
                if not existing:
                    sev = 'CRITICAL' if abs(t.y - 0.5) > 0.25 else 'WARNING'
                    self._alerts.append(IncursionAlert(
                        target_id=t.target_id,
                        severity=sev,
                        message=f'{t.callsign} on runway zone',
                    ))

    def get_targets(self) -> List[GroundTarget]:
        with self._lock:
            return list(self._targets)

    def get_alerts(self) -> List[IncursionAlert]:
        with self._lock:
            return list(self._alerts)

    def acknowledge_alert(self, target_id: str) -> None:
        with self._lock:
            for a in self._alerts:
                if a.target_id == target_id:
                    a.acknowledged = True


# ─────────────────────────────────────────────────────────────────────────────
# DataAcquisitionThread
# ─────────────────────────────────────────────────────────────────────────────

class DataAcquisitionThread(QThread):
    """Background thread that reads from VORConnectionHandler and emits data."""

    data_received = pyqtSignal(VORData)
    connection_error = pyqtSignal(str)

    def __init__(self, handler: VORConnectionHandler, processor: VORDataProcessor,
                 parent=None):
        super().__init__(parent)
        self._handler = handler
        self._processor = processor
        self._active = True

    def run(self) -> None:
        while self._active and self._handler.connected:
            line = self._handler.readline()
            if line:
                data = self._processor.process(line)
                if data:
                    self.data_received.emit(data)
            else:
                time.sleep(0.05)

    def stop(self) -> None:
        self._active = False
        self.wait(2000)


# ─────────────────────────────────────────────────────────────────────────────
# VORAirportConfig – wraps static data with helper methods
# ─────────────────────────────────────────────────────────────────────────────

class VORAirportConfig:
    """Provides access to VOR station and airport configuration."""

    def __init__(self):
        self.vor_stations = VOR_STATIONS
        self.saaf_bases = SAAF_BASES

    def get_station(self, station_id: str) -> Optional[Dict]:
        return self.vor_stations.get(station_id)

    def get_base(self, icao: str) -> Optional[Dict]:
        return self.saaf_bases.get(icao)

    def station_ids(self) -> List[str]:
        return list(self.vor_stations.keys())

    def base_icaos(self) -> List[str]:
        return list(self.saaf_bases.keys())

    def frequency_label(self, station_id: str) -> str:
        s = self.get_station(station_id)
        if s:
            return f"{s['ident']}  {s['freq']:.2f} MHz"
        return station_id


# ─────────────────────────────────────────────────────────────────────────────
# CDIDisplay  – Course Deviation Indicator
# ─────────────────────────────────────────────────────────────────────────────

class CDIDisplay(QWidget):
    """
    Draws a CDI (Course Deviation Indicator).
    deviation  – float, dots, range ±2.5
    bearing    – float, degrees 0-360
    signal_ok  – bool
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.deviation: float = 0.0
        self.bearing: float = 0.0
        self.signal_ok: bool = True
        self.setMinimumSize(160, 160)

    def set_data(self, deviation: float, bearing: float, signal_ok: bool = True) -> None:
        self.deviation = max(-2.5, min(2.5, deviation))
        self.bearing = bearing % 360
        self.signal_ok = signal_ok
        self.update()

    def paintEvent(self, event) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        w = self.width()
        h = self.height()
        cx = int(w / 2)
        cy = int(h / 2)
        r = int(min(w, h) / 2 - 8)

        # Background
        painter.fillRect(0, 0, w, h, QColor(15, 15, 30))

        # Outer circle
        painter.setPen(QPen(QColor(100, 200, 100), 2))
        painter.drawEllipse(cx - r, cy - r, r * 2, r * 2)

        # Dot scale: 5 dots, ±2.5
        dot_spacing = int(r * 0.38)
        painter.setPen(QPen(QColor(180, 180, 180), 1))
        for d in (-2, -1, 0, 1, 2):
            dx = int(cx + d * dot_spacing)
            painter.drawEllipse(dx - 4, cy - 4, 8, 8)

        # Horizontal CDI bar
        bar_w = int(r * 1.6)
        painter.setPen(QPen(QColor(60, 60, 80), 1))
        painter.drawLine(int(cx - bar_w / 2), cy, int(cx + bar_w / 2), cy)

        if self.signal_ok:
            # Needle
            needle_x = int(cx + self.deviation * dot_spacing)
            needle_x = max(cx - r + 4, min(cx + r - 4, needle_x))
            painter.setPen(QPen(QColor(0, 220, 0), 4))
            painter.drawLine(needle_x, int(cy - r * 0.55), needle_x, int(cy + r * 0.55))

            # Bearing pointer (triangle at top)
            bx = cx
            by = int(cy - r + 4)
            pts = QPolygon([
                QPoint(bx, by),
                QPoint(bx - 8, by + 14),
                QPoint(bx + 8, by + 14),
            ])
            painter.setBrush(QBrush(QColor(255, 200, 0)))
            painter.setPen(Qt.NoPen)
            painter.drawPolygon(pts)

            # Bearing text
            painter.setPen(QPen(QColor(255, 200, 0)))
            painter.setFont(QFont('Monospace', 10, QFont.Bold))
            painter.drawText(int(cx - 25), int(cy + r - 18), 50, 18,
                             Qt.AlignCenter, f'{int(self.bearing):03d}°')
        else:
            # No signal
            painter.setPen(QPen(QColor(220, 60, 60), 3))
            painter.drawLine(cx - r + 10, cy - r + 10, cx + r - 10, cy + r - 10)
            painter.drawLine(cx - r + 10, cy + r - 10, cx + r - 10, cy - r + 10)
            painter.setPen(QPen(QColor(220, 60, 60)))
            painter.setFont(QFont('Monospace', 9, QFont.Bold))
            painter.drawText(cx - 30, cy + r - 20, 60, 18, Qt.AlignCenter, 'NO SIG')

        painter.end()


# ─────────────────────────────────────────────────────────────────────────────
# ApproachGuidanceDisplay
# ─────────────────────────────────────────────────────────────────────────────

class ApproachGuidanceDisplay(QWidget):
    """
    Shows glideslope and localizer bars with deviation indicators.
    gs_dev   – float, feet (+above / -below)
    loc_dev  – float, dots ±2.5
    dist_nm  – float, nautical miles to threshold
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.gs_dev: float = 0.0
        self.loc_dev: float = 0.0
        self.dist_nm: float = 0.0
        self.signal_ok: bool = True
        self.setMinimumSize(220, 180)

    def set_data(self, gs_dev: float, loc_dev: float, dist_nm: float,
                 signal_ok: bool = True) -> None:
        self.gs_dev = gs_dev
        self.loc_dev = max(-2.5, min(2.5, loc_dev))
        self.dist_nm = dist_nm
        self.signal_ok = signal_ok
        self.update()

    def paintEvent(self, event) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        w = self.width()
        h = self.height()
        cx = int(w / 2)
        cy = int(h / 2)

        # Background
        painter.fillRect(0, 0, w, h, QColor(15, 15, 30))

        dot_r = 5
        # ── Localizer (horizontal) ────────────────────────────────────────
        loc_y = int(h * 0.70)
        bar_half = int(w * 0.38)
        dot_spacing = int(bar_half * 0.45)

        painter.setPen(QPen(QColor(80, 80, 100), 1))
        painter.drawLine(cx - bar_half, loc_y, cx + bar_half, loc_y)

        painter.setPen(QPen(QColor(160, 160, 160), 1))
        painter.setBrush(QBrush(QColor(80, 80, 80)))
        for d in (-2, -1, 1, 2):
            dx = int(cx + d * dot_spacing)
            painter.drawEllipse(dx - dot_r, loc_y - dot_r, dot_r * 2, dot_r * 2)

        if self.signal_ok:
            loc_x = int(cx + self.loc_dev * dot_spacing)
            loc_x = max(cx - bar_half + dot_r, min(cx + bar_half - dot_r, loc_x))
            col = QColor(0, 255, 0) if abs(self.loc_dev) < 1.0 else QColor(255, 165, 0)
            painter.setPen(QPen(col, 4))
            painter.drawLine(loc_x, int(loc_y - 18), loc_x, int(loc_y + 18))

        # ── Glideslope (vertical) ─────────────────────────────────────────
        gs_x = int(w * 0.82)
        gs_half = int(h * 0.32)

        painter.setPen(QPen(QColor(80, 80, 100), 1))
        painter.drawLine(gs_x, cy - gs_half, gs_x, cy + gs_half)

        painter.setPen(QPen(QColor(160, 160, 160), 1))
        painter.setBrush(QBrush(QColor(80, 80, 80)))
        for d in (-2, -1, 1, 2):
            dy = int(cy + d * int(gs_half * 0.45))
            painter.drawEllipse(gs_x - dot_r, dy - dot_r, dot_r * 2, dot_r * 2)

        if self.signal_ok:
            # gs_dev in feet; clamp to ±200 ft then map to ±2.5 dots
            gs_dots = max(-2.5, min(2.5, self.gs_dev / 80.0))
            gs_y = int(cy - gs_dots * int(gs_half * 0.45))
            gs_y = max(cy - gs_half + dot_r, min(cy + gs_half - dot_r, gs_y))
            col = QColor(0, 255, 0) if abs(self.gs_dev) < 50 else QColor(255, 165, 0)
            painter.setPen(QPen(col, 4))
            painter.drawLine(int(gs_x - 18), gs_y, int(gs_x + 18), gs_y)

        # ── Labels ────────────────────────────────────────────────────────
        painter.setPen(QPen(QColor(200, 200, 200)))
        painter.setFont(QFont('Monospace', 8))
        painter.drawText(cx - 40, int(h * 0.10), 80, 18, Qt.AlignCenter, 'APPROACH')
        gs_label = f'GS: {self.gs_dev:+.0f} ft' if self.signal_ok else 'GS: ---'
        loc_label = f'LOC: {self.loc_dev:+.2f} dot' if self.signal_ok else 'LOC: ---'
        dist_label = f'DIST: {self.dist_nm:.1f} nm'
        painter.drawText(4, int(h * 0.87), int(w / 2) - 4, 16, Qt.AlignLeft, gs_label)
        painter.drawText(int(w / 2), int(h * 0.87), int(w / 2) - 4, 16, Qt.AlignRight, loc_label)
        painter.drawText(4, int(h * 0.93), w - 8, 16, Qt.AlignCenter, dist_label)

        painter.end()


# ─────────────────────────────────────────────────────────────────────────────
# RadarDisplay
# ─────────────────────────────────────────────────────────────────────────────

class RadarDisplay(QWidget):
    """
    Animated radar sweep display showing aircraft tracks within a given range.
    All painter calls use int() coordinates to avoid PyQt5 type errors.
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self._aircraft: List[AircraftData] = []
        self._sweep_angle: float = 0.0
        self._range_nm: float = 120.0
        self._centre_lat: float = -26.14
        self._centre_lon: float = 28.25
        self._lock = Lock()
        self._zoom: float = 1.0
        self.setMinimumSize(400, 400)
        self.setStyleSheet('background: #001200;')

        self._sweep_timer = QTimer(self)
        self._sweep_timer.timeout.connect(self._advance_sweep)
        self._sweep_timer.start(50)

    def _advance_sweep(self) -> None:
        self._sweep_angle = (self._sweep_angle + 3.0) % 360
        self.update()

    def set_aircraft(self, aircraft: List[AircraftData]) -> None:
        with self._lock:
            self._aircraft = list(aircraft)

    def set_range(self, range_nm: float) -> None:
        self._range_nm = max(20.0, min(500.0, range_nm))
        self.update()

    def set_centre(self, lat: float, lon: float) -> None:
        self._centre_lat = lat
        self._centre_lon = lon
        self.update()

    def _latlon_to_pixel(self, lat: float, lon: float, cx: int, cy: int,
                         pix_per_nm: float) -> Tuple[int, int]:
        """Convert lat/lon to pixel coordinates relative to display centre."""
        dlat = lat - self._centre_lat
        dlon = lon - self._centre_lon
        cos_lat = math.cos(math.radians(self._centre_lat))
        # 1 degree lat ≈ 60 NM; 1 degree lon ≈ 60 * cos(lat) NM
        x_nm = dlon * 60.0 * cos_lat
        y_nm = dlat * 60.0
        px = int(cx + x_nm * pix_per_nm)
        py = int(cy - y_nm * pix_per_nm)
        return px, py

    def paintEvent(self, event) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        w = self.width()
        h = self.height()
        cx = int(w / 2)
        cy = int(h / 2)
        r = int(min(w, h) / 2 - 10)
        effective_range = self._range_nm / self._zoom
        pix_per_nm = r / effective_range

        # ── Background ───────────────────────────────────────────────────
        painter.fillRect(0, 0, w, h, QColor(0, 15, 0))

        # Clip to radar circle
        clip_path = QPainterPath()
        clip_path.addEllipse(cx - r, cy - r, r * 2, r * 2)
        painter.setClipPath(clip_path)

        # ── Sweep fade ────────────────────────────────────────────────────
        sweep_rad = math.radians(self._sweep_angle)
        grad = QRadialGradient(cx, cy, r)
        grad.setColorAt(0.0, QColor(0, 80, 0, 180))
        grad.setColorAt(1.0, QColor(0, 80, 0, 0))
        painter.setBrush(QBrush(grad))
        painter.setPen(Qt.NoPen)
        # Sweep sector (60 deg)
        rect = QRect(cx - r, cy - r, r * 2, r * 2)
        start_angle = int((90 - self._sweep_angle) * 16)
        span_angle = int(-60 * 16)
        painter.drawPie(rect, start_angle, span_angle)

        painter.setClipping(False)

        # ── Range rings ───────────────────────────────────────────────────
        painter.setPen(QPen(QColor(0, 100, 0), 1, Qt.DotLine))
        num_rings = 4
        for i in range(1, num_rings + 1):
            ring_r = int(r * i / num_rings)
            painter.drawEllipse(cx - ring_r, cy - ring_r, ring_r * 2, ring_r * 2)
            ring_nm = int(effective_range * i / num_rings)
            painter.setPen(QPen(QColor(0, 140, 0)))
            painter.setFont(QFont('Monospace', 7))
            painter.drawText(cx + ring_r + 2, cy - 4, 40, 12, Qt.AlignLeft, f'{ring_nm}nm')
            painter.setPen(QPen(QColor(0, 100, 0), 1, Qt.DotLine))

        # ── Outer circle ──────────────────────────────────────────────────
        painter.setPen(QPen(QColor(0, 180, 0), 2))
        painter.drawEllipse(cx - r, cy - r, r * 2, r * 2)

        # ── Crosshairs ────────────────────────────────────────────────────
        # All coordinates are already int; enforce with int() for safety
        painter.setPen(QPen(QColor(0, 80, 0), 1))
        painter.drawLine(int(cx - r), int(cy), int(cx + r), int(cy))
        painter.drawLine(int(cx), int(cy - r), int(cx), int(cy + r))

        # ── Sweep line ────────────────────────────────────────────────────
        sweep_x = int(cx + r * math.sin(sweep_rad))
        sweep_y = int(cy - r * math.cos(sweep_rad))
        painter.setPen(QPen(QColor(0, 255, 60), 2))
        painter.drawLine(cx, cy, sweep_x, sweep_y)

        # ── Cardinal labels ───────────────────────────────────────────────
        painter.setPen(QPen(QColor(0, 200, 80)))
        painter.setFont(QFont('Monospace', 8, QFont.Bold))
        lbl_d = int(r + 12)
        for angle, label in [(0, 'N'), (90, 'E'), (180, 'S'), (270, 'W')]:
            rad = math.radians(angle)
            lx = int(cx + lbl_d * math.sin(rad)) - 6
            ly = int(cy - lbl_d * math.cos(rad)) - 6
            painter.drawText(lx, ly, 12, 12, Qt.AlignCenter, label)

        # ── Aircraft ──────────────────────────────────────────────────────
        with self._lock:
            aircraft_copy = list(self._aircraft)

        for ac in aircraft_copy:
            px, py = self._latlon_to_pixel(ac.lat, ac.lon, cx, cy, pix_per_nm)
            # Skip if outside radar circle
            if (px - cx) ** 2 + (py - cy) ** 2 > r * r:
                continue

            # Track history
            hist = list(ac.track_history)
            if len(hist) > 1:
                painter.setPen(QPen(QColor(0, 160, 0, 100), 1))
                for k in range(1, len(hist)):
                    hx0, hy0 = self._latlon_to_pixel(hist[k-1][0], hist[k-1][1], cx, cy, pix_per_nm)
                    hx1, hy1 = self._latlon_to_pixel(hist[k][0], hist[k][1], cx, cy, pix_per_nm)
                    painter.drawLine(int(hx0), int(hy0), int(hx1), int(hy1))

            # Aircraft blip
            painter.setPen(QPen(QColor(0, 255, 60), 1))
            painter.setBrush(QBrush(QColor(0, 255, 60)))
            painter.drawEllipse(int(px - 3), int(py - 3), 6, 6)

            # Velocity vector
            vel_len = int(ac.speed / 50.0 * pix_per_nm)
            vx = int(px + vel_len * math.sin(math.radians(ac.heading)))
            vy = int(py - vel_len * math.cos(math.radians(ac.heading)))
            painter.setPen(QPen(QColor(0, 200, 100), 1))
            painter.drawLine(int(px), int(py), int(vx), int(vy))

            # Label
            painter.setPen(QPen(QColor(180, 255, 180)))
            painter.setFont(QFont('Monospace', 7))
            painter.drawText(int(px + 5), int(py - 5), 60, 20,
                             Qt.AlignLeft, f'{ac.callsign}\n{ac.altitude}ft')

        # ── Centre dot ────────────────────────────────────────────────────
        painter.setPen(QPen(QColor(0, 255, 60), 2))
        painter.setBrush(QBrush(QColor(0, 255, 60)))
        painter.drawEllipse(int(cx - 4), int(cy - 4), 8, 8)

        # ── Aircraft count ────────────────────────────────────────────────
        painter.setPen(QPen(QColor(0, 200, 80)))
        painter.setFont(QFont('Monospace', 8))
        painter.drawText(4, 4, 120, 16, Qt.AlignLeft,
                         f'TRK: {len(aircraft_copy)}  RNG: {int(effective_range)}nm')

        painter.end()

    def wheelEvent(self, event) -> None:
        delta = event.angleDelta().y()
        if delta > 0:
            self._zoom = min(8.0, self._zoom * 1.1)
        else:
            self._zoom = max(0.3, self._zoom / 1.1)
        self.update()


# ─────────────────────────────────────────────────────────────────────────────
# Terrain3DWidget  – OpenGL terrain view
# ─────────────────────────────────────────────────────────────────────────────

class Terrain3DWidget(QOpenGLWidget if HAS_OPENGL_WIDGET else QWidget):
    """
    OpenGL-based 3-D terrain visualisation.
    Falls back to a simple 2-D placeholder when OpenGL is unavailable.
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self._rot_x: float = 30.0
        self._rot_z: float = 0.0
        self._zoom: float = 1.0
        self._mouse_last = QPoint()
        self._aircraft: List[AircraftData] = []
        self._grid_size = 30
        self._terrain: Optional[Any] = None   # numpy array
        self._gl_ready = False
        self.setMinimumSize(400, 300)

        if HAS_NUMPY:
            self._terrain = self._generate_terrain()

    def _generate_terrain(self):
        g = self._grid_size
        z = np.zeros((g, g), dtype=np.float32)
        for i in range(g):
            for j in range(g):
                z[i, j] = (
                    0.3 * math.sin(i * 0.4) * math.cos(j * 0.3) +
                    0.15 * math.sin(i * 0.9 + 1) * math.cos(j * 0.7 + 2) +
                    0.05 * math.sin(i * 2.1) * math.sin(j * 1.8)
                )
        z = (z - z.min()) / (z.max() - z.min() + 1e-9)  # normalise 0-1
        return z

    def set_aircraft(self, aircraft: List[AircraftData]) -> None:
        self._aircraft = list(aircraft)
        self.update()

    # ── OpenGL overrides ──────────────────────────────────────────────────────

    def initializeGL(self) -> None:
        if not HAS_OPENGL:
            return
        try:
            glClearColor(0.05, 0.05, 0.10, 1.0)
            glEnable(GL_DEPTH_TEST)
            glEnable(GL_LIGHTING)
            glEnable(GL_LIGHT0)
            glLightfv(GL_LIGHT0, GL_POSITION, [1.0, 1.0, 2.0, 0.0])
            glLightfv(GL_LIGHT0, GL_DIFFUSE,  [0.8, 0.8, 0.8, 1.0])
            glLightfv(GL_LIGHT0, GL_AMBIENT,  [0.3, 0.3, 0.3, 1.0])
            glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)
            glEnable(GL_COLOR_MATERIAL)
            self._gl_ready = True
        except Exception as exc:
            logger.warning('OpenGL init failed: %s', exc)

    def resizeGL(self, w: int, h: int) -> None:
        if not HAS_OPENGL or not self._gl_ready:
            return
        try:
            glViewport(0, 0, w, h)
            glMatrixMode(GL_PROJECTION)
            glLoadIdentity()
            gluPerspective(45.0, w / max(h, 1), 0.1, 100.0)
            glMatrixMode(GL_MODELVIEW)
        except Exception:
            pass

    def paintGL(self) -> None:
        if not HAS_OPENGL or not self._gl_ready:
            self._paint_2d_fallback()
            return
        try:
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
            glLoadIdentity()
            glTranslatef(0.0, -0.5, -3.0 / self._zoom)
            glRotatef(self._rot_x, 1, 0, 0)
            glRotatef(self._rot_z, 0, 0, 1)
            self._draw_terrain_gl()
            self._draw_aircraft_gl()
        except Exception as exc:
            logger.debug('paintGL error: %s', exc)

    def paintEvent(self, event) -> None:
        if not HAS_OPENGL_WIDGET:
            self._paint_2d_fallback()
        else:
            super().paintEvent(event)

    def _paint_2d_fallback(self) -> None:
        """Simple 2-D top-down terrain view when OpenGL is unavailable."""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        w = self.width()
        h = self.height()
        painter.fillRect(0, 0, w, h, QColor(20, 30, 50))

        if HAS_NUMPY and self._terrain is not None:
            g = self._grid_size
            cell_w = int(w / g)
            cell_h = int(h / g)
            for i in range(g):
                for j in range(g):
                    elev = float(self._terrain[i, j])
                    green = int(40 + elev * 160)
                    blue  = int(20 + elev * 60)
                    painter.fillRect(
                        int(j * cell_w), int(i * cell_h),
                        int(cell_w), int(cell_h),
                        QColor(30, green, blue),
                    )
        else:
            painter.setPen(QPen(QColor(100, 120, 160)))
            painter.setFont(QFont('Monospace', 10))
            painter.drawText(0, 0, w, h, Qt.AlignCenter, '3-D Terrain View\n(OpenGL not available)')

        # Aircraft overlay
        for ac in self._aircraft:
            # Map lat/lon to pixel – crude linear approximation
            norm_x = int((ac.lon - 18.0) / (32.0 - 18.0) * w)
            norm_y = int((ac.lat - (-35.0)) / ((-22.0) - (-35.0)) * h)
            norm_y = h - norm_y  # flip y
            if 0 <= norm_x < w and 0 <= norm_y < h:
                painter.setPen(QPen(QColor(255, 200, 0), 2))
                painter.setBrush(QBrush(QColor(255, 200, 0)))
                painter.drawEllipse(int(norm_x - 4), int(norm_y - 4), 8, 8)
                painter.setPen(QPen(QColor(255, 255, 200)))
                painter.setFont(QFont('Monospace', 7))
                painter.drawText(int(norm_x + 6), int(norm_y - 4), 60, 16,
                                 Qt.AlignLeft, ac.callsign)

        painter.end()

    def _draw_terrain_gl(self) -> None:
        if not HAS_OPENGL or self._terrain is None:
            return
        g = self._grid_size
        scale = 2.0 / g
        for i in range(g - 1):
            glBegin(GL_TRIANGLE_STRIP)
            for j in range(g):
                for di in range(2):
                    z0 = float(self._terrain[i + di, j])
                    x = (j - g / 2) * scale
                    y = z0 * 0.6
                    zc = (i + di - g / 2) * scale
                    r, green, b = (0.1 + z0 * 0.2, 0.3 + z0 * 0.5, 0.1)
                    glColor3f(r, green, b)
                    glNormal3f(0, 1, 0)
                    glVertex3f(x, y, zc)
            glEnd()

    def _draw_aircraft_gl(self) -> None:
        if not HAS_OPENGL:
            return
        for ac in self._aircraft:
            norm_x = (ac.lon - 28.25) * 0.5
            norm_z = (ac.lat - (-26.14)) * 0.5
            norm_y = ac.altitude / 40000.0 * 0.6
            glColor3f(1.0, 0.8, 0.0)
            glPointSize(6.0)
            glBegin(GL_POINTS)
            glVertex3f(float(norm_x), float(norm_y), float(norm_z))
            glEnd()

    # ── Mouse interaction ─────────────────────────────────────────────────────

    def mousePressEvent(self, event) -> None:
        self._mouse_last = event.pos()

    def mouseMoveEvent(self, event) -> None:
        dx = event.x() - self._mouse_last.x()
        dy = event.y() - self._mouse_last.y()
        if event.buttons() & Qt.LeftButton:
            self._rot_z += dx * 0.5
            self._rot_x += dy * 0.5
            self._rot_x = max(-90, min(90, self._rot_x))
        self._mouse_last = event.pos()
        self.update()

    def wheelEvent(self, event) -> None:
        delta = event.angleDelta().y()
        if delta > 0:
            self._zoom = min(5.0, self._zoom * 1.1)
        else:
            self._zoom = max(0.2, self._zoom / 1.1)
        self.update()


# ─────────────────────────────────────────────────────────────────────────────
# ASRACSDisplay
# ─────────────────────────────────────────────────────────────────────────────

class ASRACSDisplay(QWidget):
    """
    Draws OR Tambo surface layout and moving ground targets.
    All painter coordinates cast to int().
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self._targets: List[GroundTarget] = []
        self._alerts: List[IncursionAlert] = []
        self.setMinimumSize(500, 400)
        self.setStyleSheet('background: #0a0a0a;')

    def set_targets(self, targets: List[GroundTarget]) -> None:
        self._targets = list(targets)
        self.update()

    def set_alerts(self, alerts: List[IncursionAlert]) -> None:
        self._alerts = list(alerts)
        self.update()

    def _to_pix(self, nx: float, ny: float) -> Tuple[int, int]:
        return int(nx * self.width()), int(ny * self.height())

    def paintEvent(self, event) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        w = self.width()
        h = self.height()

        # Background – airfield green
        painter.fillRect(0, 0, w, h, QColor(10, 30, 10))

        # ── Runways ───────────────────────────────────────────────────────
        painter.setPen(QPen(QColor(160, 160, 160), 3))
        painter.setBrush(QBrush(QColor(60, 60, 60)))

        # Runway 03L/21R  (left pair)
        rwy_pts = QPolygon([
            QPoint(*self._to_pix(0.28, 0.18)),
            QPoint(*self._to_pix(0.36, 0.18)),
            QPoint(*self._to_pix(0.36, 0.82)),
            QPoint(*self._to_pix(0.28, 0.82)),
        ])
        painter.drawPolygon(rwy_pts)

        # Runway 03R/21L  (right pair)
        rwy_pts2 = QPolygon([
            QPoint(*self._to_pix(0.48, 0.18)),
            QPoint(*self._to_pix(0.56, 0.18)),
            QPoint(*self._to_pix(0.56, 0.82)),
            QPoint(*self._to_pix(0.48, 0.82)),
        ])
        painter.drawPolygon(rwy_pts2)

        # ── Taxiways ──────────────────────────────────────────────────────
        painter.setPen(QPen(QColor(120, 120, 80), 2))
        ax0, ay0 = self._to_pix(0.20, 0.50)
        ax1, ay1 = self._to_pix(0.80, 0.50)
        painter.drawLine(int(ax0), int(ay0), int(ax1), int(ay1))
        bx0, by0 = self._to_pix(0.65, 0.20)
        bx1, by1 = self._to_pix(0.65, 0.80)
        painter.drawLine(int(bx0), int(by0), int(bx1), int(by1))

        # ── Apron ─────────────────────────────────────────────────────────
        apron_x, apron_y = self._to_pix(0.67, 0.42)
        painter.setBrush(QBrush(QColor(50, 50, 50)))
        painter.setPen(QPen(QColor(100, 100, 100), 1))
        painter.drawRect(int(apron_x), int(apron_y), int(w * 0.22), int(h * 0.20))

        # ── Runway incursion zones (red tint) ─────────────────────────────
        alert_ids = {a.target_id for a in self._alerts}
        painter.setBrush(QBrush(QColor(180, 0, 0, 40)))
        painter.setPen(Qt.NoPen)
        painter.drawRect(0, 0, w, int(h * 0.35))
        painter.drawRect(0, int(h * 0.65), w, int(h * 0.35))

        # ── Labels ────────────────────────────────────────────────────────
        painter.setPen(QPen(QColor(255, 255, 200)))
        painter.setFont(QFont('Monospace', 8))
        lx, ly = self._to_pix(0.29, 0.10)
        painter.drawText(int(lx), int(ly), 40, 12, Qt.AlignCenter, '03L/21R')
        lx2, ly2 = self._to_pix(0.49, 0.10)
        painter.drawText(int(lx2), int(ly2), 40, 12, Qt.AlignCenter, '03R/21L')

        # ── Ground targets ────────────────────────────────────────────────
        for t in self._targets:
            tx, ty = self._to_pix(t.x, t.y)
            in_alert = t.target_id in alert_ids
            if in_alert:
                sev = next((a.severity for a in self._alerts if a.target_id == t.target_id), 'CAUTION')
                if sev == 'CRITICAL':
                    colour = QColor(255, 0, 0)
                elif sev == 'WARNING':
                    colour = QColor(255, 140, 0)
                else:
                    colour = QColor(255, 255, 0)
            else:
                colour = QColor(0, 200, 255) if t.target_type == 'aircraft' else QColor(0, 255, 120)

            painter.setPen(QPen(colour, 2))
            painter.setBrush(QBrush(colour))
            if t.target_type == 'aircraft':
                # Triangle pointing in heading direction
                hdg_r = math.radians(t.heading)
                size = 8
                pts = QPolygon([
                    QPoint(int(tx + size * math.sin(hdg_r)),
                           int(ty - size * math.cos(hdg_r))),
                    QPoint(int(tx + size * 0.5 * math.sin(hdg_r + math.pi * 0.75)),
                           int(ty - size * 0.5 * math.cos(hdg_r + math.pi * 0.75))),
                    QPoint(int(tx + size * 0.5 * math.sin(hdg_r - math.pi * 0.75)),
                           int(ty - size * 0.5 * math.cos(hdg_r - math.pi * 0.75))),
                ])
                painter.drawPolygon(pts)
            else:
                painter.drawRect(int(tx - 4), int(ty - 4), 8, 8)

            painter.setPen(QPen(colour))
            painter.setFont(QFont('Monospace', 6))
            painter.drawText(int(tx + 6), int(ty - 3), 50, 12, Qt.AlignLeft, t.callsign)

        # ── Airport name ──────────────────────────────────────────────────
        painter.setPen(QPen(QColor(220, 220, 220)))
        painter.setFont(QFont('Monospace', 9, QFont.Bold))
        painter.drawText(4, 4, w - 8, 16, Qt.AlignCenter, 'OR Tambo International (FAJS)')

        painter.end()


# ─────────────────────────────────────────────────────────────────────────────
# ASRACSAlertPanel
# ─────────────────────────────────────────────────────────────────────────────

class ASRACSAlertPanel(QWidget):
    """Shows active runway incursion alerts with colour-coded severity."""

    ack_requested = pyqtSignal(str)  # target_id

    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(2)

        title = QLabel('⚠  INCURSION ALERTS')
        title.setStyleSheet('color: #ff4444; font-weight: bold; font-size: 11px;')
        layout.addWidget(title)

        self._table = QTableWidget(0, 4)
        self._table.setHorizontalHeaderLabels(['ID', 'Severity', 'Message', 'Ack'])
        self._table.horizontalHeader().setSectionResizeMode(2, QHeaderView.Stretch)
        self._table.setSelectionBehavior(QTableWidget.SelectRows)
        self._table.setStyleSheet(
            'background: #0a0a0a; color: #cccccc; gridline-color: #333;'
            'QHeaderView::section { background: #1a1a2e; color: #aaaaaa; }'
        )
        layout.addWidget(self._table)

        self.setStyleSheet('background: #0a0a0a;')

    def update_alerts(self, alerts: List[IncursionAlert]) -> None:
        self._table.setRowCount(len(alerts))
        sev_colors = {'CRITICAL': '#ff2222', 'WARNING': '#ff8c00', 'CAUTION': '#ffff00'}
        for row, alert in enumerate(alerts):
            self._table.setItem(row, 0, QTableWidgetItem(alert.target_id))
            sev_item = QTableWidgetItem(alert.severity)
            sev_item.setForeground(QBrush(QColor(sev_colors.get(alert.severity, '#ffffff'))))
            self._table.setItem(row, 1, sev_item)
            self._table.setItem(row, 2, QTableWidgetItem(alert.message))
            ack_btn = QPushButton('ACK')
            ack_btn.setFixedHeight(20)
            tid = alert.target_id
            ack_btn.clicked.connect(lambda _, t=tid: self.ack_requested.emit(t))
            self._table.setCellWidget(row, 3, ack_btn)


# ─────────────────────────────────────────────────────────────────────────────
# SAAFOverviewMap  – national map of all 10 SAAF bases
# ─────────────────────────────────────────────────────────────────────────────

class SAAFOverviewMap(QWidget):
    """
    Draws a simplified South Africa outline with SAAF base positions.
    All coordinates passed to painter are cast to int().
    """

    base_selected = pyqtSignal(str)  # ICAO code

    # Approximate SA outline (normalised lon/lat offsets within 16°E-34°E, 34°S-22°S)
    _SA_OUTLINE = [
        (16.5, -29.3), (17.9, -32.7), (18.5, -34.4), (19.9, -34.8),
        (21.8, -34.5), (23.6, -33.7), (25.6, -33.8), (27.4, -33.2),
        (29.9, -31.0), (31.3, -29.9), (32.9, -28.0), (32.9, -26.8),
        (32.0, -25.6), (31.2, -25.7), (30.3, -22.4), (28.5, -22.2),
        (27.0, -22.9), (25.9, -23.0), (22.2, -22.3), (20.0, -22.8),
        (19.0, -24.8), (17.4, -28.9), (16.5, -29.3),
    ]

    LON_MIN, LON_MAX = 15.5, 33.5
    LAT_MIN, LAT_MAX = -35.5, -21.5

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(420, 300)
        self._selected: Optional[str] = None
        self._hover: Optional[str] = None
        self.setMouseTracking(True)

    def _geo_to_pix(self, lon: float, lat: float) -> Tuple[int, int]:
        w, h = self.width(), self.height()
        margin = 20
        x = int(margin + (lon - self.LON_MIN) / (self.LON_MAX - self.LON_MIN) * (w - 2 * margin))
        y = int(margin + (self.LAT_MAX - lat) / (self.LAT_MAX - self.LAT_MIN) * (h - 2 * margin))
        return x, y

    def paintEvent(self, event) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        w = self.width()
        h = self.height()

        # Ocean background
        painter.fillRect(0, 0, w, h, QColor(10, 30, 60))

        # SA outline
        outline_pts = QPolygon([
            QPoint(*self._geo_to_pix(lon, lat)) for lon, lat in self._SA_OUTLINE
        ])
        painter.setBrush(QBrush(QColor(40, 80, 40)))
        painter.setPen(QPen(QColor(100, 160, 80), 1))
        painter.drawPolygon(outline_pts)

        # Grid lines
        painter.setPen(QPen(QColor(50, 70, 90), 1, Qt.DotLine))
        for lon in range(17, 34, 3):
            x0, y0 = self._geo_to_pix(lon, self.LAT_MAX)
            x1, y1 = self._geo_to_pix(lon, self.LAT_MIN)
            painter.drawLine(int(x0), int(y0), int(x1), int(y1))
        for lat in range(-34, -21, 3):
            x0, y0 = self._geo_to_pix(self.LON_MIN, lat)
            x1, y1 = self._geo_to_pix(self.LON_MAX, lat)
            painter.drawLine(int(x0), int(y0), int(x1), int(y1))

        # SAAF bases
        for icao, info in SAAF_BASES.items():
            bx, by = self._geo_to_pix(info['lon'], info['lat'])
            selected = (icao == self._selected)
            hover = (icao == self._hover)
            if selected:
                colour = QColor(255, 220, 0)
                size = 10
            elif hover:
                colour = QColor(200, 220, 255)
                size = 8
            else:
                colour = QColor(0, 200, 255) if info['ils_available'] else QColor(200, 140, 0)
                size = 7
            painter.setPen(QPen(colour, 2))
            painter.setBrush(QBrush(colour))
            painter.drawEllipse(int(bx - size // 2), int(by - size // 2), size, size)

            painter.setFont(QFont('Monospace', 7, QFont.Bold if selected else QFont.Normal))
            painter.setPen(QPen(QColor(255, 255, 200)))
            painter.drawText(int(bx + size), int(by - 4), 60, 14, Qt.AlignLeft,
                             f"{icao}\n{info['vor_ident']}")

        # Legend
        painter.setFont(QFont('Monospace', 7))
        painter.setPen(QPen(QColor(0, 200, 255)))
        painter.drawText(4, h - 30, 120, 12, Qt.AlignLeft, '● ILS equipped')
        painter.setPen(QPen(QColor(200, 140, 0)))
        painter.drawText(4, h - 16, 120, 12, Qt.AlignLeft, '● VOR only')

        painter.end()

    def mouseMoveEvent(self, event) -> None:
        self._hover = self._hit_test(event.x(), event.y())
        self.update()

    def mousePressEvent(self, event) -> None:
        hit = self._hit_test(event.x(), event.y())
        if hit:
            self._selected = hit
            self.base_selected.emit(hit)
            self.update()

    def _hit_test(self, mx: int, my: int) -> Optional[str]:
        for icao, info in SAAF_BASES.items():
            bx, by = self._geo_to_pix(info['lon'], info['lat'])
            if abs(mx - bx) < 12 and abs(my - by) < 12:
                return icao
        return None


# ─────────────────────────────────────────────────────────────────────────────
# SAAFBaseMapWidget  – individual base schematic
# ─────────────────────────────────────────────────────────────────────────────

class SAAFBaseMapWidget(QWidget):
    """
    Draws a simple runway schematic for a selected SAAF base.
    All painter coords cast to int().
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self._base_info: Optional[Dict] = None
        self.setMinimumSize(200, 160)

    def set_base(self, info: Optional[Dict]) -> None:
        self._base_info = info
        self.update()

    def paintEvent(self, event) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        w = self.width()
        h = self.height()

        painter.fillRect(0, 0, w, h, QColor(15, 35, 15))

        if not self._base_info:
            painter.setPen(QPen(QColor(120, 120, 120)))
            painter.drawText(0, 0, w, h, Qt.AlignCenter, 'Select a base')
            painter.end()
            return

        info = self._base_info
        # Parse runway designation
        rwy = info.get('runway', '00/18')
        parts = rwy.split('/')
        rwy_num = int(parts[0]) if parts[0].isdigit() else 0
        rwy_angle = rwy_num * 10  # degrees magnetic

        cx = int(w / 2)
        cy = int(h / 2)
        rwy_len = int(min(w, h) * 0.45)
        rwy_width = 8

        rad = math.radians(rwy_angle)
        dx = int(rwy_len * math.sin(rad))
        dy = int(rwy_len * math.cos(rad))

        # Runway rectangle (simplified as a line with thickness)
        painter.setPen(QPen(QColor(140, 140, 140), rwy_width))
        painter.drawLine(int(cx - dx), int(cy + dy), int(cx + dx), int(cy - dy))

        # Threshold marks
        painter.setPen(QPen(QColor(255, 255, 255), 2))
        painter.drawLine(int(cx - dx - 8), int(cy + dy), int(cx - dx + 8), int(cy + dy))
        painter.drawLine(int(cx + dx - 8), int(cy - dy), int(cx + dx + 8), int(cy - dy))

        # ILS indicator
        if info.get('ils_available'):
            painter.setPen(QPen(QColor(0, 200, 255), 1))
            painter.setFont(QFont('Monospace', 7))
            ils_rwy = info.get('ils_runway', '?')
            painter.drawText(cx + dx + 5, cy - dy - 4, 40, 12, Qt.AlignLeft,
                             f'ILS {ils_rwy}')

        # Runway labels
        painter.setPen(QPen(QColor(255, 255, 200)))
        painter.setFont(QFont('Monospace', 8, QFont.Bold))
        label_offset = rwy_len + 14
        painter.drawText(int(cx - dx - label_offset // 2 - 10),
                         int(cy + dy - 6), 24, 14,
                         Qt.AlignCenter, parts[0].zfill(2))
        if len(parts) > 1:
            painter.drawText(int(cx + dx - 4),
                             int(cy - dy - 8), 24, 14,
                             Qt.AlignCenter, parts[1].zfill(2))

        # Base name
        painter.setFont(QFont('Monospace', 8))
        painter.setPen(QPen(QColor(200, 220, 200)))
        painter.drawText(0, 4, w, 14, Qt.AlignCenter, info.get('name', ''))

        painter.end()


# ─────────────────────────────────────────────────────────────────────────────
# SAAFBaseInfoWidget
# ─────────────────────────────────────────────────────────────────────────────

class SAAFBaseInfoWidget(QWidget):
    """Displays detailed text information for a selected SAAF base."""

    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(6, 6, 6, 6)
        self._label = QLabel('Select a base on the map')
        self._label.setWordWrap(True)
        self._label.setStyleSheet('color: #cccccc; font-family: Monospace; font-size: 10px;')
        layout.addWidget(self._label)
        self.setStyleSheet('background: #0a0a0a;')

    def set_base(self, icao: str, info: Optional[Dict]) -> None:
        if not info:
            self._label.setText('No information available')
            return
        ils_str = f"ILS RWY {info['ils_runway']}" if info.get('ils_available') else 'VOR only'
        text = (
            f"<b>{info['name']} ({icao})</b><br>"
            f"City: {info.get('city', '')}<br>"
            f"Runway: {info.get('runway', '')}<br>"
            f"VOR: {info.get('vor_ident', '')} {info.get('vor_freq', ''):.2f} MHz<br>"
            f"Navaid: {ils_str}<br>"
            f"Elevation: {info.get('elevation', 0)} ft<br>"
            f"Squadron: {info.get('squadron', '')}<br>"
            f"Lat: {info.get('lat', 0):.4f}°  Lon: {info.get('lon', 0):.4f}°"
        )
        self._label.setText(text)


# ─────────────────────────────────────────────────────────────────────────────
# VORAirportMonitorApp  – Main application window
# ─────────────────────────────────────────────────────────────────────────────

class VORAirportMonitorApp(QMainWindow):
    """
    Main window: 8-tab monitoring interface for VOR/ILS/ASRACS/terrain.
    """

    def __init__(self):
        super().__init__()
        self.setWindowTitle('VOR / ASRACS / SAAF Monitoring System  v5.3')
        self.setMinimumSize(1100, 700)

        # Config & state
        self._config = VORAirportConfig()
        self._handler = VORConnectionHandler()
        self._processor = VORDataProcessor()
        self._tracker = AircraftTracker()
        self._sim_engine = SimulationEngine()
        self._sim_engine.set_tracker(self._tracker)
        self._asracs_engine = ASRACSSimEngine()
        self._mock_server = MockVORTCPServer()
        self._daq_thread: Optional[DataAcquisitionThread] = None
        self._lda_parser: Optional[LDAFileParser] = None
        self._slope_analyzer = SurfaceSlopeAnalyzer()
        self._gs_detector = GlideSlopeDetector()
        self._current_vor_id: str = list(VOR_STATIONS.keys())[0]
        self._current_base_icao: Optional[str] = None

        # Config YAML path
        self._config_path = Path('vor_config.yaml')

        self._load_config()
        self._build_ui()
        self._setup_timers()
        self._populate_vor_combo()

        logger.info('Application started  v5.3')

    # ──────────────────────────────────────────────────────────────────────────
    # Config load / save
    # ──────────────────────────────────────────────────────────────────────────

    def _load_config(self) -> None:
        try:
            if self._config_path.exists():
                with open(self._config_path) as fh:
                    yaml.safe_load(fh)
                logger.info('Config loaded from %s', self._config_path)
            else:
                self._save_config()
        except Exception as exc:
            logger.error('Config load error: %s', exc)

    def _save_config(self) -> None:
        try:
            data = {'airports': {}, 'vor_stations': {}}
            for k, v in VOR_STATIONS.items():
                data['vor_stations'][k] = {'name': v['name'], 'ident': v['ident'],
                                            'freq': v['freq']}
            with open(self._config_path, 'w') as fh:
                yaml.dump(data, fh, default_flow_style=False)
        except Exception as exc:
            logger.error('Config save error: %s', exc)

    # ──────────────────────────────────────────────────────────────────────────
    # UI construction
    # ──────────────────────────────────────────────────────────────────────────

    def _build_ui(self) -> None:
        self._create_menu()
        self._create_toolbar()
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
        self._status_bar = QStatusBar()
        self.setStatusBar(self._status_bar)
        self._status_bar.showMessage('Ready  –  VOR/ASRACS/SAAF Monitor v5.3')

    def _create_menu(self) -> None:
        mb = self.menuBar()
        file_menu = mb.addMenu('&File')
        act_import = QAction('Import LDA File…', self)
        act_import.triggered.connect(self._import_lda)
        file_menu.addAction(act_import)
        file_menu.addSeparator()
        act_quit = QAction('&Quit', self)
        act_quit.triggered.connect(self.close)
        file_menu.addAction(act_quit)

        help_menu = mb.addMenu('&Help')
        act_about = QAction('About', self)
        act_about.triggered.connect(self._show_about)
        help_menu.addAction(act_about)

    def _create_toolbar(self) -> None:
        tb = QToolBar('Main Toolbar')
        self.addToolBar(tb)
        act_sim = QAction('▶ Start Sim', self)
        act_sim.triggered.connect(self._toggle_simulation)
        tb.addAction(act_sim)
        self._act_sim = act_sim
        act_asracs = QAction('▶ Start ASRACS', self)
        act_asracs.triggered.connect(self._toggle_asracs)
        tb.addAction(act_asracs)
        self._act_asracs = act_asracs

    # ── Tab: VOR Monitor ──────────────────────────────────────────────────────

    def _tab_vor(self) -> None:
        w = QWidget()
        layout = QVBoxLayout(w)

        # Station selector
        top = QHBoxLayout()
        top.addWidget(QLabel('VOR Station:'))
        self.vor_combo = QComboBox()
        self.vor_combo.setMinimumWidth(280)
        self.vor_combo.currentIndexChanged.connect(self._on_vor_index_changed)
        top.addWidget(self.vor_combo)
        top.addStretch()
        layout.addLayout(top)

        # Main display split
        split = QSplitter(Qt.Horizontal)

        # Left – CDI + values
        left = QWidget()
        left_layout = QVBoxLayout(left)
        self.cdi_display = CDIDisplay()
        left_layout.addWidget(self.cdi_display)

        info_grp = QGroupBox('VOR Readout')
        info_layout = QGridLayout(info_grp)
        self._vor_labels: Dict[str, QLabel] = {}
        for row, (key, label) in enumerate([
            ('station', 'Station'), ('frequency', 'Frequency'),
            ('bearing', 'Bearing'), ('deviation', 'Deviation'),
            ('signal', 'Signal'), ('timestamp', 'Last Update'),
        ]):
            info_layout.addWidget(QLabel(label + ':'), row, 0)
            lbl = QLabel('—')
            lbl.setStyleSheet('color: #00cc44; font-family: Monospace;')
            info_layout.addWidget(lbl, row, 1)
            self._vor_labels[key] = lbl
        left_layout.addWidget(info_grp)
        split.addWidget(left)

        # Right – signal chart (or plain label if PyQtChart unavailable)
        right = QWidget()
        right_layout = QVBoxLayout(right)
        if HAS_CHARTS:
            self._vor_series = QLineSeries()
            chart = QChart()
            chart.addSeries(self._vor_series)
            chart.setTitle('Signal Strength History')
            chart.setBackgroundBrush(QBrush(QColor(15, 15, 30)))
            chart.setTitleBrush(QBrush(QColor(200, 200, 200)))
            ax_x = QValueAxis()
            ax_x.setRange(0, 100)
            ax_x.setLabelFormat('%d')
            ax_y = QValueAxis()
            ax_y.setRange(0, 100)
            ax_y.setTitleText('Signal (%)')
            chart.addAxis(ax_x, Qt.AlignBottom)
            chart.addAxis(ax_y, Qt.AlignLeft)
            self._vor_series.attachAxis(ax_x)
            self._vor_series.attachAxis(ax_y)
            chart_view = QChartView(chart)
            chart_view.setMinimumHeight(200)
            right_layout.addWidget(chart_view)
            self._vor_chart = chart
            self._vor_ax_x = ax_x
            self._vor_point_count = 0
        else:
            self._sig_log = QTextEdit()
            self._sig_log.setReadOnly(True)
            self._sig_log.setStyleSheet('background: #0a0a1a; color: #00cc44; font-family: Monospace;')
            right_layout.addWidget(QLabel('Signal Log:'))
            right_layout.addWidget(self._sig_log)

        split.addWidget(right)
        layout.addWidget(split)
        self._tabs.addTab(w, 'VOR Monitor')

    # ── Tab: Radar ────────────────────────────────────────────────────────────

    def _tab_radar(self) -> None:
        w = QWidget()
        layout = QHBoxLayout(w)

        left = QWidget()
        left_layout = QVBoxLayout(left)
        self.radar = RadarDisplay()
        left_layout.addWidget(self.radar)
        layout.addWidget(left, 3)

        # Aircraft table
        right = QWidget()
        right_layout = QVBoxLayout(right)
        right_layout.addWidget(QLabel('Tracked Aircraft:'))
        self._ac_table = QTableWidget(0, 7)
        self._ac_table.setHorizontalHeaderLabels(
            ['Callsign', 'Alt (ft)', 'Spd (kt)', 'Hdg', 'Lat', 'Lon', 'Type']
        )
        self._ac_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        self._ac_table.setStyleSheet(
            'background: #050510; color: #aaccaa; gridline-color: #1a1a2a;'
        )
        right_layout.addWidget(self._ac_table)

        # Range control
        rng_layout = QHBoxLayout()
        rng_layout.addWidget(QLabel('Range (nm):'))
        self._rng_spin = QSpinBox()
        self._rng_spin.setRange(20, 500)
        self._rng_spin.setValue(120)
        self._rng_spin.valueChanged.connect(lambda v: self.radar.set_range(v))
        rng_layout.addWidget(self._rng_spin)
        rng_layout.addStretch()
        right_layout.addLayout(rng_layout)
        layout.addWidget(right, 1)

        self._tabs.addTab(w, 'Aircraft Tracking')

    # ── Tab: Approach Guidance ────────────────────────────────────────────────

    def _tab_approach(self) -> None:
        w = QWidget()
        layout = QVBoxLayout(w)

        top = QHBoxLayout()
        top.addWidget(QLabel('Base:'))
        self._appr_base_combo = QComboBox()
        for icao, info in SAAF_BASES.items():
            self._appr_base_combo.addItem(f"{info['name']} ({icao})", icao)
        self._appr_base_combo.currentIndexChanged.connect(self._update_approach_runway_combo)
        top.addWidget(self._appr_base_combo)
        top.addWidget(QLabel('Runway:'))
        self._appr_rwy_combo = QComboBox()
        top.addWidget(self._appr_rwy_combo)
        top.addStretch()
        layout.addLayout(top)

        split = QSplitter(Qt.Horizontal)
        self.approach_display = ApproachGuidanceDisplay()
        split.addWidget(self.approach_display)

        info_grp = QGroupBox('Approach Data')
        info_layout = QGridLayout(info_grp)
        self._appr_labels: Dict[str, QLabel] = {}
        for row, (key, lbl_text) in enumerate([
            ('glideslope', 'Glide Slope Dev'),
            ('localizer', 'Localizer Dev'),
            ('distance', 'Distance'),
            ('altitude', 'Altitude'),
            ('req_rate', 'Req. Desc. Rate'),
            ('max_slope', 'Max Surface Slope'),
            ('avg_slope', 'Avg Surface Slope'),
        ]):
            info_layout.addWidget(QLabel(lbl_text + ':'), row, 0)
            lbl = QLabel('—')
            lbl.setStyleSheet('color: #00cc44; font-family: Monospace;')
            info_layout.addWidget(lbl, row, 1)
            self._appr_labels[key] = lbl
        split.addWidget(info_grp)
        layout.addWidget(split)

        self._update_approach_runway_combo()
        self._tabs.addTab(w, 'Approach Guidance')

    def _update_approach_runway_combo(self) -> None:
        icao = self._appr_base_combo.currentData()
        self._appr_rwy_combo.clear()
        if icao and icao in SAAF_BASES:
            info = SAAF_BASES[icao]
            for rwy in info.get('runway', '').split('/'):
                self._appr_rwy_combo.addItem(rwy)

    # ── Tab: Terrain 3-D ──────────────────────────────────────────────────────

    def _tab_terrain(self) -> None:
        w = QWidget()
        layout = QVBoxLayout(w)

        controls = QHBoxLayout()
        act_sim_terrain = QPushButton('▶ En-Route Sim')
        act_sim_terrain.clicked.connect(self._toggle_simulation)
        controls.addWidget(act_sim_terrain)
        controls.addStretch()
        layout.addLayout(controls)

        self.terrain_widget = Terrain3DWidget()
        layout.addWidget(self.terrain_widget)
        self._tabs.addTab(w, 'Terrain 3-D')

    # ── Tab: ASRACS Surface ───────────────────────────────────────────────────

    def _tab_asracs(self) -> None:
        w = QWidget()
        layout = QVBoxLayout(w)

        controls = QHBoxLayout()
        act_start = QPushButton('▶ Start ASRACS Sim')
        act_start.clicked.connect(self._toggle_asracs)
        controls.addWidget(act_start)
        controls.addStretch()
        layout.addLayout(controls)

        split = QSplitter(Qt.Vertical)
        self.asracs_display = ASRACSDisplay()
        split.addWidget(self.asracs_display)

        self.asracs_alerts = ASRACSAlertPanel()
        self.asracs_alerts.ack_requested.connect(self._ack_asracs_alert)
        split.addWidget(self.asracs_alerts)
        split.setSizes([300, 150])
        layout.addWidget(split)

        self._tabs.addTab(w, 'ASRACS Surface')

    # ── Tab: SAAF Bases ───────────────────────────────────────────────────────

    def _tab_saaf(self) -> None:
        w = QWidget()
        layout = QHBoxLayout(w)

        left = QWidget()
        left_layout = QVBoxLayout(left)
        self._saaf_map = SAAFOverviewMap()
        self._saaf_map.base_selected.connect(self._on_base_selected)
        left_layout.addWidget(self._saaf_map)
        layout.addWidget(left, 3)

        right = QWidget()
        right_layout = QVBoxLayout(right)
        self._base_map_widget = SAAFBaseMapWidget()
        right_layout.addWidget(self._base_map_widget)
        self._base_info_widget = SAAFBaseInfoWidget()
        right_layout.addWidget(self._base_info_widget)
        layout.addWidget(right, 2)

        self._tabs.addTab(w, 'SAAF Bases')

    # ── Tab: Connection ───────────────────────────────────────────────────────

    def _tab_connection(self) -> None:
        w = QWidget()
        layout = QVBoxLayout(w)

        # Serial group
        serial_grp = QGroupBox('Serial RS-232')
        serial_layout = QGridLayout(serial_grp)
        serial_layout.addWidget(QLabel('Port:'), 0, 0)
        self._serial_port_combo = QComboBox()
        self._refresh_serial_ports()
        serial_layout.addWidget(self._serial_port_combo, 0, 1)
        serial_layout.addWidget(QLabel('Baud:'), 1, 0)
        self._baud_combo = QComboBox()
        for b in [1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200]:
            self._baud_combo.addItem(str(b), b)
        self._baud_combo.setCurrentText('9600')
        serial_layout.addWidget(self._baud_combo, 1, 1)
        btn_serial_connect = QPushButton('Connect Serial')
        btn_serial_connect.clicked.connect(self._connect_serial)
        serial_layout.addWidget(btn_serial_connect, 2, 0, 1, 2)
        layout.addWidget(serial_grp)

        # Mock server group
        mock_grp = QGroupBox('Mock VOR Server (Testing)')
        mock_layout = QHBoxLayout(mock_grp)
        self._mock_btn = QPushButton('Start Mock Server')
        self._mock_btn.clicked.connect(self._toggle_mock_server)
        mock_layout.addWidget(self._mock_btn)
        self._mock_status_lbl = QLabel('Stopped')
        self._mock_status_lbl.setStyleSheet('color: #ff4444;')
        mock_layout.addWidget(self._mock_status_lbl)
        layout.addWidget(mock_grp)

        # TCP group
        tcp_grp = QGroupBox('TCP/IP Connection')
        tcp_layout = QGridLayout(tcp_grp)
        tcp_layout.addWidget(QLabel('Host:'), 0, 0)
        self._tcp_host = QLineEdit('127.0.0.1')
        tcp_layout.addWidget(self._tcp_host, 0, 1)
        tcp_layout.addWidget(QLabel('Port:'), 1, 0)
        self._tcp_port = QSpinBox()
        self._tcp_port.setRange(1024, 65535)
        self._tcp_port.setValue(5005)
        tcp_layout.addWidget(self._tcp_port, 1, 1)
        btn_tcp = QPushButton('Connect TCP')
        btn_tcp.clicked.connect(self._connect_tcp)
        tcp_layout.addWidget(btn_tcp, 2, 0, 1, 2)
        btn_disconnect = QPushButton('Disconnect')
        btn_disconnect.clicked.connect(self._disconnect)
        tcp_layout.addWidget(btn_disconnect, 3, 0, 1, 2)
        layout.addWidget(tcp_grp)

        # Status
        conn_status_grp = QGroupBox('Connection Status')
        conn_status_layout = QHBoxLayout(conn_status_grp)
        self._conn_status_lbl = QLabel('Disconnected')
        self._conn_status_lbl.setStyleSheet('color: #ff4444; font-weight: bold;')
        conn_status_layout.addWidget(self._conn_status_lbl)
        layout.addWidget(conn_status_grp)
        layout.addStretch()

        # Command buttons
        cmd_grp = QGroupBox('Test Commands')
        cmd_layout = QHBoxLayout(cmd_grp)
        for cmd in ['Status', 'Ident', 'Health', 'Reset']:
            btn = QPushButton(cmd)
            btn.clicked.connect(lambda _, c=cmd: self._send_cmd(c))
            cmd_layout.addWidget(btn)
        layout.addWidget(cmd_grp)

        self._tabs.addTab(w, 'Connection')

    # ── Tab: Diagnostics ──────────────────────────────────────────────────────

    def _tab_diagnostics(self) -> None:
        w = QWidget()
        layout = QVBoxLayout(w)

        layout.addWidget(QLabel('System Diagnostics', font=QFont('Monospace', 10, QFont.Bold)))

        metrics_grp = QGroupBox('System Metrics')
        metrics_layout = QGridLayout(metrics_grp)
        self._diag_labels: Dict[str, QLabel] = {}
        for row, (key, lbl_text) in enumerate([
            ('aircraft_count', 'Tracked Aircraft'),
            ('vor_updates', 'VOR Updates/min'),
            ('connection', 'Connection'),
            ('sim_running', 'Simulation'),
            ('asracs_running', 'ASRACS'),
            ('alerts_active', 'Active Alerts'),
        ]):
            metrics_layout.addWidget(QLabel(lbl_text + ':'), row, 0)
            lbl = QLabel('—')
            lbl.setStyleSheet('color: #00cc44; font-family: Monospace;')
            metrics_layout.addWidget(lbl, row, 1)
            self._diag_labels[key] = lbl
        layout.addWidget(metrics_grp)

        layout.addWidget(QLabel('Event Log:'))
        self._diag_log = QTextEdit()
        self._diag_log.setReadOnly(True)
        self._diag_log.setStyleSheet('background: #050508; color: #88cc88; font-family: Monospace; font-size: 9px;')
        layout.addWidget(self._diag_log)

        layout.addStretch()
        self._tabs.addTab(w, 'Diagnostics')

    # ──────────────────────────────────────────────────────────────────────────
    # VOR combo population
    # ──────────────────────────────────────────────────────────────────────────

    def _populate_vor_combo(self) -> None:
        self.vor_combo.blockSignals(True)
        self.vor_combo.clear()
        for sid, sinfo in VOR_STATIONS.items():
            label = f"{sinfo['ident']}  {sinfo['freq']:.2f} MHz  –  {sinfo['name']}"
            self.vor_combo.addItem(label, sid)
        self.vor_combo.blockSignals(False)
        if self.vor_combo.count():
            self.vor_combo.setCurrentIndex(0)
            self._on_vor_index_changed(0)

    def _on_vor_index_changed(self, index: int) -> None:
        sid = self.vor_combo.currentData()
        if sid:
            self._current_vor_id = sid
        sinfo = VOR_STATIONS.get(self._current_vor_id, {})
        if hasattr(self, '_vor_labels'):
            self._vor_labels['station'].setText(
                f"{sinfo.get('ident', '?')}  ({self._current_vor_id})")
            self._vor_labels['frequency'].setText(f"{sinfo.get('freq', 0):.2f} MHz")
        if hasattr(self, 'radar'):
            lat = sinfo.get('lat', -26.14)
            lon = sinfo.get('lon', 28.25)
            self.radar.set_centre(lat, lon)

    # ──────────────────────────────────────────────────────────────────────────
    # Timers
    # ──────────────────────────────────────────────────────────────────────────

    def _setup_timers(self) -> None:
        self._vor_update_count = 0

        self._ui_timer = QTimer(self)
        self._ui_timer.timeout.connect(self._refresh_ui)
        self._ui_timer.start(1000)

        self._fast_timer = QTimer(self)
        self._fast_timer.timeout.connect(self._fast_refresh)
        self._fast_timer.start(200)

    def _fast_refresh(self) -> None:
        """Update radar and ASRACS display at ~5 Hz."""
        aircraft = self._tracker.get_all()
        if hasattr(self, 'radar'):
            self.radar.set_aircraft(aircraft)
        if hasattr(self, 'terrain_widget'):
            self.terrain_widget.set_aircraft(aircraft)
        if hasattr(self, 'asracs_display'):
            targets = self._asracs_engine.get_targets()
            alerts = self._asracs_engine.get_alerts()
            self.asracs_display.set_targets(targets)
            self.asracs_display.set_alerts(alerts)
            if hasattr(self, 'asracs_alerts'):
                self.asracs_alerts.update_alerts(alerts)

    def _refresh_ui(self) -> None:
        """Update 1-Hz displays: aircraft table, approach, diagnostics, VOR."""
        self._update_aircraft_table()
        self._update_approach_display()
        self._update_diagnostics()
        self._update_vor_synthetic()

    def _update_vor_synthetic(self) -> None:
        """If no real VOR data, show synthetic values for demo."""
        sinfo = VOR_STATIONS.get(self._current_vor_id, {})
        t = time.time()
        bearing = (t * 6) % 360
        deviation = math.sin(t * 0.3) * 2.0
        signal = 70 + math.sin(t * 0.07) * 15
        if hasattr(self, 'cdi_display'):
            self.cdi_display.set_data(deviation, bearing, signal > 20)
        if hasattr(self, '_vor_labels'):
            self._vor_labels['bearing'].setText(f'{bearing:.1f}°')
            self._vor_labels['deviation'].setText(f'{deviation:+.2f} dots')
            self._vor_labels['signal'].setText(f'{signal:.0f}%')
            self._vor_labels['timestamp'].setText(datetime.now().strftime('%H:%M:%S'))
        if HAS_CHARTS and hasattr(self, '_vor_series'):
            self._vor_series.append(float(self._vor_point_count), signal)
            self._vor_point_count += 1
            if self._vor_point_count > 100:
                self._vor_series.remove(0)
                self._vor_ax_x.setRange(self._vor_point_count - 100, self._vor_point_count)
            else:
                self._vor_ax_x.setRange(0, max(self._vor_point_count, 1))
        elif hasattr(self, '_sig_log'):
            ts = datetime.now().strftime('%H:%M:%S')
            self._sig_log.append(f'[{ts}] BRG={bearing:.1f}° DEV={deviation:+.2f} SIG={signal:.0f}%')

    def _update_aircraft_table(self) -> None:
        if not hasattr(self, '_ac_table'):
            return
        aircraft = self._tracker.get_all()
        self._ac_table.setRowCount(len(aircraft))
        for row, ac in enumerate(aircraft):
            self._ac_table.setItem(row, 0, QTableWidgetItem(ac.callsign))
            self._ac_table.setItem(row, 1, QTableWidgetItem(str(ac.altitude)))
            self._ac_table.setItem(row, 2, QTableWidgetItem(str(ac.speed)))
            self._ac_table.setItem(row, 3, QTableWidgetItem(f'{ac.heading:.0f}°'))
            self._ac_table.setItem(row, 4, QTableWidgetItem(f'{ac.lat:.3f}°'))
            self._ac_table.setItem(row, 5, QTableWidgetItem(f'{ac.lon:.3f}°'))
            self._ac_table.setItem(row, 6, QTableWidgetItem(ac.aircraft_type))

    def _update_approach_display(self) -> None:
        if not hasattr(self, 'approach_display'):
            return
        icao = self._appr_base_combo.currentData() if hasattr(self, '_appr_base_combo') else None
        if not icao:
            return
        binfo = SAAF_BASES.get(icao, {})
        t = time.time()
        dist = max(0.5, 15 - (t % 30) * 0.5)
        alt = int(dist * 318 + binfo.get('elevation', 0))
        gs_dev = self._gs_detector.check_glideslope(alt, dist)
        gs_dev += math.sin(t * 0.2) * 40
        loc_dev = math.sin(t * 0.15) * 0.8
        slope_info = self._slope_analyzer.analyze(binfo.get('lat', 0), binfo.get('lon', 0))
        req_rate = self._slope_analyzer.required_descent_rate(3.0, 140)
        self.approach_display.set_data(gs_dev, loc_dev, dist)
        if hasattr(self, '_appr_labels'):
            self._appr_labels['glideslope'].setText(f'{gs_dev:+.0f} ft')
            self._appr_labels['localizer'].setText(f'{loc_dev:+.2f} dot')
            self._appr_labels['distance'].setText(f'{dist:.1f} nm')
            self._appr_labels['altitude'].setText(f'{alt} ft MSL')
            self._appr_labels['req_rate'].setText(f'{req_rate} ft/min')
            self._appr_labels['max_slope'].setText(f"{slope_info['max_gradient']:.1f}%")
            self._appr_labels['avg_slope'].setText(f"{slope_info['avg_gradient']:.1f}%")

    def _update_diagnostics(self) -> None:
        if not hasattr(self, '_diag_labels'):
            return
        self._diag_labels['aircraft_count'].setText(str(self._tracker.count()))
        self._diag_labels['connection'].setText(
            f'Connected ({self._handler.mode})' if self._handler.connected else 'Disconnected'
        )
        sim_running = self._sim_engine._running
        self._diag_labels['sim_running'].setText('Running' if sim_running else 'Stopped')
        asracs_running = self._asracs_engine._running
        self._diag_labels['asracs_running'].setText('Running' if asracs_running else 'Stopped')
        alerts = self._asracs_engine.get_alerts()
        self._diag_labels['alerts_active'].setText(str(len(alerts)))

        ts = datetime.now().strftime('%H:%M:%S')
        self._diag_log.append(
            f'[{ts}] TRK={self._tracker.count()}  '
            f'SIM={"ON" if sim_running else "OFF"}  '
            f'ASRACS={"ON" if asracs_running else "OFF"}  '
            f'ALERTS={len(alerts)}'
        )

    # ──────────────────────────────────────────────────────────────────────────
    # Simulation toggle
    # ──────────────────────────────────────────────────────────────────────────

    def _toggle_simulation(self) -> None:
        if self._sim_engine._running:
            self._sim_engine.stop()
            self._act_sim.setText('▶ Start Sim')
            self._log_event('En-route simulation stopped')
        else:
            self._sim_engine.start()
            self._act_sim.setText('⏹ Stop Sim')
            self._log_event('En-route simulation started')

    def _toggle_asracs(self) -> None:
        if self._asracs_engine._running:
            self._asracs_engine.stop()
            self._act_asracs.setText('▶ Start ASRACS')
            self._log_event('ASRACS simulation stopped')
        else:
            self._asracs_engine.start()
            self._act_asracs.setText('⏹ Stop ASRACS')
            self._log_event('ASRACS simulation started')

    # ──────────────────────────────────────────────────────────────────────────
    # Connection actions
    # ──────────────────────────────────────────────────────────────────────────

    def _refresh_serial_ports(self) -> None:
        self._serial_port_combo.clear()
        if HAS_SERIAL:
            for p in serial.tools.list_ports.comports():
                self._serial_port_combo.addItem(p.device)
        if self._serial_port_combo.count() == 0:
            self._serial_port_combo.addItem('COM1')

    def _connect_serial(self) -> None:
        port = self._serial_port_combo.currentText()
        baud = self._baud_combo.currentData() or 9600
        if self._handler.connect_serial(port, baud):
            self._start_daq()
            self._conn_status_lbl.setText(f'Serial: {port} @ {baud}')
            self._conn_status_lbl.setStyleSheet('color: #00cc44; font-weight: bold;')
        else:
            QMessageBox.warning(self, 'Connection Failed', f'Could not connect to {port}')

    def _connect_tcp(self) -> None:
        host = self._tcp_host.text().strip() or '127.0.0.1'
        port = self._tcp_port.value()
        if self._handler.connect_tcp(host, port):
            self._start_daq()
            self._conn_status_lbl.setText(f'TCP: {host}:{port}')
            self._conn_status_lbl.setStyleSheet('color: #00cc44; font-weight: bold;')
        else:
            QMessageBox.warning(self, 'Connection Failed',
                                f'Could not connect to {host}:{port}')

    def _disconnect(self) -> None:
        if self._daq_thread:
            self._daq_thread.stop()
            self._daq_thread = None
        self._handler.disconnect()
        self._conn_status_lbl.setText('Disconnected')
        self._conn_status_lbl.setStyleSheet('color: #ff4444; font-weight: bold;')

    def _toggle_mock_server(self) -> None:
        if self._mock_server._running:
            self._mock_server.stop()
            self._mock_btn.setText('Start Mock Server')
            self._mock_status_lbl.setText('Stopped')
            self._mock_status_lbl.setStyleSheet('color: #ff4444;')
        else:
            if self._mock_server.start():
                self._mock_btn.setText('Stop Mock Server')
                self._mock_status_lbl.setText('Running on :5005')
                self._mock_status_lbl.setStyleSheet('color: #00cc44;')

    def _start_daq(self) -> None:
        if self._daq_thread:
            self._daq_thread.stop()
        self._daq_thread = DataAcquisitionThread(self._handler, self._processor, self)
        self._daq_thread.data_received.connect(self._on_vor_data)
        self._daq_thread.start()

    def _on_vor_data(self, data: VORData) -> None:
        self._vor_update_count += 1
        if hasattr(self, 'cdi_display'):
            self.cdi_display.set_data(data.deviation, data.bearing, data.signal_strength > 20)
        if hasattr(self, '_vor_labels'):
            self._vor_labels['bearing'].setText(f'{data.bearing:.1f}°')
            self._vor_labels['deviation'].setText(f'{data.deviation:+.2f} dots')
            self._vor_labels['signal'].setText(f'{data.signal_strength:.0f}%')
            self._vor_labels['timestamp'].setText(datetime.now().strftime('%H:%M:%S'))

    def _send_cmd(self, cmd: str) -> None:
        logger.info('Test command: %s', cmd)
        self._log_event(f'CMD: {cmd}')

    # ──────────────────────────────────────────────────────────────────────────
    # SAAF base selection
    # ──────────────────────────────────────────────────────────────────────────

    def _on_base_selected(self, icao: str) -> None:
        self._current_base_icao = icao
        info = SAAF_BASES.get(icao)
        if hasattr(self, '_base_map_widget'):
            self._base_map_widget.set_base(info)
        if hasattr(self, '_base_info_widget'):
            self._base_info_widget.set_base(icao, info)
        self._log_event(f'Base selected: {icao} – {info["name"] if info else "?"}')

    # ──────────────────────────────────────────────────────────────────────────
    # ASRACS alert acknowledge
    # ──────────────────────────────────────────────────────────────────────────

    def _ack_asracs_alert(self, target_id: str) -> None:
        self._asracs_engine.acknowledge_alert(target_id)
        self._log_event(f'Alert acknowledged: {target_id}')

    # ──────────────────────────────────────────────────────────────────────────
    # LDA import
    # ──────────────────────────────────────────────────────────────────────────

    def _import_lda(self) -> None:
        path, _ = QFileDialog.getOpenFileName(
            self, 'Import LDA File', '', 'LDA Files (*.lda);;All Files (*)'
        )
        if not path:
            return
        parser = LDAFileParser(path)
        if parser.parse():
            self._lda_parser = parser
            info = (
                f"LDA file imported successfully!\n\n"
                f"Station Type: {parser.get_station_type()}\n"
                f"Localizer Freq: {parser.get_localizer_frequency():.2f} MHz\n"
                f"Glide Slope Freq: {parser.get_glideslope_frequency():.2f} MHz\n"
                f"Waveforms: {', '.join(parser.get_waveform_names())}\n"
                f"Nominal DDM: {parser.get_nominal_values().get('crs_ddm', '?')}"
            )
            QMessageBox.information(self, 'LDA Import', info)
            self._log_event(f'LDA imported: {Path(path).name}')
        else:
            QMessageBox.warning(self, 'LDA Import Failed',
                                'Could not parse the selected LDA file.')

    # ──────────────────────────────────────────────────────────────────────────
    # About dialog
    # ──────────────────────────────────────────────────────────────────────────

    def _show_about(self) -> None:
        QMessageBox.about(
            self,
            'About VOR Monitor v5.3',
            '<b>VOR / ASRACS / SAAF Airport Monitoring System v5.3</b><br><br>'
            'South African Air Force aviation monitoring system.<br>'
            'Monitors 10 SAAF bases and 3 civilian airports.<br><br>'
            '<b>Features:</b><br>'
            '• VOR/ILS station monitoring (13 total)<br>'
            '• Radar aircraft tracking (120 nm)<br>'
            '• Approach guidance (glideslope/localizer)<br>'
            '• 3-D terrain visualization (OpenGL)<br>'
            '• ASRACS surface movement<br>'
            '• LDA file import (Thales ILS config)<br><br>'
            'For support: github.com/schalkpieterse76-afk/VOR-Monitoring--System',
        )

    # ──────────────────────────────────────────────────────────────────────────
    # Utility
    # ──────────────────────────────────────────────────────────────────────────

    def _log_event(self, msg: str) -> None:
        ts = datetime.now().strftime('%H:%M:%S')
        entry = f'[{ts}] {msg}'
        logger.info(msg)
        if hasattr(self, '_diag_log'):
            self._diag_log.append(entry)
        if hasattr(self, '_status_bar'):
            self._status_bar.showMessage(msg, 5000)

    # ──────────────────────────────────────────────────────────────────────────
    # Cleanup
    # ──────────────────────────────────────────────────────────────────────────

    def closeEvent(self, event) -> None:
        self._ui_timer.stop()
        self._fast_timer.stop()
        if self._daq_thread:
            self._daq_thread.stop()
        self._sim_engine.stop()
        self._asracs_engine.stop()
        self._mock_server.stop()
        self._handler.disconnect()
        logger.info('Application closing')
        event.accept()



# ─────────────────────────────────────────────────────────────────────────────
# Self-test (headless CLI validation)
# ─────────────────────────────────────────────────────────────────────────────

def _self_test() -> int:
    """
    Headless validation. Returns 0 on success, 1 on failure.
    Run with:  python CVOR1.py --self-test
    """
    import traceback
    errors: List[str] = []
    passed = 0

    def check(name: str, fn) -> None:
        nonlocal passed
        try:
            fn()
            print(f'  PASS  {name}')
            passed += 1
        except Exception as exc:
            msg = f'{name}: {exc}\n{traceback.format_exc(limit=3)}'
            errors.append(msg)
            print(f'  FAIL  {msg}')

    print('=== CVOR1 Self-Test ===')

    check('VORData creation', lambda: VORData(station_id='WKV', frequency=116.9))
    check('AircraftData creation', lambda: AircraftData(callsign='SAA101'))
    check('GroundTarget creation', lambda: GroundTarget(target_id='AC1'))
    check('IncursionAlert creation', lambda: IncursionAlert(target_id='AC1', severity='WARNING'))

    check('GlideSlopeDetector basic', lambda: (
        lambda gs: (
            assert_close(gs.ideal_altitude_ft(10), 3188, tol=50),
            assert_close(gs.check_glideslope(3188, 10), 0, tol=100),
        )
    )(GlideSlopeDetector()))

    check('SurfaceSlopeAnalyzer', lambda: (
        lambda r: (
            assert_('max_gradient' in r),
            assert_('avg_gradient' in r),
        )
    )(SurfaceSlopeAnalyzer().analyze(-25.83, 28.22)))

    check('gc_distance_nm JNB-CPT', lambda: (
        lambda d: assert_(400 < d < 900, f'unexpected distance {d}')
    )(gc_distance_nm(-26.14, 28.25, -33.96, 18.60)))

    check('bearing_to north', lambda: (
        lambda b: assert_(b < 5 or b > 355, f'expected ~0° got {b}')
    )(bearing_to(0, 0, 1, 0)))

    check('VORDataProcessor parse valid', lambda: (
        lambda p: assert_(p.process('$VOR,180.0,-0.5,75.0,WKV,116.90*') is not None)
    )(VORDataProcessor()))

    check('VORDataProcessor parse invalid', lambda: (
        lambda p: assert_(p.process('GARBAGE') is None)
    )(VORDataProcessor()))

    check('AircraftTracker update+count', lambda: (
        lambda t: (
            t.update(AircraftData(callsign='TST1', lat=-26.0, lon=28.0)),
            assert_(t.count() == 1, f'expected 1 got {t.count()}'),
        )
    )(AircraftTracker()))

    check('VORAirportConfig station_ids', lambda: (
        lambda c: assert_(len(c.station_ids()) >= 13, f'got {len(c.station_ids())}')
    )(VORAirportConfig()))

    check('VORAirportConfig base_icaos', lambda: (
        lambda c: assert_(len(c.base_icaos()) >= 10, f'got {len(c.base_icaos())}')
    )(VORAirportConfig()))

    check('VOR_STATIONS count', lambda: assert_(
        len(VOR_STATIONS) >= 13, f'only {len(VOR_STATIONS)} stations'
    ))

    check('SAAF_BASES count', lambda: assert_(
        len(SAAF_BASES) >= 10, f'only {len(SAAF_BASES)} bases'
    ))

    check('SAAF_BASES all have VOR freq', lambda: [
        assert_(108.0 <= b['vor_freq'] <= 117.95, f"{icao}: freq {b['vor_freq']}")
        for icao, b in SAAF_BASES.items()
    ])

    check('SimulationEngine init', lambda: (
        lambda e: assert_(len(e.get_aircraft()) > 0)
    )(SimulationEngine(count=3)))

    check('ASRACSSimEngine init', lambda: (
        lambda e: assert_(len(e.get_targets()) > 0)
    )(ASRACSSimEngine()))

    check('project_position round-trip', lambda: (
        lambda lat2, lon2: assert_(
            abs(lat2 - (-26.14)) < 5 and abs(lon2 - 28.25) < 5
        )
    )(*project_position(-26.14, 28.25, 0, 0)))

    print()
    if errors:
        print(f'RESULT: {passed} passed, {len(errors)} FAILED')
        for e in errors:
            print(e)
        return 1
    print(f'RESULT: {passed} passed, 0 failed  ✓ ALL TESTS PASSED')
    return 0


def assert_(cond: bool, msg: str = '') -> None:
    if not cond:
        raise AssertionError(msg or 'assertion failed')


def assert_close(a: float, b: float, tol: float = 1e-6) -> None:
    if abs(a - b) > tol:
        raise AssertionError(f'{a} != {b} (tol={tol})')


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────

def main() -> None:
    if '--self-test' in sys.argv:
        sys.exit(_self_test())

    app = QApplication(sys.argv)
    app.setApplicationName('VOR Monitor')
    app.setApplicationVersion('5.3')
    app.setStyle('Fusion')

    # Dark palette
    palette = QPalette()
    palette.setColor(QPalette.Window,        QColor(30, 30, 50))
    palette.setColor(QPalette.WindowText,    QColor(200, 200, 200))
    palette.setColor(QPalette.Base,          QColor(15, 15, 30))
    palette.setColor(QPalette.AlternateBase, QColor(20, 20, 40))
    palette.setColor(QPalette.Text,          QColor(200, 200, 200))
    palette.setColor(QPalette.Button,        QColor(40, 40, 65))
    palette.setColor(QPalette.ButtonText,    QColor(200, 200, 200))
    palette.setColor(QPalette.Highlight,     QColor(0, 120, 80))
    palette.setColor(QPalette.HighlightedText, QColor(255, 255, 255))
    app.setPalette(palette)

    window = VORAirportMonitorApp()
    window.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
