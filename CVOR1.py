#!/usr/bin/env python3
"""
VOR / ASRACS / SAAF Airport Monitoring System  v5.3
=====================================================
v5.3 Enhancements:
  - NEW: LDA Data Viewer tab – searchable/filterable table of imported LDA records
  - NEW: Terrain Analysis tab – slope classification, safety assessment, 3-D profile
  - NEW: Cross-linked interactions – LDA record selection updates terrain analysis
  - NEW: CSV export combining LDA records with terrain analysis results
  - NEW: LDABinaryFileParser returns List[VORDataRecord] for structured display

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

import sys, logging, csv, socket, math, random, time
import numpy as np
from pathlib import Path
from datetime import datetime, timezone
from typing import List, Dict, Optional, Tuple
from collections import deque
from dataclasses import dataclass, field
from threading import Lock, Thread

# Optional imports – graceful fallback for headless / CI environments
try:
    import yaml
    _YAML_OK = True
except ImportError:
    _YAML_OK = False

try:
    import serial
    import serial.tools.list_ports
    _SERIAL_OK = True
except ImportError:
    _SERIAL_OK = False

_GUI_OK = False
try:
    from PyQt5.QtWidgets import (
        QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
        QTabWidget, QAction, QDialog, QLabel, QLineEdit, QPushButton,
        QGridLayout, QMessageBox, QTableWidget, QTableWidgetItem, QHeaderView,
        QComboBox, QGroupBox, QFileDialog, QTextEdit, QToolBar,
        QSplitter, QOpenGLWidget, QScrollArea, QListWidget, QListWidgetItem,
    )
    from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QThread, QPoint, QRectF, QPointF
    from PyQt5.QtGui import (
        QColor, QFont, QPainter, QPen, QBrush, QPolygon, QPalette,
        QPolygonF, QStandardItem, QIcon,
    )
    from PyQt5.QtChart import QChart, QChartView, QLineSeries
    _GUI_OK = True
except ImportError:
    pass

_GL_OK = False
if _GUI_OK:
    try:
        from OpenGL.GL import *
        from OpenGL.GLU import *
        _GL_OK = True
    except ImportError:
        pass

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('vor_monitor.log'),
        logging.StreamHandler(),
    ],
)
logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Constants & Configuration
# ---------------------------------------------------------------------------

APP_VERSION = "5.3"

# TERRAIN_LIBRARY: 15 sample altitude points (ft) per airport/base
TERRAIN_LIBRARY: Dict[str, List[float]] = {
    "JNB":  [5558, 5560, 5565, 5572, 5580, 5590, 5600, 5610, 5618, 5620, 5625, 5628, 5630, 5632, 5635],
    "CPT":  [148,  155,  165,  180,  210,  250,  300,  370,  450,  540,  620,  680,  720,  750,  780],
    "DUR":  [12,   15,   20,   28,   40,   55,   75,   100,  130,  165,  205,  250,  300,  355,  415],
    "FAWK": [4830, 4835, 4842, 4852, 4865, 4880, 4898, 4918, 4940, 4963, 4987, 5010, 5032, 5052, 5068],
    "FAOR": [5525, 5528, 5532, 5538, 5546, 5556, 5568, 5582, 5597, 5613, 5630, 5648, 5666, 5682, 5695],
    "FAGM": [4035, 4040, 4048, 4060, 4075, 4092, 4112, 4134, 4158, 4183, 4208, 4232, 4255, 4276, 4294],
    "FALA": [4938, 4942, 4948, 4957, 4969, 4983, 5000, 5018, 5038, 5059, 5081, 5102, 5122, 5140, 5156],
    "FABL": [4458, 4462, 4468, 4477, 4490, 4505, 4522, 4541, 4561, 4582, 4603, 4623, 4641, 4657, 4671],
    "FAPE": [188,  195,  205,  220,  240,  266,  296,  332,  372,  416,  464,  514,  566,  618,  670],
    "FAYP": [30,   36,   45,   57,   72,   90,   111,  135,  162,  192,  225,  261,  300,  341,  385],
    "FADN": [20,   25,   32,   42,   55,   72,   92,   116,  143,  174,  209,  247,  288,  332,  378],
    "FAHS": [4018, 4023, 4030, 4040, 4053, 4069, 4088, 4109, 4133, 4159, 4186, 4213, 4240, 4265, 4288],
}

# VOR Station definitions
VOR_STATIONS: List[Dict] = [
    {"code": "JHB",  "name": "OR Tambo Intl",      "freq": 114.90, "base": "JNB",  "vor": True,  "ils": True,  "gs": 3.0},
    {"code": "CTV",  "name": "Cape Town Intl",      "freq": 115.70, "base": "CPT",  "vor": True,  "ils": True,  "gs": 3.0},
    {"code": "DNV",  "name": "Durban King Shaka",   "freq": 112.50, "base": "DUR",  "vor": True,  "ils": True,  "gs": 3.0},
    {"code": "WKF",  "name": "AFB Waterkloof",      "freq": 111.10, "base": "FAWK", "vor": True,  "ils": True,  "gs": 3.0},
    {"code": "HLW",  "name": "AFB Hoedspruit",      "freq": 112.70, "base": "FAHS", "vor": True,  "ils": False, "gs": 3.0},
    {"code": "GMK",  "name": "AFB Makhado",         "freq": 109.80, "base": "FAGM", "vor": True,  "ils": True,  "gs": 3.0},
    {"code": "LAA",  "name": "Lanseria Intl",        "freq": 113.30, "base": "FALA", "vor": True,  "ils": True,  "gs": 3.0},
    {"code": "BLM",  "name": "AFB Bloemspruit",     "freq": 116.10, "base": "FABL", "vor": True,  "ils": False, "gs": 3.0},
    {"code": "PEV",  "name": "Port Elizabeth Intl",  "freq": 113.40, "base": "FAPE", "vor": True,  "ils": True,  "gs": 3.0},
    {"code": "YSP",  "name": "AFB Ysterplaat",      "freq": 115.70, "base": "FAYP", "vor": False, "ils": False, "gs": 3.0},
]

# ---------------------------------------------------------------------------
# Data Structures
# ---------------------------------------------------------------------------

@dataclass
class VORDataRecord:
    """Single VOR data sample, e.g. from an LDA archive or live feed."""
    timestamp: datetime
    station_key: str
    ident: str
    frequency_mhz: float
    bearing_deg: float
    deviation_deg: float
    signal_percent: float
    distance_nm: float


@dataclass
class VORData:
    """Parsed VOR telemetry frame."""
    station: str = ""
    bearing: float = 0.0
    signal_strength: float = 0.0
    timestamp: datetime = field(default_factory=lambda: datetime.now(timezone.utc))
    frequency: float = 0.0
    ident: str = ""
    deviation: float = 0.0
    distance_nm: float = 0.0


@dataclass
class AircraftData:
    """Simulated or tracked aircraft position."""
    callsign: str = ""
    bearing: float = 0.0
    distance_nm: float = 0.0
    altitude_ft: int = 0
    speed_kts: int = 0
    heading: float = 0.0
    x: float = 0.0
    y: float = 0.0


# ---------------------------------------------------------------------------
# Utility helpers
# ---------------------------------------------------------------------------

def _safe_float(s: str) -> Optional[float]:
    """Return float(s) or None if s cannot be parsed as a float."""
    try:
        return float(s)
    except (ValueError, TypeError):
        return None


# ---------------------------------------------------------------------------
# LDA File Parsers
# ---------------------------------------------------------------------------

class LDAFileParser:
    """
    Parse Thales/Rohde & Schwarz ILS .lda configuration files.
    Extracts station type, frequencies, waveform names, and nominal values.
    """

    def __init__(self, path: str) -> None:
        self._path = Path(path)
        self._station_type: str = ""
        self._localizer_freq: float = 0.0
        self._glideslope_freq: float = 0.0
        self._waveform_names: List[str] = []
        self._nominal_values: Dict[str, float] = {}
        self._parsed = False

    def parse(self) -> bool:
        try:
            raw = self._path.read_bytes()
            text = raw.decode("latin-1", errors="replace")
            self._parse_text_section(text)
            self._parse_binary_section(raw)
            self._parsed = True
            return True
        except Exception as exc:
            logger.warning("LDAFileParser: %s", exc)
            return False

    def _parse_text_section(self, text: str) -> None:
        for line in text.splitlines():
            low = line.strip().lower()
            if "station type" in low or "equipment" in low:
                parts = line.split(":", 1)
                if len(parts) == 2:
                    self._station_type = parts[1].strip()
            elif "localizer" in low and "freq" in low:
                val = next((_safe_float(t) for t in line.split() if _safe_float(t) is not None), None)
                if val is not None:
                    self._localizer_freq = val
            elif "glide" in low and "freq" in low:
                val = next((_safe_float(t) for t in line.split() if _safe_float(t) is not None), None)
                if val is not None:
                    self._glideslope_freq = val
            elif "waveform" in low and ":" in line:
                name = line.split(":", 1)[-1].strip()
                if name and len(self._waveform_names) < 8:
                    self._waveform_names.append(name)
            elif "crs ddm" in low:
                val = next((_safe_float(t) for t in line.split() if _safe_float(t) is not None), None)
                if val is not None:
                    self._nominal_values["crs_ddm"] = val
            elif "crs sdm" in low:
                val = next(
                    (_safe_float(t) for t in line.split() if _safe_float(t) is not None),
                    None,
                )
                if val is not None:
                    self._nominal_values["crs_sdm"] = val
            elif "clr ddm" in low:
                val = next(
                    (_safe_float(t) for t in line.split() if _safe_float(t) is not None),
                    None,
                )
                if val is not None:
                    self._nominal_values["clr_ddm"] = val
            elif "clr sdm" in low:
                val = next(
                    (_safe_float(t) for t in line.split() if _safe_float(t) is not None),
                    None,
                )
                if val is not None:
                    self._nominal_values["clr_sdm"] = val

    def _parse_binary_section(self, raw: bytes) -> None:
        # Look for ILS frequency embedded in binary section
        begin = raw.find(b"BEGIN_PROG")
        if begin < 0:
            return
        chunk = raw[begin:begin + 512]
        # Extract 108-112 MHz ILS localizer frequency pattern
        for i in range(len(chunk) - 4):
            val = int.from_bytes(chunk[i:i+2], "little")
            if 10800 <= val <= 11195:
                if self._localizer_freq == 0.0:
                    self._localizer_freq = val / 100.0
                break

    def get_station_type(self) -> str:
        return self._station_type or "Unknown ILS"

    def get_localizer_frequency(self) -> float:
        return self._localizer_freq

    def get_glideslope_frequency(self) -> float:
        return self._glideslope_freq

    def get_waveform_names(self) -> List[str]:
        return list(self._waveform_names)

    def get_nominal_values(self) -> Dict[str, float]:
        return dict(self._nominal_values)


class LDABinaryFileParser:
    """
    Parse LDA binary archives that contain VOR data sample records.
    Returns a list of VORDataRecord instances for display in the LDA Viewer.
    """

    _DEFAULT_STATIONS = list(TERRAIN_LIBRARY.keys())

    def __init__(self, path: Path) -> None:
        self._path = path

    def parse_file(self) -> List[VORDataRecord]:
        records: List[VORDataRecord] = []
        try:
            raw = self._path.read_bytes()
            # Try text (CSV) format first; fall back to raw binary only if needed.
            records = self._try_text(raw)
            if not records:
                records = self._try_binary(raw)
        except Exception as exc:
            logger.warning("LDABinaryFileParser: %s", exc)
        return records

    def _try_text(self, raw: bytes) -> List[VORDataRecord]:
        records: List[VORDataRecord] = []
        text = raw.decode("latin-1", errors="replace")
        for line in text.splitlines():
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split(",")
            if len(parts) >= 7:
                try:
                    ts = datetime.fromisoformat(parts[0].strip())
                    rec = VORDataRecord(
                        timestamp=ts,
                        station_key=parts[1].strip(),
                        ident=parts[2].strip(),
                        frequency_mhz=float(parts[3]),
                        bearing_deg=float(parts[4]),
                        deviation_deg=float(parts[5]),
                        signal_percent=float(parts[6]),
                        distance_nm=float(parts[7]) if len(parts) > 7 else 0.0,
                    )
                    records.append(rec)
                except (ValueError, IndexError) as exc:
                    logger.debug("LDABinaryFileParser: skipping malformed line %r – %s", line, exc)
                    continue
        return records

    def _try_binary(self, raw: bytes) -> List[VORDataRecord]:
        """Attempt to decode a simple packed binary VOR record format."""
        records: List[VORDataRecord] = []
        RECORD_SIZE = 32
        if len(raw) < RECORD_SIZE:
            return records
        i = 0
        station_pool = self._DEFAULT_STATIONS
        idx = 0
        while i + RECORD_SIZE <= len(raw):
            try:
                bearing = int.from_bytes(raw[i:i+2], "little") / 100.0
                if bearing > 360.0:
                    i += 1
                    continue
                freq_raw = int.from_bytes(raw[i+2:i+4], "little")
                freq = freq_raw / 100.0
                if not (108.0 <= freq <= 118.0):
                    i += 1
                    continue
                dev_raw = int.from_bytes(raw[i+4:i+6], "little", signed=True)
                deviation = dev_raw / 100.0
                sig_raw = int.from_bytes(raw[i+6:i+8], "little")
                signal = min(100.0, sig_raw / 10.0)
                dist_raw = int.from_bytes(raw[i+8:i+10], "little")
                distance = dist_raw / 10.0
                station = station_pool[idx % len(station_pool)]
                rec = VORDataRecord(
                    timestamp=datetime.now(timezone.utc),
                    station_key=station,
                    ident=station[:3],
                    frequency_mhz=freq,
                    bearing_deg=bearing,
                    deviation_deg=deviation,
                    signal_percent=signal,
                    distance_nm=distance,
                )
                records.append(rec)
                i += RECORD_SIZE
                idx += 1
            except Exception:
                i += 1
        return records


# ---------------------------------------------------------------------------
# Analysis Classes
# ---------------------------------------------------------------------------

class GlideSlopeDetector:
    """Detect glide slope errors relative to a standard approach path."""

    def __init__(self, glide_slope_deg: float = 3.0) -> None:
        self._gs_deg = glide_slope_deg

    def calculate_glide_slope_error(
        self,
        aircraft_alt_ft: float,
        distance_nm: float,
        runway_elev_ft: float = 0.0,
    ) -> float:
        if distance_nm <= 0:
            return 0.0
        ideal_alt = runway_elev_ft + math.tan(math.radians(self._gs_deg)) * distance_nm * 6076.12
        return aircraft_alt_ft - ideal_alt

    def get_status(self, error_ft: float) -> str:
        if abs(error_ft) < 50:
            return "ON GS"
        return "HIGH" if error_ft > 0 else "LOW"


class SurfaceSlopeAnalyzer:
    """
    Analyse a terrain elevation profile for slope gradient and approach safety.
    """

    _CLASSIFICATIONS = [
        (0.0,  2.0,  "FLAT"),
        (2.0,  5.0,  "MODERATE"),
        (5.0, 10.0,  "CAUTION"),
        (10.0, float("inf"), "CRITICAL"),
    ]

    def __init__(self, profile: List[float]) -> None:
        self._profile = list(profile)

    def analyze_profile(self) -> Dict:
        if not self._profile:
            return {
                "sample_count": 0, "lowest_ft": 0.0, "highest_ft": 0.0,
                "average_slope_percent": 0.0, "classification": "UNKNOWN",
            }
        n = len(self._profile)
        lo = min(self._profile)
        hi = max(self._profile)
        slopes = []
        for i in range(1, n):
            delta_alt = self._profile[i] - self._profile[i - 1]
            # Assume 1 NM spacing ≈ 6076 ft horizontal
            slope_pct = abs(delta_alt) / 6076.0 * 100.0
            slopes.append(slope_pct)
        avg_slope = sum(slopes) / len(slopes) if slopes else 0.0
        classification = "FLAT"
        for lo_bound, hi_bound, label in self._CLASSIFICATIONS:
            if lo_bound <= avg_slope < hi_bound:
                classification = label
                break
        return {
            "sample_count": n,
            "lowest_ft": lo,
            "highest_ft": hi,
            "average_slope_percent": avg_slope,
            "classification": classification,
        }

    def assess_approach_safety(self, threshold_altitude_ft: float = 0.0) -> Dict:
        analysis = self.analyze_profile()
        obstacles = [a for a in self._profile if a > threshold_altitude_ft]
        cls = analysis["classification"]
        risk = {"FLAT": "LOW", "MODERATE": "LOW", "CAUTION": "MEDIUM", "CRITICAL": "HIGH"}.get(cls, "UNKNOWN")
        return {"risk": risk, "obstacle_count": len(obstacles)}


# ---------------------------------------------------------------------------
# Simulation Engines
# ---------------------------------------------------------------------------

class SimulationEngine:
    """Generate synthetic VOR telemetry and aircraft positions."""

    def __init__(self, station_list: List[Dict]) -> None:
        self._stations = station_list
        self._aircraft: List[AircraftData] = self._init_aircraft()
        self._t = 0.0

    def _init_aircraft(self) -> List[AircraftData]:
        aircraft = []
        callsigns = ["SAA101", "MNX202", "CUL303", "SAAF1", "SAAF2"]
        for i, cs in enumerate(callsigns):
            aircraft.append(AircraftData(
                callsign=cs,
                bearing=random.uniform(0, 360),
                distance_nm=random.uniform(5, 80),
                altitude_ft=random.randint(3000, 35000),
                speed_kts=random.randint(220, 450),
                heading=random.uniform(0, 360),
            ))
        return aircraft

    def tick(self, dt_sec: float = 1.0) -> None:
        self._t += dt_sec
        for ac in self._aircraft:
            ac.bearing = (ac.bearing + dt_sec * 0.05) % 360
            ac.distance_nm = max(5.0, ac.distance_nm - dt_sec * 0.01)
            ac.x = ac.distance_nm * math.sin(math.radians(ac.bearing))
            ac.y = ac.distance_nm * math.cos(math.radians(ac.bearing))

    def get_vor_data(self, station_code: str) -> VORData:
        st = next((s for s in self._stations if s["code"] == station_code), self._stations[0])
        return VORData(
            station=st["code"],
            bearing=random.uniform(0, 360),
            signal_strength=random.uniform(70, 100),
            timestamp=datetime.now(timezone.utc),
            frequency=st["freq"],
            ident=st["code"],
            deviation=random.uniform(-2.5, 2.5),
            distance_nm=random.uniform(2, 40),
        )

    @property
    def aircraft(self) -> List[AircraftData]:
        return self._aircraft


class DataAcquisitionThread:
    """Background thread facade for VOR data acquisition."""

    def __init__(self, engine: SimulationEngine, callback) -> None:
        self._engine = engine
        self._callback = callback
        self._running = False
        self._thread: Optional[Thread] = None

    def start(self) -> None:
        self._running = True
        self._thread = Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False

    def _run(self) -> None:
        while self._running:
            self._engine.tick(1.0)
            try:
                self._callback(self._engine)
            except Exception:
                pass
            time.sleep(1.0)


# ---------------------------------------------------------------------------
# GUI Widgets  (only compiled when PyQt5 is available)
# ---------------------------------------------------------------------------

if _GUI_OK:

    class CDIDisplay(QWidget):
        """Course Deviation Indicator."""

        def __init__(self, parent=None) -> None:
            super().__init__(parent)
            self._deviation = 0.0
            self.setMinimumSize(200, 200)

        def set_deviation(self, deg: float) -> None:
            self._deviation = max(-2.5, min(2.5, deg))
            self.update()

        def paintEvent(self, event) -> None:
            p = QPainter(self)
            p.setRenderHint(QPainter.Antialiasing)
            w, h = self.width(), self.height()
            cx, cy = w // 2, h // 2
            r = min(w, h) // 2 - 10
            p.setPen(QPen(QColor("#00ff00"), 2))
            p.drawEllipse(cx - r, cy - r, 2 * r, 2 * r)
            # Needle
            offset = int((self._deviation / 2.5) * r * 0.8)
            p.setPen(QPen(QColor("#ffcc00"), 3))
            p.drawLine(cx + offset, cy - r + 20, cx + offset, cy + r - 20)
            # Centre dot
            p.setBrush(QBrush(QColor("#ffffff")))
            p.drawEllipse(cx - 4, cy - 4, 8, 8)

    class ApproachGuidanceDisplay(QWidget):
        """Glide slope + localizer approach display."""

        def __init__(self, parent=None) -> None:
            super().__init__(parent)
            self._gs_error = 0.0
            self._loc_dev = 0.0
            self._distance_nm = 10.0
            self.setMinimumSize(300, 220)

        def update_data(self, gs_error: float, loc_dev: float, distance_nm: float) -> None:
            self._gs_error = gs_error
            self._loc_dev = loc_dev
            self._distance_nm = distance_nm
            self.update()

        def paintEvent(self, event) -> None:
            p = QPainter(self)
            p.setRenderHint(QPainter.Antialiasing)
            w, h = self.width(), self.height()
            p.fillRect(0, 0, w, h, QColor("#1a1a2e"))
            p.setPen(QPen(QColor("#00ff88"), 2))
            # Glide slope bar
            gs_y = h // 2 + int((self._gs_error / 500.0) * h * 0.4)
            p.drawLine(20, gs_y, w - 20, gs_y)
            # Localizer bar
            loc_x = w // 2 + int((self._loc_dev / 2.5) * w * 0.4)
            p.drawLine(loc_x, 20, loc_x, h - 20)
            p.setPen(QColor("#ffffff"))
            p.drawText(5, 15, f"D {self._distance_nm:.1f} NM")

    class RadarDisplay(QWidget):
        """Simplified radar sweep display."""

        def __init__(self, parent=None) -> None:
            super().__init__(parent)
            self._aircraft: List[AircraftData] = []
            self._sweep_angle = 0.0
            self._range_nm = 120.0
            self.setMinimumSize(400, 400)

        def set_aircraft(self, aircraft: List[AircraftData]) -> None:
            self._aircraft = aircraft
            self.update()

        def advance_sweep(self, deg: float = 3.0) -> None:
            self._sweep_angle = (self._sweep_angle + deg) % 360
            self.update()

        def paintEvent(self, event) -> None:
            p = QPainter(self)
            p.setRenderHint(QPainter.Antialiasing)
            w, h = self.width(), self.height()
            cx, cy = w // 2, h // 2
            r = min(w, h) // 2 - 10
            p.fillRect(0, 0, w, h, QColor("#001a00"))
            p.setPen(QPen(QColor("#004400"), 1))
            for ring in [0.25, 0.5, 0.75, 1.0]:
                rr = int(r * ring)
                p.drawEllipse(cx - rr, cy - rr, 2 * rr, 2 * rr)
            # Sweep line
            p.setPen(QPen(QColor("#00ff00"), 2))
            sx = cx + int(r * math.sin(math.radians(self._sweep_angle)))
            sy = cy - int(r * math.cos(math.radians(self._sweep_angle)))
            p.drawLine(cx, cy, sx, sy)
            # Aircraft blips
            p.setBrush(QBrush(QColor("#00ff88")))
            p.setPen(Qt.NoPen)
            for ac in self._aircraft:
                bx = cx + int((ac.x / self._range_nm) * r)
                by = cy - int((ac.y / self._range_nm) * r)
                p.drawEllipse(bx - 4, by - 4, 8, 8)

    class Terrain3DWidget(QWidget):
        """Terrain profile display – QPainter fallback (OpenGL optional)."""

        def __init__(self, parent=None) -> None:
            super().__init__(parent)
            self._profile: List[float] = []
            self.setMinimumSize(400, 200)

        def set_terrain(self, profile: List[float]) -> None:
            self._profile = list(profile)
            self.update()

        def paintEvent(self, event) -> None:
            p = QPainter(self)
            p.setRenderHint(QPainter.Antialiasing)
            w, h = self.width(), self.height()
            p.fillRect(0, 0, w, h, QColor("#0d1117"))
            if not self._profile:
                p.setPen(QColor("#888888"))
                p.drawText(w // 2 - 60, h // 2, "No terrain data")
                return
            lo = min(self._profile)
            hi = max(self._profile)
            span = hi - lo or 1.0
            n = len(self._profile)
            dx = w / max(n - 1, 1)
            pts = []
            for i, alt in enumerate(self._profile):
                x = int(i * dx)
                y = int(h - 20 - ((alt - lo) / span) * (h - 40))
                pts.append((x, y))
            # Fill terrain
            poly = QPolygonF()
            poly.append(QPointF(0, h))
            for x, y in pts:
                poly.append(QPointF(x, y))
            poly.append(QPointF(w, h))
            gradient_color = QColor("#2d6a4f")
            p.setBrush(QBrush(gradient_color))
            p.setPen(Qt.NoPen)
            p.drawPolygon(poly)
            # Profile line
            p.setPen(QPen(QColor("#52b788"), 2))
            for i in range(1, len(pts)):
                p.drawLine(pts[i-1][0], pts[i-1][1], pts[i][0], pts[i][1])
            # Axis labels
            p.setPen(QColor("#aaaaaa"))
            p.setFont(QFont("Monospace", 8))
            p.drawText(2, 12, f"{hi:.0f} ft")
            p.drawText(2, h - 4, f"{lo:.0f} ft")

    class ASRACSDisplay(QWidget):
        """Surface movement display placeholder."""

        def __init__(self, parent=None) -> None:
            super().__init__(parent)
            self.setMinimumSize(400, 400)

        def paintEvent(self, event) -> None:
            p = QPainter(self)
            p.fillRect(0, 0, self.width(), self.height(), QColor("#1a1a1a"))
            p.setPen(QColor("#888888"))
            p.drawText(20, self.height() // 2, "ASRACS Surface – OR Tambo (FAOR)")

    # -----------------------------------------------------------------------
    # Main Application
    # -----------------------------------------------------------------------

    class VORAirportMonitorApp(QMainWindow):
        """
        VOR / ASRACS / SAAF Airport Monitoring System – Main Window.
        Tabs: VOR Monitor | Aircraft Tracking | Approach Guidance |
              Terrain 3-D | ASRACS Surface | SAAF Bases |
              Connection | Diagnostics | LDA Viewer | Terrain Analysis
        """

        def __init__(self) -> None:
            super().__init__()
            self.setWindowTitle(f"VOR / ASRACS / SAAF Monitor  v{APP_VERSION}")
            self.resize(1280, 800)

            self._sim = SimulationEngine(VOR_STATIONS)
            self._acq = DataAcquisitionThread(self._sim, self._on_sim_tick)
            self._lda_records: List[VORDataRecord] = []
            self._current_vor: Optional[VORData] = None

            self._build_ui()
            self._populate_vor_combo()

            # Timers
            self._radar_timer = QTimer(self)
            self._radar_timer.timeout.connect(self._on_radar_tick)
            self._radar_timer.start(200)

            self._acq.start()

        # -------------------------------------------------------------------
        # UI Construction
        # -------------------------------------------------------------------

        def _build_ui(self) -> None:
            central = QWidget()
            self.setCentralWidget(central)
            root = QVBoxLayout(central)

            self._tabs = QTabWidget()
            root.addWidget(self._tabs)

            self._tab_vor()
            self._tab_radar()
            self._tab_approach()
            self._tab_terrain3d()
            self._tab_asracs()
            self._tab_saaf_bases()
            self._tab_connection()
            self._tab_diagnostics()
            self._tab_lda_viewer()      # NEW v5.3
            self._tab_terrain_analysis()  # NEW v5.3

            self._populate_vor_combo()

        def _tab_vor(self) -> None:
            page = QWidget()
            layout = QVBoxLayout(page)

            controls = QHBoxLayout()
            controls.addWidget(QLabel("Station:"))
            self.vor_combo = QComboBox()
            controls.addWidget(self.vor_combo)
            self.vor_combo.currentIndexChanged.connect(self._on_vor_index_changed)
            layout.addLayout(controls)

            self.cdi = CDIDisplay()
            layout.addWidget(self.cdi, 1)

            info_grid = QGridLayout()
            self.vor_bearing_lbl = QLabel("Bearing: —")
            self.vor_signal_lbl = QLabel("Signal: —")
            self.vor_freq_lbl = QLabel("Freq: —")
            self.vor_dist_lbl = QLabel("Distance: —")
            info_grid.addWidget(self.vor_bearing_lbl, 0, 0)
            info_grid.addWidget(self.vor_signal_lbl, 0, 1)
            info_grid.addWidget(self.vor_freq_lbl, 1, 0)
            info_grid.addWidget(self.vor_dist_lbl, 1, 1)
            layout.addLayout(info_grid)

            self._tabs.addTab(page, "VOR Monitor")

        def _tab_radar(self) -> None:
            page = QWidget()
            layout = QHBoxLayout(page)

            self.radar = RadarDisplay()
            layout.addWidget(self.radar, 2)

            right = QVBoxLayout()
            right.addWidget(QLabel("Aircraft"))
            self.ac_table = QTableWidget(0, 5)
            self.ac_table.setHorizontalHeaderLabels(["Callsign", "Brg°", "Dist NM", "Alt ft", "Spd kts"])
            if hasattr(self.ac_table, "horizontalHeader"):
                self.ac_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
            right.addWidget(self.ac_table)
            layout.addLayout(right, 1)

            self._tabs.addTab(page, "Aircraft Tracking")

        def _tab_approach(self) -> None:
            page = QWidget()
            layout = QVBoxLayout(page)

            hdr = QHBoxLayout()
            hdr.addWidget(QLabel("Runway:"))
            self.approach_runway_combo = QComboBox()
            for rwy in ["03L", "03R", "21L", "21R", "06", "24", "16", "34"]:
                self.approach_runway_combo.addItem(rwy)
            hdr.addWidget(self.approach_runway_combo)
            layout.addLayout(hdr)

            self.approach_display = ApproachGuidanceDisplay()
            layout.addWidget(self.approach_display, 1)

            self.gs_status_lbl = QLabel("Glide Slope: —")
            layout.addWidget(self.gs_status_lbl)

            self._tabs.addTab(page, "Approach Guidance")

        def _tab_terrain3d(self) -> None:
            page = QWidget()
            layout = QVBoxLayout(page)

            hdr = QHBoxLayout()
            hdr.addWidget(QLabel("Airport:"))
            self.terrain3d_combo = QComboBox()
            for code in sorted(TERRAIN_LIBRARY.keys()):
                self.terrain3d_combo.addItem(code)
            self.terrain3d_combo.currentIndexChanged.connect(self._on_terrain3d_changed)
            hdr.addWidget(self.terrain3d_combo)
            layout.addLayout(hdr)

            self.terrain3d_widget = Terrain3DWidget()
            layout.addWidget(self.terrain3d_widget, 1)

            self._tabs.addTab(page, "Terrain 3-D")
            self._on_terrain3d_changed()

        def _tab_asracs(self) -> None:
            page = QWidget()
            layout = QVBoxLayout(page)
            self.asracs_display = ASRACSDisplay()
            layout.addWidget(self.asracs_display, 1)
            self.asracs_alert_lbl = QLabel("Status: Nominal")
            layout.addWidget(self.asracs_alert_lbl)
            self._tabs.addTab(page, "ASRACS Surface")

        def _tab_saaf_bases(self) -> None:
            page = QWidget()
            layout = QVBoxLayout(page)
            layout.addWidget(QLabel("SAAF Bases – VOR / ILS Directory"))
            tbl = QTableWidget(len(VOR_STATIONS), 5)
            tbl.setHorizontalHeaderLabels(["Code", "Name", "Freq MHz", "VOR", "ILS"])
            if hasattr(tbl, "horizontalHeader"):
                tbl.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
            for row, st in enumerate(VOR_STATIONS):
                tbl.setItem(row, 0, QTableWidgetItem(st["code"]))
                tbl.setItem(row, 1, QTableWidgetItem(st["name"]))
                tbl.setItem(row, 2, QTableWidgetItem(f"{st['freq']:.2f}"))
                tbl.setItem(row, 3, QTableWidgetItem("✓" if st["vor"] else "—"))
                tbl.setItem(row, 4, QTableWidgetItem("✓" if st["ils"] else "—"))
            layout.addWidget(tbl, 1)
            self._tabs.addTab(page, "SAAF Bases")

        def _tab_connection(self) -> None:
            page = QWidget()
            layout = QVBoxLayout(page)
            layout.addWidget(QLabel("Connection Settings"))
            grp = QGroupBox("Source")
            grp_lay = QVBoxLayout(grp)
            grp_lay.addWidget(QLabel("Mode: Simulation (synthetic VOR data)"))
            self.conn_status_lbl = QLabel("Status: Running")
            grp_lay.addWidget(self.conn_status_lbl)
            layout.addWidget(grp)
            layout.addStretch()
            self._tabs.addTab(page, "Connection")

        def _tab_diagnostics(self) -> None:
            page = QWidget()
            layout = QVBoxLayout(page)
            layout.addWidget(QLabel("System Diagnostics"))
            self.diag_text = QTextEdit()
            self.diag_text.setReadOnly(True)
            self.diag_text.setFont(QFont("Monospace", 9))
            layout.addWidget(self.diag_text, 1)
            self._tabs.addTab(page, "Diagnostics")

        # -------------------------------------------------------------------
        # NEW: LDA Viewer Tab (v5.3)
        # -------------------------------------------------------------------

        def _tab_lda_viewer(self) -> None:
            """Create the LDA import and data browser tab."""
            page = QWidget()
            layout = QVBoxLayout(page)

            # Import controls row
            controls = QHBoxLayout()
            self.lda_import_btn = QPushButton("Import LDA File…")
            self.lda_import_btn.clicked.connect(self._import_lda_file_with_display)
            controls.addWidget(self.lda_import_btn)

            self.lda_filter_station = QLineEdit()
            self.lda_filter_station.setPlaceholderText("Filter by station code…")
            self.lda_filter_station.textChanged.connect(self._update_lda_table)
            controls.addWidget(self.lda_filter_station)

            self.lda_export_btn = QPushButton("Export CSV…")
            self.lda_export_btn.clicked.connect(self._export_lda_and_terrain_report)
            controls.addWidget(self.lda_export_btn)

            layout.addLayout(controls)

            # Data table
            self.lda_table = QTableWidget(0, 8)
            self.lda_table.setHorizontalHeaderLabels([
                "Timestamp", "Station", "Ident", "Freq (MHz)",
                "Bearing (°)", "Deviation (°)", "Signal (%)", "Distance (NM)",
            ])
            if hasattr(self.lda_table, "horizontalHeader"):
                self.lda_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
            self.lda_table.setSelectionBehavior(QTableWidget.SelectRows)
            self.lda_table.itemSelectionChanged.connect(self._on_lda_row_selected)
            layout.addWidget(self.lda_table, 1)

            # Summary stats label
            self.lda_stats_lbl = QLabel("Import an LDA file to view records.")
            layout.addWidget(self.lda_stats_lbl)

            self._tabs.addTab(page, "LDA Viewer")

        # -------------------------------------------------------------------
        # NEW: Terrain Analysis Tab (v5.3)
        # -------------------------------------------------------------------

        def _tab_terrain_analysis(self) -> None:
            """Create terrain slope analysis and profiling tab."""
            page = QWidget()
            root_layout = QVBoxLayout(page)

            # Station selector
            controls = QHBoxLayout()
            controls.addWidget(QLabel("Analyse terrain for:"))
            self.terrain_station_combo = QComboBox()
            for code in sorted(TERRAIN_LIBRARY.keys()):
                self.terrain_station_combo.addItem(code)
            self.terrain_station_combo.currentIndexChanged.connect(self._update_terrain_analysis)
            controls.addWidget(self.terrain_station_combo)
            controls.addStretch()
            root_layout.addLayout(controls)

            # Profile + analysis panel
            body = QHBoxLayout()

            self.terrain_analysis_display = Terrain3DWidget()
            body.addWidget(self.terrain_analysis_display, 2)

            analysis_grp = QGroupBox("Slope Analysis")
            analysis_lay = QVBoxLayout(analysis_grp)
            self.terrain_info_text = QTextEdit()
            self.terrain_info_text.setReadOnly(True)
            self.terrain_info_text.setFont(QFont("Monospace", 9))
            analysis_lay.addWidget(self.terrain_info_text)
            body.addWidget(analysis_grp, 1)

            root_layout.addLayout(body, 1)
            self._tabs.addTab(page, "Terrain Analysis")

            # Trigger initial analysis for first station
            self._update_terrain_analysis()

        # -------------------------------------------------------------------
        # Helpers
        # -------------------------------------------------------------------

        def _populate_vor_combo(self) -> None:
            if not hasattr(self, "vor_combo"):
                return
            self.vor_combo.blockSignals(True)
            self.vor_combo.clear()
            for st in VOR_STATIONS:
                self.vor_combo.addItem(f"{st['code']} – {st['name']}", st["code"])
            self.vor_combo.blockSignals(False)
            self._on_vor_index_changed()

        def _on_terrain3d_changed(self) -> None:
            if not hasattr(self, "terrain3d_combo"):
                return
            code = self.terrain3d_combo.currentText()
            profile = TERRAIN_LIBRARY.get(code, [])
            if hasattr(self, "terrain3d_widget"):
                self.terrain3d_widget.set_terrain(profile)

        # -------------------------------------------------------------------
        # Slot: VOR combo selection
        # -------------------------------------------------------------------

        def _on_vor_index_changed(self) -> None:
            if not hasattr(self, "vor_combo"):
                return
            code = self.vor_combo.currentData()
            if not code:
                return
            data = self._sim.get_vor_data(code)
            self._current_vor = data
            if hasattr(self, "cdi"):
                self.cdi.set_deviation(data.deviation)
            if hasattr(self, "vor_bearing_lbl"):
                self.vor_bearing_lbl.setText(f"Bearing: {data.bearing:.1f}°")
            if hasattr(self, "vor_signal_lbl"):
                self.vor_signal_lbl.setText(f"Signal: {data.signal_strength:.1f}%")
            if hasattr(self, "vor_freq_lbl"):
                self.vor_freq_lbl.setText(f"Freq: {data.frequency:.2f} MHz")
            if hasattr(self, "vor_dist_lbl"):
                self.vor_dist_lbl.setText(f"Distance: {data.distance_nm:.1f} NM")

        # -------------------------------------------------------------------
        # Slot: simulation tick
        # -------------------------------------------------------------------

        def _on_sim_tick(self, engine: SimulationEngine) -> None:
            pass  # UI updates are timer-driven to stay on main thread

        # -------------------------------------------------------------------
        # Slot: radar timer
        # -------------------------------------------------------------------

        def _on_radar_tick(self) -> None:
            self._sim.tick(0.2)
            if hasattr(self, "radar"):
                self.radar.set_aircraft(self._sim.aircraft)
                self.radar.advance_sweep(2.0)
            if hasattr(self, "ac_table"):
                self._refresh_aircraft_table()

        def _refresh_aircraft_table(self) -> None:
            ac_list = self._sim.aircraft
            self.ac_table.setRowCount(len(ac_list))
            for row, ac in enumerate(ac_list):
                self.ac_table.setItem(row, 0, QTableWidgetItem(ac.callsign))
                self.ac_table.setItem(row, 1, QTableWidgetItem(f"{ac.bearing:.1f}"))
                self.ac_table.setItem(row, 2, QTableWidgetItem(f"{ac.distance_nm:.1f}"))
                self.ac_table.setItem(row, 3, QTableWidgetItem(str(ac.altitude_ft)))
                self.ac_table.setItem(row, 4, QTableWidgetItem(str(ac.speed_kts)))

        # -------------------------------------------------------------------
        # LDA import & display (v5.3)
        # -------------------------------------------------------------------

        def _import_lda_file_with_display(self) -> None:
            """Open a file dialog, parse the LDA file, and populate the viewer table."""
            path, _ = QFileDialog.getOpenFileName(
                self, "Import LDA File", "",
                "LDA Files (*.lda);;Text Files (*.txt);;All Files (*)",
            )
            if not path:
                return
            try:
                self._lda_records = LDABinaryFileParser(Path(path)).parse_file()
            except Exception as exc:
                self.diag_text.append(f"[ERROR] LDA import failed: {exc}")
                return
            n = len(self._lda_records)
            self.lda_stats_lbl.setText(
                f"Loaded {n} record(s) from {Path(path).name}"
            )
            self._update_lda_table()
            self.diag_text.append(f"[INFO] LDA import complete: {n} record(s) from {Path(path).name}")

        def _update_lda_table(self) -> None:
            """Refresh the LDA viewer table, applying the station filter."""
            filter_text = ""
            if hasattr(self, "lda_filter_station"):
                filter_text = self.lda_filter_station.text().strip().upper()

            filtered = [
                r for r in self._lda_records
                if not filter_text or filter_text in r.station_key.upper()
            ]

            self.lda_table.setRowCount(len(filtered))
            for row, rec in enumerate(filtered):
                self.lda_table.setItem(row, 0, QTableWidgetItem(rec.timestamp.isoformat()))
                self.lda_table.setItem(row, 1, QTableWidgetItem(rec.station_key))
                self.lda_table.setItem(row, 2, QTableWidgetItem(rec.ident))
                self.lda_table.setItem(row, 3, QTableWidgetItem(f"{rec.frequency_mhz:.2f}"))
                self.lda_table.setItem(row, 4, QTableWidgetItem(f"{rec.bearing_deg:.1f}"))
                self.lda_table.setItem(row, 5, QTableWidgetItem(f"{rec.deviation_deg:+.2f}"))
                self.lda_table.setItem(row, 6, QTableWidgetItem(f"{rec.signal_percent:.1f}"))
                self.lda_table.setItem(row, 7, QTableWidgetItem(f"{rec.distance_nm:.2f}"))

            if hasattr(self, "lda_stats_lbl") and filter_text:
                self.lda_stats_lbl.setText(
                    f"Showing {len(filtered)} of {len(self._lda_records)} record(s) "
                    f"(filter: '{filter_text}')"
                )

        def _on_lda_row_selected(self) -> None:
            """Cross-link: selected LDA row updates terrain analysis and approach display."""
            rows = self.lda_table.selectionModel().selectedRows()
            if not rows:
                return
            row = rows[0].row()
            # Map visible row back to a record (honouring filter)
            filter_text = ""
            if hasattr(self, "lda_filter_station"):
                filter_text = self.lda_filter_station.text().strip().upper()
            filtered = [
                r for r in self._lda_records
                if not filter_text or filter_text in r.station_key.upper()
            ]
            if row >= len(filtered):
                return
            rec = filtered[row]

            # Update terrain analysis tab if station is in TERRAIN_LIBRARY
            if hasattr(self, "terrain_station_combo") and rec.station_key in TERRAIN_LIBRARY:
                idx = self.terrain_station_combo.findText(rec.station_key)
                if idx >= 0:
                    self.terrain_station_combo.setCurrentIndex(idx)

            # Update approach display – derive a nominal altitude from distance on a 3° path
            if hasattr(self, "approach_display"):
                gs_detector = GlideSlopeDetector(3.0)
                # Estimate altitude on a standard 3° glide path from distance
                nominal_alt_ft = math.tan(math.radians(3.0)) * rec.distance_nm * 6076.12
                gs_error = gs_detector.calculate_glide_slope_error(
                    aircraft_alt_ft=nominal_alt_ft,
                    distance_nm=rec.distance_nm,
                )
                self.approach_display.update_data(gs_error, rec.deviation_deg, rec.distance_nm)
                if hasattr(self, "gs_status_lbl"):
                    self.gs_status_lbl.setText(
                        f"Glide Slope: {gs_detector.get_status(gs_error)}  "
                        f"({gs_error:+.0f} ft)"
                    )

            if hasattr(self, "diag_text"):
                self.diag_text.append(
                    f"[LDA] Selected: {rec.station_key} / {rec.ident}  "
                    f"Brg {rec.bearing_deg:.1f}°  Dev {rec.deviation_deg:+.2f}°  "
                    f"Sig {rec.signal_percent:.1f}%"
                )

        # -------------------------------------------------------------------
        # Terrain Analysis computation (v5.3)
        # -------------------------------------------------------------------

        def _update_terrain_analysis(self) -> None:
            """Compute and display slope metrics for the selected station."""
            if not hasattr(self, "terrain_station_combo"):
                return
            station = self.terrain_station_combo.currentText()
            if not station or station not in TERRAIN_LIBRARY:
                return

            profile = TERRAIN_LIBRARY[station]
            if hasattr(self, "terrain_analysis_display"):
                self.terrain_analysis_display.set_terrain(profile)

            analyzer = SurfaceSlopeAnalyzer(profile)
            analysis = analyzer.analyze_profile()
            safety = analyzer.assess_approach_safety(threshold_altitude_ft=analysis["lowest_ft"])

            elevation_span = analysis["highest_ft"] - analysis["lowest_ft"]
            cls_color = {
                "FLAT": "#52b788", "MODERATE": "#f4a261",
                "CAUTION": "#e76f51", "CRITICAL": "#e63946",
            }.get(analysis["classification"], "#888888")

            html = (
                f"<b>Terrain Profile – {station}</b><br><br>"
                f"<b>Elevation</b><br>"
                f"&nbsp;&nbsp;Samples : {analysis['sample_count']}<br>"
                f"&nbsp;&nbsp;Min     : {analysis['lowest_ft']:.0f} ft<br>"
                f"&nbsp;&nbsp;Max     : {analysis['highest_ft']:.0f} ft<br>"
                f"&nbsp;&nbsp;Span    : {elevation_span:.0f} ft<br><br>"
                f"<b>Slope Analysis</b><br>"
                f"&nbsp;&nbsp;Avg slope : {analysis['average_slope_percent']:.2f} %<br>"
                f"&nbsp;&nbsp;Class     : <span style='color:{cls_color}'>"
                f"<b>{analysis['classification']}</b></span><br><br>"
                f"<b>Approach Safety</b><br>"
                f"&nbsp;&nbsp;Risk level  : <b>{safety['risk']}</b><br>"
                f"&nbsp;&nbsp;Obstacles   : {safety['obstacle_count']}<br>"
            )

            if hasattr(self, "terrain_info_text"):
                self.terrain_info_text.setHtml(html)

            if hasattr(self, "diag_text"):
                self.diag_text.append(
                    f"[TERRAIN] {station}: {analysis['classification']}  "
                    f"slope={analysis['average_slope_percent']:.2f}%  "
                    f"risk={safety['risk']}"
                )

        # -------------------------------------------------------------------
        # CSV Export (v5.3)
        # -------------------------------------------------------------------

        def _export_lda_and_terrain_report(self) -> None:
            """Export all LDA records combined with terrain analysis to CSV."""
            if not self._lda_records:
                QMessageBox.information(self, "Export", "No LDA records to export.  Import a file first.")
                return

            path, _ = QFileDialog.getSaveFileName(
                self, "Export LDA + Terrain Report", "lda_terrain_report.csv",
                "CSV Files (*.csv);;All Files (*)",
            )
            if not path:
                return

            try:
                with open(path, "w", newline="", encoding="utf-8") as fh:
                    writer = csv.writer(fh)
                    writer.writerow([
                        "Timestamp", "Station", "Ident", "Frequency_MHz",
                        "Bearing_deg", "Deviation_deg", "Signal_pct", "Distance_NM",
                        "Terrain_Min_ft", "Terrain_Max_ft", "Avg_Slope_pct", "Classification",
                        "Approach_Risk", "Obstacle_Count",
                    ])
                    for rec in self._lda_records:
                        profile = TERRAIN_LIBRARY.get(rec.station_key, [])
                        if profile:
                            analyzer = SurfaceSlopeAnalyzer(profile)
                            analysis = analyzer.analyze_profile()
                            safety = analyzer.assess_approach_safety(analysis["lowest_ft"])
                        else:
                            analysis = {
                                "lowest_ft": 0.0, "highest_ft": 0.0,
                                "average_slope_percent": 0.0, "classification": "N/A",
                            }
                            safety = {"risk": "N/A", "obstacle_count": 0}

                        writer.writerow([
                            rec.timestamp.isoformat(),
                            rec.station_key,
                            rec.ident,
                            f"{rec.frequency_mhz:.2f}",
                            f"{rec.bearing_deg:.1f}",
                            f"{rec.deviation_deg:+.2f}",
                            f"{rec.signal_percent:.1f}",
                            f"{rec.distance_nm:.2f}",
                            f"{analysis['lowest_ft']:.0f}",
                            f"{analysis['highest_ft']:.0f}",
                            f"{analysis['average_slope_percent']:.2f}",
                            analysis["classification"],
                            safety["risk"],
                            safety["obstacle_count"],
                        ])

                if hasattr(self, "diag_text"):
                    self.diag_text.append(f"[EXPORT] Saved {len(self._lda_records)} records to {path}")
                QMessageBox.information(self, "Export Complete",
                                        f"Exported {len(self._lda_records)} records to:\n{path}")
            except Exception as exc:
                if hasattr(self, "diag_text"):
                    self.diag_text.append(f"[ERROR] Export failed: {exc}")
                QMessageBox.critical(self, "Export Failed", str(exc))


# ---------------------------------------------------------------------------
# Self-test  (headless – no display required)
# ---------------------------------------------------------------------------

def _self_test() -> int:
    """Run 6 headless unit tests.  Returns 0 on success, 1 on any failure."""
    failures = 0

    def _ok(name: str) -> None:
        print(f"  PASS  {name}")

    def _fail(name: str, reason: str) -> None:
        nonlocal failures
        failures += 1
        print(f"  FAIL  {name}: {reason}")

    print("=== CVOR1 Self-Test ===")

    # 1. TERRAIN_LIBRARY sanity
    try:
        assert len(TERRAIN_LIBRARY) >= 10
        for code, profile in TERRAIN_LIBRARY.items():
            assert len(profile) == 15, f"{code} profile length != 15"
        _ok("TERRAIN_LIBRARY structure")
    except AssertionError as e:
        _fail("TERRAIN_LIBRARY structure", str(e))

    # 2. SurfaceSlopeAnalyzer – flat terrain
    try:
        flat = [1000.0] * 15
        result = SurfaceSlopeAnalyzer(flat).analyze_profile()
        assert result["classification"] == "FLAT", result["classification"]
        assert result["average_slope_percent"] == 0.0
        _ok("SurfaceSlopeAnalyzer flat terrain")
    except AssertionError as e:
        _fail("SurfaceSlopeAnalyzer flat terrain", str(e))

    # 3. SurfaceSlopeAnalyzer – approach safety
    try:
        profile = TERRAIN_LIBRARY["JNB"]
        safety = SurfaceSlopeAnalyzer(profile).assess_approach_safety(0.0)
        assert safety["risk"] in ("LOW", "MEDIUM", "HIGH")
        assert isinstance(safety["obstacle_count"], int)
        _ok("SurfaceSlopeAnalyzer approach safety")
    except AssertionError as e:
        _fail("SurfaceSlopeAnalyzer approach safety", str(e))

    # 4. GlideSlopeDetector
    try:
        gsd = GlideSlopeDetector(3.0)
        err = gsd.calculate_glide_slope_error(3000.0, 10.0, 5558.0)
        assert isinstance(err, float)
        status = gsd.get_status(0.0)
        assert status == "ON GS"
        _ok("GlideSlopeDetector")
    except AssertionError as e:
        _fail("GlideSlopeDetector", str(e))

    # 5. LDABinaryFileParser – text CSV format
    try:
        import tempfile, os
        ts = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%S")
        csv_content = f"{ts},JNB,JHB,114.90,045.0,+0.25,92.5,12.3\n"
        with tempfile.NamedTemporaryFile(
            mode="w", suffix=".lda", delete=False, encoding="utf-8"
        ) as tmp:
            tmp.write(csv_content)
            tmp_path = tmp.name
        records = LDABinaryFileParser(Path(tmp_path)).parse_file()
        os.unlink(tmp_path)
        assert len(records) == 1, f"expected 1 record, got {len(records)}"
        r = records[0]
        assert r.station_key == "JNB"
        assert r.ident == "JHB"
        assert abs(r.frequency_mhz - 114.90) < 0.01
        assert abs(r.bearing_deg - 45.0) < 0.01
        assert abs(r.deviation_deg - 0.25) < 0.01
        assert abs(r.signal_percent - 92.5) < 0.01
        assert abs(r.distance_nm - 12.3) < 0.01
        _ok("LDABinaryFileParser CSV text format")
    except (AssertionError, Exception) as e:
        _fail("LDABinaryFileParser CSV text format", str(e))

    # 6. VORDataRecord dataclass
    try:
        rec = VORDataRecord(
            timestamp=datetime.now(timezone.utc),
            station_key="CPT",
            ident="CTV",
            frequency_mhz=115.70,
            bearing_deg=180.0,
            deviation_deg=-0.5,
            signal_percent=88.0,
            distance_nm=25.0,
        )
        assert rec.station_key == "CPT"
        assert rec.ident == "CTV"
        _ok("VORDataRecord dataclass")
    except AssertionError as e:
        _fail("VORDataRecord dataclass", str(e))

    print(f"=== {'PASS' if failures == 0 else 'FAIL'} ({failures} failure(s)) ===")
    return failures


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    if "--self-test" in sys.argv:
        sys.exit(_self_test())

    if not _GUI_OK:
        print("ERROR: PyQt5 is not available.  Install with: pip install PyQt5 PyQtChart")
        sys.exit(1)

    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    palette = QPalette()
    palette.setColor(QPalette.Window, QColor("#1a1a2e"))
    palette.setColor(QPalette.WindowText, QColor("#e0e0e0"))
    palette.setColor(QPalette.Base, QColor("#16213e"))
    palette.setColor(QPalette.AlternateBase, QColor("#0f3460"))
    palette.setColor(QPalette.Text, QColor("#e0e0e0"))
    palette.setColor(QPalette.Button, QColor("#0f3460"))
    palette.setColor(QPalette.ButtonText, QColor("#e0e0e0"))
    app.setPalette(palette)

    window = VORAirportMonitorApp()
    window.show()
    sys.exit(app.exec_())
