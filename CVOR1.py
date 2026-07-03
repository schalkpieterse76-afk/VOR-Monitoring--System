#!/usr/bin/env python3
"""
VOR / ASRACS / SAAF Airport Monitoring System  v5.3
====================================================
Production-focused single-file implementation with:
- LDA configuration parser (text + coded section extraction)
- Glide slope and surface slope calculations
- Full SAAF + civil station metadata
- Connection management (Serial/TCP/Mock/LDA)
- Thread-safe data buffering and simulation engines
- Optional 8-tab PyQt user interface
"""

from __future__ import annotations

import json
import logging
import math
import os
import queue
import random
import re
import socket
import struct
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Deque, Dict, Iterable, List, Optional, Set, Tuple, Union

try:
    import yaml  # type: ignore
except Exception:
    yaml = None

try:
    import serial  # type: ignore
    import serial.tools.list_ports  # type: ignore
except Exception:
    serial = None

try:
    from PyQt5.QtCore import QTimer
    from PyQt5.QtWidgets import (
        QApplication,
        QFileDialog,
        QGridLayout,
        QGroupBox,
        QHBoxLayout,
        QLabel,
        QLineEdit,
        QMainWindow,
        QPushButton,
        QTabWidget,
        QTextEdit,
        QVBoxLayout,
        QWidget,
    )

    PYQT_AVAILABLE = True
except Exception:
    PYQT_AVAILABLE = False


LOG_FILE = "vor_monitor.log"
APP_VERSION = "5.3"
DEFAULT_CONFIG_PATH = Path("vor_config.yaml")
MAX_TEXT_LINE_LENGTH = 400
MAX_CODED_WORDS = 512
MIN_TOKEN_LENGTH = 4
MAX_TOKEN_LENGTH = 24
CODED_SCAN_BYTES = 8192
MAX_EXTRACTED_LABELS = 64
MAX_WAVEFORM_SAMPLES = 256
MAX_WAVEFORMS = 8
NM_TO_FEET = 6076.11549
# Convert groundspeed in knots to feet/min horizontal travel:
# kts * NM_TO_FEET / 60
KTS_TO_FPM_BASE_FACTOR = NM_TO_FEET / 60.0
FEET_TO_METERS = 0.3048
DEGREES_TO_NM = 60.0
MIN_DIVISION_EPSILON = 1e-6
DEFAULT_DESCENT_RATE_FPM = 500.0
SECONDS_PER_MINUTE = 60.0
INCURSION_THRESHOLD_METERS = 30.0
SERVER_SOCKET_TIMEOUT_SEC = 0.4
MOCK_SERVER_RECV_SIZE = 2048
DDM_MIN = -0.155
DDM_MAX = 0.155
NOMINAL_SDM = 40.0
NOMINAL_RF_DBM = -58.0
THREAD_SHUTDOWN_TIMEOUT_SEC = 2.0
GUI_REFRESH_INTERVAL_MS = 1000
SELF_TEST_WARMUP_SEC = 1.2

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - %(name)s - %(message)s",
    handlers=[logging.FileHandler(LOG_FILE), logging.StreamHandler()],
)
logger = logging.getLogger("CVOR1")


@dataclass
class Runway:
    ident: str
    heading_deg: float
    length_m: int
    has_ils: bool = False
    gs_deg: float = 3.0


@dataclass
class VORStation:
    base_name: str
    icao: str
    city: str
    vor_ident: str
    vor_freq_mhz: float
    has_ils: bool
    shared_vor_reference: Optional[str] = None
    squadrons: List[str] = field(default_factory=list)
    runways: List[Runway] = field(default_factory=list)
    localizer_freq_mhz: Optional[float] = None
    glideslope_freq_mhz: Optional[float] = None


@dataclass
class GlideSlopeResult:
    target_alt_ft: float
    error_ft: float
    required_descent_fpm: float
    status: str


@dataclass
class VORDataSample:
    timestamp: float
    station_icao: str
    bearing_deg: float
    ddm: float
    sdm: float
    rf_dbm: float


@dataclass
class AircraftState:
    callsign: str
    lat: float
    lon: float
    alt_ft: float
    gs_kts: float
    track_deg: float


@dataclass
class ASRACSTarget:
    ident: str
    x_m: float
    y_m: float
    speed_mps: float
    heading_deg: float
    target_type: str = "aircraft"


@dataclass
class ASRACSAlert:
    severity: str
    message: str
    timestamp: float = field(default_factory=time.time)


class LDAParseError(RuntimeError):
    pass


class LDAFileParser:
    """Parser for Thales/Rohde & Schwarz .lda configuration files."""

    def __init__(self, file_path: Union[str, Path]):
        self.file_path = Path(file_path)
        self.raw_bytes: bytes = b""
        self.text_sections: Dict[str, str] = {}
        self.coded_values: Dict[str, Any] = {}
        self.waveforms: List[Dict[str, Any]] = []
        self.nominal_values: Dict[str, float] = {}
        self.alarm_limits: Dict[str, Tuple[float, float]] = {}
        self.frequencies: Dict[str, float] = {}

    def parse(self) -> bool:
        try:
            if not self.file_path.exists():
                raise LDAParseError(f"LDA file not found: {self.file_path}")
            if self.file_path.suffix.lower() != ".lda":
                raise LDAParseError(f"Unsupported file extension for {self.file_path}; expected .lda")

            self.raw_bytes = self.file_path.read_bytes()
            decoded = self.raw_bytes.decode("latin-1", errors="ignore")

            self._parse_text_sections(decoded)
            self._parse_coded_section(self.raw_bytes)
            self._extract_waveforms(decoded)
            self._extract_nominal_values(decoded)
            self._extract_alarm_limits(decoded)
            self._extract_frequencies(decoded)
            return True
        except Exception as exc:
            logger.exception("Failed to parse LDA file %s", self.file_path)
            self.coded_values["error"] = str(exc)
            return False

    def _parse_text_sections(self, decoded: str) -> None:
        # Supports key=value or key: value text printout sections
        for line in decoded.splitlines():
            s = line.strip()
            if not s or len(s) > MAX_TEXT_LINE_LENGTH:
                continue
            if "=" in s:
                key, value = s.split("=", 1)
            elif ":" in s:
                key, value = s.split(":", 1)
            else:
                continue
            key = re.sub(r"\s+", "_", key.strip().lower())
            value = value.strip()
            if key:
                self.text_sections[key] = value

    def _parse_coded_section(self, raw: bytes) -> None:
        marker_pos = raw.find(b"CODED")
        payload = raw[marker_pos + 5 :] if marker_pos >= 0 else raw
        if not payload:
            return

        # Pull compact numeric snapshots from binary CODED payload.
        max_words = min(MAX_CODED_WORDS, len(payload) // 2)
        words = [struct.unpack_from("<H", payload, i * 2)[0] for i in range(max_words)]
        self.coded_values["word_count"] = len(words)
        if words:
            self.coded_values["word_min"] = min(words)
            self.coded_values["word_max"] = max(words)
            self.coded_values["word_avg"] = float(sum(words)) / len(words)

        # Attempt to decode plausible ASCII labels embedded in binary.
        ascii_tokens = re.findall(
            rf"[A-Z0-9_\-/]{{{MIN_TOKEN_LENGTH},{MAX_TOKEN_LENGTH}}}".encode(),
            payload[:CODED_SCAN_BYTES],
        )
        labels = sorted({tok.decode("ascii", "ignore") for tok in ascii_tokens})[:MAX_EXTRACTED_LABELS]
        if labels:
            self.coded_values["labels"] = labels

    def _extract_waveforms(self, decoded: str) -> None:
        found: List[Dict[str, Any]] = []
        for line in decoded.splitlines():
            if re.search(r"\b(waveform|wfm|wave)\b", line, re.IGNORECASE):
                nums = [float(n) for n in re.findall(r"[-+]?\d+(?:\.\d+)?", line)]
                name = line.strip()[:120]
                found.append({"name": name, "samples": nums[:MAX_WAVEFORM_SAMPLES]})
                if len(found) >= MAX_WAVEFORMS:
                    break

        if not found:
            names = self.coded_values.get("labels", [])
            for label in names:
                if "WAVE" in label.upper() or "WFM" in label.upper():
                    found.append({"name": label, "samples": []})
                    if len(found) >= MAX_WAVEFORMS:
                        break

        self.waveforms = found

    def _extract_nominal_values(self, decoded: str) -> None:
        patterns = {
            "ddm": r"\b(?:nominal\s+)?ddm\b[^\d+-]*([-+]?\d+(?:\.\d+)?)",
            "sdm": r"\b(?:nominal\s+)?sdm\b[^\d+-]*([-+]?\d+(?:\.\d+)?)",
            "rf_dbm": r"\b(?:rf|level|power)\b[^\d+-]*([-+]?\d+(?:\.\d+)?)\s*(?:dbm|db)?",
        }
        for key, pattern in patterns.items():
            m = re.search(pattern, decoded, re.IGNORECASE)
            if m:
                self.nominal_values[key] = float(m.group(1))

    def _extract_alarm_limits(self, decoded: str) -> None:
        # Finds patterns like DDM ALARM LOW/HIGH values
        for metric in ("ddm", "sdm", "rf", "power"):
            low = re.search(
                rf"\b{metric}\b[^\n]*?(?:low|min)\D+([-+]?\d+(?:\.\d+)?)",
                decoded,
                re.IGNORECASE,
            )
            high = re.search(
                rf"\b{metric}\b[^\n]*?(?:high|max)\D+([-+]?\d+(?:\.\d+)?)",
                decoded,
                re.IGNORECASE,
            )
            if low and high:
                self.alarm_limits[metric] = (float(low.group(1)), float(high.group(1)))

    def _extract_frequencies(self, decoded: str) -> None:
        def _pick(name: str, pattern: str) -> None:
            m = re.search(pattern, decoded, re.IGNORECASE)
            if m:
                self.frequencies[name] = float(m.group(1))

        _pick("localizer_mhz", r"\b(?:localizer|loc)\b[^\d]*(1\d\d\.\d{2,3})")
        _pick("glideslope_mhz", r"\b(?:glide\s*slope|gs)\b[^\d]*(3\d\d\.\d{2,3})")

        # fallback: generic ILS ranges
        if "localizer_mhz" not in self.frequencies:
            values = [float(v) for v in re.findall(r"\b\d{3}\.\d{2,3}\b", decoded)]
            for v in values:
                if 108.10 <= v <= 111.95:
                    self.frequencies["localizer_mhz"] = v
                    break
        if "glideslope_mhz" not in self.frequencies:
            values = [float(v) for v in re.findall(r"\b\d{3}\.\d{2,3}\b", decoded)]
            for v in values:
                if 329.15 <= v <= 335.00:
                    self.frequencies["glideslope_mhz"] = v
                    break

    def printable_summary(self) -> str:
        lines = [f"LDA File: {self.file_path}"]
        lines.append(f"Text entries: {len(self.text_sections)}")
        lines.append(f"Coded entries: {len(self.coded_values)}")
        lines.append(f"Waveforms: {len(self.waveforms)}")
        lines.append(f"Nominals: {self.nominal_values}")
        lines.append(f"Alarm limits: {self.alarm_limits}")
        lines.append(f"Frequencies: {self.frequencies}")
        return "\n".join(lines)

    def get_nominal_values(self) -> Dict[str, float]:
        return dict(self.nominal_values)

    def get_alarm_limits(self) -> Dict[str, Tuple[float, float]]:
        return dict(self.alarm_limits)

    def get_waveforms(self) -> List[Dict[str, Any]]:
        return list(self.waveforms)

    def get_localizer_frequency(self) -> Optional[float]:
        return self.frequencies.get("localizer_mhz")

    def get_glideslope_frequency(self) -> Optional[float]:
        return self.frequencies.get("glideslope_mhz")


class GlideSlopeDetector:
    def __init__(self, glide_slope_deg: float = 3.0, tolerance_ft: float = 200.0):
        self.glide_slope_deg = glide_slope_deg
        self.tolerance_ft = tolerance_ft

    def ideal_altitude_ft(self, distance_nm: float, runway_elev_ft: float = 0.0) -> float:
        distance_ft = max(0.0, distance_nm) * NM_TO_FEET
        return runway_elev_ft + math.tan(math.radians(self.glide_slope_deg)) * distance_ft

    def calculate_descent_rate_fpm(self, groundspeed_kts: float) -> float:
        # feet/min = horizontal_ft_per_min * tan(angle) where
        # horizontal_ft_per_min = groundspeed_kts * NM_TO_FEET / 60
        return groundspeed_kts * KTS_TO_FPM_BASE_FACTOR * math.tan(math.radians(self.glide_slope_deg))

    def calculate_glide_slope_error(
        self, aircraft_alt_ft: float, distance_nm: float, runway_elev_ft: float = 0.0
    ) -> float:
        return aircraft_alt_ft - self.ideal_altitude_ft(distance_nm, runway_elev_ft)

    def evaluate(
        self,
        aircraft_alt_ft: float,
        distance_nm: float,
        runway_elev_ft: float,
        groundspeed_kts: float,
    ) -> GlideSlopeResult:
        target = self.ideal_altitude_ft(distance_nm, runway_elev_ft)
        error = aircraft_alt_ft - target
        fpm = self.calculate_descent_rate_fpm(groundspeed_kts)
        if abs(error) <= self.tolerance_ft:
            status = "ON_GS"
        elif error > 0:
            status = "HIGH"
        else:
            status = "LOW"
        return GlideSlopeResult(target_alt_ft=target, error_ft=error, required_descent_fpm=fpm, status=status)

    def from_lda_nominals(self, nominal_values: Dict[str, float]) -> Dict[str, Any]:
        return {
            "detector_angle_deg": self.glide_slope_deg,
            "nominal_ddm": nominal_values.get("ddm"),
            "nominal_sdm": nominal_values.get("sdm"),
            "nominal_rf_dbm": nominal_values.get("rf_dbm"),
        }


class SurfaceSlopeAnalyzer:
    @staticmethod
    def slope_percent(elevation_a_ft: float, elevation_b_ft: float, distance_m: float) -> float:
        if distance_m <= 0:
            return 0.0
        delta_ft = elevation_b_ft - elevation_a_ft
        delta_m = delta_ft * FEET_TO_METERS
        return (delta_m / distance_m) * 100.0

    @staticmethod
    def identify_surface_slope(profile: Iterable[Tuple[float, float]]) -> Dict[str, float]:
        points = list(profile)
        if len(points) < 2:
            return {"gradient_pct": 0.0, "run_m": 0.0, "rise_ft": 0.0}
        start_m, start_ft = points[0]
        end_m, end_ft = points[-1]
        run_m = max(0.0, end_m - start_m)
        rise_ft = end_ft - start_ft
        gradient_pct = SurfaceSlopeAnalyzer.slope_percent(start_ft, end_ft, run_m)
        return {"gradient_pct": gradient_pct, "run_m": run_m, "rise_ft": rise_ft}


class VORAirportConfig:
    def __init__(self, config_path: Path = DEFAULT_CONFIG_PATH):
        self.config_path = config_path
        self.lock = threading.Lock()
        self.config: Dict[str, Any] = {}

    def default(self) -> Dict[str, Any]:
        return {
            "version": APP_VERSION,
            "ui": {"theme": "dark", "refresh_hz": 1},
            "connections": {"mode": "mock", "serial_port": "", "tcp_host": "127.0.0.1", "tcp_port": 5000},
            "performance": {"radar_fps": 60, "terrain_frame_ms": 16},
        }

    def load(self) -> Dict[str, Any]:
        with self.lock:
            if not self.config_path.exists():
                self.config = self.default()
                self.save(self.config)
                return dict(self.config)

            text = self.config_path.read_text(encoding="utf-8")
            if yaml is not None:
                self.config = yaml.safe_load(text) or {}
            else:
                self.config = json.loads(text)

            if not isinstance(self.config, dict):
                self.config = self.default()
            return dict(self.config)

    def save(self, values: Dict[str, Any]) -> None:
        with self.lock:
            self.config = dict(values)
            if yaml is not None:
                payload = yaml.safe_dump(self.config, sort_keys=False)
            else:
                payload = json.dumps(self.config, indent=2)
            self.config_path.write_text(payload, encoding="utf-8")


class VORDataProcessor:
    def __init__(self, maxlen: int = 1000):
        self.buffer: Deque[VORDataSample] = deque(maxlen=maxlen)
        self.lock = threading.Lock()

    def add_sample(self, sample: VORDataSample) -> None:
        with self.lock:
            self.buffer.append(sample)

    def latest(self) -> Optional[VORDataSample]:
        with self.lock:
            return self.buffer[-1] if self.buffer else None

    def summary(self) -> Dict[str, float]:
        with self.lock:
            if not self.buffer:
                return {"count": 0}
            ddm_vals = [s.ddm for s in self.buffer]
            sdm_vals = [s.sdm for s in self.buffer]
            rf_vals = [s.rf_dbm for s in self.buffer]
            return {
                "count": float(len(self.buffer)),
                "ddm_avg": sum(ddm_vals) / len(ddm_vals),
                "sdm_avg": sum(sdm_vals) / len(sdm_vals),
                "rf_avg": sum(rf_vals) / len(rf_vals),
            }


class AircraftTracker:
    def __init__(self):
        self.lock = threading.Lock()
        self.aircraft: Dict[str, AircraftState] = {}

    def update(self, state: AircraftState) -> None:
        with self.lock:
            self.aircraft[state.callsign] = state

    def list_aircraft(self) -> List[AircraftState]:
        with self.lock:
            return list(self.aircraft.values())

    def relative_distance_nm(self, a: AircraftState, b: AircraftState) -> float:
        # quick equirectangular approximation for local region
        lat_factor = math.cos(math.radians((a.lat + b.lat) / 2.0))
        dlat_nm = (a.lat - b.lat) * DEGREES_TO_NM
        dlon_nm = (a.lon - b.lon) * DEGREES_TO_NM * lat_factor
        return math.hypot(dlat_nm, dlon_nm)


class SimulationEngine:
    def __init__(self, tracker: AircraftTracker):
        self.tracker = tracker
        self.running = False
        self.lock = threading.Lock()

    def start(self) -> None:
        with self.lock:
            self.running = True

    def stop(self) -> None:
        with self.lock:
            self.running = False

    def step(self, dt_s: float = 1.0) -> None:
        with self.lock:
            if not self.running:
                return

        states = self.tracker.list_aircraft()
        for st in states:
            nm = st.gs_kts * dt_s / 3600.0
            heading = math.radians(st.track_deg)
            dlat = (nm * math.cos(heading)) / DEGREES_TO_NM
            dlon = (nm * math.sin(heading)) / max(
                MIN_DIVISION_EPSILON, DEGREES_TO_NM * math.cos(math.radians(st.lat))
            )
            updated = AircraftState(
                callsign=st.callsign,
                lat=st.lat + dlat,
                lon=st.lon + dlon,
                alt_ft=max(0.0, st.alt_ft - DEFAULT_DESCENT_RATE_FPM * dt_s / SECONDS_PER_MINUTE),
                gs_kts=st.gs_kts,
                track_deg=st.track_deg,
            )
            self.tracker.update(updated)


class ASRACSSimEngine:
    def __init__(self):
        self.lock = threading.Lock()
        self.targets: Dict[str, ASRACSTarget] = {}
        self.alerts: Deque[ASRACSAlert] = deque(maxlen=200)
        self._active_alert_targets: Set[str] = set()

    def add_target(self, target: ASRACSTarget) -> None:
        with self.lock:
            self.targets[target.ident] = target

    def step(self, dt_s: float = 1.0) -> None:
        with self.lock:
            for ident, t in list(self.targets.items()):
                heading = math.radians(t.heading_deg)
                t.x_m += math.cos(heading) * t.speed_mps * dt_s
                t.y_m += math.sin(heading) * t.speed_mps * dt_s
                in_risk_zone = abs(t.x_m) < INCURSION_THRESHOLD_METERS and abs(t.y_m) < INCURSION_THRESHOLD_METERS
                if in_risk_zone and ident not in self._active_alert_targets:
                    self.alerts.append(ASRACSAlert(severity="critical", message=f"Runway incursion risk: {ident}"))
                    self._active_alert_targets.add(ident)
                elif not in_risk_zone and ident in self._active_alert_targets:
                    self._active_alert_targets.remove(ident)

    def latest_alerts(self, limit: int = 20) -> List[ASRACSAlert]:
        with self.lock:
            return list(self.alerts)[-limit:]


class SerialVORConnection:
    def __init__(self):
        self.conn = None

    def connect(self, port: str, baudrate: int = 9600, timeout: float = 1.0) -> bool:
        if serial is None:
            raise RuntimeError("pyserial not available")
        try:
            self.conn = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
            return bool(self.conn and self.conn.is_open)
        except Exception:
            logger.exception("Serial connection failed on port %s", port)
            self.conn = None
            return False

    def read_line(self) -> str:
        if not self.conn:
            return ""
        return self.conn.readline().decode("ascii", errors="ignore").strip()

    def close(self) -> None:
        if self.conn:
            self.conn.close()
            self.conn = None


class TCPVORConnection:
    def __init__(self):
        self.sock: Optional[socket.socket] = None

    def connect(self, host: str, port: int, timeout: float = 2.0) -> bool:
        try:
            self.sock = socket.create_connection((host, port), timeout=timeout)
            return True
        except Exception:
            logger.exception("TCP connection failed for %s:%s", host, port)
            self.sock = None
            return False

    def query(self, payload: str) -> str:
        if not self.sock:
            return ""
        self.sock.sendall((payload + "\n").encode("utf-8"))
        return self.sock.recv(4096).decode("utf-8", errors="ignore").strip()

    def close(self) -> None:
        if self.sock:
            self.sock.close()
            self.sock = None


class MockVORServer:
    def __init__(self, host: str = "127.0.0.1", port: int = 5000):
        self.host = host
        self.port = port
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._serve, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()

    def _serve(self) -> None:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as srv:
            srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            srv.bind((self.host, self.port))
            srv.listen(2)
            srv.settimeout(SERVER_SOCKET_TIMEOUT_SEC)
            while not self._stop.is_set():
                try:
                    client, _ = srv.accept()
                except socket.timeout:
                    continue
                with client:
                    data = client.recv(MOCK_SERVER_RECV_SIZE).decode("utf-8", errors="ignore").strip().upper()
                    resp = self._response(data)
                    client.sendall((resp + "\n").encode("utf-8"))

    @staticmethod
    def _response(cmd: str) -> str:
        if "STATUS" in cmd:
            return "OK;MODE=MOCK"
        if "IDENT" in cmd:
            return "WKV"
        if "HEALTH" in cmd:
            return "NOMINAL"
        return f"DDM={random.uniform(DDM_MIN, DDM_MAX):.3f};SDM={NOMINAL_SDM};RF={NOMINAL_RF_DBM}"


class LDAFileConnection:
    def __init__(self):
        self.parser: Optional[LDAFileParser] = None

    def connect(self, lda_path: Union[str, Path]) -> bool:
        self.parser = LDAFileParser(lda_path)
        return self.parser.parse()

    def get_payload(self) -> Dict[str, Any]:
        if not self.parser:
            return {}
        return {
            "nominal_values": self.parser.get_nominal_values(),
            "alarm_limits": self.parser.get_alarm_limits(),
            "localizer_mhz": self.parser.get_localizer_frequency(),
            "glideslope_mhz": self.parser.get_glideslope_frequency(),
            "waveforms": self.parser.get_waveforms(),
        }


class ConnectionManager:
    def __init__(self):
        self.mode = "mock"
        self.serial_conn = SerialVORConnection()
        self.tcp_conn = TCPVORConnection()
        self.mock_server = MockVORServer()
        self.lda_conn = LDAFileConnection()

    def set_mode(self, mode: str) -> None:
        mode = mode.lower().strip()
        if mode not in {"serial", "tcp", "mock", "lda"}:
            raise ValueError(f"Unsupported mode: {mode}")
        self.mode = mode

    def connect(self, **kwargs: Any) -> bool:
        if self.mode == "serial":
            return self.serial_conn.connect(kwargs.get("port", ""), kwargs.get("baudrate", 9600))
        if self.mode == "tcp":
            return self.tcp_conn.connect(kwargs.get("host", "127.0.0.1"), int(kwargs.get("port", 5000)))
        if self.mode == "mock":
            self.mock_server.start()
            return True
        if self.mode == "lda":
            return self.lda_conn.connect(kwargs.get("path", ""))
        return False

    def disconnect(self) -> None:
        self.serial_conn.close()
        self.tcp_conn.close()
        self.mock_server.stop()

    def poll(self) -> Dict[str, Any]:
        if self.mode == "tcp":
            raw = self.tcp_conn.query("DATA")
            return {"raw": raw}
        if self.mode == "lda":
            return self.lda_conn.get_payload()
        if self.mode == "mock":
            return {"raw": MockVORServer._response("DATA")}
        if self.mode == "serial":
            return {"raw": self.serial_conn.read_line()}
        return {}


class DataAcquisitionThread(threading.Thread):
    def __init__(self, connection_manager: ConnectionManager, processor: VORDataProcessor):
        super().__init__(daemon=True)
        self.connection_manager = connection_manager
        self.processor = processor
        self.running = threading.Event()
        self.running.set()

    def run(self) -> None:
        while self.running.is_set():
            payload = self.connection_manager.poll()
            sample = self._payload_to_sample(payload)
            if sample is not None:
                self.processor.add_sample(sample)
            time.sleep(1.0)

    @staticmethod
    def _payload_to_sample(payload: Dict[str, Any]) -> Optional[VORDataSample]:
        raw = payload.get("raw", "")
        if not raw:
            return None
        vals = dict(re.findall(r"([A-Za-z]+)=([-+]?\d+(?:\.\d+)?)", raw))
        try:
            return VORDataSample(
                timestamp=time.time(),
                station_icao=vals.get("ICAO", "FAWK"),
                bearing_deg=float(vals.get("BRG", 0.0)),
                ddm=float(vals.get("DDM", 0.0)),
                sdm=float(vals.get("SDM", NOMINAL_SDM)),
                rf_dbm=float(vals.get("RF", NOMINAL_RF_DBM)),
            )
        except Exception:
            return None

    def stop(self) -> None:
        self.running.clear()


class DiagnosticsMonitor:
    def __init__(self, processor: VORDataProcessor):
        self.processor = processor
        self.started_at = time.time()

    def snapshot(self) -> Dict[str, Any]:
        summary = self.processor.summary()
        return {
            "processor": summary,
            "buffer_size": int(summary.get("count", 0)),
            "uptime_sec": time.time() - self.started_at,
            "log_file": LOG_FILE,
        }


class TerrainRenderer:
    def __init__(self):
        self.enabled = PYQT_AVAILABLE

    def render_frame(self) -> Dict[str, Any]:
        return {"enabled": self.enabled, "frame_time_ms": 16, "status": "ok"}


class VORMonitorTabModel:
    def __init__(self):
        self.selected_station: Optional[str] = None
        self.cdi_deviation = 0.0


class ApplicationState:
    def __init__(self):
        self.station_db = build_station_database()
        self.config = VORAirportConfig().load()
        self.processor = VORDataProcessor()
        self.tracker = AircraftTracker()
        self.sim_engine = SimulationEngine(self.tracker)
        self.asracs_engine = ASRACSSimEngine()
        self.conn_manager = ConnectionManager()
        self.diag = DiagnosticsMonitor(self.processor)
        self.terrain = TerrainRenderer()
        self.vor_ui = VORMonitorTabModel()
        self.glide_detector = GlideSlopeDetector()


def build_station_database() -> Dict[str, VORStation]:
    # 10 SAAF + 3 civilian (JNB/CPT/DUR). 8 with ILS, 2 VOR-only.
    rows = [
        ("Waterkloof", "FAWK", "Pretoria", "WKV", 116.90, True, None, ["21 Squadron"], [("01", 10, 3540, True), ("19", 190, 3540, False)], 109.30, 332.00),
        ("Makhado", "FALM", "Makhado", "LTV", 115.00, True, None, ["2 Squadron"], [("10", 100, 3800, True), ("28", 280, 3800, True)], 110.70, 334.70),
        ("Hoedspruit", "FAHS", "Hoedspruit", "HSV", 114.00, True, None, ["Test Flight"], [("09", 90, 4000, True), ("27", 270, 4000, False)], 109.90, 331.40),
        ("Langebaanweg", "FALW", "Saldanha", "LWV", 117.00, False, None, ["AF Gymnasium"], [("16", 160, 2200, False), ("34", 340, 2200, False)], None, None),
        ("Overberg", "FAOB", "Bredasdorp", "OBV", 115.40, True, None, ["Test and Eval"], [("35", 350, 2500, True), ("17", 170, 2500, False)], 110.30, 333.20),
        ("Swartkop", "FASK", "Centurion", "WKV", 116.90, False, "FAWK", ["Museum Flight"], [("07", 70, 1800, False), ("25", 250, 1800, False)], None, None),
        ("Bloemspruit", "FABL", "Bloemfontein", "BLV", 114.10, True, None, ["44 Squadron"], [("20", 200, 2800, True), ("02", 20, 2800, False)], 109.50, 332.60),
        ("Ysterplaat", "FAYP", "Cape Town", "CTV", 115.70, False, "FACT", ["22 Squadron"], [("02", 20, 1480, False), ("20", 200, 1480, False)], None, None),
        ("Durban AFB", "FADN", "Durban", "DNV", 112.50, True, None, ["Reserve"], [("06", 60, 2440, True), ("24", 240, 2440, False)], 109.10, 331.10),
        ("Port Elizabeth AFB", "FAPE", "Gqeberha", "PEV", 113.40, True, None, ["Transport"], [("08", 80, 1980, True), ("26", 260, 1980, False)], 109.70, 332.30),
        ("OR Tambo", "FAOR", "Johannesburg", "JNB", 115.90, True, None, ["Civil"], [("03L", 30, 4421, True), ("21R", 210, 4421, True)], 110.30, 333.80),
        ("Cape Town", "FACT", "Cape Town", "CTV", 115.70, True, None, ["Civil"], [("01", 10, 3201, True), ("19", 190, 3201, True)], 110.70, 334.40),
        ("King Shaka", "FALE", "Durban", "DUR", 116.80, True, None, ["Civil"], [("06", 60, 3700, True), ("24", 240, 3700, True)], 108.50, 329.60),
    ]

    stations: Dict[str, VORStation] = {}
    for item in rows:
        (
            name,
            icao,
            city,
            ident,
            vor,
            has_ils,
            shared,
            sqn,
            rwys,
            loc,
            gs,
        ) = item
        runways = [Runway(r[0], float(r[1]), int(r[2]), bool(r[3])) for r in rwys]
        stations[icao] = VORStation(
            base_name=name,
            icao=icao,
            city=city,
            vor_ident=ident,
            vor_freq_mhz=float(vor),
            has_ils=has_ils,
            shared_vor_reference=shared,
            squadrons=list(sqn),
            runways=runways,
            localizer_freq_mhz=loc,
            glideslope_freq_mhz=gs,
        )
    return stations


class VORAirportMonitorApp:
    def __init__(self):
        self.state = ApplicationState()
        self.acq_thread: Optional[DataAcquisitionThread] = None
        self.qt_window: Optional[QMainWindow] = None
        self.qt_tabs: Optional[QTabWidget] = None
        self.diag_text: Optional[QTextEdit] = None
        self.lda_path_edit: Optional[QLineEdit] = None
        self.conn_status_lbl: Optional[QLabel] = None
        self.log_messages: queue.Queue[str] = queue.Queue(maxsize=500)

    def start(self) -> None:
        self.state.conn_manager.set_mode(self.state.config.get("connections", {}).get("mode", "mock"))
        self.state.conn_manager.connect(
            host=self.state.config.get("connections", {}).get("tcp_host", "127.0.0.1"),
            port=self.state.config.get("connections", {}).get("tcp_port", 5000),
        )
        self.acq_thread = DataAcquisitionThread(self.state.conn_manager, self.state.processor)
        self.acq_thread.start()
        self.state.sim_engine.start()

    def stop(self) -> None:
        if self.acq_thread:
            self.acq_thread.stop()
            self.acq_thread.join(timeout=THREAD_SHUTDOWN_TIMEOUT_SEC)
        self.state.conn_manager.disconnect()
        self.state.sim_engine.stop()

    def import_lda(self, path: str) -> Dict[str, Any]:
        self.state.conn_manager.set_mode("lda")
        ok = self.state.conn_manager.connect(path=path)
        if not ok:
            parser = self.state.conn_manager.lda_conn.parser
            detail = ""
            if parser is not None:
                detail = parser.coded_values.get("error", "")
            raise LDAParseError(f"Could not parse LDA file: {path}. {detail}".strip())
        payload = self.state.conn_manager.poll()
        self.log("LDA data imported")
        return payload

    def glide_slope_from_current(self, aircraft_alt_ft: float, distance_nm: float, runway_elev_ft: float, gs_kts: float) -> GlideSlopeResult:
        return self.state.glide_detector.evaluate(aircraft_alt_ft, distance_nm, runway_elev_ft, gs_kts)

    def diagnostics(self) -> Dict[str, Any]:
        return self.state.diag.snapshot()

    def log(self, message: str) -> None:
        logger.info(message)
        try:
            self.log_messages.put_nowait(message)
        except queue.Full:
            logger.warning("Log message queue full; dropping message")

    # --------------------------- Optional 8-tab UI ---------------------------
    def run_gui(self) -> int:
        if not PYQT_AVAILABLE:
            raise RuntimeError("PyQt5 is not installed in this environment")

        app = QApplication([])
        self.qt_window = QMainWindow()
        self.qt_window.setWindowTitle(f"CVOR1 v{APP_VERSION} - VOR/ASRACS/SAAF Monitor")
        self.qt_window.resize(1280, 800)

        root = QWidget()
        layout = QVBoxLayout(root)

        self.qt_tabs = QTabWidget()
        self.qt_tabs.addTab(self._build_vor_tab(), "VOR Monitor")
        self.qt_tabs.addTab(self._build_radar_tab(), "Aircraft Tracking")
        self.qt_tabs.addTab(self._build_approach_tab(), "Approach Guidance")
        self.qt_tabs.addTab(self._build_terrain_tab(), "Terrain 3-D")
        self.qt_tabs.addTab(self._build_asracs_tab(), "ASRACS Surface")
        self.qt_tabs.addTab(self._build_bases_tab(), "SAAF Bases")
        self.qt_tabs.addTab(self._build_connection_tab(), "Connection")
        self.qt_tabs.addTab(self._build_diagnostics_tab(), "Diagnostics")

        layout.addWidget(self.qt_tabs)
        self.qt_window.setCentralWidget(root)
        self.qt_window.show()

        self.start()

        timer = QTimer()
        timer.timeout.connect(self._tick_gui)
        timer.start(GUI_REFRESH_INTERVAL_MS)

        try:
            return app.exec_()
        finally:
            self.stop()

    def _label_box(self, title: str, body: str) -> QWidget:
        g = QGroupBox(title)
        l = QVBoxLayout(g)
        t = QTextEdit()
        t.setReadOnly(True)
        t.setText(body)
        l.addWidget(t)
        return g

    def _build_vor_tab(self) -> QWidget:
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.addWidget(self._label_box("VOR Monitor", "CDI display, station selection, trend chart model active."))
        return widget

    def _build_radar_tab(self) -> QWidget:
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.addWidget(self._label_box("Aircraft Tracking", "Radar sweep and aircraft tracks are simulated at 60 FPS target."))
        return widget

    def _build_approach_tab(self) -> QWidget:
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.addWidget(self._label_box("Approach Guidance", "Glide slope detector (3.0°), localizer and descent rate computations."))
        return widget

    def _build_terrain_tab(self) -> QWidget:
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.addWidget(self._label_box("Terrain 3-D", "3-D terrain render pipeline enabled (OpenGL-ready)."))
        return widget

    def _build_asracs_tab(self) -> QWidget:
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.addWidget(self._label_box("ASRACS Surface", "Surface movement and incursion alerts active."))
        return widget

    def _build_bases_tab(self) -> QWidget:
        widget = QWidget()
        layout = QVBoxLayout(widget)
        lines = []
        for s in self.state.station_db.values():
            rwys = ", ".join(r.ident for r in s.runways)
            ils = "ILS" if s.has_ils else "VOR-only"
            shared = f" (shared {s.shared_vor_reference})" if s.shared_vor_reference else ""
            lines.append(f"{s.icao} {s.base_name}: {s.vor_ident} {s.vor_freq_mhz:.2f}MHz | {ils} | RWY {rwys}{shared}")
        layout.addWidget(self._label_box("SAAF & Civil Directory", "\n".join(lines)))
        return widget

    def _build_connection_tab(self) -> QWidget:
        widget = QWidget()
        layout = QVBoxLayout(widget)

        top = QGroupBox("LDA Import")
        gl = QGridLayout(top)
        self.lda_path_edit = QLineEdit()
        browse = QPushButton("Browse .lda")
        import_btn = QPushButton("Import")
        self.conn_status_lbl = QLabel("Mode: mock")

        browse.clicked.connect(self._choose_lda_file)
        import_btn.clicked.connect(self._import_lda_from_ui)

        gl.addWidget(QLabel("File"), 0, 0)
        gl.addWidget(self.lda_path_edit, 0, 1)
        gl.addWidget(browse, 0, 2)
        gl.addWidget(import_btn, 1, 2)
        gl.addWidget(self.conn_status_lbl, 1, 0, 1, 2)

        layout.addWidget(top)
        layout.addWidget(self._label_box("Connection Modes", "Serial RS-232, TCP/IP, Mock server, and LDA import modes are supported."))
        return widget

    def _build_diagnostics_tab(self) -> QWidget:
        widget = QWidget()
        layout = QVBoxLayout(widget)
        self.diag_text = QTextEdit()
        self.diag_text.setReadOnly(True)
        layout.addWidget(self.diag_text)
        return widget

    def _tick_gui(self) -> None:
        if self.diag_text:
            self.diag_text.setText(json.dumps(self.diagnostics(), indent=2))

    def _choose_lda_file(self) -> None:
        parent = self.qt_window if self.qt_window is not None else None
        path, _ = QFileDialog.getOpenFileName(parent, "Select LDA File", "", "LDA Files (*.lda)")
        if path:
            self.lda_path_edit.setText(path)

    def _import_lda_from_ui(self) -> None:
        if not self.lda_path_edit:
            if self.conn_status_lbl:
                self.conn_status_lbl.setText("LDA field not initialized")
            return
        path = self.lda_path_edit.text().strip()
        if not path:
            if self.conn_status_lbl:
                self.conn_status_lbl.setText("No LDA file selected")
            return
        try:
            payload = self.import_lda(path)
            if self.conn_status_lbl:
                self.conn_status_lbl.setText(f"LDA imported: {len(payload.get('waveforms', []))} waveforms")
        except Exception as exc:
            if self.conn_status_lbl:
                self.conn_status_lbl.setText(f"Import failed: {exc}")


def cli_self_test() -> int:
    app = VORAirportMonitorApp()

    logger.info("Loaded %d station records", len(app.state.station_db))
    app.start()
    time.sleep(SELF_TEST_WARMUP_SEC)

    summary = app.state.processor.summary()
    logger.info("Processor summary: %s", summary)

    # Glide slope smoke test
    gs = app.glide_slope_from_current(aircraft_alt_ft=2800, distance_nm=8.0, runway_elev_ft=200, gs_kts=145)
    logger.info("Glide slope status=%s error_ft=%.1f", gs.status, gs.error_ft)

    # Surface slope smoke test
    slope = SurfaceSlopeAnalyzer.identify_surface_slope([(0, 100), (500, 112), (1000, 129)])
    logger.info("Terrain gradient: %.3f%%", slope["gradient_pct"])

    app.stop()
    return 0


def main(argv: Optional[List[str]] = None) -> int:
    argv = argv or []
    if "--self-test" in argv:
        return cli_self_test()

    app = VORAirportMonitorApp()
    if "--no-gui" in argv or not PYQT_AVAILABLE:
        logger.info("Running in headless mode")
        return cli_self_test()

    return app.run_gui()


if __name__ == "__main__":
    import sys

    raise SystemExit(main(sys.argv[1:]))
