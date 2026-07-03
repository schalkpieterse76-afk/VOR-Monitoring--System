#!/usr/bin/env python3
"""
VOR / ASRACS / SAAF Airport Monitoring System v5.3
==================================================
Production-ready monitoring, simulation, diagnostics and LDA import utility for
South African Air Force bases and selected civilian airports.

v5.3 highlights
---------------
* Complete SAAF base and VOR database integration
* LDA parser for Thales / Rohde & Schwarz ILS load-data files
* ASRACS surface-movement simulation with runway-incursion detection
* En-route aircraft simulation, terrain visualisation and approach guidance
* Serial / TCP connectivity with mock-server support
* Robust timer-driven PyQt5 application architecture
"""

import sys
import logging
import csv
import socket
import yaml
import math
import random
import time
import re
import struct
import traceback
import copy
from dataclasses import dataclass, field
from pathlib import Path
from datetime import datetime
from typing import List, Dict, Optional, Tuple
from collections import deque
from threading import Lock, Thread, Event

import numpy as np
import serial
import serial.tools.list_ports

from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QTabWidget,
    QAction,
    QLabel,
    QLineEdit,
    QPushButton,
    QGridLayout,
    QMessageBox,
    QTableWidget,
    QTableWidgetItem,
    QHeaderView,
    QComboBox,
    QGroupBox,
    QFileDialog,
    QTextEdit,
    QToolBar,
    QSplitter,
    QOpenGLWidget,
    QScrollArea,
    QListWidget,
    QListWidgetItem,
    QFrame,
    QFormLayout,
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QThread, QPoint, QRectF, QPointF
from PyQt5.QtGui import (
    QColor,
    QFont,
    QPainter,
    QPen,
    QBrush,
    QPolygon,
    QPalette,
    QPolygonF,
    QStandardItem,
    QIcon,
    QPainterPath,
)
from PyQt5.QtChart import QChart, QChartView, QLineSeries

from OpenGL.GL import *
from OpenGL.GLU import *

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    handlers=[logging.FileHandler("vor_monitor.log"), logging.StreamHandler()],
)
logger = logging.getLogger(__name__)

CONFIG_PATH = Path(__file__).with_name("vor_config.yaml")
EARTH_RADIUS_NM = 3440.065
NM_TO_METERS = 1852.0
FT_PER_NM_AT_3_DEG = math.tan(math.radians(3.0)) * 6076.12


DEFAULT_SAAF_BASES: Dict[str, Dict] = {
    "FAWK": {
        "name": "AFB Waterkloof",
        "icao": "FAWK",
        "lat": -25.8300,
        "lon": 28.2225,
        "elev_ft": 4944,
        "vor_ident": "WKV",
        "vor_freq": 116.90,
        "vor_type": "VORTAC",
        "ils": True,
        "ils_rwy": "01",
        "ils_freq": 111.50,
        "glide_angle": 3.0,
        "runways": ["01/19"],
        "rwy_length_m": 4500,
        "squadrons": ["21 Sqn (C-130)", "28 Sqn (Transport)"],
        "twr_freq": 118.1,
        "app_freq": 119.1,
        "atis_freq": 127.65,
        "gnd_freq": 121.7,
        "ndb_freq": 315,
        "ndb_ident": "WK",
        "description": "Primary SAAF strategic base near Pretoria",
    },
    "FALM": {
        "name": "AFB Makhado",
        "icao": "FALM",
        "lat": -23.1600,
        "lon": 29.6967,
        "elev_ft": 3900,
        "vor_ident": "LTV",
        "vor_freq": 115.00,
        "vor_type": "VOR/DME",
        "ils": True,
        "ils_rwy": "10/28",
        "ils_freq": 110.10,
        "glide_angle": 3.0,
        "runways": ["10/28"],
        "rwy_length_m": 4020,
        "squadrons": ["2 Sqn (Gripen)", "85 CFS (Hawk)"],
        "twr_freq": 118.3,
        "app_freq": 120.1,
        "atis_freq": 126.2,
        "gnd_freq": 121.9,
        "ndb_freq": 457,
        "ndb_ident": "MK",
        "description": "Premier fighter base supporting Gripen and Hawk operations",
    },
    "FAHS": {
        "name": "AFB Hoedspruit",
        "icao": "FAHS",
        "lat": -24.3547,
        "lon": 31.0503,
        "elev_ft": 1743,
        "vor_ident": "HSV",
        "vor_freq": 114.00,
        "vor_type": "VOR/DME",
        "ils": True,
        "ils_rwy": "09",
        "ils_freq": 109.50,
        "glide_angle": 3.0,
        "runways": ["09/27"],
        "rwy_length_m": 3991,
        "squadrons": ["17 Sqn (Rooivalk)", "19 Sqn (Oryx)"],
        "twr_freq": 118.5,
        "app_freq": 119.8,
        "atis_freq": 125.3,
        "gnd_freq": 121.8,
        "ndb_freq": 265,
        "ndb_ident": "HA",
        "description": "Lowveld operational base supporting helicopter and joint-force missions",
    },
    "FALW": {
        "name": "AFB Langebaanweg",
        "icao": "FALW",
        "lat": -32.9689,
        "lon": 18.1653,
        "elev_ft": 151,
        "vor_ident": "LWV",
        "vor_freq": 117.00,
        "vor_type": "VORTAC",
        "ils": False,
        "ils_rwy": "",
        "ils_freq": None,
        "glide_angle": 3.0,
        "runways": ["01/19"],
        "rwy_length_m": 2430,
        "squadrons": ["41 Sqn (PC-7 Mk II)", "Flight Test Support"],
        "twr_freq": 118.7,
        "app_freq": 119.7,
        "atis_freq": 126.0,
        "gnd_freq": 121.6,
        "ndb_freq": 345,
        "ndb_ident": "LW",
        "description": "Primary SAAF fixed-wing training base on the West Coast",
    },
    "FAOB": {
        "name": "AFB Overberg",
        "icao": "FAOB",
        "lat": -34.5547,
        "lon": 20.2506,
        "elev_ft": 52,
        "vor_ident": "OBV",
        "vor_freq": 115.40,
        "vor_type": "VOR/DME",
        "ils": True,
        "ils_rwy": "35",
        "ils_freq": 110.50,
        "glide_angle": 3.0,
        "runways": ["17/35"],
        "rwy_length_m": 3115,
        "squadrons": ["TFDC", "UAV Test Flights"],
        "twr_freq": 118.9,
        "app_freq": 120.3,
        "atis_freq": 125.8,
        "gnd_freq": 121.75,
        "ndb_freq": 428,
        "ndb_ident": "OB",
        "description": "Weapons, test and evaluation base with restricted-airspace support",
    },
    "FASK": {
        "name": "AFB Swartkop",
        "icao": "FASK",
        "lat": -25.8069,
        "lon": 28.1644,
        "elev_ft": 4940,
        "vor_ident": "WKV",
        "vor_freq": 116.90,
        "vor_type": "VORTAC",
        "ils": False,
        "ils_rwy": "",
        "ils_freq": None,
        "glide_angle": 3.0,
        "runways": ["02/20"],
        "rwy_length_m": 2000,
        "squadrons": ["SAAF Museum", "Heritage Flights"],
        "twr_freq": 118.1,
        "app_freq": 119.1,
        "atis_freq": 127.65,
        "gnd_freq": 121.7,
        "ndb_freq": 390,
        "ndb_ident": "SK",
        "description": "Historic SAAF base and museum field using Waterkloof VORTAC coverage",
    },
    "FABL": {
        "name": "AFB Bloemspruit",
        "icao": "FABL",
        "lat": -29.0939,
        "lon": 26.3039,
        "elev_ft": 4458,
        "vor_ident": "BLV",
        "vor_freq": 114.10,
        "vor_type": "VOR/DME",
        "ils": True,
        "ils_rwy": "20",
        "ils_freq": 109.90,
        "glide_angle": 3.0,
        "runways": ["02/20"],
        "rwy_length_m": 2559,
        "squadrons": ["16 Sqn Det", "28 Sqn Det"],
        "twr_freq": 118.1,
        "app_freq": 119.6,
        "atis_freq": 126.8,
        "gnd_freq": 121.7,
        "ndb_freq": 380,
        "ndb_ident": "BL",
        "description": "Central South African support base co-located with civil airport infrastructure",
    },
    "FAYP": {
        "name": "AFB Ysterplaat",
        "icao": "FAYP",
        "lat": -33.9011,
        "lon": 18.4833,
        "elev_ft": 52,
        "vor_ident": "CTV",
        "vor_freq": 115.70,
        "vor_type": "VORTAC",
        "ils": False,
        "ils_rwy": "",
        "ils_freq": None,
        "glide_angle": 3.0,
        "runways": ["02/20"],
        "rwy_length_m": 1487,
        "squadrons": ["22 Sqn (Oryx)", "35 Sqn Det"],
        "twr_freq": 119.0,
        "app_freq": 120.2,
        "atis_freq": 127.3,
        "gnd_freq": 121.9,
        "ndb_freq": None,
        "ndb_ident": "",
        "description": "Western Cape support base served by Cape Town VORTAC coverage",
    },
    "FADN": {
        "name": "AFB Durban",
        "icao": "FADN",
        "lat": -29.9686,
        "lon": 30.9478,
        "elev_ft": 89,
        "vor_ident": "DNV",
        "vor_freq": 112.50,
        "vor_type": "VOR/DME",
        "ils": True,
        "ils_rwy": "06",
        "ils_freq": 109.70,
        "glide_angle": 3.0,
        "runways": ["06/24"],
        "rwy_length_m": 3700,
        "squadrons": ["35 Sqn Det", "Maritime Liaison"],
        "twr_freq": 118.6,
        "app_freq": 120.6,
        "atis_freq": 127.0,
        "gnd_freq": 121.8,
        "ndb_freq": 393,
        "ndb_ident": "DU",
        "description": "KwaZulu-Natal support base integrated with Durban area navigation aids",
    },
    "FAPE": {
        "name": "Port Elizabeth AFS",
        "icao": "FAPE",
        "lat": -33.9850,
        "lon": 25.6103,
        "elev_ft": 226,
        "vor_ident": "PEV",
        "vor_freq": 113.40,
        "vor_type": "VOR/DME",
        "ils": True,
        "ils_rwy": "08",
        "ils_freq": 110.30,
        "glide_angle": 3.0,
        "runways": ["08/26"],
        "rwy_length_m": 1980,
        "squadrons": ["Liaison Flight", "Reserve Support"],
        "twr_freq": 118.1,
        "app_freq": 120.0,
        "atis_freq": 127.4,
        "gnd_freq": 121.9,
        "ndb_freq": None,
        "ndb_ident": "",
        "description": "Eastern Cape support station using co-located civil navigation infrastructure",
    },
}

DEFAULT_AIRPORTS: Dict[str, Dict] = {
    "JNB": {"name": "OR Tambo International", "latitude": -26.1337, "longitude": 28.2420, "elevation": 5558},
    "CPT": {"name": "Cape Town International", "latitude": -33.9648, "longitude": 18.6017, "elevation": 151},
    "DUR": {"name": "King Shaka International", "latitude": -29.6144, "longitude": 31.1197, "elevation": 295},
    "FAWK": {"name": "AFB Waterkloof", "latitude": -25.8300, "longitude": 28.2225, "elevation": 4944},
    "FALM": {"name": "AFB Makhado", "latitude": -23.1600, "longitude": 29.6967, "elevation": 3900},
    "FAHS": {"name": "AFB Hoedspruit", "latitude": -24.3547, "longitude": 31.0503, "elevation": 1743},
    "FALW": {"name": "AFB Langebaanweg", "latitude": -32.9689, "longitude": 18.1653, "elevation": 151},
    "FAOB": {"name": "AFB Overberg", "latitude": -34.5547, "longitude": 20.2506, "elevation": 52},
    "FASK": {"name": "AFB Swartkop", "latitude": -25.8069, "longitude": 28.1644, "elevation": 4940},
    "FABL": {"name": "AFB Bloemspruit", "latitude": -29.0939, "longitude": 26.3039, "elevation": 4458},
    "FAYP": {"name": "AFB Ysterplaat", "latitude": -33.9011, "longitude": 18.4833, "elevation": 52},
    "FADN": {"name": "AFB Durban", "latitude": -29.9686, "longitude": 30.9478, "elevation": 89},
    "FAPE": {"name": "Port Elizabeth AFS", "latitude": -33.9850, "longitude": 25.6103, "elevation": 226},
}

DEFAULT_VOR_STATIONS: Dict[str, Dict] = {
    "JNB_VOR": {
        "airport": "JNB",
        "name": "OR Tambo VOR/DME",
        "ident": "JHB",
        "frequency": 114.90,
        "channel": 96,
        "latitude": -26.1337,
        "longitude": 28.2420,
        "type": "VOR/DME",
        "remarks": "Primary Johannesburg terminal-area VOR.",
        "vor_available": True,
        "ils_available": True,
        "ils_runways": [{"runway": "03L", "frequency": 110.30, "ident": "IJB", "glide_slope_deg": 3.0}],
        "ndb_freq": None,
        "ndb_ident": "",
    },
    "CPT_VOR": {
        "airport": "CPT",
        "name": "Cape Town VORTAC",
        "ident": "CTV",
        "frequency": 115.70,
        "channel": 104,
        "latitude": -33.9648,
        "longitude": 18.6017,
        "type": "VORTAC",
        "remarks": "Cape Town approach VOR; also supports Ysterplaat coverage.",
        "vor_available": True,
        "ils_available": True,
        "ils_runways": [{"runway": "02", "frequency": 110.90, "ident": "ICT", "glide_slope_deg": 3.0}],
        "ndb_freq": None,
        "ndb_ident": "",
    },
    "DUR_VOR": {
        "airport": "DUR",
        "name": "Durban VOR/DME",
        "ident": "DNV",
        "frequency": 112.50,
        "channel": 72,
        "latitude": -29.6144,
        "longitude": 31.1197,
        "type": "VOR/DME",
        "remarks": "King Shaka / Durban area VOR serving AFB Durban.",
        "vor_available": True,
        "ils_available": True,
        "ils_runways": [{"runway": "06", "frequency": 109.70, "ident": "IDN", "glide_slope_deg": 3.0}],
        "ndb_freq": 393,
        "ndb_ident": "DU",
    },
    "FAWK_VOR": {
        "airport": "FAWK",
        "name": "Waterkloof VORTAC",
        "ident": "WKV",
        "frequency": 116.90,
        "channel": 116,
        "latitude": -25.8300,
        "longitude": 28.2225,
        "type": "VORTAC",
        "remarks": "Primary SAAF strategic base VORTAC with ILS RWY 01.",
        "vor_available": True,
        "ils_available": True,
        "ils_runways": [{"runway": "01", "frequency": 111.50, "ident": "IWK", "glide_slope_deg": 3.0}],
        "ndb_freq": 315,
        "ndb_ident": "WK",
    },
    "FALM_VOR": {
        "airport": "FALM",
        "name": "Makhado VOR/DME",
        "ident": "LTV",
        "frequency": 115.00,
        "channel": 97,
        "latitude": -23.1600,
        "longitude": 29.6967,
        "type": "VOR/DME",
        "remarks": "Fighter-base VOR with dual-end ILS support.",
        "vor_available": True,
        "ils_available": True,
        "ils_runways": [
            {"runway": "10", "frequency": 110.10, "ident": "ILM", "glide_slope_deg": 3.0},
            {"runway": "28", "frequency": 111.30, "ident": "ILM2", "glide_slope_deg": 3.0},
        ],
        "ndb_freq": 457,
        "ndb_ident": "MK",
    },
    "FAHS_VOR": {
        "airport": "FAHS",
        "name": "Hoedspruit VOR/DME",
        "ident": "HSV",
        "frequency": 114.00,
        "channel": 87,
        "latitude": -24.3547,
        "longitude": 31.0503,
        "type": "VOR/DME",
        "remarks": "Lowveld VOR supporting helicopter operations.",
        "vor_available": True,
        "ils_available": True,
        "ils_runways": [{"runway": "09", "frequency": 109.50, "ident": "IHS", "glide_slope_deg": 3.0}],
        "ndb_freq": 265,
        "ndb_ident": "HA",
    },
    "FALW_VOR": {
        "airport": "FALW",
        "name": "Langebaanweg VORTAC",
        "ident": "LWV",
        "frequency": 117.00,
        "channel": 117,
        "latitude": -32.9689,
        "longitude": 18.1653,
        "type": "VORTAC",
        "remarks": "Training-base VORTAC; no ILS installed.",
        "vor_available": True,
        "ils_available": False,
        "ils_runways": [],
        "ndb_freq": 345,
        "ndb_ident": "LW",
    },
    "FAOB_VOR": {
        "airport": "FAOB",
        "name": "Overberg VOR/DME",
        "ident": "OBV",
        "frequency": 115.40,
        "channel": 101,
        "latitude": -34.5547,
        "longitude": 20.2506,
        "type": "VOR/DME",
        "remarks": "Test and evaluation VOR with ILS on RWY 35.",
        "vor_available": True,
        "ils_available": True,
        "ils_runways": [{"runway": "35", "frequency": 110.50, "ident": "IOB", "glide_slope_deg": 3.0}],
        "ndb_freq": 428,
        "ndb_ident": "OB",
    },
    "FASK_VOR": {
        "airport": "FASK",
        "name": "Swartkop / WKV Coverage",
        "ident": "WKV",
        "frequency": 116.90,
        "channel": 116,
        "latitude": -25.8069,
        "longitude": 28.1644,
        "type": "VORTAC",
        "remarks": "Swartkop uses Waterkloof WKV VORTAC and SK NDB.",
        "vor_available": True,
        "ils_available": False,
        "ils_runways": [],
        "ndb_freq": 390,
        "ndb_ident": "SK",
    },
    "FABL_VOR": {
        "airport": "FABL",
        "name": "Bloemfontein VOR/DME",
        "ident": "BLV",
        "frequency": 114.10,
        "channel": 88,
        "latitude": -29.0939,
        "longitude": 26.3039,
        "type": "VOR/DME",
        "remarks": "Shared civil-military VOR with ILS support.",
        "vor_available": True,
        "ils_available": True,
        "ils_runways": [{"runway": "20", "frequency": 109.90, "ident": "IBL", "glide_slope_deg": 3.0}],
        "ndb_freq": 380,
        "ndb_ident": "BL",
    },
    "FAYP_VOR": {
        "airport": "FAYP",
        "name": "Ysterplaat / CTV Coverage",
        "ident": "CTV",
        "frequency": 115.70,
        "channel": 104,
        "latitude": -33.9011,
        "longitude": 18.4833,
        "type": "VORTAC",
        "remarks": "Ysterplaat uses Cape Town VORTAC for terminal-area coverage.",
        "vor_available": True,
        "ils_available": False,
        "ils_runways": [],
        "ndb_freq": None,
        "ndb_ident": "",
    },
    "FADN_VOR": {
        "airport": "FADN",
        "name": "AFB Durban / DNV",
        "ident": "DNV",
        "frequency": 112.50,
        "channel": 72,
        "latitude": -29.9686,
        "longitude": 30.9478,
        "type": "VOR/DME",
        "remarks": "Durban-area VOR shared with military support operations.",
        "vor_available": True,
        "ils_available": True,
        "ils_runways": [{"runway": "06", "frequency": 109.70, "ident": "IDN", "glide_slope_deg": 3.0}],
        "ndb_freq": 393,
        "ndb_ident": "DU",
    },
    "FAPE_VOR": {
        "airport": "FAPE",
        "name": "Port Elizabeth VOR/DME",
        "ident": "PEV",
        "frequency": 113.40,
        "channel": 81,
        "latitude": -33.9850,
        "longitude": 25.6103,
        "type": "VOR/DME",
        "remarks": "Eastern Cape terminal VOR supporting AFS Port Elizabeth.",
        "vor_available": True,
        "ils_available": True,
        "ils_runways": [{"runway": "08", "frequency": 110.30, "ident": "IPE", "glide_slope_deg": 3.0}],
        "ndb_freq": None,
        "ndb_ident": "",
    },
}

JNB_LAYOUT_DATA: Dict[str, List[Dict]] = {
    "runways": [
        {
            "name": "03L/21R",
            "x1": -1300,
            "y1": 900,
            "x2": 1500,
            "y2": -900,
            "length_m": 4418,
            "width_m": 60,
            "heading": 33,
            "active": True,
        },
        {
            "name": "03R/21L",
            "x1": -1600,
            "y1": 1250,
            "x2": 1200,
            "y2": -550,
            "length_m": 3400,
            "width_m": 46,
            "heading": 33,
            "active": False,
        },
    ],
    "taxiways": [
        {"name": "A", "points": [(-1700, 1150), (-1200, 820), (-600, 420), (100, -50), (900, -580)]},
        {"name": "B", "points": [(-1200, 1380), (-700, 1050), (-50, 610), (740, 120), (1420, -330)]},
        {"name": "C", "points": [(-1500, 400), (-900, 50), (-200, -360), (500, -770)]},
        {"name": "D", "points": [(-500, 1450), (-150, 980), (310, 650), (870, 300), (1450, -120)]},
        {"name": "E", "points": [(-1850, -100), (-1240, -300), (-650, -530), (180, -880), (940, -1200)]},
    ],
    "aprons": [
        {"name": "North Apron", "rect": (-1880, 1260, 560, 360)},
        {"name": "Terminal Apron", "rect": (-420, 780, 620, 340)},
        {"name": "South Cargo", "rect": (300, 160, 720, 400)},
    ],
    "gates": [
        {"name": "A1", "x": -1780, "y": 1470},
        {"name": "A2", "x": -1680, "y": 1420},
        {"name": "A3", "x": -1580, "y": 1370},
        {"name": "B1", "x": -290, "y": 960},
        {"name": "B2", "x": -150, "y": 940},
        {"name": "B3", "x": -20, "y": 920},
        {"name": "C1", "x": 450, "y": 300},
        {"name": "C2", "x": 610, "y": 280},
        {"name": "C3", "x": 780, "y": 250},
    ],
    "hotspots": [
        {"name": "HS1", "x": -980, "y": 650, "radius": 90, "description": "Runway 03L/21R and Taxiway A crossing"},
        {"name": "HS2", "x": -480, "y": 430, "radius": 80, "description": "Parallel runway entry risk"},
        {"name": "HS3", "x": 220, "y": -70, "radius": 70, "description": "Terminal apron runway crossing"},
        {"name": "HS4", "x": 890, "y": -600, "radius": 75, "description": "Cargo taxiway merge point"},
    ],
}


TERRAIN_LIBRARY: Dict[str, List[float]] = {
    "FAWK": [4944, 4920, 4880, 4840, 4790, 4720, 4660, 4610, 4550, 4490, 4440, 4380, 4300, 4210, 4120],
    "FALM": [3900, 3880, 3840, 3780, 3720, 3650, 3560, 3490, 3420, 3350, 3290, 3230, 3180, 3110, 3040],
    "FAHS": [1743, 1735, 1710, 1680, 1650, 1610, 1570, 1540, 1505, 1460, 1420, 1380, 1340, 1290, 1240],
    "FALW": [151, 145, 138, 130, 124, 118, 111, 104, 98, 94, 90, 88, 86, 84, 82],
    "FAOB": [52, 49, 46, 43, 40, 38, 36, 33, 30, 28, 25, 23, 21, 18, 16],
    "FASK": [4940, 4915, 4880, 4840, 4795, 4730, 4680, 4620, 4565, 4510, 4460, 4400, 4350, 4290, 4230],
    "FABL": [4458, 4440, 4415, 4380, 4340, 4300, 4250, 4200, 4140, 4090, 4030, 3980, 3925, 3880, 3810],
    "FAYP": [52, 50, 48, 46, 44, 42, 39, 36, 32, 30, 28, 26, 25, 24, 23],
    "FADN": [89, 87, 84, 80, 77, 73, 69, 65, 61, 57, 54, 50, 47, 44, 41],
    "FAPE": [226, 220, 214, 208, 201, 195, 188, 180, 173, 167, 160, 154, 147, 140, 133],
    "JNB": [5558, 5510, 5450, 5370, 5290, 5220, 5140, 5060, 4990, 4920, 4850, 4780, 4700, 4620, 4550],
}


SYSTEM_OPERATIONS_REFERENCE = """
CVOR1 v5.3 embedded operations reference
This block provides on-screen and maintenance reference data for operators, maintainers and test personnel.
The reference is intentionally verbose so the deployed single-file application remains self-describing.

SECTION A - SAAF BASE OPERATING NOTES
FAWK NOTE 01 | Base AFB Waterkloof | Navaid WKV 116.90 MHz | Runway 01/19 | Primary strategic transport and VIP support hub.
FAWK NOTE 01 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAWK NOTE 01 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAWK NOTE 01 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAWK NOTE 01 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAWK NOTE 02 | Base AFB Waterkloof | Navaid WKV 116.90 MHz | Runway 01/19 | Primary strategic transport and VIP support hub.
FAWK NOTE 02 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAWK NOTE 02 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAWK NOTE 02 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAWK NOTE 02 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAWK NOTE 03 | Base AFB Waterkloof | Navaid WKV 116.90 MHz | Runway 01/19 | Primary strategic transport and VIP support hub.
FAWK NOTE 03 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAWK NOTE 03 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAWK NOTE 03 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAWK NOTE 03 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAWK NOTE 04 | Base AFB Waterkloof | Navaid WKV 116.90 MHz | Runway 01/19 | Primary strategic transport and VIP support hub.
FAWK NOTE 04 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAWK NOTE 04 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAWK NOTE 04 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAWK NOTE 04 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAWK NOTE 05 | Base AFB Waterkloof | Navaid WKV 116.90 MHz | Runway 01/19 | Primary strategic transport and VIP support hub.
FAWK NOTE 05 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAWK NOTE 05 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAWK NOTE 05 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAWK NOTE 05 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAWK NOTE 06 | Base AFB Waterkloof | Navaid WKV 116.90 MHz | Runway 01/19 | Primary strategic transport and VIP support hub.
FAWK NOTE 06 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAWK NOTE 06 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAWK NOTE 06 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAWK NOTE 06 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAWK NOTE 07 | Base AFB Waterkloof | Navaid WKV 116.90 MHz | Runway 01/19 | Primary strategic transport and VIP support hub.
FAWK NOTE 07 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAWK NOTE 07 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAWK NOTE 07 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAWK NOTE 07 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAWK NOTE 08 | Base AFB Waterkloof | Navaid WKV 116.90 MHz | Runway 01/19 | Primary strategic transport and VIP support hub.
FAWK NOTE 08 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAWK NOTE 08 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAWK NOTE 08 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAWK NOTE 08 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAWK NOTE 09 | Base AFB Waterkloof | Navaid WKV 116.90 MHz | Runway 01/19 | Primary strategic transport and VIP support hub.
FAWK NOTE 09 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAWK NOTE 09 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAWK NOTE 09 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAWK NOTE 09 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAWK NOTE 10 | Base AFB Waterkloof | Navaid WKV 116.90 MHz | Runway 01/19 | Primary strategic transport and VIP support hub.
FAWK NOTE 10 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAWK NOTE 10 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAWK NOTE 10 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAWK NOTE 10 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAWK NOTE 11 | Base AFB Waterkloof | Navaid WKV 116.90 MHz | Runway 01/19 | Primary strategic transport and VIP support hub.
FAWK NOTE 11 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAWK NOTE 11 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAWK NOTE 11 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAWK NOTE 11 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAWK NOTE 12 | Base AFB Waterkloof | Navaid WKV 116.90 MHz | Runway 01/19 | Primary strategic transport and VIP support hub.
FAWK NOTE 12 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAWK NOTE 12 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAWK NOTE 12 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAWK NOTE 12 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAWK NOTE 13 | Base AFB Waterkloof | Navaid WKV 116.90 MHz | Runway 01/19 | Primary strategic transport and VIP support hub.
FAWK NOTE 13 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAWK NOTE 13 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAWK NOTE 13 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAWK NOTE 13 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAWK NOTE 14 | Base AFB Waterkloof | Navaid WKV 116.90 MHz | Runway 01/19 | Primary strategic transport and VIP support hub.
FAWK NOTE 14 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAWK NOTE 14 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAWK NOTE 14 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAWK NOTE 14 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAWK NOTE 15 | Base AFB Waterkloof | Navaid WKV 116.90 MHz | Runway 01/19 | Primary strategic transport and VIP support hub.
FAWK NOTE 15 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAWK NOTE 15 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAWK NOTE 15 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAWK NOTE 15 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAWK NOTE 16 | Base AFB Waterkloof | Navaid WKV 116.90 MHz | Runway 01/19 | Primary strategic transport and VIP support hub.
FAWK NOTE 16 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAWK NOTE 16 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAWK NOTE 16 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAWK NOTE 16 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAWK NOTE 17 | Base AFB Waterkloof | Navaid WKV 116.90 MHz | Runway 01/19 | Primary strategic transport and VIP support hub.
FAWK NOTE 17 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAWK NOTE 17 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAWK NOTE 17 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAWK NOTE 17 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAWK NOTE 18 | Base AFB Waterkloof | Navaid WKV 116.90 MHz | Runway 01/19 | Primary strategic transport and VIP support hub.
FAWK NOTE 18 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAWK NOTE 18 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAWK NOTE 18 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAWK NOTE 18 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAWK NOTE 19 | Base AFB Waterkloof | Navaid WKV 116.90 MHz | Runway 01/19 | Primary strategic transport and VIP support hub.
FAWK NOTE 19 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAWK NOTE 19 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAWK NOTE 19 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAWK NOTE 19 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAWK NOTE 20 | Base AFB Waterkloof | Navaid WKV 116.90 MHz | Runway 01/19 | Primary strategic transport and VIP support hub.
FAWK NOTE 20 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAWK NOTE 20 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAWK NOTE 20 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAWK NOTE 20 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FALM NOTE 01 | Base AFB Makhado | Navaid LTV 115.00 MHz | Runway 10/28 | Fighter operations and weapons training base.
FALM NOTE 01 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FALM NOTE 01 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FALM NOTE 01 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FALM NOTE 01 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FALM NOTE 02 | Base AFB Makhado | Navaid LTV 115.00 MHz | Runway 10/28 | Fighter operations and weapons training base.
FALM NOTE 02 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FALM NOTE 02 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FALM NOTE 02 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FALM NOTE 02 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FALM NOTE 03 | Base AFB Makhado | Navaid LTV 115.00 MHz | Runway 10/28 | Fighter operations and weapons training base.
FALM NOTE 03 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FALM NOTE 03 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FALM NOTE 03 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FALM NOTE 03 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FALM NOTE 04 | Base AFB Makhado | Navaid LTV 115.00 MHz | Runway 10/28 | Fighter operations and weapons training base.
FALM NOTE 04 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FALM NOTE 04 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FALM NOTE 04 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FALM NOTE 04 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FALM NOTE 05 | Base AFB Makhado | Navaid LTV 115.00 MHz | Runway 10/28 | Fighter operations and weapons training base.
FALM NOTE 05 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FALM NOTE 05 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FALM NOTE 05 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FALM NOTE 05 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FALM NOTE 06 | Base AFB Makhado | Navaid LTV 115.00 MHz | Runway 10/28 | Fighter operations and weapons training base.
FALM NOTE 06 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FALM NOTE 06 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FALM NOTE 06 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FALM NOTE 06 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FALM NOTE 07 | Base AFB Makhado | Navaid LTV 115.00 MHz | Runway 10/28 | Fighter operations and weapons training base.
FALM NOTE 07 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FALM NOTE 07 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FALM NOTE 07 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FALM NOTE 07 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FALM NOTE 08 | Base AFB Makhado | Navaid LTV 115.00 MHz | Runway 10/28 | Fighter operations and weapons training base.
FALM NOTE 08 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FALM NOTE 08 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FALM NOTE 08 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FALM NOTE 08 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FALM NOTE 09 | Base AFB Makhado | Navaid LTV 115.00 MHz | Runway 10/28 | Fighter operations and weapons training base.
FALM NOTE 09 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FALM NOTE 09 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FALM NOTE 09 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FALM NOTE 09 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FALM NOTE 10 | Base AFB Makhado | Navaid LTV 115.00 MHz | Runway 10/28 | Fighter operations and weapons training base.
FALM NOTE 10 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FALM NOTE 10 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FALM NOTE 10 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FALM NOTE 10 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FALM NOTE 11 | Base AFB Makhado | Navaid LTV 115.00 MHz | Runway 10/28 | Fighter operations and weapons training base.
FALM NOTE 11 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FALM NOTE 11 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FALM NOTE 11 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FALM NOTE 11 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FALM NOTE 12 | Base AFB Makhado | Navaid LTV 115.00 MHz | Runway 10/28 | Fighter operations and weapons training base.
FALM NOTE 12 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FALM NOTE 12 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FALM NOTE 12 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FALM NOTE 12 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FALM NOTE 13 | Base AFB Makhado | Navaid LTV 115.00 MHz | Runway 10/28 | Fighter operations and weapons training base.
FALM NOTE 13 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FALM NOTE 13 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FALM NOTE 13 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FALM NOTE 13 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FALM NOTE 14 | Base AFB Makhado | Navaid LTV 115.00 MHz | Runway 10/28 | Fighter operations and weapons training base.
FALM NOTE 14 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FALM NOTE 14 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FALM NOTE 14 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FALM NOTE 14 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FALM NOTE 15 | Base AFB Makhado | Navaid LTV 115.00 MHz | Runway 10/28 | Fighter operations and weapons training base.
FALM NOTE 15 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FALM NOTE 15 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FALM NOTE 15 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FALM NOTE 15 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FALM NOTE 16 | Base AFB Makhado | Navaid LTV 115.00 MHz | Runway 10/28 | Fighter operations and weapons training base.
FALM NOTE 16 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FALM NOTE 16 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FALM NOTE 16 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FALM NOTE 16 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FALM NOTE 17 | Base AFB Makhado | Navaid LTV 115.00 MHz | Runway 10/28 | Fighter operations and weapons training base.
FALM NOTE 17 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FALM NOTE 17 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FALM NOTE 17 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FALM NOTE 17 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FALM NOTE 18 | Base AFB Makhado | Navaid LTV 115.00 MHz | Runway 10/28 | Fighter operations and weapons training base.
FALM NOTE 18 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FALM NOTE 18 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FALM NOTE 18 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FALM NOTE 18 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FALM NOTE 19 | Base AFB Makhado | Navaid LTV 115.00 MHz | Runway 10/28 | Fighter operations and weapons training base.
FALM NOTE 19 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FALM NOTE 19 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FALM NOTE 19 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FALM NOTE 19 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FALM NOTE 20 | Base AFB Makhado | Navaid LTV 115.00 MHz | Runway 10/28 | Fighter operations and weapons training base.
FALM NOTE 20 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FALM NOTE 20 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FALM NOTE 20 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FALM NOTE 20 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAHS NOTE 01 | Base AFB Hoedspruit | Navaid HSV 114.00 MHz | Runway 09/27 | Helicopter operations and joint-force support base.
FAHS NOTE 01 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAHS NOTE 01 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAHS NOTE 01 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAHS NOTE 01 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAHS NOTE 02 | Base AFB Hoedspruit | Navaid HSV 114.00 MHz | Runway 09/27 | Helicopter operations and joint-force support base.
FAHS NOTE 02 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAHS NOTE 02 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAHS NOTE 02 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAHS NOTE 02 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAHS NOTE 03 | Base AFB Hoedspruit | Navaid HSV 114.00 MHz | Runway 09/27 | Helicopter operations and joint-force support base.
FAHS NOTE 03 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAHS NOTE 03 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAHS NOTE 03 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAHS NOTE 03 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAHS NOTE 04 | Base AFB Hoedspruit | Navaid HSV 114.00 MHz | Runway 09/27 | Helicopter operations and joint-force support base.
FAHS NOTE 04 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAHS NOTE 04 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAHS NOTE 04 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAHS NOTE 04 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAHS NOTE 05 | Base AFB Hoedspruit | Navaid HSV 114.00 MHz | Runway 09/27 | Helicopter operations and joint-force support base.
FAHS NOTE 05 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAHS NOTE 05 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAHS NOTE 05 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAHS NOTE 05 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAHS NOTE 06 | Base AFB Hoedspruit | Navaid HSV 114.00 MHz | Runway 09/27 | Helicopter operations and joint-force support base.
FAHS NOTE 06 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAHS NOTE 06 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAHS NOTE 06 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAHS NOTE 06 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAHS NOTE 07 | Base AFB Hoedspruit | Navaid HSV 114.00 MHz | Runway 09/27 | Helicopter operations and joint-force support base.
FAHS NOTE 07 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAHS NOTE 07 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAHS NOTE 07 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAHS NOTE 07 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAHS NOTE 08 | Base AFB Hoedspruit | Navaid HSV 114.00 MHz | Runway 09/27 | Helicopter operations and joint-force support base.
FAHS NOTE 08 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAHS NOTE 08 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAHS NOTE 08 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAHS NOTE 08 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAHS NOTE 09 | Base AFB Hoedspruit | Navaid HSV 114.00 MHz | Runway 09/27 | Helicopter operations and joint-force support base.
FAHS NOTE 09 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAHS NOTE 09 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAHS NOTE 09 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAHS NOTE 09 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAHS NOTE 10 | Base AFB Hoedspruit | Navaid HSV 114.00 MHz | Runway 09/27 | Helicopter operations and joint-force support base.
FAHS NOTE 10 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAHS NOTE 10 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAHS NOTE 10 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAHS NOTE 10 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAHS NOTE 11 | Base AFB Hoedspruit | Navaid HSV 114.00 MHz | Runway 09/27 | Helicopter operations and joint-force support base.
FAHS NOTE 11 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAHS NOTE 11 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAHS NOTE 11 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAHS NOTE 11 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAHS NOTE 12 | Base AFB Hoedspruit | Navaid HSV 114.00 MHz | Runway 09/27 | Helicopter operations and joint-force support base.
FAHS NOTE 12 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAHS NOTE 12 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAHS NOTE 12 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAHS NOTE 12 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAHS NOTE 13 | Base AFB Hoedspruit | Navaid HSV 114.00 MHz | Runway 09/27 | Helicopter operations and joint-force support base.
FAHS NOTE 13 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAHS NOTE 13 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAHS NOTE 13 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAHS NOTE 13 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAHS NOTE 14 | Base AFB Hoedspruit | Navaid HSV 114.00 MHz | Runway 09/27 | Helicopter operations and joint-force support base.
FAHS NOTE 14 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAHS NOTE 14 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAHS NOTE 14 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAHS NOTE 14 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAHS NOTE 15 | Base AFB Hoedspruit | Navaid HSV 114.00 MHz | Runway 09/27 | Helicopter operations and joint-force support base.
FAHS NOTE 15 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAHS NOTE 15 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAHS NOTE 15 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAHS NOTE 15 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAHS NOTE 16 | Base AFB Hoedspruit | Navaid HSV 114.00 MHz | Runway 09/27 | Helicopter operations and joint-force support base.
FAHS NOTE 16 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAHS NOTE 16 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAHS NOTE 16 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAHS NOTE 16 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAHS NOTE 17 | Base AFB Hoedspruit | Navaid HSV 114.00 MHz | Runway 09/27 | Helicopter operations and joint-force support base.
FAHS NOTE 17 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAHS NOTE 17 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAHS NOTE 17 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAHS NOTE 17 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAHS NOTE 18 | Base AFB Hoedspruit | Navaid HSV 114.00 MHz | Runway 09/27 | Helicopter operations and joint-force support base.
FAHS NOTE 18 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAHS NOTE 18 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAHS NOTE 18 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAHS NOTE 18 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAHS NOTE 19 | Base AFB Hoedspruit | Navaid HSV 114.00 MHz | Runway 09/27 | Helicopter operations and joint-force support base.
FAHS NOTE 19 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAHS NOTE 19 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAHS NOTE 19 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAHS NOTE 19 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAHS NOTE 20 | Base AFB Hoedspruit | Navaid HSV 114.00 MHz | Runway 09/27 | Helicopter operations and joint-force support base.
FAHS NOTE 20 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAHS NOTE 20 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAHS NOTE 20 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAHS NOTE 20 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FALW NOTE 01 | Base AFB Langebaanweg | Navaid LWV 117.00 MHz | Runway 01/19 | Pilot training base on the West Coast.
FALW NOTE 01 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FALW NOTE 01 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FALW NOTE 01 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FALW NOTE 01 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FALW NOTE 02 | Base AFB Langebaanweg | Navaid LWV 117.00 MHz | Runway 01/19 | Pilot training base on the West Coast.
FALW NOTE 02 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FALW NOTE 02 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FALW NOTE 02 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FALW NOTE 02 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FALW NOTE 03 | Base AFB Langebaanweg | Navaid LWV 117.00 MHz | Runway 01/19 | Pilot training base on the West Coast.
FALW NOTE 03 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FALW NOTE 03 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FALW NOTE 03 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FALW NOTE 03 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FALW NOTE 04 | Base AFB Langebaanweg | Navaid LWV 117.00 MHz | Runway 01/19 | Pilot training base on the West Coast.
FALW NOTE 04 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FALW NOTE 04 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FALW NOTE 04 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FALW NOTE 04 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FALW NOTE 05 | Base AFB Langebaanweg | Navaid LWV 117.00 MHz | Runway 01/19 | Pilot training base on the West Coast.
FALW NOTE 05 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FALW NOTE 05 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FALW NOTE 05 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FALW NOTE 05 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FALW NOTE 06 | Base AFB Langebaanweg | Navaid LWV 117.00 MHz | Runway 01/19 | Pilot training base on the West Coast.
FALW NOTE 06 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FALW NOTE 06 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FALW NOTE 06 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FALW NOTE 06 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FALW NOTE 07 | Base AFB Langebaanweg | Navaid LWV 117.00 MHz | Runway 01/19 | Pilot training base on the West Coast.
FALW NOTE 07 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FALW NOTE 07 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FALW NOTE 07 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FALW NOTE 07 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FALW NOTE 08 | Base AFB Langebaanweg | Navaid LWV 117.00 MHz | Runway 01/19 | Pilot training base on the West Coast.
FALW NOTE 08 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FALW NOTE 08 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FALW NOTE 08 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FALW NOTE 08 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FALW NOTE 09 | Base AFB Langebaanweg | Navaid LWV 117.00 MHz | Runway 01/19 | Pilot training base on the West Coast.
FALW NOTE 09 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FALW NOTE 09 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FALW NOTE 09 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FALW NOTE 09 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FALW NOTE 10 | Base AFB Langebaanweg | Navaid LWV 117.00 MHz | Runway 01/19 | Pilot training base on the West Coast.
FALW NOTE 10 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FALW NOTE 10 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FALW NOTE 10 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FALW NOTE 10 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FALW NOTE 11 | Base AFB Langebaanweg | Navaid LWV 117.00 MHz | Runway 01/19 | Pilot training base on the West Coast.
FALW NOTE 11 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FALW NOTE 11 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FALW NOTE 11 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FALW NOTE 11 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FALW NOTE 12 | Base AFB Langebaanweg | Navaid LWV 117.00 MHz | Runway 01/19 | Pilot training base on the West Coast.
FALW NOTE 12 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FALW NOTE 12 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FALW NOTE 12 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FALW NOTE 12 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FALW NOTE 13 | Base AFB Langebaanweg | Navaid LWV 117.00 MHz | Runway 01/19 | Pilot training base on the West Coast.
FALW NOTE 13 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FALW NOTE 13 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FALW NOTE 13 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FALW NOTE 13 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FALW NOTE 14 | Base AFB Langebaanweg | Navaid LWV 117.00 MHz | Runway 01/19 | Pilot training base on the West Coast.
FALW NOTE 14 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FALW NOTE 14 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FALW NOTE 14 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FALW NOTE 14 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FALW NOTE 15 | Base AFB Langebaanweg | Navaid LWV 117.00 MHz | Runway 01/19 | Pilot training base on the West Coast.
FALW NOTE 15 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FALW NOTE 15 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FALW NOTE 15 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FALW NOTE 15 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FALW NOTE 16 | Base AFB Langebaanweg | Navaid LWV 117.00 MHz | Runway 01/19 | Pilot training base on the West Coast.
FALW NOTE 16 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FALW NOTE 16 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FALW NOTE 16 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FALW NOTE 16 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FALW NOTE 17 | Base AFB Langebaanweg | Navaid LWV 117.00 MHz | Runway 01/19 | Pilot training base on the West Coast.
FALW NOTE 17 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FALW NOTE 17 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FALW NOTE 17 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FALW NOTE 17 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FALW NOTE 18 | Base AFB Langebaanweg | Navaid LWV 117.00 MHz | Runway 01/19 | Pilot training base on the West Coast.
FALW NOTE 18 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FALW NOTE 18 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FALW NOTE 18 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FALW NOTE 18 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FALW NOTE 19 | Base AFB Langebaanweg | Navaid LWV 117.00 MHz | Runway 01/19 | Pilot training base on the West Coast.
FALW NOTE 19 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FALW NOTE 19 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FALW NOTE 19 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FALW NOTE 19 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FALW NOTE 20 | Base AFB Langebaanweg | Navaid LWV 117.00 MHz | Runway 01/19 | Pilot training base on the West Coast.
FALW NOTE 20 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FALW NOTE 20 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FALW NOTE 20 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FALW NOTE 20 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAOB NOTE 01 | Base AFB Overberg | Navaid OBV 115.40 MHz | Runway 17/35 | Test and evaluation base with restricted airspace.
FAOB NOTE 01 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAOB NOTE 01 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAOB NOTE 01 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAOB NOTE 01 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAOB NOTE 02 | Base AFB Overberg | Navaid OBV 115.40 MHz | Runway 17/35 | Test and evaluation base with restricted airspace.
FAOB NOTE 02 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAOB NOTE 02 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAOB NOTE 02 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAOB NOTE 02 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAOB NOTE 03 | Base AFB Overberg | Navaid OBV 115.40 MHz | Runway 17/35 | Test and evaluation base with restricted airspace.
FAOB NOTE 03 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAOB NOTE 03 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAOB NOTE 03 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAOB NOTE 03 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAOB NOTE 04 | Base AFB Overberg | Navaid OBV 115.40 MHz | Runway 17/35 | Test and evaluation base with restricted airspace.
FAOB NOTE 04 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAOB NOTE 04 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAOB NOTE 04 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAOB NOTE 04 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAOB NOTE 05 | Base AFB Overberg | Navaid OBV 115.40 MHz | Runway 17/35 | Test and evaluation base with restricted airspace.
FAOB NOTE 05 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAOB NOTE 05 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAOB NOTE 05 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAOB NOTE 05 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAOB NOTE 06 | Base AFB Overberg | Navaid OBV 115.40 MHz | Runway 17/35 | Test and evaluation base with restricted airspace.
FAOB NOTE 06 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAOB NOTE 06 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAOB NOTE 06 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAOB NOTE 06 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAOB NOTE 07 | Base AFB Overberg | Navaid OBV 115.40 MHz | Runway 17/35 | Test and evaluation base with restricted airspace.
FAOB NOTE 07 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAOB NOTE 07 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAOB NOTE 07 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAOB NOTE 07 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAOB NOTE 08 | Base AFB Overberg | Navaid OBV 115.40 MHz | Runway 17/35 | Test and evaluation base with restricted airspace.
FAOB NOTE 08 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAOB NOTE 08 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAOB NOTE 08 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAOB NOTE 08 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAOB NOTE 09 | Base AFB Overberg | Navaid OBV 115.40 MHz | Runway 17/35 | Test and evaluation base with restricted airspace.
FAOB NOTE 09 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAOB NOTE 09 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAOB NOTE 09 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAOB NOTE 09 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAOB NOTE 10 | Base AFB Overberg | Navaid OBV 115.40 MHz | Runway 17/35 | Test and evaluation base with restricted airspace.
FAOB NOTE 10 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAOB NOTE 10 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAOB NOTE 10 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAOB NOTE 10 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAOB NOTE 11 | Base AFB Overberg | Navaid OBV 115.40 MHz | Runway 17/35 | Test and evaluation base with restricted airspace.
FAOB NOTE 11 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAOB NOTE 11 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAOB NOTE 11 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAOB NOTE 11 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAOB NOTE 12 | Base AFB Overberg | Navaid OBV 115.40 MHz | Runway 17/35 | Test and evaluation base with restricted airspace.
FAOB NOTE 12 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAOB NOTE 12 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAOB NOTE 12 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAOB NOTE 12 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAOB NOTE 13 | Base AFB Overberg | Navaid OBV 115.40 MHz | Runway 17/35 | Test and evaluation base with restricted airspace.
FAOB NOTE 13 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAOB NOTE 13 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAOB NOTE 13 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAOB NOTE 13 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAOB NOTE 14 | Base AFB Overberg | Navaid OBV 115.40 MHz | Runway 17/35 | Test and evaluation base with restricted airspace.
FAOB NOTE 14 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAOB NOTE 14 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAOB NOTE 14 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAOB NOTE 14 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAOB NOTE 15 | Base AFB Overberg | Navaid OBV 115.40 MHz | Runway 17/35 | Test and evaluation base with restricted airspace.
FAOB NOTE 15 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAOB NOTE 15 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAOB NOTE 15 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAOB NOTE 15 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAOB NOTE 16 | Base AFB Overberg | Navaid OBV 115.40 MHz | Runway 17/35 | Test and evaluation base with restricted airspace.
FAOB NOTE 16 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAOB NOTE 16 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAOB NOTE 16 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAOB NOTE 16 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAOB NOTE 17 | Base AFB Overberg | Navaid OBV 115.40 MHz | Runway 17/35 | Test and evaluation base with restricted airspace.
FAOB NOTE 17 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAOB NOTE 17 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAOB NOTE 17 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAOB NOTE 17 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAOB NOTE 18 | Base AFB Overberg | Navaid OBV 115.40 MHz | Runway 17/35 | Test and evaluation base with restricted airspace.
FAOB NOTE 18 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAOB NOTE 18 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAOB NOTE 18 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAOB NOTE 18 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAOB NOTE 19 | Base AFB Overberg | Navaid OBV 115.40 MHz | Runway 17/35 | Test and evaluation base with restricted airspace.
FAOB NOTE 19 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAOB NOTE 19 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAOB NOTE 19 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAOB NOTE 19 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAOB NOTE 20 | Base AFB Overberg | Navaid OBV 115.40 MHz | Runway 17/35 | Test and evaluation base with restricted airspace.
FAOB NOTE 20 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAOB NOTE 20 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAOB NOTE 20 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAOB NOTE 20 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FASK NOTE 01 | Base AFB Swartkop | Navaid WKV 116.90 MHz | Runway 02/20 | Historic museum and ceremonial flight base.
FASK NOTE 01 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FASK NOTE 01 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FASK NOTE 01 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FASK NOTE 01 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FASK NOTE 02 | Base AFB Swartkop | Navaid WKV 116.90 MHz | Runway 02/20 | Historic museum and ceremonial flight base.
FASK NOTE 02 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FASK NOTE 02 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FASK NOTE 02 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FASK NOTE 02 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FASK NOTE 03 | Base AFB Swartkop | Navaid WKV 116.90 MHz | Runway 02/20 | Historic museum and ceremonial flight base.
FASK NOTE 03 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FASK NOTE 03 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FASK NOTE 03 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FASK NOTE 03 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FASK NOTE 04 | Base AFB Swartkop | Navaid WKV 116.90 MHz | Runway 02/20 | Historic museum and ceremonial flight base.
FASK NOTE 04 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FASK NOTE 04 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FASK NOTE 04 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FASK NOTE 04 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FASK NOTE 05 | Base AFB Swartkop | Navaid WKV 116.90 MHz | Runway 02/20 | Historic museum and ceremonial flight base.
FASK NOTE 05 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FASK NOTE 05 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FASK NOTE 05 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FASK NOTE 05 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FASK NOTE 06 | Base AFB Swartkop | Navaid WKV 116.90 MHz | Runway 02/20 | Historic museum and ceremonial flight base.
FASK NOTE 06 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FASK NOTE 06 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FASK NOTE 06 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FASK NOTE 06 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FASK NOTE 07 | Base AFB Swartkop | Navaid WKV 116.90 MHz | Runway 02/20 | Historic museum and ceremonial flight base.
FASK NOTE 07 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FASK NOTE 07 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FASK NOTE 07 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FASK NOTE 07 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FASK NOTE 08 | Base AFB Swartkop | Navaid WKV 116.90 MHz | Runway 02/20 | Historic museum and ceremonial flight base.
FASK NOTE 08 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FASK NOTE 08 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FASK NOTE 08 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FASK NOTE 08 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FASK NOTE 09 | Base AFB Swartkop | Navaid WKV 116.90 MHz | Runway 02/20 | Historic museum and ceremonial flight base.
FASK NOTE 09 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FASK NOTE 09 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FASK NOTE 09 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FASK NOTE 09 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FASK NOTE 10 | Base AFB Swartkop | Navaid WKV 116.90 MHz | Runway 02/20 | Historic museum and ceremonial flight base.
FASK NOTE 10 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FASK NOTE 10 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FASK NOTE 10 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FASK NOTE 10 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FASK NOTE 11 | Base AFB Swartkop | Navaid WKV 116.90 MHz | Runway 02/20 | Historic museum and ceremonial flight base.
FASK NOTE 11 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FASK NOTE 11 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FASK NOTE 11 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FASK NOTE 11 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FASK NOTE 12 | Base AFB Swartkop | Navaid WKV 116.90 MHz | Runway 02/20 | Historic museum and ceremonial flight base.
FASK NOTE 12 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FASK NOTE 12 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FASK NOTE 12 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FASK NOTE 12 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FASK NOTE 13 | Base AFB Swartkop | Navaid WKV 116.90 MHz | Runway 02/20 | Historic museum and ceremonial flight base.
FASK NOTE 13 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FASK NOTE 13 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FASK NOTE 13 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FASK NOTE 13 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FASK NOTE 14 | Base AFB Swartkop | Navaid WKV 116.90 MHz | Runway 02/20 | Historic museum and ceremonial flight base.
FASK NOTE 14 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FASK NOTE 14 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FASK NOTE 14 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FASK NOTE 14 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FASK NOTE 15 | Base AFB Swartkop | Navaid WKV 116.90 MHz | Runway 02/20 | Historic museum and ceremonial flight base.
FASK NOTE 15 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FASK NOTE 15 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FASK NOTE 15 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FASK NOTE 15 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FASK NOTE 16 | Base AFB Swartkop | Navaid WKV 116.90 MHz | Runway 02/20 | Historic museum and ceremonial flight base.
FASK NOTE 16 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FASK NOTE 16 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FASK NOTE 16 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FASK NOTE 16 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FASK NOTE 17 | Base AFB Swartkop | Navaid WKV 116.90 MHz | Runway 02/20 | Historic museum and ceremonial flight base.
FASK NOTE 17 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FASK NOTE 17 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FASK NOTE 17 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FASK NOTE 17 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FASK NOTE 18 | Base AFB Swartkop | Navaid WKV 116.90 MHz | Runway 02/20 | Historic museum and ceremonial flight base.
FASK NOTE 18 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FASK NOTE 18 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FASK NOTE 18 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FASK NOTE 18 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FASK NOTE 19 | Base AFB Swartkop | Navaid WKV 116.90 MHz | Runway 02/20 | Historic museum and ceremonial flight base.
FASK NOTE 19 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FASK NOTE 19 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FASK NOTE 19 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FASK NOTE 19 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FASK NOTE 20 | Base AFB Swartkop | Navaid WKV 116.90 MHz | Runway 02/20 | Historic museum and ceremonial flight base.
FASK NOTE 20 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FASK NOTE 20 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FASK NOTE 20 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FASK NOTE 20 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FABL NOTE 01 | Base AFB Bloemspruit | Navaid BLV 114.10 MHz | Runway 02/20 | Central support base with civil integration.
FABL NOTE 01 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FABL NOTE 01 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FABL NOTE 01 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FABL NOTE 01 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FABL NOTE 02 | Base AFB Bloemspruit | Navaid BLV 114.10 MHz | Runway 02/20 | Central support base with civil integration.
FABL NOTE 02 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FABL NOTE 02 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FABL NOTE 02 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FABL NOTE 02 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FABL NOTE 03 | Base AFB Bloemspruit | Navaid BLV 114.10 MHz | Runway 02/20 | Central support base with civil integration.
FABL NOTE 03 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FABL NOTE 03 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FABL NOTE 03 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FABL NOTE 03 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FABL NOTE 04 | Base AFB Bloemspruit | Navaid BLV 114.10 MHz | Runway 02/20 | Central support base with civil integration.
FABL NOTE 04 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FABL NOTE 04 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FABL NOTE 04 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FABL NOTE 04 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FABL NOTE 05 | Base AFB Bloemspruit | Navaid BLV 114.10 MHz | Runway 02/20 | Central support base with civil integration.
FABL NOTE 05 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FABL NOTE 05 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FABL NOTE 05 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FABL NOTE 05 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FABL NOTE 06 | Base AFB Bloemspruit | Navaid BLV 114.10 MHz | Runway 02/20 | Central support base with civil integration.
FABL NOTE 06 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FABL NOTE 06 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FABL NOTE 06 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FABL NOTE 06 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FABL NOTE 07 | Base AFB Bloemspruit | Navaid BLV 114.10 MHz | Runway 02/20 | Central support base with civil integration.
FABL NOTE 07 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FABL NOTE 07 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FABL NOTE 07 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FABL NOTE 07 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FABL NOTE 08 | Base AFB Bloemspruit | Navaid BLV 114.10 MHz | Runway 02/20 | Central support base with civil integration.
FABL NOTE 08 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FABL NOTE 08 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FABL NOTE 08 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FABL NOTE 08 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FABL NOTE 09 | Base AFB Bloemspruit | Navaid BLV 114.10 MHz | Runway 02/20 | Central support base with civil integration.
FABL NOTE 09 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FABL NOTE 09 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FABL NOTE 09 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FABL NOTE 09 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FABL NOTE 10 | Base AFB Bloemspruit | Navaid BLV 114.10 MHz | Runway 02/20 | Central support base with civil integration.
FABL NOTE 10 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FABL NOTE 10 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FABL NOTE 10 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FABL NOTE 10 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FABL NOTE 11 | Base AFB Bloemspruit | Navaid BLV 114.10 MHz | Runway 02/20 | Central support base with civil integration.
FABL NOTE 11 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FABL NOTE 11 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FABL NOTE 11 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FABL NOTE 11 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FABL NOTE 12 | Base AFB Bloemspruit | Navaid BLV 114.10 MHz | Runway 02/20 | Central support base with civil integration.
FABL NOTE 12 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FABL NOTE 12 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FABL NOTE 12 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FABL NOTE 12 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FABL NOTE 13 | Base AFB Bloemspruit | Navaid BLV 114.10 MHz | Runway 02/20 | Central support base with civil integration.
FABL NOTE 13 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FABL NOTE 13 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FABL NOTE 13 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FABL NOTE 13 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FABL NOTE 14 | Base AFB Bloemspruit | Navaid BLV 114.10 MHz | Runway 02/20 | Central support base with civil integration.
FABL NOTE 14 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FABL NOTE 14 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FABL NOTE 14 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FABL NOTE 14 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FABL NOTE 15 | Base AFB Bloemspruit | Navaid BLV 114.10 MHz | Runway 02/20 | Central support base with civil integration.
FABL NOTE 15 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FABL NOTE 15 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FABL NOTE 15 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FABL NOTE 15 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FABL NOTE 16 | Base AFB Bloemspruit | Navaid BLV 114.10 MHz | Runway 02/20 | Central support base with civil integration.
FABL NOTE 16 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FABL NOTE 16 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FABL NOTE 16 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FABL NOTE 16 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FABL NOTE 17 | Base AFB Bloemspruit | Navaid BLV 114.10 MHz | Runway 02/20 | Central support base with civil integration.
FABL NOTE 17 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FABL NOTE 17 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FABL NOTE 17 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FABL NOTE 17 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FABL NOTE 18 | Base AFB Bloemspruit | Navaid BLV 114.10 MHz | Runway 02/20 | Central support base with civil integration.
FABL NOTE 18 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FABL NOTE 18 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FABL NOTE 18 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FABL NOTE 18 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FABL NOTE 19 | Base AFB Bloemspruit | Navaid BLV 114.10 MHz | Runway 02/20 | Central support base with civil integration.
FABL NOTE 19 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FABL NOTE 19 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FABL NOTE 19 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FABL NOTE 19 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FABL NOTE 20 | Base AFB Bloemspruit | Navaid BLV 114.10 MHz | Runway 02/20 | Central support base with civil integration.
FABL NOTE 20 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FABL NOTE 20 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FABL NOTE 20 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FABL NOTE 20 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAYP NOTE 01 | Base AFB Ysterplaat | Navaid CTV 115.70 MHz | Runway 02/20 | Western Cape rotary support base.
FAYP NOTE 01 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAYP NOTE 01 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAYP NOTE 01 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAYP NOTE 01 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAYP NOTE 02 | Base AFB Ysterplaat | Navaid CTV 115.70 MHz | Runway 02/20 | Western Cape rotary support base.
FAYP NOTE 02 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAYP NOTE 02 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAYP NOTE 02 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAYP NOTE 02 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAYP NOTE 03 | Base AFB Ysterplaat | Navaid CTV 115.70 MHz | Runway 02/20 | Western Cape rotary support base.
FAYP NOTE 03 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAYP NOTE 03 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAYP NOTE 03 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAYP NOTE 03 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAYP NOTE 04 | Base AFB Ysterplaat | Navaid CTV 115.70 MHz | Runway 02/20 | Western Cape rotary support base.
FAYP NOTE 04 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAYP NOTE 04 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAYP NOTE 04 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAYP NOTE 04 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAYP NOTE 05 | Base AFB Ysterplaat | Navaid CTV 115.70 MHz | Runway 02/20 | Western Cape rotary support base.
FAYP NOTE 05 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAYP NOTE 05 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAYP NOTE 05 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAYP NOTE 05 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAYP NOTE 06 | Base AFB Ysterplaat | Navaid CTV 115.70 MHz | Runway 02/20 | Western Cape rotary support base.
FAYP NOTE 06 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAYP NOTE 06 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAYP NOTE 06 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAYP NOTE 06 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAYP NOTE 07 | Base AFB Ysterplaat | Navaid CTV 115.70 MHz | Runway 02/20 | Western Cape rotary support base.
FAYP NOTE 07 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAYP NOTE 07 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAYP NOTE 07 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAYP NOTE 07 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAYP NOTE 08 | Base AFB Ysterplaat | Navaid CTV 115.70 MHz | Runway 02/20 | Western Cape rotary support base.
FAYP NOTE 08 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAYP NOTE 08 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAYP NOTE 08 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAYP NOTE 08 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAYP NOTE 09 | Base AFB Ysterplaat | Navaid CTV 115.70 MHz | Runway 02/20 | Western Cape rotary support base.
FAYP NOTE 09 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAYP NOTE 09 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAYP NOTE 09 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAYP NOTE 09 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAYP NOTE 10 | Base AFB Ysterplaat | Navaid CTV 115.70 MHz | Runway 02/20 | Western Cape rotary support base.
FAYP NOTE 10 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAYP NOTE 10 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAYP NOTE 10 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAYP NOTE 10 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAYP NOTE 11 | Base AFB Ysterplaat | Navaid CTV 115.70 MHz | Runway 02/20 | Western Cape rotary support base.
FAYP NOTE 11 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAYP NOTE 11 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAYP NOTE 11 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAYP NOTE 11 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAYP NOTE 12 | Base AFB Ysterplaat | Navaid CTV 115.70 MHz | Runway 02/20 | Western Cape rotary support base.
FAYP NOTE 12 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAYP NOTE 12 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAYP NOTE 12 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAYP NOTE 12 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAYP NOTE 13 | Base AFB Ysterplaat | Navaid CTV 115.70 MHz | Runway 02/20 | Western Cape rotary support base.
FAYP NOTE 13 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAYP NOTE 13 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAYP NOTE 13 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAYP NOTE 13 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAYP NOTE 14 | Base AFB Ysterplaat | Navaid CTV 115.70 MHz | Runway 02/20 | Western Cape rotary support base.
FAYP NOTE 14 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAYP NOTE 14 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAYP NOTE 14 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAYP NOTE 14 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAYP NOTE 15 | Base AFB Ysterplaat | Navaid CTV 115.70 MHz | Runway 02/20 | Western Cape rotary support base.
FAYP NOTE 15 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAYP NOTE 15 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAYP NOTE 15 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAYP NOTE 15 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAYP NOTE 16 | Base AFB Ysterplaat | Navaid CTV 115.70 MHz | Runway 02/20 | Western Cape rotary support base.
FAYP NOTE 16 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAYP NOTE 16 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAYP NOTE 16 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAYP NOTE 16 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAYP NOTE 17 | Base AFB Ysterplaat | Navaid CTV 115.70 MHz | Runway 02/20 | Western Cape rotary support base.
FAYP NOTE 17 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAYP NOTE 17 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAYP NOTE 17 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAYP NOTE 17 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAYP NOTE 18 | Base AFB Ysterplaat | Navaid CTV 115.70 MHz | Runway 02/20 | Western Cape rotary support base.
FAYP NOTE 18 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAYP NOTE 18 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAYP NOTE 18 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAYP NOTE 18 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAYP NOTE 19 | Base AFB Ysterplaat | Navaid CTV 115.70 MHz | Runway 02/20 | Western Cape rotary support base.
FAYP NOTE 19 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAYP NOTE 19 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAYP NOTE 19 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAYP NOTE 19 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAYP NOTE 20 | Base AFB Ysterplaat | Navaid CTV 115.70 MHz | Runway 02/20 | Western Cape rotary support base.
FAYP NOTE 20 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAYP NOTE 20 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAYP NOTE 20 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAYP NOTE 20 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FADN NOTE 01 | Base AFB Durban | Navaid DNV 112.50 MHz | Runway 06/24 | KwaZulu-Natal maritime support base.
FADN NOTE 01 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FADN NOTE 01 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FADN NOTE 01 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FADN NOTE 01 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FADN NOTE 02 | Base AFB Durban | Navaid DNV 112.50 MHz | Runway 06/24 | KwaZulu-Natal maritime support base.
FADN NOTE 02 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FADN NOTE 02 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FADN NOTE 02 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FADN NOTE 02 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FADN NOTE 03 | Base AFB Durban | Navaid DNV 112.50 MHz | Runway 06/24 | KwaZulu-Natal maritime support base.
FADN NOTE 03 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FADN NOTE 03 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FADN NOTE 03 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FADN NOTE 03 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FADN NOTE 04 | Base AFB Durban | Navaid DNV 112.50 MHz | Runway 06/24 | KwaZulu-Natal maritime support base.
FADN NOTE 04 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FADN NOTE 04 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FADN NOTE 04 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FADN NOTE 04 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FADN NOTE 05 | Base AFB Durban | Navaid DNV 112.50 MHz | Runway 06/24 | KwaZulu-Natal maritime support base.
FADN NOTE 05 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FADN NOTE 05 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FADN NOTE 05 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FADN NOTE 05 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FADN NOTE 06 | Base AFB Durban | Navaid DNV 112.50 MHz | Runway 06/24 | KwaZulu-Natal maritime support base.
FADN NOTE 06 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FADN NOTE 06 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FADN NOTE 06 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FADN NOTE 06 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FADN NOTE 07 | Base AFB Durban | Navaid DNV 112.50 MHz | Runway 06/24 | KwaZulu-Natal maritime support base.
FADN NOTE 07 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FADN NOTE 07 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FADN NOTE 07 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FADN NOTE 07 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FADN NOTE 08 | Base AFB Durban | Navaid DNV 112.50 MHz | Runway 06/24 | KwaZulu-Natal maritime support base.
FADN NOTE 08 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FADN NOTE 08 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FADN NOTE 08 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FADN NOTE 08 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FADN NOTE 09 | Base AFB Durban | Navaid DNV 112.50 MHz | Runway 06/24 | KwaZulu-Natal maritime support base.
FADN NOTE 09 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FADN NOTE 09 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FADN NOTE 09 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FADN NOTE 09 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FADN NOTE 10 | Base AFB Durban | Navaid DNV 112.50 MHz | Runway 06/24 | KwaZulu-Natal maritime support base.
FADN NOTE 10 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FADN NOTE 10 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FADN NOTE 10 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FADN NOTE 10 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FADN NOTE 11 | Base AFB Durban | Navaid DNV 112.50 MHz | Runway 06/24 | KwaZulu-Natal maritime support base.
FADN NOTE 11 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FADN NOTE 11 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FADN NOTE 11 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FADN NOTE 11 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FADN NOTE 12 | Base AFB Durban | Navaid DNV 112.50 MHz | Runway 06/24 | KwaZulu-Natal maritime support base.
FADN NOTE 12 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FADN NOTE 12 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FADN NOTE 12 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FADN NOTE 12 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FADN NOTE 13 | Base AFB Durban | Navaid DNV 112.50 MHz | Runway 06/24 | KwaZulu-Natal maritime support base.
FADN NOTE 13 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FADN NOTE 13 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FADN NOTE 13 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FADN NOTE 13 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FADN NOTE 14 | Base AFB Durban | Navaid DNV 112.50 MHz | Runway 06/24 | KwaZulu-Natal maritime support base.
FADN NOTE 14 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FADN NOTE 14 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FADN NOTE 14 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FADN NOTE 14 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FADN NOTE 15 | Base AFB Durban | Navaid DNV 112.50 MHz | Runway 06/24 | KwaZulu-Natal maritime support base.
FADN NOTE 15 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FADN NOTE 15 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FADN NOTE 15 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FADN NOTE 15 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FADN NOTE 16 | Base AFB Durban | Navaid DNV 112.50 MHz | Runway 06/24 | KwaZulu-Natal maritime support base.
FADN NOTE 16 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FADN NOTE 16 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FADN NOTE 16 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FADN NOTE 16 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FADN NOTE 17 | Base AFB Durban | Navaid DNV 112.50 MHz | Runway 06/24 | KwaZulu-Natal maritime support base.
FADN NOTE 17 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FADN NOTE 17 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FADN NOTE 17 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FADN NOTE 17 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FADN NOTE 18 | Base AFB Durban | Navaid DNV 112.50 MHz | Runway 06/24 | KwaZulu-Natal maritime support base.
FADN NOTE 18 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FADN NOTE 18 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FADN NOTE 18 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FADN NOTE 18 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FADN NOTE 19 | Base AFB Durban | Navaid DNV 112.50 MHz | Runway 06/24 | KwaZulu-Natal maritime support base.
FADN NOTE 19 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FADN NOTE 19 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FADN NOTE 19 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FADN NOTE 19 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FADN NOTE 20 | Base AFB Durban | Navaid DNV 112.50 MHz | Runway 06/24 | KwaZulu-Natal maritime support base.
FADN NOTE 20 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FADN NOTE 20 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FADN NOTE 20 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FADN NOTE 20 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAPE NOTE 01 | Base Port Elizabeth AFS | Navaid PEV 113.40 MHz | Runway 08/26 | Eastern Cape liaison and support station.
FAPE NOTE 01 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAPE NOTE 01 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAPE NOTE 01 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAPE NOTE 01 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAPE NOTE 02 | Base Port Elizabeth AFS | Navaid PEV 113.40 MHz | Runway 08/26 | Eastern Cape liaison and support station.
FAPE NOTE 02 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAPE NOTE 02 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAPE NOTE 02 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAPE NOTE 02 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAPE NOTE 03 | Base Port Elizabeth AFS | Navaid PEV 113.40 MHz | Runway 08/26 | Eastern Cape liaison and support station.
FAPE NOTE 03 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAPE NOTE 03 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAPE NOTE 03 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAPE NOTE 03 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAPE NOTE 04 | Base Port Elizabeth AFS | Navaid PEV 113.40 MHz | Runway 08/26 | Eastern Cape liaison and support station.
FAPE NOTE 04 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAPE NOTE 04 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAPE NOTE 04 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAPE NOTE 04 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAPE NOTE 05 | Base Port Elizabeth AFS | Navaid PEV 113.40 MHz | Runway 08/26 | Eastern Cape liaison and support station.
FAPE NOTE 05 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAPE NOTE 05 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAPE NOTE 05 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAPE NOTE 05 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAPE NOTE 06 | Base Port Elizabeth AFS | Navaid PEV 113.40 MHz | Runway 08/26 | Eastern Cape liaison and support station.
FAPE NOTE 06 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAPE NOTE 06 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAPE NOTE 06 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAPE NOTE 06 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAPE NOTE 07 | Base Port Elizabeth AFS | Navaid PEV 113.40 MHz | Runway 08/26 | Eastern Cape liaison and support station.
FAPE NOTE 07 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAPE NOTE 07 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAPE NOTE 07 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAPE NOTE 07 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAPE NOTE 08 | Base Port Elizabeth AFS | Navaid PEV 113.40 MHz | Runway 08/26 | Eastern Cape liaison and support station.
FAPE NOTE 08 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAPE NOTE 08 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAPE NOTE 08 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAPE NOTE 08 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAPE NOTE 09 | Base Port Elizabeth AFS | Navaid PEV 113.40 MHz | Runway 08/26 | Eastern Cape liaison and support station.
FAPE NOTE 09 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAPE NOTE 09 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAPE NOTE 09 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAPE NOTE 09 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAPE NOTE 10 | Base Port Elizabeth AFS | Navaid PEV 113.40 MHz | Runway 08/26 | Eastern Cape liaison and support station.
FAPE NOTE 10 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAPE NOTE 10 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAPE NOTE 10 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAPE NOTE 10 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAPE NOTE 11 | Base Port Elizabeth AFS | Navaid PEV 113.40 MHz | Runway 08/26 | Eastern Cape liaison and support station.
FAPE NOTE 11 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAPE NOTE 11 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAPE NOTE 11 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAPE NOTE 11 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAPE NOTE 12 | Base Port Elizabeth AFS | Navaid PEV 113.40 MHz | Runway 08/26 | Eastern Cape liaison and support station.
FAPE NOTE 12 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAPE NOTE 12 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAPE NOTE 12 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAPE NOTE 12 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAPE NOTE 13 | Base Port Elizabeth AFS | Navaid PEV 113.40 MHz | Runway 08/26 | Eastern Cape liaison and support station.
FAPE NOTE 13 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAPE NOTE 13 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAPE NOTE 13 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAPE NOTE 13 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAPE NOTE 14 | Base Port Elizabeth AFS | Navaid PEV 113.40 MHz | Runway 08/26 | Eastern Cape liaison and support station.
FAPE NOTE 14 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAPE NOTE 14 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAPE NOTE 14 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAPE NOTE 14 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAPE NOTE 15 | Base Port Elizabeth AFS | Navaid PEV 113.40 MHz | Runway 08/26 | Eastern Cape liaison and support station.
FAPE NOTE 15 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAPE NOTE 15 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAPE NOTE 15 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAPE NOTE 15 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAPE NOTE 16 | Base Port Elizabeth AFS | Navaid PEV 113.40 MHz | Runway 08/26 | Eastern Cape liaison and support station.
FAPE NOTE 16 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAPE NOTE 16 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAPE NOTE 16 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAPE NOTE 16 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAPE NOTE 17 | Base Port Elizabeth AFS | Navaid PEV 113.40 MHz | Runway 08/26 | Eastern Cape liaison and support station.
FAPE NOTE 17 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAPE NOTE 17 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAPE NOTE 17 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAPE NOTE 17 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAPE NOTE 18 | Base Port Elizabeth AFS | Navaid PEV 113.40 MHz | Runway 08/26 | Eastern Cape liaison and support station.
FAPE NOTE 18 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAPE NOTE 18 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAPE NOTE 18 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAPE NOTE 18 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAPE NOTE 19 | Base Port Elizabeth AFS | Navaid PEV 113.40 MHz | Runway 08/26 | Eastern Cape liaison and support station.
FAPE NOTE 19 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAPE NOTE 19 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAPE NOTE 19 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAPE NOTE 19 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

FAPE NOTE 20 | Base Port Elizabeth AFS | Navaid PEV 113.40 MHz | Runway 08/26 | Eastern Cape liaison and support station.
FAPE NOTE 20 | Operational focus: monitoring, navigation integrity, runway status awareness and contingency readiness.
FAPE NOTE 20 | Controller cue: confirm ident, signal trend, glide-path availability and local weather impact.
FAPE NOTE 20 | Maintenance cue: review alarms, executive thresholds, standby indications and trend history before release.
FAPE NOTE 20 | Training cue: use simulator scenarios to rehearse degraded VOR, ILS and surface-movement states.

SECTION B - VOR STATION VALIDATION GUIDE
JNB_VOR CHECK 01 | Ident JHB | Frequency 114.90 MHz | Type VOR/DME.
JNB_VOR CHECK 01 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
JNB_VOR CHECK 01 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
JNB_VOR CHECK 01 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

JNB_VOR CHECK 02 | Ident JHB | Frequency 114.90 MHz | Type VOR/DME.
JNB_VOR CHECK 02 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
JNB_VOR CHECK 02 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
JNB_VOR CHECK 02 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

JNB_VOR CHECK 03 | Ident JHB | Frequency 114.90 MHz | Type VOR/DME.
JNB_VOR CHECK 03 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
JNB_VOR CHECK 03 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
JNB_VOR CHECK 03 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

JNB_VOR CHECK 04 | Ident JHB | Frequency 114.90 MHz | Type VOR/DME.
JNB_VOR CHECK 04 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
JNB_VOR CHECK 04 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
JNB_VOR CHECK 04 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

JNB_VOR CHECK 05 | Ident JHB | Frequency 114.90 MHz | Type VOR/DME.
JNB_VOR CHECK 05 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
JNB_VOR CHECK 05 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
JNB_VOR CHECK 05 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

JNB_VOR CHECK 06 | Ident JHB | Frequency 114.90 MHz | Type VOR/DME.
JNB_VOR CHECK 06 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
JNB_VOR CHECK 06 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
JNB_VOR CHECK 06 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

JNB_VOR CHECK 07 | Ident JHB | Frequency 114.90 MHz | Type VOR/DME.
JNB_VOR CHECK 07 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
JNB_VOR CHECK 07 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
JNB_VOR CHECK 07 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

JNB_VOR CHECK 08 | Ident JHB | Frequency 114.90 MHz | Type VOR/DME.
JNB_VOR CHECK 08 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
JNB_VOR CHECK 08 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
JNB_VOR CHECK 08 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

JNB_VOR CHECK 09 | Ident JHB | Frequency 114.90 MHz | Type VOR/DME.
JNB_VOR CHECK 09 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
JNB_VOR CHECK 09 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
JNB_VOR CHECK 09 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

JNB_VOR CHECK 10 | Ident JHB | Frequency 114.90 MHz | Type VOR/DME.
JNB_VOR CHECK 10 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
JNB_VOR CHECK 10 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
JNB_VOR CHECK 10 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

JNB_VOR CHECK 11 | Ident JHB | Frequency 114.90 MHz | Type VOR/DME.
JNB_VOR CHECK 11 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
JNB_VOR CHECK 11 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
JNB_VOR CHECK 11 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

JNB_VOR CHECK 12 | Ident JHB | Frequency 114.90 MHz | Type VOR/DME.
JNB_VOR CHECK 12 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
JNB_VOR CHECK 12 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
JNB_VOR CHECK 12 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

JNB_VOR CHECK 13 | Ident JHB | Frequency 114.90 MHz | Type VOR/DME.
JNB_VOR CHECK 13 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
JNB_VOR CHECK 13 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
JNB_VOR CHECK 13 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

JNB_VOR CHECK 14 | Ident JHB | Frequency 114.90 MHz | Type VOR/DME.
JNB_VOR CHECK 14 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
JNB_VOR CHECK 14 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
JNB_VOR CHECK 14 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

JNB_VOR CHECK 15 | Ident JHB | Frequency 114.90 MHz | Type VOR/DME.
JNB_VOR CHECK 15 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
JNB_VOR CHECK 15 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
JNB_VOR CHECK 15 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

CPT_VOR CHECK 01 | Ident CTV | Frequency 115.70 MHz | Type VORTAC.
CPT_VOR CHECK 01 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
CPT_VOR CHECK 01 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
CPT_VOR CHECK 01 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

CPT_VOR CHECK 02 | Ident CTV | Frequency 115.70 MHz | Type VORTAC.
CPT_VOR CHECK 02 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
CPT_VOR CHECK 02 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
CPT_VOR CHECK 02 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

CPT_VOR CHECK 03 | Ident CTV | Frequency 115.70 MHz | Type VORTAC.
CPT_VOR CHECK 03 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
CPT_VOR CHECK 03 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
CPT_VOR CHECK 03 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

CPT_VOR CHECK 04 | Ident CTV | Frequency 115.70 MHz | Type VORTAC.
CPT_VOR CHECK 04 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
CPT_VOR CHECK 04 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
CPT_VOR CHECK 04 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

CPT_VOR CHECK 05 | Ident CTV | Frequency 115.70 MHz | Type VORTAC.
CPT_VOR CHECK 05 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
CPT_VOR CHECK 05 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
CPT_VOR CHECK 05 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

CPT_VOR CHECK 06 | Ident CTV | Frequency 115.70 MHz | Type VORTAC.
CPT_VOR CHECK 06 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
CPT_VOR CHECK 06 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
CPT_VOR CHECK 06 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

CPT_VOR CHECK 07 | Ident CTV | Frequency 115.70 MHz | Type VORTAC.
CPT_VOR CHECK 07 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
CPT_VOR CHECK 07 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
CPT_VOR CHECK 07 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

CPT_VOR CHECK 08 | Ident CTV | Frequency 115.70 MHz | Type VORTAC.
CPT_VOR CHECK 08 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
CPT_VOR CHECK 08 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
CPT_VOR CHECK 08 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

CPT_VOR CHECK 09 | Ident CTV | Frequency 115.70 MHz | Type VORTAC.
CPT_VOR CHECK 09 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
CPT_VOR CHECK 09 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
CPT_VOR CHECK 09 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

CPT_VOR CHECK 10 | Ident CTV | Frequency 115.70 MHz | Type VORTAC.
CPT_VOR CHECK 10 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
CPT_VOR CHECK 10 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
CPT_VOR CHECK 10 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

CPT_VOR CHECK 11 | Ident CTV | Frequency 115.70 MHz | Type VORTAC.
CPT_VOR CHECK 11 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
CPT_VOR CHECK 11 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
CPT_VOR CHECK 11 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

CPT_VOR CHECK 12 | Ident CTV | Frequency 115.70 MHz | Type VORTAC.
CPT_VOR CHECK 12 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
CPT_VOR CHECK 12 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
CPT_VOR CHECK 12 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

CPT_VOR CHECK 13 | Ident CTV | Frequency 115.70 MHz | Type VORTAC.
CPT_VOR CHECK 13 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
CPT_VOR CHECK 13 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
CPT_VOR CHECK 13 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

CPT_VOR CHECK 14 | Ident CTV | Frequency 115.70 MHz | Type VORTAC.
CPT_VOR CHECK 14 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
CPT_VOR CHECK 14 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
CPT_VOR CHECK 14 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

CPT_VOR CHECK 15 | Ident CTV | Frequency 115.70 MHz | Type VORTAC.
CPT_VOR CHECK 15 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
CPT_VOR CHECK 15 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
CPT_VOR CHECK 15 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

DUR_VOR CHECK 01 | Ident DNV | Frequency 112.50 MHz | Type VOR/DME.
DUR_VOR CHECK 01 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
DUR_VOR CHECK 01 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
DUR_VOR CHECK 01 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

DUR_VOR CHECK 02 | Ident DNV | Frequency 112.50 MHz | Type VOR/DME.
DUR_VOR CHECK 02 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
DUR_VOR CHECK 02 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
DUR_VOR CHECK 02 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

DUR_VOR CHECK 03 | Ident DNV | Frequency 112.50 MHz | Type VOR/DME.
DUR_VOR CHECK 03 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
DUR_VOR CHECK 03 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
DUR_VOR CHECK 03 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

DUR_VOR CHECK 04 | Ident DNV | Frequency 112.50 MHz | Type VOR/DME.
DUR_VOR CHECK 04 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
DUR_VOR CHECK 04 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
DUR_VOR CHECK 04 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

DUR_VOR CHECK 05 | Ident DNV | Frequency 112.50 MHz | Type VOR/DME.
DUR_VOR CHECK 05 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
DUR_VOR CHECK 05 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
DUR_VOR CHECK 05 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

DUR_VOR CHECK 06 | Ident DNV | Frequency 112.50 MHz | Type VOR/DME.
DUR_VOR CHECK 06 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
DUR_VOR CHECK 06 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
DUR_VOR CHECK 06 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

DUR_VOR CHECK 07 | Ident DNV | Frequency 112.50 MHz | Type VOR/DME.
DUR_VOR CHECK 07 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
DUR_VOR CHECK 07 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
DUR_VOR CHECK 07 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

DUR_VOR CHECK 08 | Ident DNV | Frequency 112.50 MHz | Type VOR/DME.
DUR_VOR CHECK 08 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
DUR_VOR CHECK 08 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
DUR_VOR CHECK 08 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

DUR_VOR CHECK 09 | Ident DNV | Frequency 112.50 MHz | Type VOR/DME.
DUR_VOR CHECK 09 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
DUR_VOR CHECK 09 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
DUR_VOR CHECK 09 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

DUR_VOR CHECK 10 | Ident DNV | Frequency 112.50 MHz | Type VOR/DME.
DUR_VOR CHECK 10 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
DUR_VOR CHECK 10 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
DUR_VOR CHECK 10 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

DUR_VOR CHECK 11 | Ident DNV | Frequency 112.50 MHz | Type VOR/DME.
DUR_VOR CHECK 11 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
DUR_VOR CHECK 11 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
DUR_VOR CHECK 11 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

DUR_VOR CHECK 12 | Ident DNV | Frequency 112.50 MHz | Type VOR/DME.
DUR_VOR CHECK 12 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
DUR_VOR CHECK 12 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
DUR_VOR CHECK 12 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

DUR_VOR CHECK 13 | Ident DNV | Frequency 112.50 MHz | Type VOR/DME.
DUR_VOR CHECK 13 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
DUR_VOR CHECK 13 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
DUR_VOR CHECK 13 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

DUR_VOR CHECK 14 | Ident DNV | Frequency 112.50 MHz | Type VOR/DME.
DUR_VOR CHECK 14 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
DUR_VOR CHECK 14 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
DUR_VOR CHECK 14 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

DUR_VOR CHECK 15 | Ident DNV | Frequency 112.50 MHz | Type VOR/DME.
DUR_VOR CHECK 15 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
DUR_VOR CHECK 15 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
DUR_VOR CHECK 15 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FAWK_VOR CHECK 01 | Ident WKV | Frequency 116.90 MHz | Type VORTAC.
FAWK_VOR CHECK 01 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FAWK_VOR CHECK 01 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FAWK_VOR CHECK 01 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FAWK_VOR CHECK 02 | Ident WKV | Frequency 116.90 MHz | Type VORTAC.
FAWK_VOR CHECK 02 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FAWK_VOR CHECK 02 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FAWK_VOR CHECK 02 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FAWK_VOR CHECK 03 | Ident WKV | Frequency 116.90 MHz | Type VORTAC.
FAWK_VOR CHECK 03 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FAWK_VOR CHECK 03 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FAWK_VOR CHECK 03 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FAWK_VOR CHECK 04 | Ident WKV | Frequency 116.90 MHz | Type VORTAC.
FAWK_VOR CHECK 04 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FAWK_VOR CHECK 04 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FAWK_VOR CHECK 04 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FAWK_VOR CHECK 05 | Ident WKV | Frequency 116.90 MHz | Type VORTAC.
FAWK_VOR CHECK 05 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FAWK_VOR CHECK 05 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FAWK_VOR CHECK 05 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FAWK_VOR CHECK 06 | Ident WKV | Frequency 116.90 MHz | Type VORTAC.
FAWK_VOR CHECK 06 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FAWK_VOR CHECK 06 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FAWK_VOR CHECK 06 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FAWK_VOR CHECK 07 | Ident WKV | Frequency 116.90 MHz | Type VORTAC.
FAWK_VOR CHECK 07 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FAWK_VOR CHECK 07 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FAWK_VOR CHECK 07 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FAWK_VOR CHECK 08 | Ident WKV | Frequency 116.90 MHz | Type VORTAC.
FAWK_VOR CHECK 08 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FAWK_VOR CHECK 08 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FAWK_VOR CHECK 08 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FAWK_VOR CHECK 09 | Ident WKV | Frequency 116.90 MHz | Type VORTAC.
FAWK_VOR CHECK 09 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FAWK_VOR CHECK 09 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FAWK_VOR CHECK 09 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FAWK_VOR CHECK 10 | Ident WKV | Frequency 116.90 MHz | Type VORTAC.
FAWK_VOR CHECK 10 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FAWK_VOR CHECK 10 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FAWK_VOR CHECK 10 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FAWK_VOR CHECK 11 | Ident WKV | Frequency 116.90 MHz | Type VORTAC.
FAWK_VOR CHECK 11 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FAWK_VOR CHECK 11 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FAWK_VOR CHECK 11 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FAWK_VOR CHECK 12 | Ident WKV | Frequency 116.90 MHz | Type VORTAC.
FAWK_VOR CHECK 12 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FAWK_VOR CHECK 12 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FAWK_VOR CHECK 12 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FAWK_VOR CHECK 13 | Ident WKV | Frequency 116.90 MHz | Type VORTAC.
FAWK_VOR CHECK 13 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FAWK_VOR CHECK 13 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FAWK_VOR CHECK 13 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FAWK_VOR CHECK 14 | Ident WKV | Frequency 116.90 MHz | Type VORTAC.
FAWK_VOR CHECK 14 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FAWK_VOR CHECK 14 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FAWK_VOR CHECK 14 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FAWK_VOR CHECK 15 | Ident WKV | Frequency 116.90 MHz | Type VORTAC.
FAWK_VOR CHECK 15 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FAWK_VOR CHECK 15 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FAWK_VOR CHECK 15 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FALM_VOR CHECK 01 | Ident LTV | Frequency 115.00 MHz | Type VOR/DME.
FALM_VOR CHECK 01 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FALM_VOR CHECK 01 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FALM_VOR CHECK 01 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FALM_VOR CHECK 02 | Ident LTV | Frequency 115.00 MHz | Type VOR/DME.
FALM_VOR CHECK 02 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FALM_VOR CHECK 02 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FALM_VOR CHECK 02 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FALM_VOR CHECK 03 | Ident LTV | Frequency 115.00 MHz | Type VOR/DME.
FALM_VOR CHECK 03 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FALM_VOR CHECK 03 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FALM_VOR CHECK 03 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FALM_VOR CHECK 04 | Ident LTV | Frequency 115.00 MHz | Type VOR/DME.
FALM_VOR CHECK 04 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FALM_VOR CHECK 04 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FALM_VOR CHECK 04 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FALM_VOR CHECK 05 | Ident LTV | Frequency 115.00 MHz | Type VOR/DME.
FALM_VOR CHECK 05 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FALM_VOR CHECK 05 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FALM_VOR CHECK 05 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FALM_VOR CHECK 06 | Ident LTV | Frequency 115.00 MHz | Type VOR/DME.
FALM_VOR CHECK 06 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FALM_VOR CHECK 06 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FALM_VOR CHECK 06 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FALM_VOR CHECK 07 | Ident LTV | Frequency 115.00 MHz | Type VOR/DME.
FALM_VOR CHECK 07 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FALM_VOR CHECK 07 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FALM_VOR CHECK 07 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FALM_VOR CHECK 08 | Ident LTV | Frequency 115.00 MHz | Type VOR/DME.
FALM_VOR CHECK 08 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FALM_VOR CHECK 08 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FALM_VOR CHECK 08 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FALM_VOR CHECK 09 | Ident LTV | Frequency 115.00 MHz | Type VOR/DME.
FALM_VOR CHECK 09 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FALM_VOR CHECK 09 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FALM_VOR CHECK 09 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FALM_VOR CHECK 10 | Ident LTV | Frequency 115.00 MHz | Type VOR/DME.
FALM_VOR CHECK 10 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FALM_VOR CHECK 10 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FALM_VOR CHECK 10 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FALM_VOR CHECK 11 | Ident LTV | Frequency 115.00 MHz | Type VOR/DME.
FALM_VOR CHECK 11 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FALM_VOR CHECK 11 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FALM_VOR CHECK 11 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FALM_VOR CHECK 12 | Ident LTV | Frequency 115.00 MHz | Type VOR/DME.
FALM_VOR CHECK 12 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FALM_VOR CHECK 12 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FALM_VOR CHECK 12 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FALM_VOR CHECK 13 | Ident LTV | Frequency 115.00 MHz | Type VOR/DME.
FALM_VOR CHECK 13 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FALM_VOR CHECK 13 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FALM_VOR CHECK 13 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FALM_VOR CHECK 14 | Ident LTV | Frequency 115.00 MHz | Type VOR/DME.
FALM_VOR CHECK 14 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FALM_VOR CHECK 14 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FALM_VOR CHECK 14 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FALM_VOR CHECK 15 | Ident LTV | Frequency 115.00 MHz | Type VOR/DME.
FALM_VOR CHECK 15 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FALM_VOR CHECK 15 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FALM_VOR CHECK 15 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FAHS_VOR CHECK 01 | Ident HSV | Frequency 114.00 MHz | Type VOR/DME.
FAHS_VOR CHECK 01 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FAHS_VOR CHECK 01 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FAHS_VOR CHECK 01 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FAHS_VOR CHECK 02 | Ident HSV | Frequency 114.00 MHz | Type VOR/DME.
FAHS_VOR CHECK 02 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FAHS_VOR CHECK 02 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FAHS_VOR CHECK 02 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FAHS_VOR CHECK 03 | Ident HSV | Frequency 114.00 MHz | Type VOR/DME.
FAHS_VOR CHECK 03 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FAHS_VOR CHECK 03 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FAHS_VOR CHECK 03 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FAHS_VOR CHECK 04 | Ident HSV | Frequency 114.00 MHz | Type VOR/DME.
FAHS_VOR CHECK 04 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FAHS_VOR CHECK 04 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FAHS_VOR CHECK 04 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FAHS_VOR CHECK 05 | Ident HSV | Frequency 114.00 MHz | Type VOR/DME.
FAHS_VOR CHECK 05 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FAHS_VOR CHECK 05 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FAHS_VOR CHECK 05 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FAHS_VOR CHECK 06 | Ident HSV | Frequency 114.00 MHz | Type VOR/DME.
FAHS_VOR CHECK 06 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FAHS_VOR CHECK 06 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FAHS_VOR CHECK 06 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FAHS_VOR CHECK 07 | Ident HSV | Frequency 114.00 MHz | Type VOR/DME.
FAHS_VOR CHECK 07 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FAHS_VOR CHECK 07 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FAHS_VOR CHECK 07 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FAHS_VOR CHECK 08 | Ident HSV | Frequency 114.00 MHz | Type VOR/DME.
FAHS_VOR CHECK 08 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FAHS_VOR CHECK 08 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FAHS_VOR CHECK 08 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FAHS_VOR CHECK 09 | Ident HSV | Frequency 114.00 MHz | Type VOR/DME.
FAHS_VOR CHECK 09 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FAHS_VOR CHECK 09 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FAHS_VOR CHECK 09 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FAHS_VOR CHECK 10 | Ident HSV | Frequency 114.00 MHz | Type VOR/DME.
FAHS_VOR CHECK 10 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FAHS_VOR CHECK 10 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FAHS_VOR CHECK 10 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FAHS_VOR CHECK 11 | Ident HSV | Frequency 114.00 MHz | Type VOR/DME.
FAHS_VOR CHECK 11 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FAHS_VOR CHECK 11 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FAHS_VOR CHECK 11 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FAHS_VOR CHECK 12 | Ident HSV | Frequency 114.00 MHz | Type VOR/DME.
FAHS_VOR CHECK 12 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FAHS_VOR CHECK 12 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FAHS_VOR CHECK 12 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FAHS_VOR CHECK 13 | Ident HSV | Frequency 114.00 MHz | Type VOR/DME.
FAHS_VOR CHECK 13 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FAHS_VOR CHECK 13 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FAHS_VOR CHECK 13 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FAHS_VOR CHECK 14 | Ident HSV | Frequency 114.00 MHz | Type VOR/DME.
FAHS_VOR CHECK 14 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FAHS_VOR CHECK 14 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FAHS_VOR CHECK 14 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FAHS_VOR CHECK 15 | Ident HSV | Frequency 114.00 MHz | Type VOR/DME.
FAHS_VOR CHECK 15 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FAHS_VOR CHECK 15 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FAHS_VOR CHECK 15 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FALW_VOR CHECK 01 | Ident LWV | Frequency 117.00 MHz | Type VORTAC.
FALW_VOR CHECK 01 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FALW_VOR CHECK 01 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FALW_VOR CHECK 01 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FALW_VOR CHECK 02 | Ident LWV | Frequency 117.00 MHz | Type VORTAC.
FALW_VOR CHECK 02 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FALW_VOR CHECK 02 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FALW_VOR CHECK 02 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FALW_VOR CHECK 03 | Ident LWV | Frequency 117.00 MHz | Type VORTAC.
FALW_VOR CHECK 03 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FALW_VOR CHECK 03 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FALW_VOR CHECK 03 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FALW_VOR CHECK 04 | Ident LWV | Frequency 117.00 MHz | Type VORTAC.
FALW_VOR CHECK 04 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FALW_VOR CHECK 04 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FALW_VOR CHECK 04 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FALW_VOR CHECK 05 | Ident LWV | Frequency 117.00 MHz | Type VORTAC.
FALW_VOR CHECK 05 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FALW_VOR CHECK 05 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FALW_VOR CHECK 05 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FALW_VOR CHECK 06 | Ident LWV | Frequency 117.00 MHz | Type VORTAC.
FALW_VOR CHECK 06 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FALW_VOR CHECK 06 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FALW_VOR CHECK 06 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FALW_VOR CHECK 07 | Ident LWV | Frequency 117.00 MHz | Type VORTAC.
FALW_VOR CHECK 07 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FALW_VOR CHECK 07 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FALW_VOR CHECK 07 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FALW_VOR CHECK 08 | Ident LWV | Frequency 117.00 MHz | Type VORTAC.
FALW_VOR CHECK 08 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FALW_VOR CHECK 08 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FALW_VOR CHECK 08 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FALW_VOR CHECK 09 | Ident LWV | Frequency 117.00 MHz | Type VORTAC.
FALW_VOR CHECK 09 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FALW_VOR CHECK 09 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FALW_VOR CHECK 09 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FALW_VOR CHECK 10 | Ident LWV | Frequency 117.00 MHz | Type VORTAC.
FALW_VOR CHECK 10 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FALW_VOR CHECK 10 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FALW_VOR CHECK 10 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FALW_VOR CHECK 11 | Ident LWV | Frequency 117.00 MHz | Type VORTAC.
FALW_VOR CHECK 11 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FALW_VOR CHECK 11 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FALW_VOR CHECK 11 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FALW_VOR CHECK 12 | Ident LWV | Frequency 117.00 MHz | Type VORTAC.
FALW_VOR CHECK 12 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FALW_VOR CHECK 12 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FALW_VOR CHECK 12 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FALW_VOR CHECK 13 | Ident LWV | Frequency 117.00 MHz | Type VORTAC.
FALW_VOR CHECK 13 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FALW_VOR CHECK 13 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FALW_VOR CHECK 13 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FALW_VOR CHECK 14 | Ident LWV | Frequency 117.00 MHz | Type VORTAC.
FALW_VOR CHECK 14 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FALW_VOR CHECK 14 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FALW_VOR CHECK 14 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FALW_VOR CHECK 15 | Ident LWV | Frequency 117.00 MHz | Type VORTAC.
FALW_VOR CHECK 15 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FALW_VOR CHECK 15 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FALW_VOR CHECK 15 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FAOB_VOR CHECK 01 | Ident OBV | Frequency 115.40 MHz | Type VOR/DME.
FAOB_VOR CHECK 01 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FAOB_VOR CHECK 01 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FAOB_VOR CHECK 01 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FAOB_VOR CHECK 02 | Ident OBV | Frequency 115.40 MHz | Type VOR/DME.
FAOB_VOR CHECK 02 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FAOB_VOR CHECK 02 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FAOB_VOR CHECK 02 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FAOB_VOR CHECK 03 | Ident OBV | Frequency 115.40 MHz | Type VOR/DME.
FAOB_VOR CHECK 03 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FAOB_VOR CHECK 03 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FAOB_VOR CHECK 03 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FAOB_VOR CHECK 04 | Ident OBV | Frequency 115.40 MHz | Type VOR/DME.
FAOB_VOR CHECK 04 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FAOB_VOR CHECK 04 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FAOB_VOR CHECK 04 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FAOB_VOR CHECK 05 | Ident OBV | Frequency 115.40 MHz | Type VOR/DME.
FAOB_VOR CHECK 05 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FAOB_VOR CHECK 05 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FAOB_VOR CHECK 05 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FAOB_VOR CHECK 06 | Ident OBV | Frequency 115.40 MHz | Type VOR/DME.
FAOB_VOR CHECK 06 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FAOB_VOR CHECK 06 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FAOB_VOR CHECK 06 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FAOB_VOR CHECK 07 | Ident OBV | Frequency 115.40 MHz | Type VOR/DME.
FAOB_VOR CHECK 07 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FAOB_VOR CHECK 07 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FAOB_VOR CHECK 07 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FAOB_VOR CHECK 08 | Ident OBV | Frequency 115.40 MHz | Type VOR/DME.
FAOB_VOR CHECK 08 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FAOB_VOR CHECK 08 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FAOB_VOR CHECK 08 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FAOB_VOR CHECK 09 | Ident OBV | Frequency 115.40 MHz | Type VOR/DME.
FAOB_VOR CHECK 09 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FAOB_VOR CHECK 09 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FAOB_VOR CHECK 09 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FAOB_VOR CHECK 10 | Ident OBV | Frequency 115.40 MHz | Type VOR/DME.
FAOB_VOR CHECK 10 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FAOB_VOR CHECK 10 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FAOB_VOR CHECK 10 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FAOB_VOR CHECK 11 | Ident OBV | Frequency 115.40 MHz | Type VOR/DME.
FAOB_VOR CHECK 11 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FAOB_VOR CHECK 11 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FAOB_VOR CHECK 11 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FAOB_VOR CHECK 12 | Ident OBV | Frequency 115.40 MHz | Type VOR/DME.
FAOB_VOR CHECK 12 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FAOB_VOR CHECK 12 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FAOB_VOR CHECK 12 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FAOB_VOR CHECK 13 | Ident OBV | Frequency 115.40 MHz | Type VOR/DME.
FAOB_VOR CHECK 13 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FAOB_VOR CHECK 13 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FAOB_VOR CHECK 13 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FAOB_VOR CHECK 14 | Ident OBV | Frequency 115.40 MHz | Type VOR/DME.
FAOB_VOR CHECK 14 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FAOB_VOR CHECK 14 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FAOB_VOR CHECK 14 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FAOB_VOR CHECK 15 | Ident OBV | Frequency 115.40 MHz | Type VOR/DME.
FAOB_VOR CHECK 15 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FAOB_VOR CHECK 15 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FAOB_VOR CHECK 15 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FASK_VOR CHECK 01 | Ident WKV | Frequency 116.90 MHz | Type VORTAC.
FASK_VOR CHECK 01 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FASK_VOR CHECK 01 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FASK_VOR CHECK 01 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FASK_VOR CHECK 02 | Ident WKV | Frequency 116.90 MHz | Type VORTAC.
FASK_VOR CHECK 02 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FASK_VOR CHECK 02 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FASK_VOR CHECK 02 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FASK_VOR CHECK 03 | Ident WKV | Frequency 116.90 MHz | Type VORTAC.
FASK_VOR CHECK 03 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FASK_VOR CHECK 03 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FASK_VOR CHECK 03 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FASK_VOR CHECK 04 | Ident WKV | Frequency 116.90 MHz | Type VORTAC.
FASK_VOR CHECK 04 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FASK_VOR CHECK 04 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FASK_VOR CHECK 04 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FASK_VOR CHECK 05 | Ident WKV | Frequency 116.90 MHz | Type VORTAC.
FASK_VOR CHECK 05 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FASK_VOR CHECK 05 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FASK_VOR CHECK 05 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FASK_VOR CHECK 06 | Ident WKV | Frequency 116.90 MHz | Type VORTAC.
FASK_VOR CHECK 06 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FASK_VOR CHECK 06 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FASK_VOR CHECK 06 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FASK_VOR CHECK 07 | Ident WKV | Frequency 116.90 MHz | Type VORTAC.
FASK_VOR CHECK 07 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FASK_VOR CHECK 07 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FASK_VOR CHECK 07 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FASK_VOR CHECK 08 | Ident WKV | Frequency 116.90 MHz | Type VORTAC.
FASK_VOR CHECK 08 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FASK_VOR CHECK 08 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FASK_VOR CHECK 08 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FASK_VOR CHECK 09 | Ident WKV | Frequency 116.90 MHz | Type VORTAC.
FASK_VOR CHECK 09 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FASK_VOR CHECK 09 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FASK_VOR CHECK 09 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FASK_VOR CHECK 10 | Ident WKV | Frequency 116.90 MHz | Type VORTAC.
FASK_VOR CHECK 10 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FASK_VOR CHECK 10 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FASK_VOR CHECK 10 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FASK_VOR CHECK 11 | Ident WKV | Frequency 116.90 MHz | Type VORTAC.
FASK_VOR CHECK 11 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FASK_VOR CHECK 11 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FASK_VOR CHECK 11 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FASK_VOR CHECK 12 | Ident WKV | Frequency 116.90 MHz | Type VORTAC.
FASK_VOR CHECK 12 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FASK_VOR CHECK 12 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FASK_VOR CHECK 12 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FASK_VOR CHECK 13 | Ident WKV | Frequency 116.90 MHz | Type VORTAC.
FASK_VOR CHECK 13 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FASK_VOR CHECK 13 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FASK_VOR CHECK 13 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FASK_VOR CHECK 14 | Ident WKV | Frequency 116.90 MHz | Type VORTAC.
FASK_VOR CHECK 14 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FASK_VOR CHECK 14 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FASK_VOR CHECK 14 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FASK_VOR CHECK 15 | Ident WKV | Frequency 116.90 MHz | Type VORTAC.
FASK_VOR CHECK 15 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FASK_VOR CHECK 15 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FASK_VOR CHECK 15 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FABL_VOR CHECK 01 | Ident BLV | Frequency 114.10 MHz | Type VOR/DME.
FABL_VOR CHECK 01 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FABL_VOR CHECK 01 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FABL_VOR CHECK 01 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FABL_VOR CHECK 02 | Ident BLV | Frequency 114.10 MHz | Type VOR/DME.
FABL_VOR CHECK 02 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FABL_VOR CHECK 02 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FABL_VOR CHECK 02 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FABL_VOR CHECK 03 | Ident BLV | Frequency 114.10 MHz | Type VOR/DME.
FABL_VOR CHECK 03 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FABL_VOR CHECK 03 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FABL_VOR CHECK 03 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FABL_VOR CHECK 04 | Ident BLV | Frequency 114.10 MHz | Type VOR/DME.
FABL_VOR CHECK 04 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FABL_VOR CHECK 04 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FABL_VOR CHECK 04 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FABL_VOR CHECK 05 | Ident BLV | Frequency 114.10 MHz | Type VOR/DME.
FABL_VOR CHECK 05 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FABL_VOR CHECK 05 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FABL_VOR CHECK 05 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FABL_VOR CHECK 06 | Ident BLV | Frequency 114.10 MHz | Type VOR/DME.
FABL_VOR CHECK 06 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FABL_VOR CHECK 06 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FABL_VOR CHECK 06 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FABL_VOR CHECK 07 | Ident BLV | Frequency 114.10 MHz | Type VOR/DME.
FABL_VOR CHECK 07 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FABL_VOR CHECK 07 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FABL_VOR CHECK 07 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FABL_VOR CHECK 08 | Ident BLV | Frequency 114.10 MHz | Type VOR/DME.
FABL_VOR CHECK 08 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FABL_VOR CHECK 08 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FABL_VOR CHECK 08 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FABL_VOR CHECK 09 | Ident BLV | Frequency 114.10 MHz | Type VOR/DME.
FABL_VOR CHECK 09 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FABL_VOR CHECK 09 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FABL_VOR CHECK 09 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FABL_VOR CHECK 10 | Ident BLV | Frequency 114.10 MHz | Type VOR/DME.
FABL_VOR CHECK 10 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FABL_VOR CHECK 10 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FABL_VOR CHECK 10 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FABL_VOR CHECK 11 | Ident BLV | Frequency 114.10 MHz | Type VOR/DME.
FABL_VOR CHECK 11 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FABL_VOR CHECK 11 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FABL_VOR CHECK 11 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FABL_VOR CHECK 12 | Ident BLV | Frequency 114.10 MHz | Type VOR/DME.
FABL_VOR CHECK 12 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FABL_VOR CHECK 12 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FABL_VOR CHECK 12 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FABL_VOR CHECK 13 | Ident BLV | Frequency 114.10 MHz | Type VOR/DME.
FABL_VOR CHECK 13 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FABL_VOR CHECK 13 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FABL_VOR CHECK 13 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FABL_VOR CHECK 14 | Ident BLV | Frequency 114.10 MHz | Type VOR/DME.
FABL_VOR CHECK 14 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FABL_VOR CHECK 14 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FABL_VOR CHECK 14 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FABL_VOR CHECK 15 | Ident BLV | Frequency 114.10 MHz | Type VOR/DME.
FABL_VOR CHECK 15 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FABL_VOR CHECK 15 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FABL_VOR CHECK 15 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FAYP_VOR CHECK 01 | Ident CTV | Frequency 115.70 MHz | Type VORTAC.
FAYP_VOR CHECK 01 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FAYP_VOR CHECK 01 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FAYP_VOR CHECK 01 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FAYP_VOR CHECK 02 | Ident CTV | Frequency 115.70 MHz | Type VORTAC.
FAYP_VOR CHECK 02 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FAYP_VOR CHECK 02 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FAYP_VOR CHECK 02 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FAYP_VOR CHECK 03 | Ident CTV | Frequency 115.70 MHz | Type VORTAC.
FAYP_VOR CHECK 03 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FAYP_VOR CHECK 03 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FAYP_VOR CHECK 03 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FAYP_VOR CHECK 04 | Ident CTV | Frequency 115.70 MHz | Type VORTAC.
FAYP_VOR CHECK 04 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FAYP_VOR CHECK 04 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FAYP_VOR CHECK 04 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FAYP_VOR CHECK 05 | Ident CTV | Frequency 115.70 MHz | Type VORTAC.
FAYP_VOR CHECK 05 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FAYP_VOR CHECK 05 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FAYP_VOR CHECK 05 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FAYP_VOR CHECK 06 | Ident CTV | Frequency 115.70 MHz | Type VORTAC.
FAYP_VOR CHECK 06 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FAYP_VOR CHECK 06 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FAYP_VOR CHECK 06 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FAYP_VOR CHECK 07 | Ident CTV | Frequency 115.70 MHz | Type VORTAC.
FAYP_VOR CHECK 07 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FAYP_VOR CHECK 07 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FAYP_VOR CHECK 07 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FAYP_VOR CHECK 08 | Ident CTV | Frequency 115.70 MHz | Type VORTAC.
FAYP_VOR CHECK 08 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FAYP_VOR CHECK 08 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FAYP_VOR CHECK 08 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FAYP_VOR CHECK 09 | Ident CTV | Frequency 115.70 MHz | Type VORTAC.
FAYP_VOR CHECK 09 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FAYP_VOR CHECK 09 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FAYP_VOR CHECK 09 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FAYP_VOR CHECK 10 | Ident CTV | Frequency 115.70 MHz | Type VORTAC.
FAYP_VOR CHECK 10 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FAYP_VOR CHECK 10 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FAYP_VOR CHECK 10 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FAYP_VOR CHECK 11 | Ident CTV | Frequency 115.70 MHz | Type VORTAC.
FAYP_VOR CHECK 11 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FAYP_VOR CHECK 11 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FAYP_VOR CHECK 11 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FAYP_VOR CHECK 12 | Ident CTV | Frequency 115.70 MHz | Type VORTAC.
FAYP_VOR CHECK 12 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FAYP_VOR CHECK 12 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FAYP_VOR CHECK 12 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FAYP_VOR CHECK 13 | Ident CTV | Frequency 115.70 MHz | Type VORTAC.
FAYP_VOR CHECK 13 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FAYP_VOR CHECK 13 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FAYP_VOR CHECK 13 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FAYP_VOR CHECK 14 | Ident CTV | Frequency 115.70 MHz | Type VORTAC.
FAYP_VOR CHECK 14 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FAYP_VOR CHECK 14 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FAYP_VOR CHECK 14 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FAYP_VOR CHECK 15 | Ident CTV | Frequency 115.70 MHz | Type VORTAC.
FAYP_VOR CHECK 15 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FAYP_VOR CHECK 15 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FAYP_VOR CHECK 15 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FADN_VOR CHECK 01 | Ident DNV | Frequency 112.50 MHz | Type VOR/DME.
FADN_VOR CHECK 01 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FADN_VOR CHECK 01 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FADN_VOR CHECK 01 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FADN_VOR CHECK 02 | Ident DNV | Frequency 112.50 MHz | Type VOR/DME.
FADN_VOR CHECK 02 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FADN_VOR CHECK 02 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FADN_VOR CHECK 02 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FADN_VOR CHECK 03 | Ident DNV | Frequency 112.50 MHz | Type VOR/DME.
FADN_VOR CHECK 03 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FADN_VOR CHECK 03 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FADN_VOR CHECK 03 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FADN_VOR CHECK 04 | Ident DNV | Frequency 112.50 MHz | Type VOR/DME.
FADN_VOR CHECK 04 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FADN_VOR CHECK 04 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FADN_VOR CHECK 04 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FADN_VOR CHECK 05 | Ident DNV | Frequency 112.50 MHz | Type VOR/DME.
FADN_VOR CHECK 05 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FADN_VOR CHECK 05 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FADN_VOR CHECK 05 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FADN_VOR CHECK 06 | Ident DNV | Frequency 112.50 MHz | Type VOR/DME.
FADN_VOR CHECK 06 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FADN_VOR CHECK 06 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FADN_VOR CHECK 06 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FADN_VOR CHECK 07 | Ident DNV | Frequency 112.50 MHz | Type VOR/DME.
FADN_VOR CHECK 07 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FADN_VOR CHECK 07 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FADN_VOR CHECK 07 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FADN_VOR CHECK 08 | Ident DNV | Frequency 112.50 MHz | Type VOR/DME.
FADN_VOR CHECK 08 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FADN_VOR CHECK 08 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FADN_VOR CHECK 08 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FADN_VOR CHECK 09 | Ident DNV | Frequency 112.50 MHz | Type VOR/DME.
FADN_VOR CHECK 09 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FADN_VOR CHECK 09 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FADN_VOR CHECK 09 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FADN_VOR CHECK 10 | Ident DNV | Frequency 112.50 MHz | Type VOR/DME.
FADN_VOR CHECK 10 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FADN_VOR CHECK 10 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FADN_VOR CHECK 10 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FADN_VOR CHECK 11 | Ident DNV | Frequency 112.50 MHz | Type VOR/DME.
FADN_VOR CHECK 11 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FADN_VOR CHECK 11 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FADN_VOR CHECK 11 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FADN_VOR CHECK 12 | Ident DNV | Frequency 112.50 MHz | Type VOR/DME.
FADN_VOR CHECK 12 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FADN_VOR CHECK 12 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FADN_VOR CHECK 12 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FADN_VOR CHECK 13 | Ident DNV | Frequency 112.50 MHz | Type VOR/DME.
FADN_VOR CHECK 13 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FADN_VOR CHECK 13 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FADN_VOR CHECK 13 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FADN_VOR CHECK 14 | Ident DNV | Frequency 112.50 MHz | Type VOR/DME.
FADN_VOR CHECK 14 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FADN_VOR CHECK 14 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FADN_VOR CHECK 14 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FADN_VOR CHECK 15 | Ident DNV | Frequency 112.50 MHz | Type VOR/DME.
FADN_VOR CHECK 15 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FADN_VOR CHECK 15 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FADN_VOR CHECK 15 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FAPE_VOR CHECK 01 | Ident PEV | Frequency 113.40 MHz | Type VOR/DME.
FAPE_VOR CHECK 01 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FAPE_VOR CHECK 01 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FAPE_VOR CHECK 01 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FAPE_VOR CHECK 02 | Ident PEV | Frequency 113.40 MHz | Type VOR/DME.
FAPE_VOR CHECK 02 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FAPE_VOR CHECK 02 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FAPE_VOR CHECK 02 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FAPE_VOR CHECK 03 | Ident PEV | Frequency 113.40 MHz | Type VOR/DME.
FAPE_VOR CHECK 03 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FAPE_VOR CHECK 03 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FAPE_VOR CHECK 03 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FAPE_VOR CHECK 04 | Ident PEV | Frequency 113.40 MHz | Type VOR/DME.
FAPE_VOR CHECK 04 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FAPE_VOR CHECK 04 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FAPE_VOR CHECK 04 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FAPE_VOR CHECK 05 | Ident PEV | Frequency 113.40 MHz | Type VOR/DME.
FAPE_VOR CHECK 05 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FAPE_VOR CHECK 05 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FAPE_VOR CHECK 05 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FAPE_VOR CHECK 06 | Ident PEV | Frequency 113.40 MHz | Type VOR/DME.
FAPE_VOR CHECK 06 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FAPE_VOR CHECK 06 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FAPE_VOR CHECK 06 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FAPE_VOR CHECK 07 | Ident PEV | Frequency 113.40 MHz | Type VOR/DME.
FAPE_VOR CHECK 07 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FAPE_VOR CHECK 07 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FAPE_VOR CHECK 07 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FAPE_VOR CHECK 08 | Ident PEV | Frequency 113.40 MHz | Type VOR/DME.
FAPE_VOR CHECK 08 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FAPE_VOR CHECK 08 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FAPE_VOR CHECK 08 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FAPE_VOR CHECK 09 | Ident PEV | Frequency 113.40 MHz | Type VOR/DME.
FAPE_VOR CHECK 09 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FAPE_VOR CHECK 09 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FAPE_VOR CHECK 09 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FAPE_VOR CHECK 10 | Ident PEV | Frequency 113.40 MHz | Type VOR/DME.
FAPE_VOR CHECK 10 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FAPE_VOR CHECK 10 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FAPE_VOR CHECK 10 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FAPE_VOR CHECK 11 | Ident PEV | Frequency 113.40 MHz | Type VOR/DME.
FAPE_VOR CHECK 11 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FAPE_VOR CHECK 11 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FAPE_VOR CHECK 11 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FAPE_VOR CHECK 12 | Ident PEV | Frequency 113.40 MHz | Type VOR/DME.
FAPE_VOR CHECK 12 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FAPE_VOR CHECK 12 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FAPE_VOR CHECK 12 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FAPE_VOR CHECK 13 | Ident PEV | Frequency 113.40 MHz | Type VOR/DME.
FAPE_VOR CHECK 13 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FAPE_VOR CHECK 13 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FAPE_VOR CHECK 13 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FAPE_VOR CHECK 14 | Ident PEV | Frequency 113.40 MHz | Type VOR/DME.
FAPE_VOR CHECK 14 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FAPE_VOR CHECK 14 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FAPE_VOR CHECK 14 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

FAPE_VOR CHECK 15 | Ident PEV | Frequency 113.40 MHz | Type VOR/DME.
FAPE_VOR CHECK 15 | Verify bearing stability, recent signal samples, connection latency and configuration persistence.
FAPE_VOR CHECK 15 | Compare live monitor data with expected station metadata and runway-dependent ILS availability.
FAPE_VOR CHECK 15 | If values diverge beyond tolerance, place station in maintenance observation mode and log UTC event.

SECTION C - LDA IMPORT INTERPRETATION
LDA REFERENCE 01 | Review text section headers, coded blocks, waveform labels and parsed frequency pairs.
LDA REFERENCE 01 | Confirm station type, localizer frequency, glide slope frequency and transmitter block counts.
LDA REFERENCE 01 | Cross-check nominal DDM, SDM, RF level and executive/standby alarm thresholds.
LDA REFERENCE 01 | Preserve imported summaries in diagnostics to support engineering traceability and acceptance testing.
LDA REFERENCE 01 | Use parsed waveform names for monitor screens, simulation presets and maintenance handover notes.

LDA REFERENCE 02 | Review text section headers, coded blocks, waveform labels and parsed frequency pairs.
LDA REFERENCE 02 | Confirm station type, localizer frequency, glide slope frequency and transmitter block counts.
LDA REFERENCE 02 | Cross-check nominal DDM, SDM, RF level and executive/standby alarm thresholds.
LDA REFERENCE 02 | Preserve imported summaries in diagnostics to support engineering traceability and acceptance testing.
LDA REFERENCE 02 | Use parsed waveform names for monitor screens, simulation presets and maintenance handover notes.

LDA REFERENCE 03 | Review text section headers, coded blocks, waveform labels and parsed frequency pairs.
LDA REFERENCE 03 | Confirm station type, localizer frequency, glide slope frequency and transmitter block counts.
LDA REFERENCE 03 | Cross-check nominal DDM, SDM, RF level and executive/standby alarm thresholds.
LDA REFERENCE 03 | Preserve imported summaries in diagnostics to support engineering traceability and acceptance testing.
LDA REFERENCE 03 | Use parsed waveform names for monitor screens, simulation presets and maintenance handover notes.

LDA REFERENCE 04 | Review text section headers, coded blocks, waveform labels and parsed frequency pairs.
LDA REFERENCE 04 | Confirm station type, localizer frequency, glide slope frequency and transmitter block counts.
LDA REFERENCE 04 | Cross-check nominal DDM, SDM, RF level and executive/standby alarm thresholds.
LDA REFERENCE 04 | Preserve imported summaries in diagnostics to support engineering traceability and acceptance testing.
LDA REFERENCE 04 | Use parsed waveform names for monitor screens, simulation presets and maintenance handover notes.

LDA REFERENCE 05 | Review text section headers, coded blocks, waveform labels and parsed frequency pairs.
LDA REFERENCE 05 | Confirm station type, localizer frequency, glide slope frequency and transmitter block counts.
LDA REFERENCE 05 | Cross-check nominal DDM, SDM, RF level and executive/standby alarm thresholds.
LDA REFERENCE 05 | Preserve imported summaries in diagnostics to support engineering traceability and acceptance testing.
LDA REFERENCE 05 | Use parsed waveform names for monitor screens, simulation presets and maintenance handover notes.

LDA REFERENCE 06 | Review text section headers, coded blocks, waveform labels and parsed frequency pairs.
LDA REFERENCE 06 | Confirm station type, localizer frequency, glide slope frequency and transmitter block counts.
LDA REFERENCE 06 | Cross-check nominal DDM, SDM, RF level and executive/standby alarm thresholds.
LDA REFERENCE 06 | Preserve imported summaries in diagnostics to support engineering traceability and acceptance testing.
LDA REFERENCE 06 | Use parsed waveform names for monitor screens, simulation presets and maintenance handover notes.

LDA REFERENCE 07 | Review text section headers, coded blocks, waveform labels and parsed frequency pairs.
LDA REFERENCE 07 | Confirm station type, localizer frequency, glide slope frequency and transmitter block counts.
LDA REFERENCE 07 | Cross-check nominal DDM, SDM, RF level and executive/standby alarm thresholds.
LDA REFERENCE 07 | Preserve imported summaries in diagnostics to support engineering traceability and acceptance testing.
LDA REFERENCE 07 | Use parsed waveform names for monitor screens, simulation presets and maintenance handover notes.

LDA REFERENCE 08 | Review text section headers, coded blocks, waveform labels and parsed frequency pairs.
LDA REFERENCE 08 | Confirm station type, localizer frequency, glide slope frequency and transmitter block counts.
LDA REFERENCE 08 | Cross-check nominal DDM, SDM, RF level and executive/standby alarm thresholds.
LDA REFERENCE 08 | Preserve imported summaries in diagnostics to support engineering traceability and acceptance testing.
LDA REFERENCE 08 | Use parsed waveform names for monitor screens, simulation presets and maintenance handover notes.

LDA REFERENCE 09 | Review text section headers, coded blocks, waveform labels and parsed frequency pairs.
LDA REFERENCE 09 | Confirm station type, localizer frequency, glide slope frequency and transmitter block counts.
LDA REFERENCE 09 | Cross-check nominal DDM, SDM, RF level and executive/standby alarm thresholds.
LDA REFERENCE 09 | Preserve imported summaries in diagnostics to support engineering traceability and acceptance testing.
LDA REFERENCE 09 | Use parsed waveform names for monitor screens, simulation presets and maintenance handover notes.

LDA REFERENCE 10 | Review text section headers, coded blocks, waveform labels and parsed frequency pairs.
LDA REFERENCE 10 | Confirm station type, localizer frequency, glide slope frequency and transmitter block counts.
LDA REFERENCE 10 | Cross-check nominal DDM, SDM, RF level and executive/standby alarm thresholds.
LDA REFERENCE 10 | Preserve imported summaries in diagnostics to support engineering traceability and acceptance testing.
LDA REFERENCE 10 | Use parsed waveform names for monitor screens, simulation presets and maintenance handover notes.

LDA REFERENCE 11 | Review text section headers, coded blocks, waveform labels and parsed frequency pairs.
LDA REFERENCE 11 | Confirm station type, localizer frequency, glide slope frequency and transmitter block counts.
LDA REFERENCE 11 | Cross-check nominal DDM, SDM, RF level and executive/standby alarm thresholds.
LDA REFERENCE 11 | Preserve imported summaries in diagnostics to support engineering traceability and acceptance testing.
LDA REFERENCE 11 | Use parsed waveform names for monitor screens, simulation presets and maintenance handover notes.

LDA REFERENCE 12 | Review text section headers, coded blocks, waveform labels and parsed frequency pairs.
LDA REFERENCE 12 | Confirm station type, localizer frequency, glide slope frequency and transmitter block counts.
LDA REFERENCE 12 | Cross-check nominal DDM, SDM, RF level and executive/standby alarm thresholds.
LDA REFERENCE 12 | Preserve imported summaries in diagnostics to support engineering traceability and acceptance testing.
LDA REFERENCE 12 | Use parsed waveform names for monitor screens, simulation presets and maintenance handover notes.

LDA REFERENCE 13 | Review text section headers, coded blocks, waveform labels and parsed frequency pairs.
LDA REFERENCE 13 | Confirm station type, localizer frequency, glide slope frequency and transmitter block counts.
LDA REFERENCE 13 | Cross-check nominal DDM, SDM, RF level and executive/standby alarm thresholds.
LDA REFERENCE 13 | Preserve imported summaries in diagnostics to support engineering traceability and acceptance testing.
LDA REFERENCE 13 | Use parsed waveform names for monitor screens, simulation presets and maintenance handover notes.

LDA REFERENCE 14 | Review text section headers, coded blocks, waveform labels and parsed frequency pairs.
LDA REFERENCE 14 | Confirm station type, localizer frequency, glide slope frequency and transmitter block counts.
LDA REFERENCE 14 | Cross-check nominal DDM, SDM, RF level and executive/standby alarm thresholds.
LDA REFERENCE 14 | Preserve imported summaries in diagnostics to support engineering traceability and acceptance testing.
LDA REFERENCE 14 | Use parsed waveform names for monitor screens, simulation presets and maintenance handover notes.

LDA REFERENCE 15 | Review text section headers, coded blocks, waveform labels and parsed frequency pairs.
LDA REFERENCE 15 | Confirm station type, localizer frequency, glide slope frequency and transmitter block counts.
LDA REFERENCE 15 | Cross-check nominal DDM, SDM, RF level and executive/standby alarm thresholds.
LDA REFERENCE 15 | Preserve imported summaries in diagnostics to support engineering traceability and acceptance testing.
LDA REFERENCE 15 | Use parsed waveform names for monitor screens, simulation presets and maintenance handover notes.

LDA REFERENCE 16 | Review text section headers, coded blocks, waveform labels and parsed frequency pairs.
LDA REFERENCE 16 | Confirm station type, localizer frequency, glide slope frequency and transmitter block counts.
LDA REFERENCE 16 | Cross-check nominal DDM, SDM, RF level and executive/standby alarm thresholds.
LDA REFERENCE 16 | Preserve imported summaries in diagnostics to support engineering traceability and acceptance testing.
LDA REFERENCE 16 | Use parsed waveform names for monitor screens, simulation presets and maintenance handover notes.

LDA REFERENCE 17 | Review text section headers, coded blocks, waveform labels and parsed frequency pairs.
LDA REFERENCE 17 | Confirm station type, localizer frequency, glide slope frequency and transmitter block counts.
LDA REFERENCE 17 | Cross-check nominal DDM, SDM, RF level and executive/standby alarm thresholds.
LDA REFERENCE 17 | Preserve imported summaries in diagnostics to support engineering traceability and acceptance testing.
LDA REFERENCE 17 | Use parsed waveform names for monitor screens, simulation presets and maintenance handover notes.

LDA REFERENCE 18 | Review text section headers, coded blocks, waveform labels and parsed frequency pairs.
LDA REFERENCE 18 | Confirm station type, localizer frequency, glide slope frequency and transmitter block counts.
LDA REFERENCE 18 | Cross-check nominal DDM, SDM, RF level and executive/standby alarm thresholds.
LDA REFERENCE 18 | Preserve imported summaries in diagnostics to support engineering traceability and acceptance testing.
LDA REFERENCE 18 | Use parsed waveform names for monitor screens, simulation presets and maintenance handover notes.

LDA REFERENCE 19 | Review text section headers, coded blocks, waveform labels and parsed frequency pairs.
LDA REFERENCE 19 | Confirm station type, localizer frequency, glide slope frequency and transmitter block counts.
LDA REFERENCE 19 | Cross-check nominal DDM, SDM, RF level and executive/standby alarm thresholds.
LDA REFERENCE 19 | Preserve imported summaries in diagnostics to support engineering traceability and acceptance testing.
LDA REFERENCE 19 | Use parsed waveform names for monitor screens, simulation presets and maintenance handover notes.

LDA REFERENCE 20 | Review text section headers, coded blocks, waveform labels and parsed frequency pairs.
LDA REFERENCE 20 | Confirm station type, localizer frequency, glide slope frequency and transmitter block counts.
LDA REFERENCE 20 | Cross-check nominal DDM, SDM, RF level and executive/standby alarm thresholds.
LDA REFERENCE 20 | Preserve imported summaries in diagnostics to support engineering traceability and acceptance testing.
LDA REFERENCE 20 | Use parsed waveform names for monitor screens, simulation presets and maintenance handover notes.

LDA REFERENCE 21 | Review text section headers, coded blocks, waveform labels and parsed frequency pairs.
LDA REFERENCE 21 | Confirm station type, localizer frequency, glide slope frequency and transmitter block counts.
LDA REFERENCE 21 | Cross-check nominal DDM, SDM, RF level and executive/standby alarm thresholds.
LDA REFERENCE 21 | Preserve imported summaries in diagnostics to support engineering traceability and acceptance testing.
LDA REFERENCE 21 | Use parsed waveform names for monitor screens, simulation presets and maintenance handover notes.

LDA REFERENCE 22 | Review text section headers, coded blocks, waveform labels and parsed frequency pairs.
LDA REFERENCE 22 | Confirm station type, localizer frequency, glide slope frequency and transmitter block counts.
LDA REFERENCE 22 | Cross-check nominal DDM, SDM, RF level and executive/standby alarm thresholds.
LDA REFERENCE 22 | Preserve imported summaries in diagnostics to support engineering traceability and acceptance testing.
LDA REFERENCE 22 | Use parsed waveform names for monitor screens, simulation presets and maintenance handover notes.

LDA REFERENCE 23 | Review text section headers, coded blocks, waveform labels and parsed frequency pairs.
LDA REFERENCE 23 | Confirm station type, localizer frequency, glide slope frequency and transmitter block counts.
LDA REFERENCE 23 | Cross-check nominal DDM, SDM, RF level and executive/standby alarm thresholds.
LDA REFERENCE 23 | Preserve imported summaries in diagnostics to support engineering traceability and acceptance testing.
LDA REFERENCE 23 | Use parsed waveform names for monitor screens, simulation presets and maintenance handover notes.

LDA REFERENCE 24 | Review text section headers, coded blocks, waveform labels and parsed frequency pairs.
LDA REFERENCE 24 | Confirm station type, localizer frequency, glide slope frequency and transmitter block counts.
LDA REFERENCE 24 | Cross-check nominal DDM, SDM, RF level and executive/standby alarm thresholds.
LDA REFERENCE 24 | Preserve imported summaries in diagnostics to support engineering traceability and acceptance testing.
LDA REFERENCE 24 | Use parsed waveform names for monitor screens, simulation presets and maintenance handover notes.

LDA REFERENCE 25 | Review text section headers, coded blocks, waveform labels and parsed frequency pairs.
LDA REFERENCE 25 | Confirm station type, localizer frequency, glide slope frequency and transmitter block counts.
LDA REFERENCE 25 | Cross-check nominal DDM, SDM, RF level and executive/standby alarm thresholds.
LDA REFERENCE 25 | Preserve imported summaries in diagnostics to support engineering traceability and acceptance testing.
LDA REFERENCE 25 | Use parsed waveform names for monitor screens, simulation presets and maintenance handover notes.

LDA REFERENCE 26 | Review text section headers, coded blocks, waveform labels and parsed frequency pairs.
LDA REFERENCE 26 | Confirm station type, localizer frequency, glide slope frequency and transmitter block counts.
LDA REFERENCE 26 | Cross-check nominal DDM, SDM, RF level and executive/standby alarm thresholds.
LDA REFERENCE 26 | Preserve imported summaries in diagnostics to support engineering traceability and acceptance testing.
LDA REFERENCE 26 | Use parsed waveform names for monitor screens, simulation presets and maintenance handover notes.

LDA REFERENCE 27 | Review text section headers, coded blocks, waveform labels and parsed frequency pairs.
LDA REFERENCE 27 | Confirm station type, localizer frequency, glide slope frequency and transmitter block counts.
LDA REFERENCE 27 | Cross-check nominal DDM, SDM, RF level and executive/standby alarm thresholds.
LDA REFERENCE 27 | Preserve imported summaries in diagnostics to support engineering traceability and acceptance testing.
LDA REFERENCE 27 | Use parsed waveform names for monitor screens, simulation presets and maintenance handover notes.

LDA REFERENCE 28 | Review text section headers, coded blocks, waveform labels and parsed frequency pairs.
LDA REFERENCE 28 | Confirm station type, localizer frequency, glide slope frequency and transmitter block counts.
LDA REFERENCE 28 | Cross-check nominal DDM, SDM, RF level and executive/standby alarm thresholds.
LDA REFERENCE 28 | Preserve imported summaries in diagnostics to support engineering traceability and acceptance testing.
LDA REFERENCE 28 | Use parsed waveform names for monitor screens, simulation presets and maintenance handover notes.

LDA REFERENCE 29 | Review text section headers, coded blocks, waveform labels and parsed frequency pairs.
LDA REFERENCE 29 | Confirm station type, localizer frequency, glide slope frequency and transmitter block counts.
LDA REFERENCE 29 | Cross-check nominal DDM, SDM, RF level and executive/standby alarm thresholds.
LDA REFERENCE 29 | Preserve imported summaries in diagnostics to support engineering traceability and acceptance testing.
LDA REFERENCE 29 | Use parsed waveform names for monitor screens, simulation presets and maintenance handover notes.

LDA REFERENCE 30 | Review text section headers, coded blocks, waveform labels and parsed frequency pairs.
LDA REFERENCE 30 | Confirm station type, localizer frequency, glide slope frequency and transmitter block counts.
LDA REFERENCE 30 | Cross-check nominal DDM, SDM, RF level and executive/standby alarm thresholds.
LDA REFERENCE 30 | Preserve imported summaries in diagnostics to support engineering traceability and acceptance testing.
LDA REFERENCE 30 | Use parsed waveform names for monitor screens, simulation presets and maintenance handover notes.

LDA REFERENCE 31 | Review text section headers, coded blocks, waveform labels and parsed frequency pairs.
LDA REFERENCE 31 | Confirm station type, localizer frequency, glide slope frequency and transmitter block counts.
LDA REFERENCE 31 | Cross-check nominal DDM, SDM, RF level and executive/standby alarm thresholds.
LDA REFERENCE 31 | Preserve imported summaries in diagnostics to support engineering traceability and acceptance testing.
LDA REFERENCE 31 | Use parsed waveform names for monitor screens, simulation presets and maintenance handover notes.

LDA REFERENCE 32 | Review text section headers, coded blocks, waveform labels and parsed frequency pairs.
LDA REFERENCE 32 | Confirm station type, localizer frequency, glide slope frequency and transmitter block counts.
LDA REFERENCE 32 | Cross-check nominal DDM, SDM, RF level and executive/standby alarm thresholds.
LDA REFERENCE 32 | Preserve imported summaries in diagnostics to support engineering traceability and acceptance testing.
LDA REFERENCE 32 | Use parsed waveform names for monitor screens, simulation presets and maintenance handover notes.

LDA REFERENCE 33 | Review text section headers, coded blocks, waveform labels and parsed frequency pairs.
LDA REFERENCE 33 | Confirm station type, localizer frequency, glide slope frequency and transmitter block counts.
LDA REFERENCE 33 | Cross-check nominal DDM, SDM, RF level and executive/standby alarm thresholds.
LDA REFERENCE 33 | Preserve imported summaries in diagnostics to support engineering traceability and acceptance testing.
LDA REFERENCE 33 | Use parsed waveform names for monitor screens, simulation presets and maintenance handover notes.

LDA REFERENCE 34 | Review text section headers, coded blocks, waveform labels and parsed frequency pairs.
LDA REFERENCE 34 | Confirm station type, localizer frequency, glide slope frequency and transmitter block counts.
LDA REFERENCE 34 | Cross-check nominal DDM, SDM, RF level and executive/standby alarm thresholds.
LDA REFERENCE 34 | Preserve imported summaries in diagnostics to support engineering traceability and acceptance testing.
LDA REFERENCE 34 | Use parsed waveform names for monitor screens, simulation presets and maintenance handover notes.

LDA REFERENCE 35 | Review text section headers, coded blocks, waveform labels and parsed frequency pairs.
LDA REFERENCE 35 | Confirm station type, localizer frequency, glide slope frequency and transmitter block counts.
LDA REFERENCE 35 | Cross-check nominal DDM, SDM, RF level and executive/standby alarm thresholds.
LDA REFERENCE 35 | Preserve imported summaries in diagnostics to support engineering traceability and acceptance testing.
LDA REFERENCE 35 | Use parsed waveform names for monitor screens, simulation presets and maintenance handover notes.

LDA REFERENCE 36 | Review text section headers, coded blocks, waveform labels and parsed frequency pairs.
LDA REFERENCE 36 | Confirm station type, localizer frequency, glide slope frequency and transmitter block counts.
LDA REFERENCE 36 | Cross-check nominal DDM, SDM, RF level and executive/standby alarm thresholds.
LDA REFERENCE 36 | Preserve imported summaries in diagnostics to support engineering traceability and acceptance testing.
LDA REFERENCE 36 | Use parsed waveform names for monitor screens, simulation presets and maintenance handover notes.

LDA REFERENCE 37 | Review text section headers, coded blocks, waveform labels and parsed frequency pairs.
LDA REFERENCE 37 | Confirm station type, localizer frequency, glide slope frequency and transmitter block counts.
LDA REFERENCE 37 | Cross-check nominal DDM, SDM, RF level and executive/standby alarm thresholds.
LDA REFERENCE 37 | Preserve imported summaries in diagnostics to support engineering traceability and acceptance testing.
LDA REFERENCE 37 | Use parsed waveform names for monitor screens, simulation presets and maintenance handover notes.

LDA REFERENCE 38 | Review text section headers, coded blocks, waveform labels and parsed frequency pairs.
LDA REFERENCE 38 | Confirm station type, localizer frequency, glide slope frequency and transmitter block counts.
LDA REFERENCE 38 | Cross-check nominal DDM, SDM, RF level and executive/standby alarm thresholds.
LDA REFERENCE 38 | Preserve imported summaries in diagnostics to support engineering traceability and acceptance testing.
LDA REFERENCE 38 | Use parsed waveform names for monitor screens, simulation presets and maintenance handover notes.

LDA REFERENCE 39 | Review text section headers, coded blocks, waveform labels and parsed frequency pairs.
LDA REFERENCE 39 | Confirm station type, localizer frequency, glide slope frequency and transmitter block counts.
LDA REFERENCE 39 | Cross-check nominal DDM, SDM, RF level and executive/standby alarm thresholds.
LDA REFERENCE 39 | Preserve imported summaries in diagnostics to support engineering traceability and acceptance testing.
LDA REFERENCE 39 | Use parsed waveform names for monitor screens, simulation presets and maintenance handover notes.

LDA REFERENCE 40 | Review text section headers, coded blocks, waveform labels and parsed frequency pairs.
LDA REFERENCE 40 | Confirm station type, localizer frequency, glide slope frequency and transmitter block counts.
LDA REFERENCE 40 | Cross-check nominal DDM, SDM, RF level and executive/standby alarm thresholds.
LDA REFERENCE 40 | Preserve imported summaries in diagnostics to support engineering traceability and acceptance testing.
LDA REFERENCE 40 | Use parsed waveform names for monitor screens, simulation presets and maintenance handover notes.

SECTION D - ASRACS AND APPROACH CHECKLISTS
CHECKLIST 01 | Surface: confirm targets, hotspots, runway strip entry warnings and conflict separation trends.
CHECKLIST 01 | Approach: compare aircraft profile with ideal glide path and terrain slope assessment summary.
CHECKLIST 01 | Connection: verify serial or TCP transport, mock-server state and acquisition thread continuity.
CHECKLIST 01 | Recovery: stop simulations cleanly, clear alerts, persist configuration and restart in known-good state.

CHECKLIST 02 | Surface: confirm targets, hotspots, runway strip entry warnings and conflict separation trends.
CHECKLIST 02 | Approach: compare aircraft profile with ideal glide path and terrain slope assessment summary.
CHECKLIST 02 | Connection: verify serial or TCP transport, mock-server state and acquisition thread continuity.
CHECKLIST 02 | Recovery: stop simulations cleanly, clear alerts, persist configuration and restart in known-good state.

CHECKLIST 03 | Surface: confirm targets, hotspots, runway strip entry warnings and conflict separation trends.
CHECKLIST 03 | Approach: compare aircraft profile with ideal glide path and terrain slope assessment summary.
CHECKLIST 03 | Connection: verify serial or TCP transport, mock-server state and acquisition thread continuity.
CHECKLIST 03 | Recovery: stop simulations cleanly, clear alerts, persist configuration and restart in known-good state.

CHECKLIST 04 | Surface: confirm targets, hotspots, runway strip entry warnings and conflict separation trends.
CHECKLIST 04 | Approach: compare aircraft profile with ideal glide path and terrain slope assessment summary.
CHECKLIST 04 | Connection: verify serial or TCP transport, mock-server state and acquisition thread continuity.
CHECKLIST 04 | Recovery: stop simulations cleanly, clear alerts, persist configuration and restart in known-good state.

CHECKLIST 05 | Surface: confirm targets, hotspots, runway strip entry warnings and conflict separation trends.
CHECKLIST 05 | Approach: compare aircraft profile with ideal glide path and terrain slope assessment summary.
CHECKLIST 05 | Connection: verify serial or TCP transport, mock-server state and acquisition thread continuity.
CHECKLIST 05 | Recovery: stop simulations cleanly, clear alerts, persist configuration and restart in known-good state.

CHECKLIST 06 | Surface: confirm targets, hotspots, runway strip entry warnings and conflict separation trends.
CHECKLIST 06 | Approach: compare aircraft profile with ideal glide path and terrain slope assessment summary.
CHECKLIST 06 | Connection: verify serial or TCP transport, mock-server state and acquisition thread continuity.
CHECKLIST 06 | Recovery: stop simulations cleanly, clear alerts, persist configuration and restart in known-good state.

CHECKLIST 07 | Surface: confirm targets, hotspots, runway strip entry warnings and conflict separation trends.
CHECKLIST 07 | Approach: compare aircraft profile with ideal glide path and terrain slope assessment summary.
CHECKLIST 07 | Connection: verify serial or TCP transport, mock-server state and acquisition thread continuity.
CHECKLIST 07 | Recovery: stop simulations cleanly, clear alerts, persist configuration and restart in known-good state.

CHECKLIST 08 | Surface: confirm targets, hotspots, runway strip entry warnings and conflict separation trends.
CHECKLIST 08 | Approach: compare aircraft profile with ideal glide path and terrain slope assessment summary.
CHECKLIST 08 | Connection: verify serial or TCP transport, mock-server state and acquisition thread continuity.
CHECKLIST 08 | Recovery: stop simulations cleanly, clear alerts, persist configuration and restart in known-good state.

CHECKLIST 09 | Surface: confirm targets, hotspots, runway strip entry warnings and conflict separation trends.
CHECKLIST 09 | Approach: compare aircraft profile with ideal glide path and terrain slope assessment summary.
CHECKLIST 09 | Connection: verify serial or TCP transport, mock-server state and acquisition thread continuity.
CHECKLIST 09 | Recovery: stop simulations cleanly, clear alerts, persist configuration and restart in known-good state.

CHECKLIST 10 | Surface: confirm targets, hotspots, runway strip entry warnings and conflict separation trends.
CHECKLIST 10 | Approach: compare aircraft profile with ideal glide path and terrain slope assessment summary.
CHECKLIST 10 | Connection: verify serial or TCP transport, mock-server state and acquisition thread continuity.
CHECKLIST 10 | Recovery: stop simulations cleanly, clear alerts, persist configuration and restart in known-good state.

CHECKLIST 11 | Surface: confirm targets, hotspots, runway strip entry warnings and conflict separation trends.
CHECKLIST 11 | Approach: compare aircraft profile with ideal glide path and terrain slope assessment summary.
CHECKLIST 11 | Connection: verify serial or TCP transport, mock-server state and acquisition thread continuity.
CHECKLIST 11 | Recovery: stop simulations cleanly, clear alerts, persist configuration and restart in known-good state.

CHECKLIST 12 | Surface: confirm targets, hotspots, runway strip entry warnings and conflict separation trends.
CHECKLIST 12 | Approach: compare aircraft profile with ideal glide path and terrain slope assessment summary.
CHECKLIST 12 | Connection: verify serial or TCP transport, mock-server state and acquisition thread continuity.
CHECKLIST 12 | Recovery: stop simulations cleanly, clear alerts, persist configuration and restart in known-good state.

CHECKLIST 13 | Surface: confirm targets, hotspots, runway strip entry warnings and conflict separation trends.
CHECKLIST 13 | Approach: compare aircraft profile with ideal glide path and terrain slope assessment summary.
CHECKLIST 13 | Connection: verify serial or TCP transport, mock-server state and acquisition thread continuity.
CHECKLIST 13 | Recovery: stop simulations cleanly, clear alerts, persist configuration and restart in known-good state.

CHECKLIST 14 | Surface: confirm targets, hotspots, runway strip entry warnings and conflict separation trends.
CHECKLIST 14 | Approach: compare aircraft profile with ideal glide path and terrain slope assessment summary.
CHECKLIST 14 | Connection: verify serial or TCP transport, mock-server state and acquisition thread continuity.
CHECKLIST 14 | Recovery: stop simulations cleanly, clear alerts, persist configuration and restart in known-good state.

CHECKLIST 15 | Surface: confirm targets, hotspots, runway strip entry warnings and conflict separation trends.
CHECKLIST 15 | Approach: compare aircraft profile with ideal glide path and terrain slope assessment summary.
CHECKLIST 15 | Connection: verify serial or TCP transport, mock-server state and acquisition thread continuity.
CHECKLIST 15 | Recovery: stop simulations cleanly, clear alerts, persist configuration and restart in known-good state.

CHECKLIST 16 | Surface: confirm targets, hotspots, runway strip entry warnings and conflict separation trends.
CHECKLIST 16 | Approach: compare aircraft profile with ideal glide path and terrain slope assessment summary.
CHECKLIST 16 | Connection: verify serial or TCP transport, mock-server state and acquisition thread continuity.
CHECKLIST 16 | Recovery: stop simulations cleanly, clear alerts, persist configuration and restart in known-good state.

CHECKLIST 17 | Surface: confirm targets, hotspots, runway strip entry warnings and conflict separation trends.
CHECKLIST 17 | Approach: compare aircraft profile with ideal glide path and terrain slope assessment summary.
CHECKLIST 17 | Connection: verify serial or TCP transport, mock-server state and acquisition thread continuity.
CHECKLIST 17 | Recovery: stop simulations cleanly, clear alerts, persist configuration and restart in known-good state.

CHECKLIST 18 | Surface: confirm targets, hotspots, runway strip entry warnings and conflict separation trends.
CHECKLIST 18 | Approach: compare aircraft profile with ideal glide path and terrain slope assessment summary.
CHECKLIST 18 | Connection: verify serial or TCP transport, mock-server state and acquisition thread continuity.
CHECKLIST 18 | Recovery: stop simulations cleanly, clear alerts, persist configuration and restart in known-good state.

CHECKLIST 19 | Surface: confirm targets, hotspots, runway strip entry warnings and conflict separation trends.
CHECKLIST 19 | Approach: compare aircraft profile with ideal glide path and terrain slope assessment summary.
CHECKLIST 19 | Connection: verify serial or TCP transport, mock-server state and acquisition thread continuity.
CHECKLIST 19 | Recovery: stop simulations cleanly, clear alerts, persist configuration and restart in known-good state.

CHECKLIST 20 | Surface: confirm targets, hotspots, runway strip entry warnings and conflict separation trends.
CHECKLIST 20 | Approach: compare aircraft profile with ideal glide path and terrain slope assessment summary.
CHECKLIST 20 | Connection: verify serial or TCP transport, mock-server state and acquisition thread continuity.
CHECKLIST 20 | Recovery: stop simulations cleanly, clear alerts, persist configuration and restart in known-good state.

CHECKLIST 21 | Surface: confirm targets, hotspots, runway strip entry warnings and conflict separation trends.
CHECKLIST 21 | Approach: compare aircraft profile with ideal glide path and terrain slope assessment summary.
CHECKLIST 21 | Connection: verify serial or TCP transport, mock-server state and acquisition thread continuity.
CHECKLIST 21 | Recovery: stop simulations cleanly, clear alerts, persist configuration and restart in known-good state.

CHECKLIST 22 | Surface: confirm targets, hotspots, runway strip entry warnings and conflict separation trends.
CHECKLIST 22 | Approach: compare aircraft profile with ideal glide path and terrain slope assessment summary.
CHECKLIST 22 | Connection: verify serial or TCP transport, mock-server state and acquisition thread continuity.
CHECKLIST 22 | Recovery: stop simulations cleanly, clear alerts, persist configuration and restart in known-good state.

CHECKLIST 23 | Surface: confirm targets, hotspots, runway strip entry warnings and conflict separation trends.
CHECKLIST 23 | Approach: compare aircraft profile with ideal glide path and terrain slope assessment summary.
CHECKLIST 23 | Connection: verify serial or TCP transport, mock-server state and acquisition thread continuity.
CHECKLIST 23 | Recovery: stop simulations cleanly, clear alerts, persist configuration and restart in known-good state.

CHECKLIST 24 | Surface: confirm targets, hotspots, runway strip entry warnings and conflict separation trends.
CHECKLIST 24 | Approach: compare aircraft profile with ideal glide path and terrain slope assessment summary.
CHECKLIST 24 | Connection: verify serial or TCP transport, mock-server state and acquisition thread continuity.
CHECKLIST 24 | Recovery: stop simulations cleanly, clear alerts, persist configuration and restart in known-good state.

CHECKLIST 25 | Surface: confirm targets, hotspots, runway strip entry warnings and conflict separation trends.
CHECKLIST 25 | Approach: compare aircraft profile with ideal glide path and terrain slope assessment summary.
CHECKLIST 25 | Connection: verify serial or TCP transport, mock-server state and acquisition thread continuity.
CHECKLIST 25 | Recovery: stop simulations cleanly, clear alerts, persist configuration and restart in known-good state.

CHECKLIST 26 | Surface: confirm targets, hotspots, runway strip entry warnings and conflict separation trends.
CHECKLIST 26 | Approach: compare aircraft profile with ideal glide path and terrain slope assessment summary.
CHECKLIST 26 | Connection: verify serial or TCP transport, mock-server state and acquisition thread continuity.
CHECKLIST 26 | Recovery: stop simulations cleanly, clear alerts, persist configuration and restart in known-good state.

CHECKLIST 27 | Surface: confirm targets, hotspots, runway strip entry warnings and conflict separation trends.
CHECKLIST 27 | Approach: compare aircraft profile with ideal glide path and terrain slope assessment summary.
CHECKLIST 27 | Connection: verify serial or TCP transport, mock-server state and acquisition thread continuity.
CHECKLIST 27 | Recovery: stop simulations cleanly, clear alerts, persist configuration and restart in known-good state.

CHECKLIST 28 | Surface: confirm targets, hotspots, runway strip entry warnings and conflict separation trends.
CHECKLIST 28 | Approach: compare aircraft profile with ideal glide path and terrain slope assessment summary.
CHECKLIST 28 | Connection: verify serial or TCP transport, mock-server state and acquisition thread continuity.
CHECKLIST 28 | Recovery: stop simulations cleanly, clear alerts, persist configuration and restart in known-good state.

CHECKLIST 29 | Surface: confirm targets, hotspots, runway strip entry warnings and conflict separation trends.
CHECKLIST 29 | Approach: compare aircraft profile with ideal glide path and terrain slope assessment summary.
CHECKLIST 29 | Connection: verify serial or TCP transport, mock-server state and acquisition thread continuity.
CHECKLIST 29 | Recovery: stop simulations cleanly, clear alerts, persist configuration and restart in known-good state.

CHECKLIST 30 | Surface: confirm targets, hotspots, runway strip entry warnings and conflict separation trends.
CHECKLIST 30 | Approach: compare aircraft profile with ideal glide path and terrain slope assessment summary.
CHECKLIST 30 | Connection: verify serial or TCP transport, mock-server state and acquisition thread continuity.
CHECKLIST 30 | Recovery: stop simulations cleanly, clear alerts, persist configuration and restart in known-good state.

END OF EMBEDDED OPERATIONS REFERENCE
"""

def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def destination_point(lat: float, lon: float, bearing_deg: float, distance_nm: float) -> Tuple[float, float]:
    bearing = math.radians(bearing_deg)
    d = distance_nm / EARTH_RADIUS_NM
    lat1 = math.radians(lat)
    lon1 = math.radians(lon)
    lat2 = math.asin(math.sin(lat1) * math.cos(d) + math.cos(lat1) * math.sin(d) * math.cos(bearing))
    lon2 = lon1 + math.atan2(
        math.sin(bearing) * math.sin(d) * math.cos(lat1),
        math.cos(d) - math.sin(lat1) * math.sin(lat2),
    )
    return math.degrees(lat2), math.degrees(lon2)


def great_circle_distance_nm(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = (
        math.sin(dlat / 2) ** 2
        + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon / 2) ** 2
    )
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return EARTH_RADIUS_NM * c


def initial_bearing(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    lat1r = math.radians(lat1)
    lat2r = math.radians(lat2)
    dlon = math.radians(lon2 - lon1)
    x = math.sin(dlon) * math.cos(lat2r)
    y = math.cos(lat1r) * math.sin(lat2r) - math.sin(lat1r) * math.cos(lat2r) * math.cos(dlon)
    return (math.degrees(math.atan2(x, y)) + 360.0) % 360.0


def parse_runway_heading(runway: str) -> int:
    numbers = runway.split("/")[0]
    try:
        return int(numbers) * 10
    except Exception:
        return 30


def polyline_length(points: List[Tuple[float, float]]) -> float:
    total = 0.0
    for idx in range(1, len(points)):
        x1, y1 = points[idx - 1]
        x2, y2 = points[idx]
        total += math.hypot(x2 - x1, y2 - y1)
    return total


def interpolate_polyline(points: List[Tuple[float, float]], distance_m: float) -> Tuple[float, float]:
    if not points:
        return 0.0, 0.0
    if len(points) == 1:
        return points[0]
    remaining = distance_m
    for idx in range(1, len(points)):
        x1, y1 = points[idx - 1]
        x2, y2 = points[idx]
        seg = math.hypot(x2 - x1, y2 - y1)
        if remaining <= seg:
            ratio = 0.0 if seg == 0 else remaining / seg
            return x1 + (x2 - x1) * ratio, y1 + (y2 - y1) * ratio
        remaining -= seg
    return points[-1]


def point_line_distance(point: Tuple[float, float], line_start: Tuple[float, float], line_end: Tuple[float, float]) -> float:
    px, py = point
    x1, y1 = line_start
    x2, y2 = line_end
    dx = x2 - x1
    dy = y2 - y1
    if dx == 0 and dy == 0:
        return math.hypot(px - x1, py - y1)
    t = ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)
    t = clamp(t, 0.0, 1.0)
    proj_x = x1 + t * dx
    proj_y = y1 + t * dy
    return math.hypot(px - proj_x, py - proj_y)


@dataclass
class VORAirportConfig:
    key: str
    name: str
    ident: str
    frequency: float
    latitude: float
    longitude: float
    elevation_ft: int
    station_type: str
    airport_code: str
    ils_available: bool = False
    ils_runways: List[Dict] = field(default_factory=list)
    remarks: str = ""


@dataclass
class GroundTarget:
    target_id: str
    callsign: str
    category: str
    x_m: float
    y_m: float
    heading_deg: float
    speed_kts: float
    status: str = "TAXI"
    route_name: str = ""
    progress_m: float = 0.0
    route_points: List[Tuple[float, float]] = field(default_factory=list)
    track_history: deque = field(default_factory=lambda: deque(maxlen=40))
    metadata: Dict = field(default_factory=dict)


@dataclass
class IncursionAlert:
    alert_id: str
    timestamp: datetime
    severity: str
    message: str
    runway: str
    target_ids: List[str] = field(default_factory=list)
    active: bool = True
    location: Tuple[float, float] = (0.0, 0.0)


class LDAFileParser:
    """Parser for Thales / Rohde & Schwarz LDA files."""

    def __init__(self, filepath):
        self.filepath = Path(filepath)
        self.raw_bytes: bytes = b""
        self.text_content: str = ""
        self.parsed = False
        self.station_type = "UNKNOWN"
        self.localizer_frequency: Optional[float] = None
        self.glideslope_frequency: Optional[float] = None
        self.waveform_names: List[str] = []
        self.nominal_values: Dict = {}
        self.alarm_limits: Dict = {}
        self.equipment_config: Dict = {}
        self.binary_blocks: List[Dict] = []
        self.parse_messages: List[str] = []

    def parse(self) -> bool:
        try:
            self.raw_bytes = self.filepath.read_bytes()
            self.text_content = self.raw_bytes.decode("latin-1", errors="ignore")
            self._parse_text_section(self.text_content)
            self._parse_binary_coded_section(self.raw_bytes)
            self.parsed = True
            logger.info("Parsed LDA file: %s", self.filepath)
            return True
        except Exception as exc:
            self.parsed = False
            logger.exception("Failed to parse LDA file %s: %s", self.filepath, exc)
            self.parse_messages.append(str(exc))
            return False

    def _parse_text_section(self, content):
        lines = [line.rstrip() for line in content.splitlines() if line.strip()]
        text_region = content
        if "***PRINTOUT_TEXT_START***" in content:
            text_region = content.split("***PRINTOUT_TEXT_START***", 1)[1]
        if "BEGIN_PROG" in text_region:
            text_region = text_region.split("BEGIN_PROG", 1)[0]

        station_patterns = [
            r"Station\s*Type\s*[:\-]?\s*([A-Za-z0-9\-/ ]+)",
            r"Station:\s*:?.*?\(\s*([A-Za-z0-9\-/ ]+)\s*\)",
        ]
        for pattern in station_patterns:
            match = re.search(pattern, text_region, re.IGNORECASE)
            if match:
                self.station_type = match.group(1).strip()
                break
        if self.station_type == "UNKNOWN":
            if "glide path" in text_region.lower() or "glide slope" in text_region.lower():
                self.station_type = "GLIDE SLOPE"
            elif "localizer" in text_region.lower():
                self.station_type = "LOCALIZER"

        freq_match = re.search(r"(10[89]\.\d{2}|11[01]\.\d{2})\s*/\s*(33\d\.\d{2})\s*MHz", text_region)
        if freq_match:
            self.localizer_frequency = float(freq_match.group(1))
            self.glideslope_frequency = float(freq_match.group(2))
        else:
            loc_matches = re.findall(r"(10[89]\.\d{2}|11[01]\.\d{2})\s*MHz", text_region)
            gs_matches = re.findall(r"(32[9-9]\.\d{2}|33[0-5]\.\d{2})\s*MHz", text_region)
            if loc_matches:
                self.localizer_frequency = float(loc_matches[0])
            if gs_matches:
                self.glideslope_frequency = float(gs_matches[0])

        waveform_candidates = []
        for match in re.finditer(r"(?:Waveform\s*Name|TX\s*Waveform|Waveform)\s*[:\-]?\s*([A-Za-z0-9_ \-]+)", text_region, re.IGNORECASE):
            waveform_candidates.append(match.group(1).strip())
        ascii_names = re.findall(r"\b(Normal|Alarm\s+POS\s+Low\d*|Alarm\s+POS\s+High\d*|Standby|Monitor|Reference|Clearance|Course)\b", text_region, re.IGNORECASE)
        for item in ascii_names:
            waveform_candidates.append(str(item).strip())
        unique_waveforms = []
        for item in waveform_candidates:
            clean = re.sub(r"\s+", " ", item)
            if clean and clean not in unique_waveforms:
                unique_waveforms.append(clean)
        self.waveform_names = unique_waveforms[:8]

        def parse_named_float(keys: List[str], text: str) -> Optional[float]:
            for key in keys:
                pattern = rf"{key}\s*[:=]?\s*(-?\d+(?:\.\d+)?)"
                match = re.search(pattern, text, re.IGNORECASE)
                if match:
                    return float(match.group(1))
            return None

        nominal_map = {
            "course_ddm": [r"CRS\s*DDM", r"Course\s*DDM", r"DDM\s*course"],
            "course_sdm": [r"CRS\s*SDM", r"Course\s*SDM", r"SDM\s*course"],
            "clearance_ddm": [r"CLR\s*DDM", r"Clearance\s*DDM"],
            "clearance_sdm": [r"CLR\s*SDM", r"Clearance\s*SDM"],
            "rf_level_dbm": [r"RF\s*Level", r"RF\s*Power"],
            "mod90_pct": [r"90\s*Hz\s*Mod", r"Mod\s*90"],
            "mod150_pct": [r"150\s*Hz\s*Mod", r"Mod\s*150"],
        }
        for key, patterns in nominal_map.items():
            value = parse_named_float(patterns, text_region)
            if value is not None:
                self.nominal_values[key] = value

        alarm_map = {
            "executive_high_ddm": [r"Executive\s*High\s*DDM", r"Exec\s*DDM\s*High"],
            "executive_low_ddm": [r"Executive\s*Low\s*DDM", r"Exec\s*DDM\s*Low"],
            "standby_high_ddm": [r"Standby\s*High\s*DDM"],
            "standby_low_ddm": [r"Standby\s*Low\s*DDM"],
            "rf_low_limit": [r"RF\s*Low\s*Limit"],
            "rf_high_limit": [r"RF\s*High\s*Limit"],
        }
        for key, patterns in alarm_map.items():
            value = parse_named_float(patterns, text_region)
            if value is not None:
                self.alarm_limits[key] = value

        equipment_text = {}
        eq_patterns = {
            "site": r"Site\s*[:\-]\s*([A-Za-z0-9\-_/ ]+)",
            "equipment_mode": r"Equipment\s*configuration\s*[:\-]?\s*([A-Za-z0-9\-_/ ]+)",
            "carrier_mode": r"Frequency\s*carrier\s*[:\-]?\s*([A-Za-z0-9\-_/ ]+)",
            "timestamp": r"(\d{2}\.\d{2}\.\d{4}\s+\d{1,2}:\d{2}:\d{2})",
        }
        for key, pattern in eq_patterns.items():
            match = re.search(pattern, text_region, re.IGNORECASE)
            if match:
                equipment_text[key] = match.group(1).strip()
        if equipment_text:
            self.equipment_config.update(equipment_text)

        self.parse_messages.append(f"Text section scanned: {len(lines)} non-empty lines")

    def _parse_binary_coded_section(self, data):
        if b"BEGIN_PROG" not in data:
            self.parse_messages.append("No BEGIN_PROG marker found")
            return
        decoded = data.decode("latin-1", errors="ignore")
        coded_lines = re.findall(r"([^\n\r;]+?\s+\d+\s+\d+\s+);CODED;([^\n\r]+)", decoded)
        for header, payload in coded_lines:
            bytes_found = []
            for token in re.findall(r"\d+", payload):
                try:
                    val = int(token)
                    if 0 <= val <= 255:
                        bytes_found.append(val)
                except ValueError:
                    continue
            if not bytes_found:
                continue
            raw = bytes(bytes_found)
            ascii_chunks = re.findall(rb"[A-Za-z][A-Za-z0-9 _\-]{3,20}", raw)
            block = {
                "header": header.strip(),
                "length": len(raw),
                "ascii_strings": [chunk.decode("latin-1", errors="ignore").strip(" \x00") for chunk in ascii_chunks],
                "preview_hex": raw[:16].hex(" "),
            }
            self.binary_blocks.append(block)
            for text in block["ascii_strings"]:
                if text and text not in self.waveform_names and len(self.waveform_names) < 8:
                    if any(ch.isalpha() for ch in text):
                        self.waveform_names.append(text)

            floats = []
            for idx in range(0, min(len(raw) - 4, 32), 4):
                try:
                    value = struct.unpack("<f", raw[idx : idx + 4])[0]
                    if math.isfinite(value) and -1e6 < value < 1e6:
                        floats.append(round(float(value), 4))
                except struct.error:
                    continue
            if floats:
                block["float_preview"] = floats[:5]

        self.equipment_config["binary_block_count"] = len(self.binary_blocks)
        tx_blocks = [blk for blk in self.binary_blocks if blk["header"].startswith("TX-")]
        self.equipment_config["tx_block_count"] = len(tx_blocks)
        if tx_blocks and "waveform_source" not in self.equipment_config:
            self.equipment_config["waveform_source"] = tx_blocks[0]["header"]
        self.parse_messages.append(f"Binary section scanned: {len(self.binary_blocks)} CODED blocks")

    def get_station_type(self) -> str:
        return self.station_type

    def get_localizer_frequency(self) -> Optional[float]:
        return self.localizer_frequency

    def get_glideslope_frequency(self) -> Optional[float]:
        return self.glideslope_frequency

    def get_waveform_names(self) -> List[str]:
        return list(self.waveform_names[:8])

    def get_nominal_values(self) -> Dict:
        return dict(self.nominal_values)

    def get_alarm_limits(self) -> Dict:
        return dict(self.alarm_limits)

    def get_equipment_config(self) -> Dict:
        return dict(self.equipment_config)

    def get_summary(self) -> str:
        parts = [
            f"File: {self.filepath.name}",
            f"Station Type: {self.station_type}",
            f"Localizer Frequency: {self.localizer_frequency if self.localizer_frequency is not None else 'N/A'} MHz",
            f"Glide Slope Frequency: {self.glideslope_frequency if self.glideslope_frequency is not None else 'N/A'} MHz",
            f"Waveforms ({len(self.waveform_names)}): {', '.join(self.waveform_names) if self.waveform_names else 'None detected'}",
            f"Nominal Values: {self.nominal_values if self.nominal_values else 'None detected'}",
            f"Alarm Limits: {self.alarm_limits if self.alarm_limits else 'None detected'}",
            f"Equipment Config: {self.equipment_config if self.equipment_config else 'None detected'}",
        ]
        if self.parse_messages:
            parts.append("Notes: " + " | ".join(self.parse_messages[:6]))
        return "\n".join(parts)


class GlideSlopeDetector:
    def __init__(self, glide_angle_deg=3.0):
        self.glide_angle_deg = glide_angle_deg

    def ideal_altitude_ft(self, distance_nm) -> float:
        feet_per_nm = math.tan(math.radians(self.glide_angle_deg)) * 6076.12
        return max(0.0, distance_nm * feet_per_nm)

    def glide_slope_error_ft(self, altitude_ft, distance_nm) -> float:
        return altitude_ft - self.ideal_altitude_ft(distance_nm)

    def is_on_glide_slope(self, altitude_ft, distance_nm, tolerance_ft=50) -> bool:
        return abs(self.glide_slope_error_ft(altitude_ft, distance_nm)) <= tolerance_ft

    def validate_profile(self, points: List[Tuple[float, float]]) -> Dict:
        results = []
        errors = []
        for distance_nm, altitude_ft in points:
            ideal = self.ideal_altitude_ft(distance_nm)
            error = altitude_ft - ideal
            errors.append(error)
            results.append(
                {
                    "distance_nm": distance_nm,
                    "altitude_ft": altitude_ft,
                    "ideal_altitude_ft": ideal,
                    "error_ft": error,
                    "on_glide_slope": abs(error) <= 50,
                }
            )
        rms = math.sqrt(sum(err * err for err in errors) / len(errors)) if errors else 0.0
        return {
            "glide_angle_deg": self.glide_angle_deg,
            "points": results,
            "max_abs_error_ft": max((abs(err) for err in errors), default=0.0),
            "mean_error_ft": sum(errors) / len(errors) if errors else 0.0,
            "rms_error_ft": rms,
            "on_profile_ratio": sum(1 for err in errors if abs(err) <= 50) / len(errors) if errors else 0.0,
            "valid": all(abs(err) <= 150 for err in errors) if errors else True,
        }

    def required_descent_rate_fpm(self, speed_kts) -> float:
        return speed_kts * 101.27 * math.tan(math.radians(self.glide_angle_deg))


class SurfaceSlopeAnalyzer:
    CAUTION_THRESHOLD = 4.0
    CRITICAL_THRESHOLD = 8.0

    def __init__(self):
        self.last_profile: Dict = {}

    def analyze_profile(self, elevations: List[float], spacing_m=100) -> Dict:
        slopes = []
        critical_segments = []
        caution_segments = []
        for idx in range(1, len(elevations)):
            rise = elevations[idx] - elevations[idx - 1]
            slope_pct = (rise / spacing_m) * 100.0 if spacing_m else 0.0
            abs_slope = abs(slope_pct)
            segment = {
                "segment": idx - 1,
                "start_elev": elevations[idx - 1],
                "end_elev": elevations[idx],
                "slope_pct": round(slope_pct, 3),
            }
            slopes.append(abs_slope)
            if abs_slope >= self.CRITICAL_THRESHOLD:
                critical_segments.append(segment)
            elif abs_slope >= self.CAUTION_THRESHOLD:
                caution_segments.append(segment)
        profile = {
            "max_slope_pct": round(max(slopes) if slopes else 0.0, 3),
            "avg_slope_pct": round(sum(slopes) / len(slopes) if slopes else 0.0, 3),
            "critical_segments": critical_segments,
            "caution_segments": caution_segments,
            "sample_count": len(elevations),
            "spacing_m": spacing_m,
        }
        self.last_profile = profile
        return profile

    def assess_approach_safety(self, profile: Dict) -> str:
        if profile.get("critical_segments"):
            return "CRITICAL"
        if profile.get("caution_segments") or profile.get("max_slope_pct", 0.0) >= self.CAUTION_THRESHOLD:
            return "CAUTION"
        return "SAFE"


class JNBAirportLayout:
    RUNWAYS = copy.deepcopy(JNB_LAYOUT_DATA["runways"])
    TAXIWAYS = copy.deepcopy(JNB_LAYOUT_DATA["taxiways"])
    APRONS = copy.deepcopy(JNB_LAYOUT_DATA["aprons"])
    GATES = copy.deepcopy(JNB_LAYOUT_DATA["gates"])
    HOTSPOTS = copy.deepcopy(JNB_LAYOUT_DATA["hotspots"])

    def __init__(self):
        self.bounds = (-2200, -1500, 1800, 1700)

    def get_runways(self):
        return copy.deepcopy(self.RUNWAYS)

    def get_taxiways(self):
        return copy.deepcopy(self.TAXIWAYS)

    def get_hotspots(self):
        return copy.deepcopy(self.HOTSPOTS)


class ASRACSSimEngine:
    def __init__(self):
        self.layout = JNBAirportLayout()
        self.lock = Lock()
        self._running = False
        self._thread: Optional[Thread] = None
        self._stop_event = Event()
        self._targets: List[GroundTarget] = []
        self._alerts: List[IncursionAlert] = []
        self._seed_targets()

    def _seed_targets(self):
        taxiways = {item["name"]: item["points"] for item in self.layout.get_taxiways()}
        sample_defs = [
            ("GT001", "SAA321", "A320", "A", 120.0, 18.0),
            ("GT002", "LNX12", "B738", "B", 480.0, 22.0),
            ("GT003", "MEL45", "B737", "D", 90.0, 15.0),
            ("GT004", "CARGO9", "B744", "E", 60.0, 12.0),
        ]
        self._targets.clear()
        for target_id, callsign, category, route_name, progress, speed_kts in sample_defs:
            route_points = taxiways[route_name]
            x_m, y_m = interpolate_polyline(route_points, progress)
            self._targets.append(
                GroundTarget(
                    target_id=target_id,
                    callsign=callsign,
                    category=category,
                    x_m=x_m,
                    y_m=y_m,
                    heading_deg=0.0,
                    speed_kts=speed_kts,
                    status="TAXI",
                    route_name=route_name,
                    progress_m=progress,
                    route_points=route_points,
                )
            )

    def start(self):
        with self.lock:
            if self._running:
                return
            self._running = True
            self._stop_event.clear()
        self._thread = Thread(target=self._loop, daemon=True)
        self._thread.start()
        logger.info("ASRACS simulation started")

    def stop(self):
        with self.lock:
            self._running = False
            self._stop_event.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        logger.info("ASRACS simulation stopped")

    def _loop(self):
        while not self._stop_event.is_set():
            self._update()
            time.sleep(0.5)

    def get_targets(self) -> List[GroundTarget]:
        with self.lock:
            return copy.deepcopy(self._targets)

    def get_alerts(self) -> List[IncursionAlert]:
        with self.lock:
            return copy.deepcopy(self._alerts)

    def _make_alert(self, severity: str, message: str, runway: str, target_ids: List[str], location: Tuple[float, float]):
        alert = IncursionAlert(
            alert_id=f"ALT-{int(time.time() * 1000)}-{random.randint(10, 99)}",
            timestamp=datetime.utcnow(),
            severity=severity,
            message=message,
            runway=runway,
            target_ids=target_ids,
            location=location,
        )
        self._alerts.insert(0, alert)
        self._alerts = self._alerts[:20]

    def _runway_distance(self, target: GroundTarget, runway: Dict) -> float:
        return point_line_distance((target.x_m, target.y_m), (runway["x1"], runway["y1"]), (runway["x2"], runway["y2"]))

    def _update(self):
        with self.lock:
            if not self._running:
                return
            for target in self._targets:
                if not target.route_points:
                    continue
                travel_m = target.speed_kts * NM_TO_METERS / 3600.0 * 0.5
                target.progress_m += travel_m
                route_len = polyline_length(target.route_points)
                if target.progress_m > route_len:
                    target.progress_m = 0.0
                    if random.random() > 0.65:
                        target.status = "HOLD"
                    else:
                        target.status = "TAXI"
                old_x, old_y = target.x_m, target.y_m
                target.x_m, target.y_m = interpolate_polyline(target.route_points, target.progress_m)
                if old_x != target.x_m or old_y != target.y_m:
                    target.heading_deg = (math.degrees(math.atan2(target.x_m - old_x, target.y_m - old_y)) + 360.0) % 360.0
                target.track_history.append((target.x_m, target.y_m, time.time()))
                if random.random() > 0.985:
                    target.status = random.choice(["TAXI", "CROSS", "HOLD", "STOP"])

            self._alerts = [alert for alert in self._alerts if (time.time() - alert.timestamp.timestamp()) < 30 and alert.active]

            runways = self.layout.get_runways()
            hotspots = self.layout.get_hotspots()
            for target in self._targets:
                for runway in runways:
                    distance = self._runway_distance(target, runway)
                    if distance <= runway["width_m"] * 0.8 and target.status in {"CROSS", "TAXI"}:
                        self._make_alert(
                            "WARNING",
                            f"{target.callsign} entered protected runway strip {runway['name']}",
                            runway["name"],
                            [target.target_id],
                            (target.x_m, target.y_m),
                        )
                        break
                for hotspot in hotspots:
                    if math.hypot(target.x_m - hotspot["x"], target.y_m - hotspot["y"]) <= hotspot["radius"]:
                        self._make_alert(
                            "CAUTION",
                            f"{target.callsign} near hotspot {hotspot['name']} - {hotspot['description']}",
                            "N/A",
                            [target.target_id],
                            (target.x_m, target.y_m),
                        )
                        break

            for idx, first in enumerate(self._targets):
                for second in self._targets[idx + 1 :]:
                    separation = math.hypot(first.x_m - second.x_m, first.y_m - second.y_m)
                    if separation < 85.0:
                        self._make_alert(
                            "CRITICAL",
                            f"Potential surface collision between {first.callsign} and {second.callsign}",
                            "TAXI",
                            [first.target_id, second.target_id],
                            ((first.x_m + second.x_m) / 2.0, (first.y_m + second.y_m) / 2.0),
                        )


class SimulationEngine:
    def __init__(self):
        self.lock = Lock()
        self._running = False
        self._thread: Optional[Thread] = None
        self._stop_event = Event()
        self._aircraft: List[Dict] = []
        self._seed_aircraft()

    def _seed_aircraft(self):
        origin_lat = DEFAULT_AIRPORTS["JNB"]["latitude"]
        origin_lon = DEFAULT_AIRPORTS["JNB"]["longitude"]
        sample = [
            ("SAF001", 25, 210, 18000, 320),
            ("RCH214", 42, 30, 22000, 360),
            ("GRP201", 18, 120, 12000, 280),
            ("LNX551", 55, 300, 26000, 410),
            ("BDF720", 12, 75, 8000, 230),
            ("SPR905", 70, 165, 30000, 450),
        ]
        self._aircraft = []
        for callsign, distance_nm, bearing_deg, altitude, speed in sample:
            lat, lon = destination_point(origin_lat, origin_lon, bearing_deg, distance_nm)
            heading = (bearing_deg + 180) % 360 if distance_nm < 30 else (bearing_deg + 25) % 360
            self._aircraft.append(
                {
                    "callsign": callsign,
                    "lat": lat,
                    "lon": lon,
                    "altitude": altitude,
                    "speed": speed,
                    "heading": heading,
                    "track_history": deque(maxlen=60),
                }
            )

    def start(self):
        with self.lock:
            if self._running:
                return
            self._running = True
            self._stop_event.clear()
        self._thread = Thread(target=self._loop, daemon=True)
        self._thread.start()
        logger.info("Aircraft simulation started")

    def stop(self):
        with self.lock:
            self._running = False
            self._stop_event.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        logger.info("Aircraft simulation stopped")

    def _loop(self):
        while not self._stop_event.is_set():
            self._update()
            time.sleep(1.0)

    def get_aircraft(self) -> List[Dict]:
        with self.lock:
            return [
                {
                    **{k: v for k, v in item.items() if k != "track_history"},
                    "track_history": list(item["track_history"]),
                }
                for item in self._aircraft
            ]

    def _update(self):
        with self.lock:
            if not self._running:
                return
            origin_lat = DEFAULT_AIRPORTS["JNB"]["latitude"]
            origin_lon = DEFAULT_AIRPORTS["JNB"]["longitude"]
            for aircraft in self._aircraft:
                distance_nm = aircraft["speed"] / 3600.0
                aircraft["lat"], aircraft["lon"] = destination_point(
                    aircraft["lat"], aircraft["lon"], aircraft["heading"], distance_nm
                )
                aircraft["track_history"].append((aircraft["lat"], aircraft["lon"], aircraft["altitude"], time.time()))
                range_from_origin = great_circle_distance_nm(origin_lat, origin_lon, aircraft["lat"], aircraft["lon"])
                if range_from_origin > 110:
                    inbound_bearing = initial_bearing(aircraft["lat"], aircraft["lon"], origin_lat, origin_lon)
                    aircraft["heading"] = inbound_bearing
                else:
                    aircraft["heading"] = (aircraft["heading"] + random.uniform(-3.5, 3.5)) % 360
                desired_alt = max(3000, 30000 - range_from_origin * 180)
                aircraft["altitude"] += clamp(desired_alt - aircraft["altitude"], -500, 500)
                aircraft["altitude"] = int(clamp(aircraft["altitude"], 2500, 36000))


class MockVORTCPServer:
    def __init__(self, host='127.0.0.1', port=5000):
        self.host = host
        self.port = port
        self.lock = Lock()
        self._running = False
        self._server_socket: Optional[socket.socket] = None
        self._thread: Optional[Thread] = None
        self._stop_event = Event()
        self._last_bearing = 0.0
        self._last_strength = 80.0
        self._last_freq = 116.90

    def start(self):
        with self.lock:
            if self._running:
                return
            self._running = True
            self._stop_event.clear()
        self._thread = Thread(target=self._run, daemon=True)
        self._thread.start()
        logger.info("Mock VOR TCP server starting on %s:%s", self.host, self.port)

    def stop(self):
        with self.lock:
            self._running = False
            self._stop_event.set()
            sock = self._server_socket
            self._server_socket = None
        if sock:
            try:
                sock.close()
            except OSError:
                pass
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        logger.info("Mock VOR TCP server stopped")

    def is_running(self) -> bool:
        with self.lock:
            return self._running

    def _handle_command(self, cmd: str) -> str:
        cmd = cmd.strip().upper()
        self._last_bearing = (self._last_bearing + random.uniform(2.0, 8.0)) % 360.0
        self._last_strength = clamp(self._last_strength + random.uniform(-4.5, 4.5), 45.0, 98.0)
        self._last_freq = random.choice([114.90, 115.40, 116.90, 112.50, 115.00])
        if cmd == "STATUS":
            return f"BEARING={self._last_bearing:.2f},SIGNAL={self._last_strength:.1f},FREQ={self._last_freq:.2f},UTC={datetime.utcnow().isoformat()}"
        if cmd == "IDENT":
            return "IDENT=WKV,TYPE=VORTAC,HEALTH=NORMAL"
        if cmd == "HEALTH":
            return "HEALTH=OK,CPU=22,MEM=48,TEMP=39"
        if cmd == "PING":
            return "PONG"
        return "ERR=UNKNOWN_COMMAND"

    def _run(self):
        try:
            server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind((self.host, self.port))
            server.listen(5)
            server.settimeout(0.5)
            with self.lock:
                self._server_socket = server
            while not self._stop_event.is_set():
                try:
                    client, _addr = server.accept()
                except socket.timeout:
                    continue
                except OSError:
                    break
                with client:
                    client.settimeout(1.0)
                    try:
                        while not self._stop_event.is_set():
                            payload = client.recv(1024)
                            if not payload:
                                break
                            response = self._handle_command(payload.decode("utf-8", errors="ignore")) + "\n"
                            client.sendall(response.encode("utf-8"))
                    except (socket.timeout, ConnectionError, OSError):
                        pass
        except Exception as exc:
            logger.exception("MockVORTCPServer failure: %s", exc)
        finally:
            with self.lock:
                self._running = False
                if self._server_socket:
                    try:
                        self._server_socket.close()
                    except OSError:
                        pass
                    self._server_socket = None


class VORConnectionHandler:
    def __init__(self):
        self.lock = Lock()
        self.serial_conn: Optional[serial.Serial] = None
        self.tcp_conn: Optional[socket.socket] = None
        self.connection_mode = "DISCONNECTED"
        self.last_error = ""
        self.remote = ""

    def _disconnect_unlocked(self):
        if self.serial_conn:
            try:
                self.serial_conn.close()
            except Exception:
                pass
            self.serial_conn = None
        if self.tcp_conn:
            try:
                self.tcp_conn.close()
            except Exception:
                pass
            self.tcp_conn = None
        self.connection_mode = "DISCONNECTED"
        self.remote = ""

    def connect_serial(self, port, baud) -> bool:
        with self.lock:
            self._disconnect_unlocked()
            try:
                self.serial_conn = serial.Serial(port=port, baudrate=int(baud), timeout=1.0)
                self.connection_mode = "SERIAL"
                self.remote = f"{port}@{baud}"
                self.last_error = ""
                return True
            except Exception as exc:
                self.last_error = str(exc)
                logger.exception("Serial connect failed: %s", exc)
                self.serial_conn = None
                self.connection_mode = "DISCONNECTED"
                return False

    def connect_tcp(self, host, port) -> bool:
        with self.lock:
            self._disconnect_unlocked()
            try:
                self.tcp_conn = socket.create_connection((host, int(port)), timeout=2.0)
                self.tcp_conn.settimeout(2.0)
                self.connection_mode = "TCP"
                self.remote = f"{host}:{port}"
                self.last_error = ""
                return True
            except Exception as exc:
                self.last_error = str(exc)
                logger.exception("TCP connect failed: %s", exc)
                self.tcp_conn = None
                self.connection_mode = "DISCONNECTED"
                return False

    def disconnect(self):
        with self.lock:
            self._disconnect_unlocked()

    def send_command(self, cmd: str) -> str:
        with self.lock:
            if self.serial_conn:
                try:
                    self.serial_conn.write((cmd.strip() + "\n").encode("utf-8"))
                    time.sleep(0.1)
                    return self.serial_conn.readline().decode("utf-8", errors="ignore").strip()
                except Exception as exc:
                    self.last_error = str(exc)
                    raise
            if self.tcp_conn:
                try:
                    self.tcp_conn.sendall((cmd.strip() + "\n").encode("utf-8"))
                    return self.tcp_conn.recv(1024).decode("utf-8", errors="ignore").strip()
                except Exception as exc:
                    self.last_error = str(exc)
                    raise
            return ""

    def is_connected(self) -> bool:
        with self.lock:
            return self.serial_conn is not None or self.tcp_conn is not None

    def get_status(self) -> str:
        with self.lock:
            if self.serial_conn:
                return f"SERIAL connected to {self.remote}"
            if self.tcp_conn:
                return f"TCP connected to {self.remote}"
            return f"DISCONNECTED {self.last_error}".strip()


class VORDataProcessor:
    def __init__(self):
        self.lock = Lock()
        self.history = deque(maxlen=1000)

    def add_sample(self, bearing, signal_strength, frequency):
        with self.lock:
            self.history.append(
                {
                    "timestamp": time.time(),
                    "bearing": float(bearing),
                    "signal_strength": float(signal_strength),
                    "frequency": float(frequency),
                }
            )

    def get_bearing(self) -> float:
        with self.lock:
            if not self.history:
                return 0.0
            samples = list(self.history)[-10:]
            return sum(item["bearing"] for item in samples) / len(samples)

    def get_signal_strength(self) -> float:
        with self.lock:
            if not self.history:
                return 0.0
            samples = list(self.history)[-10:]
            return sum(item["signal_strength"] for item in samples) / len(samples)

    def get_history(self) -> deque:
        with self.lock:
            return deque(copy.deepcopy(list(self.history)), maxlen=1000)


class AircraftTracker:
    def __init__(self):
        self.lock = Lock()
        self.reference_lat = DEFAULT_AIRPORTS["JNB"]["latitude"]
        self.reference_lon = DEFAULT_AIRPORTS["JNB"]["longitude"]
        self.position = (self.reference_lat, self.reference_lon)
        self.altitude = 0.0
        self.history = deque(maxlen=40)

    def update(self, bearing, distance, altitude):
        with self.lock:
            self.position = destination_point(self.reference_lat, self.reference_lon, float(bearing), float(distance))
            self.altitude = float(altitude)
            self.history.append({
                "lat": self.position[0],
                "lon": self.position[1],
                "altitude": self.altitude,
                "bearing": float(bearing),
                "distance": float(distance),
                "timestamp": time.time(),
            })

    def get_position(self) -> Tuple[float, float]:
        with self.lock:
            return self.position

    def get_track_history(self) -> List:
        with self.lock:
            return list(self.history)


class DataAcquisitionThread(QThread):
    data_received = pyqtSignal(dict)
    connection_lost = pyqtSignal(str)

    def __init__(self, connection_handler, data_processor):
        super().__init__()
        self.connection_handler = connection_handler
        self.data_processor = data_processor
        self._running = True

    def stop(self):
        self._running = False

    def _parse_payload(self, payload: str) -> Dict:
        result = {}
        for item in payload.split(","):
            if "=" in item:
                key, value = item.split("=", 1)
                result[key.strip().lower()] = value.strip()
        try:
            result["bearing"] = float(result.get("bearing", 0.0))
            result["signal"] = float(result.get("signal", result.get("signal_strength", 0.0)))
            result["freq"] = float(result.get("freq", result.get("frequency", 0.0)))
        except (ValueError, TypeError):
            pass
        return result

    def run(self):
        while self._running:
            try:
                if not self.connection_handler.is_connected():
                    self.connection_lost.emit("Connection unavailable")
                    break
                payload = self.connection_handler.send_command("STATUS")
                if not payload:
                    self.connection_lost.emit("No data returned")
                    break
                parsed = self._parse_payload(payload)
                bearing = float(parsed.get("bearing", 0.0))
                signal = float(parsed.get("signal", 0.0))
                freq = float(parsed.get("freq", 0.0))
                self.data_processor.add_sample(bearing, signal, freq)
                self.data_received.emit(parsed)
                self.msleep(800)
            except Exception as exc:
                logger.exception("Data acquisition error: %s", exc)
                self.connection_lost.emit(str(exc))
                break


class CDIDisplay(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.deviation = 0.0
        self.setMinimumHeight(180)
        self.setMinimumWidth(220)

    def set_deviation(self, dots: float):
        self.deviation = clamp(float(dots), -2.5, 2.5)
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), QColor(16, 18, 22))
        cx = self.width() / 2
        cy = self.height() / 2
        scale = min(self.width(), self.height()) * 0.32

        painter.setPen(QPen(QColor(180, 180, 180), 2))
        painter.drawLine(int(cx), int(cy - scale), int(cx), int(cy + scale))
        painter.drawLine(int(cx - scale), int(cy), int(cx + scale), int(cy))

        painter.setPen(QPen(QColor(90, 180, 255), 2))
        for dot in range(-2, 3):
            x = cx + dot * (scale / 2)
            painter.drawEllipse(QPointF(x, cy), 4, 4)

        full_scale = scale
        needle_x = cx + (self.deviation / 2.5) * full_scale
        painter.setPen(QPen(QColor(255, 220, 0), 4))
        painter.drawLine(int(needle_x), int(cy - 55), int(needle_x), int(cy + 55))

        painter.setPen(QColor(220, 220, 220))
        painter.setFont(QFont("Arial", 10, QFont.Bold))
        painter.drawText(10, 20, "CDI")
        painter.drawText(10, self.height() - 10, f"Deviation: {self.deviation:+.2f} dots")


class RadarDisplay(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.aircraft_list: List[Dict] = []
        self.sweep_angle = 0.0
        self.origin_lat = DEFAULT_AIRPORTS["JNB"]["latitude"]
        self.origin_lon = DEFAULT_AIRPORTS["JNB"]["longitude"]
        self.setMinimumSize(500, 500)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._animate)
        self.timer.start(50)

    def _animate(self):
        self.sweep_angle = (self.sweep_angle + 4.0) % 360.0
        self.update()

    def set_aircraft(self, aircraft_list: List[Dict]):
        self.aircraft_list = aircraft_list or []
        self.update()

    def _to_screen(self, lat: float, lon: float) -> Tuple[float, float, float]:
        distance = great_circle_distance_nm(self.origin_lat, self.origin_lon, lat, lon)
        bearing = initial_bearing(self.origin_lat, self.origin_lon, lat, lon)
        radius = min(self.width(), self.height()) * 0.42 * clamp(distance / 120.0, 0.0, 1.0)
        angle = math.radians(bearing - 90.0)
        cx = self.width() / 2.0
        cy = self.height() / 2.0
        return cx + radius * math.cos(angle), cy + radius * math.sin(angle), distance

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), QColor(5, 18, 10))
        cx = self.width() / 2
        cy = self.height() / 2
        max_radius = min(self.width(), self.height()) * 0.42

        painter.setPen(QPen(QColor(0, 90, 0), 1))
        for factor, label in zip([0.25, 0.5, 0.75, 1.0], [30, 60, 90, 120]):
            radius = max_radius * factor
            painter.drawEllipse(QPointF(cx, cy), radius, radius)
            painter.drawText(int(cx + 6), int(cy - radius + 14), f"{label}nm")
        painter.drawLine(int(cx - max_radius), cy, int(cx + max_radius), cy)
        painter.drawLine(cx, int(cy - max_radius), cx, int(cy + max_radius))

        angle = math.radians(self.sweep_angle - 90)
        sweep_x = cx + math.cos(angle) * max_radius
        sweep_y = cy + math.sin(angle) * max_radius
        painter.setPen(QPen(QColor(0, 255, 120, 180), 2))
        painter.drawLine(int(cx), int(cy), int(sweep_x), int(sweep_y))

        for aircraft in self.aircraft_list:
            x, y, distance = self._to_screen(aircraft["lat"], aircraft["lon"])
            history = aircraft.get("track_history", [])[-12:]
            painter.setPen(QPen(QColor(0, 160, 120, 120), 1))
            prev = None
            for lat, lon, *_rest in history:
                hx, hy, _ = self._to_screen(lat, lon)
                if prev is not None:
                    painter.drawLine(int(prev[0]), int(prev[1]), int(hx), int(hy))
                prev = (hx, hy)
            painter.setPen(QPen(QColor(255, 255, 80), 2))
            painter.setBrush(QBrush(QColor(255, 255, 80)))
            painter.drawEllipse(QPointF(x, y), 4, 4)
            painter.setPen(QColor(220, 255, 220))
            painter.drawText(int(x + 6), int(y - 6), f"{aircraft['callsign']} {int(distance)}nm")

        painter.setPen(QColor(200, 255, 200))
        painter.drawText(10, 20, "Radar Range: 120 NM")


class ApproachGuidanceDisplay(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.distance_nm = 0.0
        self.altitude_ft = 0.0
        self.glide_error_ft = 0.0
        self.localizer_dots = 0.0
        self.terrain_profile = []
        self.setMinimumHeight(260)

    def set_approach_data(self, distance_nm, altitude_ft, glide_error_ft, localizer_dots, terrain_profile=None):
        self.distance_nm = float(distance_nm)
        self.altitude_ft = float(altitude_ft)
        self.glide_error_ft = float(glide_error_ft)
        self.localizer_dots = float(localizer_dots)
        self.terrain_profile = terrain_profile or []
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), QColor(14, 16, 25))
        margin = 18
        profile_rect = QRectF(margin, margin, self.width() * 0.64, self.height() - 2 * margin)
        cdi_rect = QRectF(profile_rect.right() + 15, margin, self.width() - profile_rect.right() - 25, self.height() - 2 * margin)

        painter.setPen(QPen(QColor(90, 90, 110), 1))
        painter.drawRect(profile_rect)
        painter.drawRect(cdi_rect)

        if self.terrain_profile:
            max_elev = max(self.terrain_profile + [self.altitude_ft, 1000])
            min_elev = min(self.terrain_profile + [0])
            path = QPainterPath()
            for idx, elev in enumerate(self.terrain_profile):
                x = profile_rect.left() + idx * (profile_rect.width() / max(1, len(self.terrain_profile) - 1))
                y = profile_rect.bottom() - ((elev - min_elev) / max(1.0, max_elev - min_elev)) * profile_rect.height()
                if idx == 0:
                    path.moveTo(x, y)
                else:
                    path.lineTo(x, y)
            path.lineTo(profile_rect.right(), profile_rect.bottom())
            path.lineTo(profile_rect.left(), profile_rect.bottom())
            painter.fillPath(path, QBrush(QColor(110, 80, 50, 180)))

        max_dist = 15.0
        max_alt = max(1000.0, self.altitude_ft + 1000.0, 4000.0)
        glide_path = QPainterPath()
        for step in range(0, 16):
            dist = max_dist - step
            alt = dist * FT_PER_NM_AT_3_DEG
            x = profile_rect.left() + (dist / max_dist) * profile_rect.width()
            y = profile_rect.bottom() - (alt / max_alt) * profile_rect.height()
            if step == 0:
                glide_path.moveTo(x, y)
            else:
                glide_path.lineTo(x, y)
        painter.setPen(QPen(QColor(0, 210, 255), 2, Qt.DashLine))
        painter.drawPath(glide_path)

        ac_x = profile_rect.left() + (clamp(self.distance_nm, 0.0, max_dist) / max_dist) * profile_rect.width()
        ac_y = profile_rect.bottom() - (clamp(self.altitude_ft, 0.0, max_alt) / max_alt) * profile_rect.height()
        painter.setPen(QPen(QColor(255, 255, 80), 2))
        painter.setBrush(QBrush(QColor(255, 255, 80)))
        painter.drawEllipse(QPointF(ac_x, ac_y), 6, 6)

        center_x = cdi_rect.center().x()
        center_y = cdi_rect.center().y()
        painter.setPen(QPen(QColor(180, 180, 180), 2))
        painter.drawLine(int(center_x), int(cdi_rect.top() + 25), int(center_x), int(cdi_rect.bottom() - 25))
        painter.drawLine(int(cdi_rect.left() + 15), int(center_y), int(cdi_rect.right() - 15), int(center_y))
        for step in range(-2, 3):
            painter.drawEllipse(QPointF(center_x + step * 24, center_y), 3, 3)
        loc_x = center_x + clamp(self.localizer_dots, -2.5, 2.5) * 20
        painter.setPen(QPen(QColor(255, 180, 0), 4))
        painter.drawLine(int(loc_x), int(center_y - 55), int(loc_x), int(center_y + 55))

        painter.setPen(QColor(230, 230, 230))
        painter.drawText(int(profile_rect.left() + 8), int(profile_rect.top() + 18), "Approach Profile")
        painter.drawText(int(cdi_rect.left() + 8), int(cdi_rect.top() + 18), "Localizer")
        painter.drawText(
            int(profile_rect.left() + 8),
            int(profile_rect.bottom() - 8),
            f"Dist {self.distance_nm:.1f}nm  Alt {self.altitude_ft:.0f}ft  GS err {self.glide_error_ft:+.0f}ft",
        )


class Terrain3DWidget(QOpenGLWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.rot_x = 35.0
        self.rot_y = -45.0
        self.zoom = -28.0
        self.last_pos = QPoint()
        self.aircraft_list: List[Dict] = []
        x = np.linspace(-12, 12, 50)
        y = np.linspace(-12, 12, 50)
        xx, yy = np.meshgrid(x, y)
        self.terrain = np.sin(xx / 3.0) * 1.2 + np.cos(yy / 2.6) * 0.8 + np.exp(-((xx + 2) ** 2 + (yy - 1) ** 2) / 30.0) * 3.0

    def set_aircraft(self, aircraft_list: List[Dict]):
        self.aircraft_list = aircraft_list or []
        self.update()

    def initializeGL(self):
        glClearColor(0.05, 0.07, 0.10, 1.0)
        glEnable(GL_DEPTH_TEST)
        glShadeModel(GL_SMOOTH)
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

    def resizeGL(self, w, h):
        glViewport(0, 0, max(1, w), max(1, h))
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45.0, max(1.0, w / max(1.0, h)), 0.1, 200.0)
        glMatrixMode(GL_MODELVIEW)

    def paintGL(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()
        glTranslatef(0.0, -2.5, self.zoom)
        glRotatef(self.rot_x, 1.0, 0.0, 0.0)
        glRotatef(self.rot_y, 0.0, 1.0, 0.0)

        rows, cols = self.terrain.shape
        for r in range(rows - 1):
            glBegin(GL_TRIANGLE_STRIP)
            for c in range(cols):
                for rr in (r, r + 1):
                    x = (c / (cols - 1) - 0.5) * 24.0
                    y = self.terrain[rr, c]
                    z = (rr / (rows - 1) - 0.5) * 24.0
                    shade = clamp((y + 2.5) / 6.0, 0.0, 1.0)
                    glColor4f(0.1 + 0.2 * shade, 0.35 + 0.4 * shade, 0.15 + 0.1 * shade, 1.0)
                    glVertex3f(x, y, z)
            glEnd()

        glColor3f(0.0, 0.7, 1.0)
        glBegin(GL_LINES)
        glVertex3f(-10.0, 0.02, 0.0)
        glVertex3f(10.0, 0.02, 0.0)
        glEnd()

        glPointSize(7.0)
        glBegin(GL_POINTS)
        for idx, _aircraft in enumerate(self.aircraft_list[:8]):
            offset = idx * 2.0 - 6.0
            glColor3f(1.0, 0.9, 0.2)
            glVertex3f(offset, 4.5 + (idx % 3) * 0.6, -offset / 2.0)
        glEnd()

    def mousePressEvent(self, event):
        self.last_pos = event.pos()

    def mouseMoveEvent(self, event):
        dx = event.x() - self.last_pos.x()
        dy = event.y() - self.last_pos.y()
        if event.buttons() & Qt.LeftButton:
            self.rot_x = clamp(self.rot_x + dy * 0.5, -90, 90)
            self.rot_y = (self.rot_y + dx * 0.5) % 360
            self.update()
        self.last_pos = event.pos()

    def wheelEvent(self, event):
        self.zoom = clamp(self.zoom + event.angleDelta().y() / 240.0, -60.0, -8.0)
        self.update()


class ASRACSDisplay(QWidget):
    def __init__(self, layout: JNBAirportLayout, parent=None):
        super().__init__(parent)
        self.layout = layout
        self.targets: List[GroundTarget] = []
        self.alerts: List[IncursionAlert] = []
        self.setMinimumSize(560, 420)

    def set_targets(self, targets: List[GroundTarget]):
        self.targets = targets or []
        self.update()

    def set_alerts(self, alerts: List[IncursionAlert]):
        self.alerts = alerts or []
        self.update()

    def _transform(self, x: float, y: float) -> QPointF:
        min_x, min_y, max_x, max_y = self.layout.bounds
        px = (x - min_x) / (max_x - min_x) * self.width()
        py = self.height() - (y - min_y) / (max_y - min_y) * self.height()
        return QPointF(px, py)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), QColor(22, 24, 28))

        for apron in self.layout.APRONS:
            x, y, w, h = apron["rect"]
            p1 = self._transform(x, y)
            p2 = self._transform(x + w, y - h)
            rect = QRectF(min(p1.x(), p2.x()), min(p1.y(), p2.y()), abs(p2.x() - p1.x()), abs(p2.y() - p1.y()))
            painter.fillRect(rect, QColor(70, 72, 76))
            painter.setPen(QColor(180, 180, 180))
            painter.drawText(rect.topLeft() + QPointF(4, 14), apron["name"])

        painter.setPen(QPen(QColor(180, 180, 180), 8, Qt.SolidLine, Qt.RoundCap))
        for runway in self.layout.get_runways():
            a = self._transform(runway["x1"], runway["y1"])
            b = self._transform(runway["x2"], runway["y2"])
            painter.drawLine(a, b)
            painter.setPen(QColor(255, 255, 255))
            painter.drawText(a + QPointF(6, -6), runway["name"])
            painter.setPen(QPen(QColor(180, 180, 180), 8, Qt.SolidLine, Qt.RoundCap))

        painter.setPen(QPen(QColor(80, 160, 255), 3, Qt.SolidLine, Qt.RoundCap))
        for taxiway in self.layout.get_taxiways():
            pts = [self._transform(x, y) for x, y in taxiway["points"]]
            for idx in range(1, len(pts)):
                painter.drawLine(pts[idx - 1], pts[idx])
            if pts:
                painter.setPen(QColor(160, 220, 255))
                painter.drawText(pts[0] + QPointF(4, -4), taxiway["name"])
                painter.setPen(QPen(QColor(80, 160, 255), 3, Qt.SolidLine, Qt.RoundCap))

        for hotspot in self.layout.get_hotspots():
            center = self._transform(hotspot["x"], hotspot["y"])
            scale_r = hotspot["radius"] / (self.layout.bounds[2] - self.layout.bounds[0]) * self.width()
            painter.setPen(QPen(QColor(255, 120, 0), 2, Qt.DashLine))
            painter.drawEllipse(center, scale_r, scale_r)
            painter.drawText(center + QPointF(8, -6), hotspot["name"])

        for target in self.targets:
            point = self._transform(target.x_m, target.y_m)
            color = QColor(255, 255, 60) if target.status != "HOLD" else QColor(255, 150, 0)
            painter.setPen(QPen(color, 2))
            painter.setBrush(QBrush(color))
            painter.drawEllipse(point, 5, 5)
            painter.drawText(point + QPointF(7, -7), f"{target.callsign} {target.status}")

        if self.alerts:
            latest = self.alerts[0]
            severity_color = {"CAUTION": QColor(255, 196, 0), "WARNING": QColor(255, 120, 0), "CRITICAL": QColor(255, 70, 70)}.get(latest.severity, QColor(255, 255, 255))
            painter.fillRect(QRectF(10, 10, self.width() - 20, 28), QColor(0, 0, 0, 140))
            painter.setPen(severity_color)
            painter.drawText(20, 29, f"{latest.severity}: {latest.message}")


class ASRACSAlertPanel(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        self.summary_label = QLabel("No active alerts")
        self.summary_label.setWordWrap(True)
        self.alert_list = QListWidget()
        layout.addWidget(self.summary_label)
        layout.addWidget(self.alert_list)

    def update_alerts(self, alerts: List[IncursionAlert]):
        self.alert_list.clear()
        alerts = alerts or []
        self.summary_label.setText(f"Active alerts: {len(alerts)}")
        for alert in alerts[:15]:
            item = QListWidgetItem(
                f"[{alert.severity}] {alert.timestamp.strftime('%H:%M:%S')} - {alert.message} ({', '.join(alert.target_ids)})"
            )
            if alert.severity == "CRITICAL":
                item.setForeground(QColor(255, 60, 60))
            elif alert.severity == "WARNING":
                item.setForeground(QColor(255, 160, 0))
            else:
                item.setForeground(QColor(255, 220, 0))
            self.alert_list.addItem(item)


class SAAFOverviewMap(QWidget):
    base_selected = pyqtSignal(str)

    def __init__(self, bases_data: Dict, parent=None):
        super().__init__(parent)
        self.bases_data = bases_data
        self._markers: Dict[str, QRectF] = {}
        self.setMinimumSize(520, 420)

    def _project(self, lat: float, lon: float) -> QPointF:
        min_lon, max_lon = 16.0, 33.5
        min_lat, max_lat = -35.5, -21.5
        x = (lon - min_lon) / (max_lon - min_lon) * (self.width() - 40) + 20
        y = (max_lat - lat) / (max_lat - min_lat) * (self.height() - 40) + 20
        return QPointF(x, y)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), QColor(18, 28, 44))
        painter.setPen(QPen(QColor(50, 70, 110), 1))
        for lon in np.linspace(16.0, 33.5, 8):
            x = self._project(-28.0, lon).x()
            painter.drawLine(int(x), 15, int(x), self.height() - 15)
        for lat in np.linspace(-35.0, -22.0, 7):
            y = self._project(lat, 24.0).y()
            painter.drawLine(15, int(y), self.width() - 15, int(y))

        outline = [
            (-34.9, 18.4), (-34.0, 18.2), (-33.0, 17.8), (-32.0, 17.5), (-31.0, 17.4), (-30.0, 17.3),
            (-29.0, 17.2), (-28.0, 16.9), (-27.0, 16.7), (-26.0, 17.0), (-25.0, 17.6), (-24.0, 18.7),
            (-23.0, 19.8), (-22.4, 21.5), (-22.2, 24.0), (-22.5, 26.0), (-23.5, 29.0), (-25.0, 31.0),
            (-27.0, 32.2), (-29.0, 32.9), (-31.0, 31.8), (-33.0, 29.8), (-34.7, 24.9), (-34.9, 18.4),
        ]
        painter.setPen(QPen(QColor(120, 180, 220), 2))
        polygon = QPolygonF([self._project(lat, lon) for lat, lon in outline])
        painter.drawPolygon(polygon)

        self._markers.clear()
        for key, base in self.bases_data.items():
            pt = self._project(base["lat"], base["lon"])
            marker_rect = QRectF(pt.x() - 5, pt.y() - 5, 10, 10)
            self._markers[key] = marker_rect
            painter.setPen(QPen(QColor(255, 220, 0), 1))
            painter.setBrush(QBrush(QColor(255, 220, 0)))
            painter.drawEllipse(marker_rect)
            painter.setPen(QColor(240, 240, 240))
            painter.drawText(pt + QPointF(8, -8), key)

        painter.drawText(20, 22, "SAAF National Overview")

    def mousePressEvent(self, event):
        pos = event.pos()
        for key, rect in self._markers.items():
            if rect.adjusted(-8, -8, 8, 8).contains(pos):
                self.base_selected.emit(key)
                break


class SAAFBaseMapWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.base_key = ""
        self.base_data: Dict = {}
        self.setMinimumHeight(280)

    def set_base(self, base_key: str, base_data: Dict):
        self.base_key = base_key
        self.base_data = base_data or {}
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), QColor(26, 26, 34))
        if not self.base_data:
            painter.setPen(QColor(220, 220, 220))
            painter.drawText(self.rect(), Qt.AlignCenter, "Select a base from the overview map")
            return

        cx = self.width() / 2
        cy = self.height() / 2
        painter.setPen(QPen(QColor(180, 180, 180), 12, Qt.SolidLine, Qt.RoundCap))
        runway = self.base_data.get("runways", ["03/21"])[0]
        heading = parse_runway_heading(runway)
        angle = math.radians(heading - 90)
        dx = math.cos(angle) * min(self.width(), self.height()) * 0.28
        dy = math.sin(angle) * min(self.width(), self.height()) * 0.28
        painter.drawLine(QPointF(cx - dx, cy - dy), QPointF(cx + dx, cy + dy))
        painter.setPen(QColor(255, 255, 255))
        painter.drawText(int(cx - dx), int(cy - dy - 10), runway)

        painter.setPen(QPen(QColor(80, 200, 255), 3))
        painter.drawLine(int(cx - 110), int(cy + 60), int(cx + 110), int(cy + 60))
        painter.drawText(int(cx - 105), int(cy + 52), "Taxiway Alpha")

        painter.setPen(QColor(255, 220, 0))
        painter.setBrush(QBrush(QColor(255, 220, 0)))
        painter.drawEllipse(QPointF(cx - 140, cy - 80), 6, 6)
        painter.drawText(int(cx - 130), int(cy - 85), f"{self.base_data.get('vor_ident', 'VOR')} {self.base_data.get('vor_freq', 0):.2f}")

        if self.base_data.get("ils"):
            painter.setPen(QColor(0, 220, 140))
            painter.drawText(14, 22, f"ILS RWY {self.base_data.get('ils_rwy')} {self.base_data.get('ils_freq')}")
        painter.setPen(QColor(220, 220, 220))
        painter.drawText(14, self.height() - 14, self.base_data.get("description", ""))


class SAAFBaseInfoWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        self.title_label = QLabel("No base selected")
        self.title_label.setFont(QFont("Arial", 12, QFont.Bold))
        self.info_text = QTextEdit()
        self.info_text.setReadOnly(True)
        layout.addWidget(self.title_label)
        layout.addWidget(self.info_text)

    def set_base(self, base_key: str, base_data: Dict):
        self.title_label.setText(f"{base_key} - {base_data.get('name', '')}")
        lines = [
            f"ICAO: {base_data.get('icao', base_key)}",
            f"Coordinates: {base_data.get('lat')} / {base_data.get('lon')}",
            f"Elevation: {base_data.get('elev_ft')} ft",
            f"Runways: {', '.join(base_data.get('runways', []))} ({base_data.get('rwy_length_m')} m)",
            f"VOR: {base_data.get('vor_ident')} {base_data.get('vor_freq')} MHz {base_data.get('vor_type')}",
            f"ILS: {'Yes' if base_data.get('ils') else 'No'} {base_data.get('ils_rwy', '')} {base_data.get('ils_freq', '')}",
            f"Tower/App/Ground: {base_data.get('twr_freq')} / {base_data.get('app_freq')} / {base_data.get('gnd_freq')}",
            f"ATIS: {base_data.get('atis_freq')}  NDB: {base_data.get('ndb_ident')} {base_data.get('ndb_freq')}",
            f"Squadrons: {', '.join(base_data.get('squadrons', []))}",
            "",
            base_data.get("description", ""),
        ]
        self.info_text.setPlainText("\n".join(lines))


class VORAirportMonitorApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("VOR / ASRACS / SAAF Monitoring System v5.3")
        self.resize(1480, 980)
        self.config = self._load_config()
        self.saaf_db = self._build_saaf_db()
        self.vor_configs: Dict[str, VORAirportConfig] = {}
        self.vor_db = self._build_vor_db()
        self.current_vor_key = ""
        self.current_base_key = "FAWK"
        self.lda_parser: Optional[LDAFileParser] = None

        self.connection_handler = VORConnectionHandler()
        self.data_processor = VORDataProcessor()
        self.aircraft_tracker = AircraftTracker()
        self.glide_detector = GlideSlopeDetector(3.0)
        self.slope_analyzer = SurfaceSlopeAnalyzer()
        self.layout_db = JNBAirportLayout()
        self.asracs_engine = ASRACSSimEngine()
        self.sim_engine = SimulationEngine()
        self.mock_server = MockVORTCPServer()
        self.data_thread: Optional[DataAcquisitionThread] = None

        self._build_ui()
        self._build_menu()
        self.statusBar().showMessage("Ready")

        self.vor_timer = QTimer(self)
        self.vor_timer.timeout.connect(self._update_vor_display)
        self.vor_timer.start(1000)

        self.radar_timer = QTimer(self)
        self.radar_timer.timeout.connect(self._update_radar_display)
        self.radar_timer.start(1000)

        self.approach_timer = QTimer(self)
        self.approach_timer.timeout.connect(self._update_approach_display)
        self.approach_timer.start(500)

        self.diagnostics_timer = QTimer(self)
        self.diagnostics_timer.timeout.connect(self._update_diagnostics)
        self.diagnostics_timer.start(2000)

        self._on_vor_index_changed(0)

    def _load_config(self) -> Dict:
        default_config = {
            "version": "5.3",
            "generated": datetime.utcnow().isoformat(),
            "airports": copy.deepcopy(DEFAULT_AIRPORTS),
            "vor_stations": copy.deepcopy(DEFAULT_VOR_STATIONS),
            "ui": {"last_vor": "FAWK_VOR", "theme": "dark"},
        }
        if not CONFIG_PATH.exists():
            CONFIG_PATH.write_text(yaml.safe_dump(default_config, sort_keys=False), encoding="utf-8")
            return default_config
        try:
            data = yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8")) or {}
            if not isinstance(data, dict):
                data = {}
            data.setdefault("version", "5.3")
            data.setdefault("airports", copy.deepcopy(DEFAULT_AIRPORTS))
            data.setdefault("vor_stations", copy.deepcopy(DEFAULT_VOR_STATIONS))
            data.setdefault("ui", {"last_vor": "FAWK_VOR", "theme": "dark"})
            for key, value in DEFAULT_AIRPORTS.items():
                data["airports"].setdefault(key, copy.deepcopy(value))
            for key, value in DEFAULT_VOR_STATIONS.items():
                data["vor_stations"].setdefault(key, copy.deepcopy(value))
            return data
        except Exception as exc:
            logger.exception("Config load failed, recreating default config: %s", exc)
            CONFIG_PATH.write_text(yaml.safe_dump(default_config, sort_keys=False), encoding="utf-8")
            return default_config

    def _save_config(self):
        try:
            CONFIG_PATH.write_text(yaml.safe_dump(self.config, sort_keys=False), encoding="utf-8")
        except Exception as exc:
            logger.exception("Failed to save config: %s", exc)

    def _build_saaf_db(self) -> Dict:
        return copy.deepcopy(DEFAULT_SAAF_BASES)

    def _build_vor_db(self) -> Dict:
        data = copy.deepcopy(self.config.get("vor_stations", DEFAULT_VOR_STATIONS))
        for key, item in data.items():
            airport = item.get("airport", "")
            airport_meta = self.config.get("airports", {}).get(airport, {})
            self.vor_configs[key] = VORAirportConfig(
                key=key,
                name=item.get("name", key),
                ident=item.get("ident", "---"),
                frequency=float(item.get("frequency", 0.0)),
                latitude=float(item.get("latitude", airport_meta.get("latitude", 0.0))),
                longitude=float(item.get("longitude", airport_meta.get("longitude", 0.0))),
                elevation_ft=int(airport_meta.get("elevation", 0)),
                station_type=item.get("type", "VOR"),
                airport_code=airport,
                ils_available=bool(item.get("ils_available", False)),
                ils_runways=copy.deepcopy(item.get("ils_runways", [])),
                remarks=item.get("remarks", ""),
            )
        return data

    def _build_ui(self):
        self.tabs = QTabWidget()
        self.setCentralWidget(self.tabs)
        self.tabs.addTab(self._tab_vor(), "VOR Monitor")
        self.tabs.addTab(self._tab_radar(), "Radar")
        self.tabs.addTab(self._tab_approach(), "Approach")
        self.tabs.addTab(self._tab_terrain(), "Terrain 3D")
        self.tabs.addTab(self._tab_asracs(), "ASRACS")
        self.tabs.addTab(self._tab_saaf(), "SAAF Bases")
        self.tabs.addTab(self._tab_connection(), "Connection")
        self.tabs.addTab(self._tab_diagnostics(), "Diagnostics")
        self._populate_vor_combo()

    def _build_menu(self):
        menu = self.menuBar()
        file_menu = menu.addMenu("File")
        import_action = QAction("Import LDA", self)
        import_action.triggered.connect(self._import_lda_file)
        file_menu.addAction(import_action)
        file_menu.addSeparator()
        save_action = QAction("Save Config", self)
        save_action.triggered.connect(self._save_config)
        file_menu.addAction(save_action)
        exit_action = QAction("Exit", self)
        exit_action.triggered.connect(self.close)
        file_menu.addAction(exit_action)

        tools_menu = menu.addMenu("Tools")
        sim_start = QAction("Start Simulation", self)
        sim_start.triggered.connect(self._start_simulation)
        tools_menu.addAction(sim_start)
        sim_stop = QAction("Stop Simulation", self)
        sim_stop.triggered.connect(self._stop_simulation)
        tools_menu.addAction(sim_stop)

        server_menu = menu.addMenu("Server")
        server_start = QAction("Start Mock Server", self)
        server_start.triggered.connect(self._start_mock_server)
        server_menu.addAction(server_start)
        server_stop = QAction("Stop Mock Server", self)
        server_stop.triggered.connect(self._stop_mock_server)
        server_menu.addAction(server_stop)

        toolbar = QToolBar("Quick")
        self.addToolBar(toolbar)
        toolbar.addAction(import_action)
        toolbar.addAction(sim_start)
        toolbar.addAction(sim_stop)
        toolbar.addAction(server_start)
        toolbar.addAction(server_stop)

    def _tab_vor(self) -> QWidget:
        widget = QWidget()
        layout = QVBoxLayout(widget)

        top = QHBoxLayout()
        self.vor_combo = QComboBox()
        self.vor_combo.currentIndexChanged.connect(self._on_vor_index_changed)
        top.addWidget(QLabel("Select VOR:"))
        top.addWidget(self.vor_combo)
        top.addStretch(1)
        layout.addLayout(top)

        info_group = QGroupBox("Station Information")
        info_grid = QGridLayout(info_group)
        self.vor_name_label = QLabel("-")
        self.vor_ident_label = QLabel("-")
        self.vor_freq_label = QLabel("-")
        self.vor_type_label = QLabel("-")
        self.vor_airport_label = QLabel("-")
        self.vor_remarks_text = QTextEdit()
        self.vor_remarks_text.setReadOnly(True)
        self.vor_remarks_text.setMaximumHeight(90)

        labels = [
            ("Name", self.vor_name_label),
            ("Ident", self.vor_ident_label),
            ("Frequency", self.vor_freq_label),
            ("Type", self.vor_type_label),
            ("Airport", self.vor_airport_label),
        ]
        for row, (name, control) in enumerate(labels):
            info_grid.addWidget(QLabel(name + ":"), row, 0)
            info_grid.addWidget(control, row, 1)
        info_grid.addWidget(QLabel("Remarks:"), len(labels), 0)
        info_grid.addWidget(self.vor_remarks_text, len(labels), 1, 1, 2)
        layout.addWidget(info_group)

        data_group = QGroupBox("Live Data")
        data_grid = QGridLayout(data_group)
        self.vor_bearing_label = QLabel("0.0°")
        self.vor_signal_label = QLabel("0.0%")
        self.vor_conn_label = QLabel("Disconnected")
        self.vor_lda_label = QLabel("No LDA imported")
        self.vor_ils_label = QLabel("-")
        for row, (name, control) in enumerate([
            ("Bearing", self.vor_bearing_label),
            ("Signal", self.vor_signal_label),
            ("Connection", self.vor_conn_label),
            ("ILS", self.vor_ils_label),
            ("LDA", self.vor_lda_label),
        ]):
            data_grid.addWidget(QLabel(name + ":"), row, 0)
            data_grid.addWidget(control, row, 1)
        layout.addWidget(data_group)

        self.vor_chart_series = QLineSeries()
        self.vor_chart = QChart()
        self.vor_chart.addSeries(self.vor_chart_series)
        self.vor_chart.createDefaultAxes()
        self.vor_chart.setTitle("Signal Strength History")
        self.vor_chart_view = QChartView(self.vor_chart)
        self.vor_chart_view.setMinimumHeight(240)
        layout.addWidget(self.vor_chart_view)

        self.vor_history_table = QTableWidget(0, 4)
        self.vor_history_table.setHorizontalHeaderLabels(["UTC", "Bearing", "Signal", "Freq"])
        self.vor_history_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        layout.addWidget(self.vor_history_table)
        return widget

    def _tab_radar(self) -> QWidget:
        widget = QWidget()
        layout = QHBoxLayout(widget)
        self.radar = RadarDisplay()
        layout.addWidget(self.radar, 2)

        right = QVBoxLayout()
        self.radar_table = QTableWidget(0, 6)
        self.radar_table.setHorizontalHeaderLabels(["Callsign", "Lat", "Lon", "Alt", "Spd", "Hdg"])
        self.radar_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        right.addWidget(self.radar_table)

        self.radar_summary = QTextEdit()
        self.radar_summary.setReadOnly(True)
        right.addWidget(self.radar_summary)
        layout.addLayout(right, 1)
        return widget

    def _tab_approach(self) -> QWidget:
        widget = QWidget()
        layout = QVBoxLayout(widget)
        self.approach_display = ApproachGuidanceDisplay()
        layout.addWidget(self.approach_display, 2)

        bottom = QHBoxLayout()
        self.cdi_display = CDIDisplay()
        bottom.addWidget(self.cdi_display, 1)
        self.approach_text = QTextEdit()
        self.approach_text.setReadOnly(True)
        bottom.addWidget(self.approach_text, 2)
        layout.addLayout(bottom)
        return widget

    def _tab_terrain(self) -> QWidget:
        widget = QWidget()
        layout = QVBoxLayout(widget)
        self.terrain_widget = Terrain3DWidget()
        layout.addWidget(self.terrain_widget, 3)

        controls = QHBoxLayout()
        self.sim_start_btn = QPushButton("Start En-Route Sim")
        self.sim_stop_btn = QPushButton("Stop En-Route Sim")
        self.sim_start_btn.clicked.connect(self._start_simulation)
        self.sim_stop_btn.clicked.connect(self._stop_simulation)
        controls.addWidget(self.sim_start_btn)
        controls.addWidget(self.sim_stop_btn)
        controls.addStretch(1)
        layout.addLayout(controls)
        return widget

    def _tab_asracs(self) -> QWidget:
        widget = QWidget()
        layout = QHBoxLayout(widget)
        self.asracs_display = ASRACSDisplay(self.layout_db)
        self.asracs_alert_panel = ASRACSAlertPanel()
        layout.addWidget(self.asracs_display, 3)
        layout.addWidget(self.asracs_alert_panel, 2)
        return widget

    def _tab_saaf(self) -> QWidget:
        widget = QWidget()
        layout = QHBoxLayout(widget)
        self.saaf_map = SAAFOverviewMap(self.saaf_db)
        self.saaf_map.base_selected.connect(self._handle_base_selected)
        layout.addWidget(self.saaf_map, 2)
        right = QVBoxLayout()
        self.saaf_base_map = SAAFBaseMapWidget()
        self.saaf_base_info = SAAFBaseInfoWidget()
        right.addWidget(self.saaf_base_map)
        right.addWidget(self.saaf_base_info)
        layout.addLayout(right, 2)
        self._handle_base_selected(self.current_base_key)
        return widget

    def _tab_connection(self) -> QWidget:
        widget = QWidget()
        layout = QVBoxLayout(widget)

        serial_group = QGroupBox("Serial Connection")
        serial_layout = QGridLayout(serial_group)
        self.serial_port_combo = QComboBox()
        for port in serial.tools.list_ports.comports():
            self.serial_port_combo.addItem(port.device)
        self.serial_baud_combo = QComboBox()
        for baud in [9600, 19200, 38400, 57600, 115200]:
            self.serial_baud_combo.addItem(str(baud))
        self.serial_connect_btn = QPushButton("Connect Serial")
        self.serial_connect_btn.clicked.connect(self._connect_serial)
        serial_layout.addWidget(QLabel("Port"), 0, 0)
        serial_layout.addWidget(self.serial_port_combo, 0, 1)
        serial_layout.addWidget(QLabel("Baud"), 1, 0)
        serial_layout.addWidget(self.serial_baud_combo, 1, 1)
        serial_layout.addWidget(self.serial_connect_btn, 2, 0, 1, 2)
        layout.addWidget(serial_group)

        tcp_group = QGroupBox("TCP/IP Connection")
        tcp_layout = QGridLayout(tcp_group)
        self.tcp_host_edit = QLineEdit("127.0.0.1")
        self.tcp_port_edit = QLineEdit("5000")
        self.tcp_connect_btn = QPushButton("Connect TCP")
        self.tcp_connect_btn.clicked.connect(self._connect_tcp)
        self.disconnect_btn = QPushButton("Disconnect")
        self.disconnect_btn.clicked.connect(self._disconnect)
        self.server_start_btn = QPushButton("Start Mock Server")
        self.server_start_btn.clicked.connect(self._start_mock_server)
        self.server_stop_btn = QPushButton("Stop Mock Server")
        self.server_stop_btn.clicked.connect(self._stop_mock_server)
        tcp_layout.addWidget(QLabel("Host"), 0, 0)
        tcp_layout.addWidget(self.tcp_host_edit, 0, 1)
        tcp_layout.addWidget(QLabel("Port"), 1, 0)
        tcp_layout.addWidget(self.tcp_port_edit, 1, 1)
        tcp_layout.addWidget(self.tcp_connect_btn, 2, 0, 1, 2)
        tcp_layout.addWidget(self.disconnect_btn, 3, 0, 1, 2)
        tcp_layout.addWidget(self.server_start_btn, 4, 0, 1, 2)
        tcp_layout.addWidget(self.server_stop_btn, 5, 0, 1, 2)
        layout.addWidget(tcp_group)

        self.connection_status_text = QTextEdit()
        self.connection_status_text.setReadOnly(True)
        layout.addWidget(self.connection_status_text)
        return widget

    def _tab_diagnostics(self) -> QWidget:
        widget = QWidget()
        layout = QVBoxLayout(widget)
        self.diagnostics_text = QTextEdit()
        self.diagnostics_text.setReadOnly(True)
        layout.addWidget(self.diagnostics_text)
        self.diagnostics_table = QTableWidget(0, 2)
        self.diagnostics_table.setHorizontalHeaderLabels(["Metric", "Value"])
        self.diagnostics_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        layout.addWidget(self.diagnostics_table)
        return widget

    def _populate_vor_combo(self):
        self.vor_combo.blockSignals(True)
        self.vor_combo.clear()
        for key, value in sorted(self.vor_db.items()):
            self.vor_combo.addItem(f"{key} - {value.get('ident')} {value.get('frequency'):.2f} MHz", key)
        desired = self.config.get("ui", {}).get("last_vor", "FAWK_VOR")
        idx = max(0, self.vor_combo.findData(desired))
        self.vor_combo.setCurrentIndex(idx)
        self.vor_combo.blockSignals(False)
        self._on_vor_index_changed(idx)

    def _handle_base_selected(self, base_key: str):
        self.current_base_key = base_key
        base_data = self.saaf_db.get(base_key, {})
        self.saaf_base_map.set_base(base_key, base_data)
        self.saaf_base_info.set_base(base_key, base_data)

    def _on_vor_index_changed(self, idx):
        if idx < 0:
            return
        key = self.vor_combo.itemData(idx) if hasattr(self, "vor_combo") else None
        if not key:
            return
        self.current_vor_key = key
        self.config.setdefault("ui", {})["last_vor"] = key
        data = self.vor_db.get(key, {})
        airport = data.get("airport", "")
        base_data = self.saaf_db.get(airport, self.config.get("airports", {}).get(airport, {}))
        ils_desc = "None"
        if data.get("ils_available") and data.get("ils_runways"):
            ils_desc = ", ".join(f"RWY {item.get('runway')} {item.get('frequency'):.2f}" for item in data.get("ils_runways", []))

        if hasattr(self, "vor_name_label"):
            self.vor_name_label.setText(data.get("name", "-"))
        if hasattr(self, "vor_ident_label"):
            self.vor_ident_label.setText(data.get("ident", "-"))
        if hasattr(self, "vor_freq_label"):
            self.vor_freq_label.setText(f"{float(data.get('frequency', 0.0)):.2f} MHz")
        if hasattr(self, "vor_type_label"):
            self.vor_type_label.setText(data.get("type", "-"))
        if hasattr(self, "vor_airport_label"):
            self.vor_airport_label.setText(base_data.get("name", airport))
        if hasattr(self, "vor_ils_label"):
            self.vor_ils_label.setText(ils_desc)
        if hasattr(self, "vor_remarks_text"):
            self.vor_remarks_text.setPlainText(data.get("remarks", ""))
        if hasattr(self, "saaf_base_map") and airport in self.saaf_db:
            self.saaf_base_map.set_base(airport, self.saaf_db[airport])
        if hasattr(self, "saaf_base_info") and airport in self.saaf_db:
            self.saaf_base_info.set_base(airport, self.saaf_db[airport])
        if hasattr(self, "saaf_map") and airport in self.saaf_db:
            self.current_base_key = airport
        if hasattr(self, "approach_text"):
            terrain = TERRAIN_LIBRARY.get(airport, TERRAIN_LIBRARY.get("JNB", []))
            slope_profile = self.slope_analyzer.analyze_profile(terrain, spacing_m=100)
            self.approach_text.setPlainText(
                f"Selected: {data.get('name')}\n"
                f"Airport: {airport}\n"
                f"Glide angle: {base_data.get('glide_angle', 3.0)}°\n"
                f"Terrain safety: {self.slope_analyzer.assess_approach_safety(slope_profile)}\n"
                f"Max slope: {slope_profile['max_slope_pct']:.2f}%\n"
                f"Average slope: {slope_profile['avg_slope_pct']:.2f}%"
            )

    def _import_lda_file(self):
        filepath, _ = QFileDialog.getOpenFileName(self, "Import LDA File", str(Path.cwd()), "LDA Files (*.lda);;All Files (*)")
        if not filepath:
            return
        parser = LDAFileParser(filepath)
        if parser.parse():
            self.lda_parser = parser
            if hasattr(self, "vor_lda_label"):
                self.vor_lda_label.setText(Path(filepath).name)
            QMessageBox.information(self, "LDA Import", parser.get_summary())
        else:
            QMessageBox.warning(self, "LDA Import", f"Failed to parse LDA file:\n{parser.get_summary()}")

    def _start_simulation(self):
        self.sim_engine.start()
        self.asracs_engine.start()
        self.statusBar().showMessage("Simulation started")

    def _stop_simulation(self):
        self.sim_engine.stop()
        self.asracs_engine.stop()
        self.statusBar().showMessage("Simulation stopped")

    def _start_mock_server(self):
        if not self.mock_server.is_running():
            self.mock_server.start()
            self.statusBar().showMessage("Mock server started")

    def _stop_mock_server(self):
        if self.mock_server.is_running():
            self.mock_server.stop()
            self.statusBar().showMessage("Mock server stopped")

    def _start_data_thread(self):
        if self.data_thread and self.data_thread.isRunning():
            self.data_thread.stop()
            self.data_thread.wait(1500)
        self.data_thread = DataAcquisitionThread(self.connection_handler, self.data_processor)
        self.data_thread.data_received.connect(self._handle_acquired_data)
        self.data_thread.connection_lost.connect(self._handle_connection_lost)
        self.data_thread.start()

    def _connect_serial(self):
        port = self.serial_port_combo.currentText().strip()
        baud = self.serial_baud_combo.currentText().strip() or "9600"
        if not port:
            QMessageBox.warning(self, "Serial", "No serial port available")
            return
        if self.connection_handler.connect_serial(port, int(baud)):
            self._start_data_thread()
            self.statusBar().showMessage(f"Connected serial {port}")
        else:
            QMessageBox.warning(self, "Serial", self.connection_handler.last_error or "Serial connection failed")

    def _connect_tcp(self):
        host = self.tcp_host_edit.text().strip() or "127.0.0.1"
        port = self.tcp_port_edit.text().strip() or "5000"
        if self.connection_handler.connect_tcp(host, int(port)):
            self._start_data_thread()
            self.statusBar().showMessage(f"Connected TCP {host}:{port}")
        else:
            QMessageBox.warning(self, "TCP", self.connection_handler.last_error or "TCP connection failed")

    def _disconnect(self):
        if self.data_thread and self.data_thread.isRunning():
            self.data_thread.stop()
            self.data_thread.wait(1500)
        self.connection_handler.disconnect()
        self.statusBar().showMessage("Disconnected")

    def _handle_acquired_data(self, parsed: Dict):
        bearing = float(parsed.get("bearing", 0.0))
        signal = float(parsed.get("signal", 0.0))
        freq = float(parsed.get("freq", 0.0))
        self.aircraft_tracker.update(bearing, distance=25.0, altitude=8000.0)
        if hasattr(self, "vor_bearing_label"):
            self.vor_bearing_label.setText(f"{bearing:.1f}°")
        if hasattr(self, "vor_signal_label"):
            self.vor_signal_label.setText(f"{signal:.1f}%")
        if hasattr(self, "vor_conn_label"):
            self.vor_conn_label.setText(self.connection_handler.get_status())
        logger.debug("Acquired sample freq=%s bearing=%s signal=%s", freq, bearing, signal)

    def _handle_connection_lost(self, reason: str):
        if hasattr(self, "vor_conn_label"):
            self.vor_conn_label.setText(f"Lost: {reason}")
        self.statusBar().showMessage(f"Connection lost: {reason}")

    def _update_vor_display(self):
        history = list(self.data_processor.get_history())
        if not history and not self.connection_handler.is_connected():
            bearing = random.uniform(0, 360)
            signal = random.uniform(65, 95)
            freq = self.vor_db.get(self.current_vor_key, {}).get("frequency", 116.90)
            self.data_processor.add_sample(bearing, signal, freq)
            history = list(self.data_processor.get_history())
        if hasattr(self, "vor_bearing_label"):
            self.vor_bearing_label.setText(f"{self.data_processor.get_bearing():.1f}°")
        if hasattr(self, "vor_signal_label"):
            self.vor_signal_label.setText(f"{self.data_processor.get_signal_strength():.1f}%")
        if hasattr(self, "vor_conn_label"):
            self.vor_conn_label.setText(self.connection_handler.get_status())

        if hasattr(self, "vor_chart_series"):
            self.vor_chart_series.clear()
            for idx, item in enumerate(history[-30:]):
                self.vor_chart_series.append(idx, item["signal_strength"])
            self.vor_chart.createDefaultAxes()

        if hasattr(self, "vor_history_table"):
            self.vor_history_table.setRowCount(min(12, len(history)))
            for row, item in enumerate(reversed(history[-12:])):
                self.vor_history_table.setItem(row, 0, QTableWidgetItem(datetime.utcfromtimestamp(item["timestamp"]).strftime("%H:%M:%S")))
                self.vor_history_table.setItem(row, 1, QTableWidgetItem(f"{item['bearing']:.1f}"))
                self.vor_history_table.setItem(row, 2, QTableWidgetItem(f"{item['signal_strength']:.1f}"))
                self.vor_history_table.setItem(row, 3, QTableWidgetItem(f"{item['frequency']:.2f}"))

    def _update_radar_display(self):
        aircraft = self.sim_engine.get_aircraft()
        if hasattr(self, "radar"):
            self.radar.set_aircraft(aircraft)
        if hasattr(self, "terrain_widget"):
            self.terrain_widget.set_aircraft(aircraft)
        if hasattr(self, "radar_table"):
            self.radar_table.setRowCount(len(aircraft))
            for row, ac in enumerate(aircraft):
                values = [
                    ac["callsign"],
                    f"{ac['lat']:.3f}",
                    f"{ac['lon']:.3f}",
                    str(ac["altitude"]),
                    str(ac["speed"]),
                    f"{ac['heading']:.0f}",
                ]
                for col, value in enumerate(values):
                    self.radar_table.setItem(row, col, QTableWidgetItem(value))
        if hasattr(self, "radar_summary"):
            lines = [f"Tracked aircraft: {len(aircraft)}"]
            for ac in aircraft[:6]:
                dist = great_circle_distance_nm(DEFAULT_AIRPORTS["JNB"]["latitude"], DEFAULT_AIRPORTS["JNB"]["longitude"], ac["lat"], ac["lon"])
                lines.append(f"{ac['callsign']}: {dist:.1f}nm / {ac['altitude']}ft / {ac['speed']}kts")
            self.radar_summary.setPlainText("\n".join(lines))

    def _update_approach_display(self):
        aircraft = self.sim_engine.get_aircraft()
        if not aircraft:
            return
        ac = min(
            aircraft,
            key=lambda item: great_circle_distance_nm(
                DEFAULT_AIRPORTS["JNB"]["latitude"],
                DEFAULT_AIRPORTS["JNB"]["longitude"],
                item["lat"],
                item["lon"],
            ),
        )
        vor_item = self.vor_db.get(self.current_vor_key, {})
        airport_key = vor_item.get("airport", "JNB")
        airport_meta = self.saaf_db.get(airport_key, DEFAULT_AIRPORTS.get(airport_key, DEFAULT_AIRPORTS["JNB"]))
        airport_lat = airport_meta.get("lat", airport_meta.get("latitude", DEFAULT_AIRPORTS["JNB"]["latitude"]))
        airport_lon = airport_meta.get("lon", airport_meta.get("longitude", DEFAULT_AIRPORTS["JNB"]["longitude"]))
        airport_elev = airport_meta.get("elev_ft", airport_meta.get("elevation", 0))
        distance_nm = great_circle_distance_nm(airport_lat, airport_lon, ac["lat"], ac["lon"])
        rel_alt = ac["altitude"] - airport_elev
        glide_error = self.glide_detector.glide_slope_error_ft(rel_alt, distance_nm)
        localizer_ref = initial_bearing(ac["lat"], ac["lon"], airport_lat, airport_lon)
        runway_heading = parse_runway_heading(airport_meta.get("ils_rwy", airport_meta.get("runways", ["03/21"])[0]))
        localizer_error_deg = ((localizer_ref - runway_heading + 540) % 360) - 180
        localizer_dots = clamp(localizer_error_deg / 2.5, -2.5, 2.5)
        terrain_profile = TERRAIN_LIBRARY.get(airport_key, TERRAIN_LIBRARY.get("JNB", []))
        if hasattr(self, "approach_display"):
            self.approach_display.set_approach_data(distance_nm, rel_alt, glide_error, localizer_dots, terrain_profile)
        if hasattr(self, "cdi_display"):
            self.cdi_display.set_deviation(localizer_dots)
        if hasattr(self, "approach_text"):
            profile = self.glide_detector.validate_profile([(distance_nm, rel_alt)])
            slope_profile = self.slope_analyzer.analyze_profile(terrain_profile)
            self.approach_text.setPlainText(
                f"Aircraft: {ac['callsign']}\n"
                f"Base: {airport_meta.get('name', airport_key)}\n"
                f"Distance: {distance_nm:.1f} nm\n"
                f"Relative altitude: {rel_alt:.0f} ft\n"
                f"Glide error: {glide_error:+.0f} ft\n"
                f"Required descent rate ({ac['speed']} kt): {self.glide_detector.required_descent_rate_fpm(ac['speed']):.0f} fpm\n"
                f"Localizer: {localizer_dots:+.2f} dots\n"
                f"Profile valid: {profile['valid']}\n"
                f"Terrain safety: {self.slope_analyzer.assess_approach_safety(slope_profile)}"
            )

    def _update_diagnostics(self):
        metrics = {
            "Current VOR": self.current_vor_key or "-",
            "Connection": self.connection_handler.get_status(),
            "Mock Server": "RUNNING" if self.mock_server.is_running() else "STOPPED",
            "Data Samples": len(self.data_processor.get_history()),
            "Tracked Aircraft": len(self.sim_engine.get_aircraft()),
            "ASRACS Targets": len(self.asracs_engine.get_targets()),
            "ASRACS Alerts": len(self.asracs_engine.get_alerts()),
            "LDA Imported": self.lda_parser.filepath.name if self.lda_parser else "No",
        }
        if hasattr(self, "diagnostics_table"):
            self.diagnostics_table.setRowCount(len(metrics))
            for row, (key, value) in enumerate(metrics.items()):
                self.diagnostics_table.setItem(row, 0, QTableWidgetItem(str(key)))
                self.diagnostics_table.setItem(row, 1, QTableWidgetItem(str(value)))

        target_lines = []
        for target in self.asracs_engine.get_targets()[:8]:
            target_lines.append(f"{target.callsign} {target.status} route {target.route_name} ({target.x_m:.0f},{target.y_m:.0f})")
        alert_lines = [f"[{a.severity}] {a.message}" for a in self.asracs_engine.get_alerts()[:6]]
        lda_text = self.lda_parser.get_summary() if self.lda_parser else "No LDA file imported"
        if hasattr(self, "diagnostics_text"):
            self.diagnostics_text.setPlainText(
                "Diagnostics\n"
                "===========\n"
                f"UTC: {datetime.utcnow().isoformat()}\n"
                f"Connection: {self.connection_handler.get_status()}\n"
                f"Mock server running: {self.mock_server.is_running()}\n"
                f"\nTargets:\n" + ("\n".join(target_lines) if target_lines else "None") +
                f"\n\nAlerts:\n" + ("\n".join(alert_lines) if alert_lines else "None") +
                f"\n\nLDA:\n{lda_text}"
            )
        if hasattr(self, "connection_status_text"):
            status_payload = "Disconnected"
            if self.connection_handler.is_connected():
                try:
                    status_payload = self.connection_handler.send_command("HEALTH")
                except Exception:
                    status_payload = self.connection_handler.get_status()
            self.connection_status_text.setPlainText(
                f"Connection Status\n=================\n{self.connection_handler.get_status()}\n\nHealth\n------\n{status_payload}"
            )
        if hasattr(self, "asracs_display"):
            self.asracs_display.set_targets(self.asracs_engine.get_targets())
            self.asracs_display.set_alerts(self.asracs_engine.get_alerts())
        if hasattr(self, "asracs_alert_panel"):
            self.asracs_alert_panel.update_alerts(self.asracs_engine.get_alerts())

    def closeEvent(self, event):
        try:
            for timer in [self.vor_timer, self.radar_timer, self.approach_timer, self.diagnostics_timer]:
                timer.stop()
        except Exception:
            pass
        try:
            if self.data_thread and self.data_thread.isRunning():
                self.data_thread.stop()
                self.data_thread.wait(1500)
        except Exception:
            logger.exception("Failed stopping data thread")
        try:
            self.connection_handler.disconnect()
        except Exception:
            logger.exception("Failed disconnecting handler")
        try:
            self.sim_engine.stop()
            self.asracs_engine.stop()
        except Exception:
            logger.exception("Failed stopping simulations")
        try:
            self.mock_server.stop()
        except Exception:
            logger.exception("Failed stopping mock server")
        self._save_config()
        event.accept()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    w = VORAirportMonitorApp()
    w.show()
    sys.exit(app.exec_())
