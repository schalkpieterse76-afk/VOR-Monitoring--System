#!/usr/bin/env python3
"""
VOR / ASRACS / SAAF Airport Monitoring System  v5.2
=====================================================
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

import sys, logging, csv, socket, yaml, math, random, time
import numpy as np
from pathlib import Path
from datetime import datetime
from typing import List, Dict, Optional, Tuple
from collections import deque
from threading import Lock, Thread

import serial
import serial.tools.list_ports

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
    QPolygonF, QStandardItem,
)
from PyQt5.QtChart import QChart, QChartView, QLineSeries

from OpenGL.GL import *
from OpenGL.GLU import *

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[logging.FileHandler('vor_monitor.log'), logging.StreamHandler()]
)
logger = logging.getLogger(__name__)


STANDARD_GLIDE_SLOPE_DEG = 3.0
KM_PER_NM = 1.852
M_PER_KM = 1000.0
APPROACH_GUIDE_PIXELS = 110
MAX_LOCALIZER_DOTS = 2.5
FULL_SCALE_LOCALIZER_DEG = 5.0


# ===========================================================================
# Glide slope / terrain helpers
# ===========================================================================
class GlideSlopeDetector:
    def __init__(self, glide_slope_deg: float = STANDARD_GLIDE_SLOPE_DEG):
        self.glide_slope_deg = glide_slope_deg

    def ideal_altitude_ft(self, distance_nm: float, runway_elev_ft: float) -> float:
        if distance_nm <= 0:
            return runway_elev_ft
        return runway_elev_ft + math.tan(math.radians(self.glide_slope_deg)) * distance_nm * 6076.12

    def calculate_glide_slope_error(self, altitude_ft: float, distance_nm: float,
                                    runway_elev_ft: float) -> float:
        return altitude_ft - self.ideal_altitude_ft(distance_nm, runway_elev_ft)

    def is_on_profile(self, altitude_ft: float, distance_nm: float,
                      runway_elev_ft: float, tolerance_ft: float = 200.0) -> bool:
        return abs(self.calculate_glide_slope_error(
            altitude_ft, distance_nm, runway_elev_ft)) <= tolerance_ft

    def descent_rate_fpm(self, groundspeed_kt: float) -> float:
        return groundspeed_kt * 101.27 * math.tan(math.radians(self.glide_slope_deg))


class SurfaceSlopeAnalyzer:
    def analyze_profile(self, heights_m: List[float], sample_spacing_m: float) -> Dict:
        if len(heights_m) < 2 or sample_spacing_m <= 0:
            return dict(safe=True, max_slope=0.0, avg_slope=0.0, warning='')
        slopes = [
            abs(heights_m[i+1] - heights_m[i]) / sample_spacing_m * 100.0
            for i in range(len(heights_m) - 1)
        ]
        max_slope = max(slopes)
        avg_slope = sum(slopes) / len(slopes)
        warning = ''
        if max_slope >= 8.0:
            warning = f"CRITICAL {max_slope:.1f}%"
        elif max_slope >= 4.0:
            warning = f"CAUTION {max_slope:.1f}%"
        return dict(
            safe=max_slope < 4.0,
            max_slope=max_slope,
            avg_slope=avg_slope,
            warning=warning,
        )


# ===========================================================================
# VOR / Navaid Station Database
# Every entry MUST contain: name, ident, frequency, channel, type,
#   airport, latitude, longitude, ndb_freq (None ok), ndb_ident, remarks
# ===========================================================================
def _build_vor_stations() -> Dict:
    return {
        # ═══ CIVIL ══════════════════════════════════════════════════════
        'JNB_VOR': dict(
            name='OR Tambo VOR/DME', ident='JHB', frequency=114.90, channel=96,
            type='VOR/DME', airport='JNB', latitude=-25.5967, longitude=28.2394,
            ndb_freq=None, ndb_ident='',
            vor_available=True, ils_available=True,
            ils_runways=[dict(runway='03L', frequency=110.30, ident='IJB',
                              glide_slope_deg=STANDARD_GLIDE_SLOPE_DEG)],
            remarks='OR Tambo approach VOR. Primary Johannesburg area navaid.',
        ),
        'CPT_VOR': dict(
            name='Cape Town VORTAC', ident='CTV', frequency=115.70, channel=104,
            type='VORTAC', airport='CPT', latitude=-33.9648, longitude=18.6017,
            ndb_freq=None, ndb_ident='',
            vor_available=True, ils_available=True,
            ils_runways=[dict(runway='02', frequency=110.90, ident='ICT',
                              glide_slope_deg=STANDARD_GLIDE_SLOPE_DEG)],
            remarks='Cape Town approach VOR. Also serves AFB Ysterplaat.',
        ),
        'DUR_VOR': dict(
            name='Durban VOR/DME', ident='DNV', frequency=112.50, channel=72,
            type='VOR/DME', airport='DUR', latitude=-29.6144, longitude=31.1197,
            ndb_freq=393, ndb_ident='DU',
            vor_available=True, ils_available=True,
            ils_runways=[dict(runway='06', frequency=109.70, ident='IDN',
                              glide_slope_deg=STANDARD_GLIDE_SLOPE_DEG)],
            remarks='King Shaka / Durban area VOR. Also serves AFB Durban.',
        ),
        # ═══ SAAF AIR FORCE BASES ═══════════════════════════════════════
        'FAWK_VOR': dict(
            name='Waterkloof VORTAC', ident='WKV', frequency=116.90, channel=116,
            type='VORTAC', airport='FAWK', latitude=-25.8300, longitude=28.2225,
            ndb_freq=315, ndb_ident='WK',
            vor_available=True, ils_available=True,
            ils_runways=[dict(runway='01', frequency=111.50, ident='IWK',
                              glide_slope_deg=STANDARD_GLIDE_SLOPE_DEG)],
            remarks='Primary SAAF strategic base. Serves Waterkloof & Swartkop. ILS RWY 01. TWR 118.1.',
        ),
        'FALM_VOR': dict(
            name='Makhado VOR/DME', ident='LTV', frequency=115.00, channel=97,
            type='VOR/DME', airport='FALM', latitude=-23.1600, longitude=29.6967,
            ndb_freq=457, ndb_ident='MK',
            vor_available=True, ils_available=True,
            ils_runways=[
                dict(runway='10', frequency=110.10, ident='ILM',
                     glide_slope_deg=STANDARD_GLIDE_SLOPE_DEG),
                dict(runway='28', frequency=111.30, ident='ILM2',
                     glide_slope_deg=STANDARD_GLIDE_SLOPE_DEG),
            ],
            remarks='Fighter base. Gripen & Hawk. RWY 10/28 x 4020m. ILS both ends. TWR 118.3.',
        ),
        'FAHS_VOR': dict(
            name='Hoedspruit VOR/DME', ident='HSV', frequency=114.00, channel=87,
            type='VOR/DME', airport='FAHS', latitude=-24.3547, longitude=31.0503,
            ndb_freq=265, ndb_ident='HA',
            vor_available=True, ils_available=True,
            ils_runways=[dict(runway='09', frequency=109.50, ident='IHS',
                              glide_slope_deg=STANDARD_GLIDE_SLOPE_DEG)],
            remarks='Rooivalk attack helicopter base. RWY 09/27 x 3991m. ILS RWY 09. TWR 118.5.',
        ),
        'FALW_VOR': dict(
            name='Langebaanweg VORTAC', ident='LWV', frequency=117.00, channel=117,
            type='VORTAC', airport='FALW', latitude=-32.9689, longitude=18.1653,
            ndb_freq=345, ndb_ident='LW',
            vor_available=True, ils_available=False, ils_runways=[],
            remarks='Pilot training base. PC-7 Mk II. RWY 01/19 x 2430m. TWR 118.7.',
        ),
        'FAOB_VOR': dict(
            name='Overberg VOR/DME', ident='OBV', frequency=115.40, channel=101,
            type='VOR/DME', airport='FAOB', latitude=-34.5547, longitude=20.2506,
            ndb_freq=428, ndb_ident='OB',
            vor_available=True, ils_available=True,
            ils_runways=[dict(runway='35', frequency=110.50, ident='IOB',
                              glide_slope_deg=STANDARD_GLIDE_SLOPE_DEG)],
            remarks='Test & Eval (TFDC). UAV & weapons. RWY 17/35 x 3115m. ILS RWY 35. Restricted R105.',
        ),
        'FASK_VOR': dict(
            name='Swartkop NDB / WKV', ident='WKV', frequency=116.90, channel=116,
            type='VORTAC', airport='FASK', latitude=-25.8069, longitude=28.1644,
            ndb_freq=390, ndb_ident='SK',
            vor_available=True, ils_available=False, ils_runways=[],
            remarks='Historic SAAF Museum base. Uses Waterkloof VORTAC WKV. NDB SK 390 kHz. TWR 118.1.',
        ),
        'FABL_VOR': dict(
            name='Bloemfontein VOR/DME', ident='BLV', frequency=114.10, channel=88,
            type='VOR/DME', airport='FABL', latitude=-29.0939, longitude=26.3039,
            ndb_freq=380, ndb_ident='BL',
            vor_available=True, ils_available=True,
            ils_runways=[dict(runway='20', frequency=109.90, ident='IBL',
                              glide_slope_deg=STANDARD_GLIDE_SLOPE_DEG)],
            remarks='Shared civil/military. Co-located Bram Fischer Airport. ILS RWY 20. TWR 118.1.',
        ),
        'FAYP_VOR': dict(
            name='Ysterplaat NDB / CTV', ident='CTV', frequency=115.70, channel=104,
            type='VORTAC', airport='FAYP', latitude=-33.9011, longitude=18.4833,
            ndb_freq=284, ndb_ident='YP',
            vor_available=True, ils_available=False, ils_runways=[],
            remarks='Maritime patrol & SAR. Uses Cape Town VORTAC CTV. NDB YP 284 kHz. TWR 118.1.',
        ),
        'FADN_VOR': dict(
            name='AFB Durban VOR/DME', ident='DNV', frequency=112.50, channel=72,
            type='VOR/DME', airport='FADN', latitude=-29.9686, longitude=30.9478,
            ndb_freq=393, ndb_ident='DU',
            vor_available=True, ils_available=True,
            ils_runways=[dict(runway='06', frequency=109.70, ident='IDN',
                              glide_slope_deg=STANDARD_GLIDE_SLOPE_DEG)],
            remarks='Maritime patrol base KZN. Shares Durban VOR DNV. ILS RWY 06. TWR 118.1.',
        ),
        'FAPE_VOR': dict(
            name='Port Elizabeth AFS VOR', ident='PEV', frequency=113.40, channel=81,
            type='VOR/DME', airport='FAPE', latitude=-33.9850, longitude=25.6103,
            ndb_freq=330, ndb_ident='PE',
            vor_available=True, ils_available=True,
            ils_runways=[dict(runway='08', frequency=110.70, ident='IPE',
                              glide_slope_deg=STANDARD_GLIDE_SLOPE_DEG)],
            remarks='AFS Port Elizabeth. Helicopter & liaison. Shares PE Airport. TWR 118.1.',
        ),
    }


def _build_airports() -> Dict:
    return {
        'JNB':  dict(name='OR Tambo International',    latitude=-25.5967, longitude=28.2394,  elevation=1623),
        'CPT':  dict(name='Cape Town International',   latitude=-33.9648, longitude=18.6017,  elevation=47),
        'DUR':  dict(name='King Shaka International',  latitude=-29.6144, longitude=31.1197,  elevation=110),
        'FAWK': dict(name='AFB Waterkloof',            latitude=-25.8300, longitude=28.2225,  elevation=1506),
        'FALM': dict(name='AFB Makhado',               latitude=-23.1600, longitude=29.6967,  elevation=1524),
        'FAHS': dict(name='AFB Hoedspruit',            latitude=-24.3547, longitude=31.0503,  elevation=479),
        'FALW': dict(name='AFB Langebaanweg',          latitude=-32.9689, longitude=18.1653,  elevation=46),
        'FAOB': dict(name='AFB Overberg',              latitude=-34.5547, longitude=20.2506,  elevation=52),
        'FASK': dict(name='AFB Swartkop',              latitude=-25.8069, longitude=28.1644,  elevation=1519),
        'FABL': dict(name='AFB Bloemspruit',           latitude=-29.0939, longitude=26.3039,  elevation=1354),
        'FAYP': dict(name='AFB Ysterplaat',            latitude=-33.9011, longitude=18.4833,  elevation=15),
        'FADN': dict(name='AFB Durban',                latitude=-29.9686, longitude=30.9478,  elevation=89),
        'FAPE': dict(name='AFS Port Elizabeth',        latitude=-33.9850, longitude=25.6103,  elevation=58),
    }


SAAF_BASES = {
    'FAWK': dict(
        name='AFB Waterkloof', full_name='Air Force Base Waterkloof',
        city='Centurion, Pretoria', province='Gauteng',
        lat=-25.8300, lon=28.2225, elevation=1506, magnetic_var=-19,
        role='Strategic transport, VIP, SAAF HQ',
        vor_available=True, ils_available=True,
        vor_station=dict(ident='WKV', frequency=116.90, type='VORTAC'),
        ils_runways=[dict(runway='01', frequency=111.50, ident='IWK',
                          glide_slope_deg=STANDARD_GLIDE_SLOPE_DEG)],
        squadrons=['21 Sqn (C-130)', '28 Sqn (B737)', '44 Sqn (Oryx)', '60 Sqn (Agusta)'],
        runways=[
            dict(designation='01/19', true_hdg_lo=10,  true_hdg_hi=190,
                 length_m=3356, width_m=45, surface='Asphalt', ils='01',
                 thr_lo_lat=-25.8142, thr_lo_lon=28.2200,
                 thr_hi_lat=-25.8440, thr_hi_lon=28.2248, pcn='80/F/B/X/T'),
            dict(designation='06/24', true_hdg_lo=62,  true_hdg_hi=242,
                 length_m=1921, width_m=45, surface='Asphalt', ils='None',
                 thr_lo_lat=-25.8213, thr_lo_lon=28.2320,
                 thr_hi_lat=-25.8335, thr_hi_lon=28.2187, pcn='55/F/B/X/T'),
        ],
        taxiways=['Alpha','Bravo','Charlie','Delta'],
        aprons=['Main Military Apron','VIP Apron','Maintenance Apron'],
        navaids=['VORTAC WKV 116.9 MHz','NDB WK 315 kHz','ILS RWY 01'],
        frequencies=dict(TWR='118.1',APP='120.5',ATIS='126.2',GND='121.9'),
        remarks='Primary SAAF strategic base. All VIP/Head-of-State movements.',
    ),
    'FALM': dict(
        name='AFB Makhado', full_name='Air Force Base Makhado',
        city='Louis Trichardt', province='Limpopo',
        lat=-23.1600, lon=29.6967, elevation=1524, magnetic_var=-18,
        role='Fighter operations, advanced training',
        vor_available=True, ils_available=True,
        vor_station=dict(ident='LTV', frequency=115.00, type='VOR/DME'),
        ils_runways=[
            dict(runway='10', frequency=110.10, ident='ILM',
                 glide_slope_deg=STANDARD_GLIDE_SLOPE_DEG),
            dict(runway='28', frequency=111.30, ident='ILM2',
                 glide_slope_deg=STANDARD_GLIDE_SLOPE_DEG),
        ],
        squadrons=['2 Sqn (Gripen)', '85 CFS (Hawk)'],
        runways=[
            dict(designation='10/28', true_hdg_lo=100, true_hdg_hi=280,
                 length_m=4020, width_m=60, surface='Asphalt/Concrete', ils='10 & 28',
                 thr_lo_lat=-23.1560, thr_lo_lon=29.6680,
                 thr_hi_lat=-23.1640, thr_hi_lon=29.7280, pcn='95/R/B/W/T'),
        ],
        taxiways=['Alpha','Bravo','Charlie','Echo','Foxtrot'],
        aprons=['Fighter Apron North','Fighter Apron South','Training Apron'],
        navaids=['VOR/DME LTV 115.0 MHz','NDB MK 457 kHz','ILS RWY 10','ILS RWY 28'],
        frequencies=dict(TWR='118.3',APP='121.1',ATIS='127.4',GND='121.9'),
        remarks='Home of SAAF fighter force. 4,020m runway.',
    ),
    'FAHS': dict(
        name='AFB Hoedspruit', full_name='Air Force Base Hoedspruit',
        city='Hoedspruit', province='Limpopo',
        lat=-24.3547, lon=31.0503, elevation=479, magnetic_var=-18,
        role='Helicopter ops, Rooivalk attack helicopter base',
        vor_available=True, ils_available=True,
        vor_station=dict(ident='HSV', frequency=114.00, type='VOR/DME'),
        ils_runways=[dict(runway='09', frequency=109.50, ident='IHS',
                          glide_slope_deg=STANDARD_GLIDE_SLOPE_DEG)],
        squadrons=['16 Sqn (Oryx)', '17 Sqn (Rooivalk)', '19 Sqn (Super Lynx)'],
        runways=[
            dict(designation='09/27', true_hdg_lo=90,  true_hdg_hi=270,
                 length_m=3991, width_m=60, surface='Asphalt', ils='09',
                 thr_lo_lat=-24.3530, thr_lo_lon=31.0320,
                 thr_hi_lat=-24.3560, thr_hi_lon=31.0710, pcn='70/F/B/X/T'),
        ],
        taxiways=['Alpha','Bravo','Charlie'],
        aprons=['Helicopter Apron','Fixed-Wing Apron','Test Apron'],
        navaids=['VOR/DME HSV 114.0 MHz','NDB HA 265 kHz','ILS RWY 09'],
        frequencies=dict(TWR='118.5',APP='120.8',ATIS='128.3',GND='121.9'),
        remarks='Primary SAAF rotary-wing base. Rooivalk home.',
    ),
    'FALW': dict(
        name='AFB Langebaanweg', full_name='Air Force Base Langebaanweg',
        city='Langebaanweg', province='Western Cape',
        lat=-32.9689, lon=18.1653, elevation=46, magnetic_var=-24,
        role='Basic/advanced pilot training',
        vor_available=True, ils_available=False,
        vor_station=dict(ident='LWV', frequency=117.00, type='VORTAC'),
        ils_runways=[],
        squadrons=['41 Sqn (Pilatus PC-7)', 'Central Flying School'],
        runways=[
            dict(designation='01/19', true_hdg_lo=10,  true_hdg_hi=190,
                 length_m=2430, width_m=45, surface='Asphalt', ils='None',
                 thr_lo_lat=-32.9520, thr_lo_lon=18.1640,
                 thr_hi_lat=-33.0120, thr_hi_lon=18.1700, pcn='50/F/B/X/T'),
            dict(designation='07/25', true_hdg_lo=70,  true_hdg_hi=250,
                 length_m=1800, width_m=30, surface='Asphalt', ils='None',
                 thr_lo_lat=-32.9700, thr_lo_lon=18.1480,
                 thr_hi_lat=-32.9680, thr_hi_lon=18.1800, pcn='35/F/B/X/T'),
        ],
        taxiways=['Alpha','Bravo','Charlie'],
        aprons=['Training Apron East','Training Apron West'],
        navaids=['VORTAC LWV 117.0 MHz','NDB LW 345 kHz'],
        frequencies=dict(TWR='118.7',APP='121.3',ATIS='N/A',GND='121.9'),
        remarks='PC-7 Mk II pilot training.',
    ),
    'FAOB': dict(
        name='AFB Overberg', full_name='Air Force Base Overberg',
        city='Bredasdorp', province='Western Cape',
        lat=-34.5547, lon=20.2506, elevation=52, magnetic_var=-25,
        role='Test & evaluation, UAV, weapons testing',
        vor_available=True, ils_available=True,
        vor_station=dict(ident='OBV', frequency=115.40, type='VOR/DME'),
        ils_runways=[dict(runway='35', frequency=110.50, ident='IOB',
                          glide_slope_deg=STANDARD_GLIDE_SLOPE_DEG)],
        squadrons=['Test Flight & Development Centre (TFDC)'],
        runways=[
            dict(designation='17/35', true_hdg_lo=170, true_hdg_hi=350,
                 length_m=3115, width_m=45, surface='Asphalt', ils='35',
                 thr_lo_lat=-34.5700, thr_lo_lon=20.2490,
                 thr_hi_lat=-34.5420, thr_hi_lon=20.2530, pcn='65/F/B/X/T'),
            dict(designation='10/28', true_hdg_lo=100, true_hdg_hi=280,
                 length_m=2111, width_m=45, surface='Asphalt', ils='None',
                 thr_lo_lat=-34.5580, thr_lo_lon=20.2310,
                 thr_hi_lat=-34.5510, thr_hi_lon=20.2700, pcn='55/F/B/X/T'),
        ],
        taxiways=['Alpha','Bravo','Charlie','Delta'],
        aprons=['TFDC Apron','Test Range Apron','UAV Apron'],
        navaids=['VOR/DME OBV 115.4 MHz','NDB OB 428 kHz','ILS RWY 35'],
        frequencies=dict(TWR='118.9',APP='121.5',ATIS='N/A',GND='121.9'),
        remarks='Weapons & systems test facility. Restricted R105.',
    ),
    'FASK': dict(
        name='AFB Swartkop', full_name='Air Force Base Swartkop',
        city='Valhalla, Pretoria', province='Gauteng',
        lat=-25.8069, lon=28.1644, elevation=1519, magnetic_var=-19,
        role='Historic base, SAAF Museum, liaison',
        vor_available=True, ils_available=False,
        vor_station=dict(ident='WKV', frequency=116.90, type='VORTAC'),
        ils_runways=[],
        squadrons=['SAAF Museum', '41 Sqn det'],
        runways=[
            dict(designation='02/20', true_hdg_lo=26,  true_hdg_hi=206,
                 length_m=1982, width_m=45, surface='Asphalt', ils='None',
                 thr_lo_lat=-25.7950, thr_lo_lon=28.1610,
                 thr_hi_lat=-25.8180, thr_hi_lon=28.1680, pcn='40/F/B/X/T'),
            dict(designation='06/24', true_hdg_lo=69,  true_hdg_hi=249,
                 length_m=1419, width_m=30, surface='Asphalt', ils='None',
                 thr_lo_lat=-25.8100, thr_lo_lon=28.1530,
                 thr_hi_lat=-25.8040, thr_hi_lon=28.1750, pcn='30/F/B/X/T'),
        ],
        taxiways=['Alpha','Bravo'],
        aprons=['Museum Apron','Liaison Apron'],
        navaids=['NDB SK 390 kHz','Uses WKV 116.9'],
        frequencies=dict(TWR='118.1',APP='N/A',ATIS='N/A',GND='121.9'),
        remarks='Oldest operating military airfield in SA. SAAF Museum.',
    ),
    'FABL': dict(
        name='AFB Bloemspruit', full_name='Air Force Base Bloemspruit',
        city='Bloemfontein', province='Free State',
        lat=-29.0939, lon=26.3039, elevation=1354, magnetic_var=-21,
        role='Shared civil/military, transport support',
        vor_available=True, ils_available=True,
        vor_station=dict(ident='BLV', frequency=114.10, type='VOR/DME'),
        ils_runways=[dict(runway='20', frequency=109.90, ident='IBL',
                          glide_slope_deg=STANDARD_GLIDE_SLOPE_DEG)],
        squadrons=['28 Sqn det', 'ATC School'],
        runways=[
            dict(designation='02/20', true_hdg_lo=21,  true_hdg_hi=201,
                 length_m=2600, width_m=45, surface='Asphalt', ils='20',
                 thr_lo_lat=-29.0740, thr_lo_lon=26.3010,
                 thr_hi_lat=-29.1140, thr_hi_lon=26.3080, pcn='60/F/B/X/T'),
            dict(designation='12/30', true_hdg_lo=117, true_hdg_hi=297,
                 length_m=2187, width_m=45, surface='Asphalt', ils='None',
                 thr_lo_lat=-29.1010, thr_lo_lon=26.2810,
                 thr_hi_lat=-29.0870, thr_hi_lon=26.3260, pcn='55/F/B/X/T'),
        ],
        taxiways=['Alpha','Bravo','Charlie'],
        aprons=['Military Apron','Civil Apron'],
        navaids=['VOR/DME BLV 114.1 MHz','NDB BL 380 kHz','ILS RWY 20'],
        frequencies=dict(TWR='118.1',APP='119.5',ATIS='126.4',GND='121.9'),
        remarks='Co-located with Bram Fischer Intl (FABM).',
    ),
    'FAYP': dict(
        name='AFB Ysterplaat', full_name='Air Force Base Ysterplaat',
        city='Cape Town', province='Western Cape',
        lat=-33.9011, lon=18.4833, elevation=15, magnetic_var=-25,
        role='Maritime patrol, SAR, helicopter',
        vor_available=True, ils_available=False,
        vor_station=dict(ident='CTV', frequency=115.70, type='VORTAC'),
        ils_runways=[],
        squadrons=['35 Sqn (C-47)', '22 Sqn (Super Lynx)', 'Maritime Command'],
        runways=[
            dict(designation='02/20', true_hdg_lo=19,  true_hdg_hi=199,
                 length_m=1585, width_m=30, surface='Asphalt', ils='None',
                 thr_lo_lat=-33.8920, thr_lo_lon=18.4820,
                 thr_hi_lat=-33.9100, thr_hi_lon=18.4850, pcn='45/F/B/X/T'),
            dict(designation='14/32', true_hdg_lo=140, true_hdg_hi=320,
                 length_m=1295, width_m=30, surface='Asphalt', ils='None',
                 thr_lo_lat=-33.8960, thr_lo_lon=18.4760,
                 thr_hi_lat=-33.9060, thr_hi_lon=18.4910, pcn='40/F/B/X/T'),
        ],
        taxiways=['Alpha','Bravo'],
        aprons=['Maritime Patrol Apron','Helicopter Apron'],
        navaids=['NDB YP 284 kHz','Uses CTV 115.7'],
        frequencies=dict(TWR='118.1',APP='N/A',ATIS='N/A',GND='121.9'),
        remarks='Primary SAAF maritime base. Adjacent Cape Town metro.',
    ),
    'FADN': dict(
        name='AFB Durban', full_name='Air Force Base Durban',
        city='Durban', province='KwaZulu-Natal',
        lat=-29.9686, lon=30.9478, elevation=89, magnetic_var=-23,
        role='Maritime patrol, transport support, SAR',
        vor_available=True, ils_available=True,
        vor_station=dict(ident='DNV', frequency=112.50, type='VOR/DME'),
        ils_runways=[dict(runway='06', frequency=109.70, ident='IDN',
                          glide_slope_deg=STANDARD_GLIDE_SLOPE_DEG)],
        squadrons=['35 Sqn det', '15 Sqn (Oryx)'],
        runways=[
            dict(designation='06/24', true_hdg_lo=60,  true_hdg_hi=240,
                 length_m=2872, width_m=45, surface='Asphalt', ils='06',
                 thr_lo_lat=-29.9760, thr_lo_lon=30.9330,
                 thr_hi_lat=-29.9600, thr_hi_lon=30.9630, pcn='60/F/B/X/T'),
            dict(designation='18/36', true_hdg_lo=180, true_hdg_hi=360,
                 length_m=1524, width_m=30, surface='Asphalt', ils='None',
                 thr_lo_lat=-29.9600, thr_lo_lon=30.9480,
                 thr_hi_lat=-29.9790, thr_hi_lon=30.9480, pcn='40/F/B/X/T'),
        ],
        taxiways=['Alpha','Bravo','Charlie'],
        aprons=['SAAF Apron','Maintenance Apron'],
        navaids=['VOR/DME DNV 112.5 MHz','NDB DU 393 kHz','ILS RWY 06'],
        frequencies=dict(TWR='118.1',APP='120.2',ATIS='127.6',GND='121.9'),
        remarks='Shares Virginia Airport. KZN maritime patrol.',
    ),
    'FAPE': dict(
        name='AFS Port Elizabeth', full_name='Air Force Station Port Elizabeth',
        city='Gqeberha (Port Elizabeth)', province='Eastern Cape',
        lat=-33.9850, lon=25.6103, elevation=58, magnetic_var=-25,
        role='Helicopter & liaison operations',
        vor_available=True, ils_available=True,
        vor_station=dict(ident='PEV', frequency=113.40, type='VOR/DME'),
        ils_runways=[dict(runway='08', frequency=110.70, ident='IPE',
                          glide_slope_deg=STANDARD_GLIDE_SLOPE_DEG)],
        squadrons=['Test Sqn det', 'Liaison Flt'],
        runways=[
            dict(designation='08/26', true_hdg_lo=80,  true_hdg_hi=260,
                 length_m=2690, width_m=45, surface='Asphalt', ils='08',
                 thr_lo_lat=-33.9848, thr_lo_lon=25.5920,
                 thr_hi_lat=-33.9852, thr_hi_lon=25.6310, pcn='55/F/B/X/T'),
        ],
        taxiways=['Alpha','Bravo'],
        aprons=['SAAF Apron'],
        navaids=['VOR/DME PEV 113.4 MHz','NDB PE 330 kHz'],
        frequencies=dict(TWR='118.1',APP='119.7',ATIS='N/A',GND='121.9'),
        remarks='AFS co-located with PE Airport (FAPE). Helicopter & liaison.',
    ),
}


def _merge_defaults(default, loaded):
    if isinstance(default, dict):
        loaded = loaded if isinstance(loaded, dict) else {}
        merged = {k: _merge_defaults(v, loaded.get(k)) for k, v in default.items()}
        for k, v in loaded.items():
            if k not in merged:
                merged[k] = v
        return merged
    if isinstance(default, list):
        return loaded if isinstance(loaded, list) else list(default)
    return default if loaded is None else loaded


def _format_ils_runways(ils_runways: List[Dict]) -> str:
    if not ils_runways:
        return "No ILS"
    return "; ".join(
        f"RWY {rw.get('runway','--')} {rw.get('frequency',0.0):.2f} MHz GS "
        f"{rw.get('glide_slope_deg', STANDARD_GLIDE_SLOPE_DEG):.1f}°"
        for rw in ils_runways
    )


def _build_base_navaids(base: Dict) -> List[str]:
    lines = []
    vor = base.get('vor_station', {})
    if base.get('vor_available') and vor:
        lines.append(
            f"{vor.get('type', 'VOR')} {vor.get('ident', '---')} "
            f"{vor.get('frequency', 0.0):.2f} MHz"
        )
    lines.extend(
        line for line in base.get('navaids', [])
        if 'NDB' in line.upper() and line not in lines
    )
    ils = base.get('ils_runways', [])
    if ils:
        lines.extend(
            f"ILS RWY {rw.get('runway', '--')}  {rw.get('frequency', 0.0):.2f} MHz  "
            f"GS {rw.get('glide_slope_deg', STANDARD_GLIDE_SLOPE_DEG):.1f}°"
            for rw in ils
        )
    elif base.get('ils_available') is False:
        lines.append("ILS not available")
    return lines or list(base.get('navaids', ['—']))


def _build_base_remarks(base: Dict) -> str:
    remarks = base.get('remarks', '')
    vor = base.get('vor_station', {})
    vor_txt = ("No operational VOR coverage" if not base.get('vor_available')
               else f"VOR {vor.get('ident', '---')} {vor.get('frequency', 0.0):.2f} MHz")
    return f"{remarks}  {vor_txt}.  {_format_ils_runways(base.get('ils_runways', []))}."


def _build_station_remarks(st: Dict) -> str:
    remarks = st.get('remarks', '')
    return f"{remarks}  {_format_ils_runways(st.get('ils_runways', []))}."


# ===========================================================================
# Configuration
# ===========================================================================
class VORAirportConfig:
    def __init__(self, config_file: str = 'vor_config.yaml'):
        self.config_file  = config_file
        self.airports: Dict    = {}
        self.vor_stations: Dict = {}
        self.load_config()

    def load_config(self):
        try:
            default_airports = _build_airports()
            default_stations = _build_vor_stations()
            if Path(self.config_file).exists():
                with open(self.config_file) as f:
                    cfg = yaml.safe_load(f) or {}
                loaded_airports = cfg.get('airports', {})
                loaded = cfg.get('vor_stations', {})
                # Rebuild defaults if any entry is missing the 'name' key
                if (loaded and all('name' in v for v in loaded.values()) and
                        (not loaded_airports or all('name' in v for v in loaded_airports.values()))):
                    self.airports = _merge_defaults(default_airports, loaded_airports)
                    self.vor_stations = _merge_defaults(default_stations, loaded)
                    logger.info(f"Config loaded from {self.config_file}")
                    if self.airports != loaded_airports or self.vor_stations != loaded:
                        logger.info("Config merged with latest VOR/ILS defaults")
                        self.save_config()
                    return
            self._defaults()
        except Exception as e:
            logger.error(f"Config load: {e}")
            self._defaults()

    def _defaults(self):
        self.airports     = _build_airports()
        self.vor_stations = _build_vor_stations()
        self.save_config()

    def save_config(self):
        try:
            with open(self.config_file, 'w') as f:
                yaml.dump({'airports': self.airports,
                           'vor_stations': self.vor_stations}, f)
        except Exception as e:
            logger.error(f"Config save: {e}")


# ===========================================================================
# ASRACS Ground Target / Alert
# ===========================================================================
class GroundTarget:
    TYPE_AIRCRAFT = 'ACFT'; TYPE_VEHICLE = 'VEH'
    STATUS_OK = 'OK'; STATUS_CAUTION = 'CAUTION'
    STATUS_WARNING = 'WARNING'; STATUS_CRITICAL = 'CRITICAL'

    def __init__(self, tid, ttype='ACFT'):
        self.target_id = tid; self.target_type = ttype; self.callsign = tid
        self.x = self.y = self.heading = self.speed = 0.0
        self.timestamp = datetime.now(); self.status = self.STATUS_OK
        self.cleared_rwy = ''; self.taxi_route = []; self.track_hist = []
        self.squawk = '2000'; self.gate = ''


class IncursionAlert:
    SEV_CAUTION = 'CAUTION'; SEV_WARNING = 'WARNING'; SEV_CRITICAL = 'CRITICAL'

    def __init__(self, severity, target_id, location, message):
        self.severity = severity; self.target_id = target_id
        self.location = location; self.message = message
        self.timestamp = datetime.now(); self.acknowledged = False


# ===========================================================================
# JNB Airport Layout
# ===========================================================================
class JNBAirportLayout:
    RUNWAYS = [
        dict(name='03L/21R', p1=(-800,-2800),  p2=(800,2800),   w=60),
        dict(name='03R/21L', p1=(-1800,-2800), p2=(-200,2800),  w=60),
    ]
    TAXIWAYS = [
        dict(name='A', pts=[(-1200,-2800),(-1200,2800)]),
        dict(name='B', pts=[(1200,-2800),(1200,2800)]),
        dict(name='C', pts=[(-1200,0),(0,0),(1200,0)]),
        dict(name='D', pts=[(-1200,-1200),(1200,-1200)]),
        dict(name='E', pts=[(-1200,1200),(1200,1200)]),
        dict(name='H', pts=[(-800,-600),(800,-600)]),
        dict(name='J', pts=[(-800,600),(800,600)]),
    ]
    APRONS = [
        dict(name='Terminal A', cx=2200,  cy=400,  w=1200, h=600),
        dict(name='Terminal B', cx=2200,  cy=-800, w=1200, h=600),
        dict(name='Cargo',      cx=-2800, cy=0,    w=1000, h=800),
    ]
    GATES = [
        ('A01',2800,600),('A02',2800,400),('A03',2800,200),('A04',2800,0),
        ('A05',2800,-200),('A06',2800,-400),('B01',2900,-600),('B02',2900,-800),
        ('B03',2900,-1000),('B04',2900,-1200),
    ]
    HOLDING_POINTS = [
        ('A1',-1200,-2600),('A2',-1200,2600),('B1',1200,-2600),('B2',1200,2600),
    ]
    HOTSPOTS = [
        dict(name='HS1',x=-900, y=-2600,r=150),
        dict(name='HS2',x=300,  y=2600, r=150),
        dict(name='HS3',x=-1200,y=0,    r=120),
        dict(name='HS4',x=1200, y=0,    r=120),
    ]
    RUNWAY_ZONES = [
        dict(rwy='03L/21R', x1=-880,  y1=-2900, x2=880,  y2=2900),
        dict(rwy='03R/21L', x1=-1880, y1=-2900, x2=-120, y2=2900),
    ]


# ===========================================================================
# ASRACS Simulation Engine
# ===========================================================================
class ASRACSSimEngine:
    _AIRCRAFT = [
        dict(id='SAA101',sq='4321',gate='A01',phase='pushback',hdg=270),
        dict(id='BAW202',sq='5432',gate='A03',phase='taxiout', hdg=90),
        dict(id='DAL303',sq='6543',gate='B02',phase='landing', hdg=210),
        dict(id='MSR404',sq='7654',gate='A05',phase='taxiin',  hdg=180),
        dict(id='ETH505',sq='2000',gate='B04',phase='holding', hdg=30),
    ]
    _VEHICLES = [
        dict(id='FOL01',type='FUEL'), dict(id='BUS02',type='BUS'),
        dict(id='TUG03',type='TUG'),  dict(id='AMB04',type='AMBL'),
        dict(id='CAT05',type='CAT'),
    ]
    _R_PUSH = [(2800,400),(2000,400),(1200,400),(1200,0),(-800,0),(-1200,0),(-1200,-2600)]
    _R_TOUT = [(2800,200),(1200,200),(1200,0),(0,0),(-800,0),(-1200,0),(-1200,-2600)]
    _R_VAC  = [(-800,-2800),(-800,-1200),(-800,0),(0,0),(1200,0),(1200,-600),(2800,-600)]
    _R_TIN  = [(300,2800),(300,1200),(1200,1200),(1200,0),(2800,0),(2800,-400)]
    _VL = [
        [(2200,400),(2200,-800),(1200,-800),(1200,400),(2200,400)],
        [(-3200,300),(-3200,-300),(-2400,-300),(-2400,300),(-3200,300)],
        [(0,0),(800,0),(800,600),(0,600),(0,0)],
    ]

    def __init__(self):
        self.targets: Dict[str, GroundTarget] = {}
        self.alerts:  List[IncursionAlert]    = []
        self._reset()

    def _reset(self):
        self.targets = {}; self.alerts = []
        for f in self._AIRCRAFT:
            t = GroundTarget(f['id'], GroundTarget.TYPE_AIRCRAFT)
            t.callsign = f['id']; t.squawk = f['sq']; t.heading = f['hdg']
            gate = next((g for g in JNBAirportLayout.GATES if g[0] == f['gate']), None)
            if gate:
                t.x = gate[1] + random.uniform(-50, 50)
                t.y = gate[2] + random.uniform(-50, 50)
            t.gate = f['gate']
            rt = {
                'pushback': list(self._R_PUSH), 'taxiout': list(self._R_TOUT),
                'landing':  list(self._R_VAC),  'taxiin':  list(self._R_TIN),
                'holding':  [(-1200,-2600)],
            }
            t.taxi_route = rt.get(f['phase'], [])
            t.speed = random.uniform(8, 18)
            self.targets[f['id']] = t
        for i, f in enumerate(self._VEHICLES):
            t = GroundTarget(f['id'], GroundTarget.TYPE_VEHICLE)
            t.callsign = f['type']
            loop = self._VL[i % len(self._VL)]
            t.x, t.y = loop[0]; t.taxi_route = loop * 3
            t.speed = random.uniform(5, 12)
            self.targets[f['id']] = t

    def tick(self):
        for tid, t in self.targets.items():
            if not t.taxi_route:
                continue
            wx, wy = t.taxi_route[0]
            dx, dy = wx - t.x, wy - t.y
            dist = math.sqrt(dx*dx + dy*dy)
            step = t.speed * 0.514444
            if dist < step + 1:
                t.x, t.y = wx, wy; t.taxi_route.pop(0)
                if not t.taxi_route:
                    if t.target_type == GroundTarget.TYPE_VEHICLE:
                        loop = random.choice(self._VL); t.taxi_route = loop * 3
                    else:
                        t.taxi_route = random.choice([
                            list(self._R_PUSH), list(self._R_TOUT),
                            list(self._R_VAC),  list(self._R_TIN)
                        ])
            else:
                t.heading = (math.degrees(math.atan2(dx, dy)) + 360) % 360
                t.x += (dx/dist) * step; t.y += (dy/dist) * step
            t.timestamp = datetime.now()
            t.track_hist.append((t.x, t.y))
            if len(t.track_hist) > 40:
                t.track_hist.pop(0)
            self._check(t)
        return self.targets

    def _check(self, t):
        for z in JNBAirportLayout.RUNWAY_ZONES:
            if z['x1'] < t.x < z['x2'] and z['y1'] < t.y < z['y2']:
                if t.target_type == GroundTarget.TYPE_VEHICLE or not t.cleared_rwy:
                    sev = (IncursionAlert.SEV_CRITICAL
                           if t.target_type == GroundTarget.TYPE_VEHICLE
                           else IncursionAlert.SEV_WARNING)
                    recent = [a for a in self.alerts
                              if a.target_id == t.target_id
                              and (datetime.now() - a.timestamp).seconds < 10]
                    if not recent:
                        a = IncursionAlert(sev, t.target_id, z['rwy'],
                                           f"{t.callsign} on {z['rwy']} without clearance")
                        self.alerts.append(a); t.status = sev
                return
        if t.status in (IncursionAlert.SEV_WARNING, IncursionAlert.SEV_CRITICAL):
            t.status = GroundTarget.STATUS_OK

    def reset(self): self._reset()


# ===========================================================================
# ASRACS Surface Display
# ===========================================================================
class ASRACSDisplay(QWidget):
    COL_BG       = QColor(20,25,20);  COL_RUNWAY  = QColor(60,60,65)
    COL_TAXIWAY  = QColor(45,45,50);  COL_APRON   = QColor(35,40,35)
    COL_HOTSPOT  = QColor(255,180,0,60); COL_RWY_ZONE = QColor(255,50,50,25)
    COL_GATE     = QColor(100,200,100);  COL_HOLD   = QColor(255,200,0)
    COL_AIRCRAFT = QColor(0,255,120);    COL_VEHICLE= QColor(0,180,255)
    COL_CAUTION  = QColor(255,200,0);    COL_WARNING= QColor(255,140,0)
    COL_CRITICAL = QColor(255,40,40);    COL_ROUTE  = QColor(0,200,255,130)
    SCALE = 0.055

    def __init__(self):
        super().__init__()
        self.setMinimumSize(600,560); self.setStyleSheet("background:#14191a;")
        self.targets = {}; self._scale = self.SCALE
        self._offset = QPointF(0,0); self._drag = None
        self._zoom = 1.0; self.setMouseTracking(True)

    def set_targets(self, t): self.targets = t; self.update()
    def _sp(self,x,y):
        cx = self.width()/2 + self._offset.x()
        cy = self.height()/2 + self._offset.y()
        s  = self._scale * self._zoom
        return QPointF(cx + x*s, cy - y*s)
    def _m(self,m): return m * self._scale * self._zoom

    def paintEvent(self, event):
        p = QPainter(self); p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), QColor(22,38,22))
        self._zones(p); self._aprons(p); self._runways(p)
        self._taxiways(p); self._hotspots(p); self._gates(p)
        self._holds(p); self._targets(p); self._overlay(p)
        p.end()

    def _zones(self,p):
        for z in JNBAirportLayout.RUNWAY_ZONES:
            p1=self._sp(z['x1'],z['y2']); p2=self._sp(z['x2'],z['y1'])
            p.setPen(QPen(QColor(255,50,50,80),1,Qt.DashLine))
            p.setBrush(QBrush(self.COL_RWY_ZONE)); p.drawRect(QRectF(p1,p2))

    def _aprons(self,p):
        for ap in JNBAirportLayout.APRONS:
            sp=self._sp(ap['cx']-ap['w']/2,ap['cy']+ap['h']/2)
            ep=self._sp(ap['cx']+ap['w']/2,ap['cy']-ap['h']/2)
            p.setPen(QPen(QColor(80,90,80),1)); p.setBrush(QBrush(self.COL_APRON))
            p.drawRect(QRectF(sp,ep))
            ctr=self._sp(ap['cx'],ap['cy'])
            p.setPen(QPen(QColor(150,180,150),1))
            p.setFont(QFont("Consolas",max(7,int(self._zoom*7))))
            p.drawText(ctr+QPointF(-30,5),ap['name'])

    def _runways(self,p):
        for rwy in JNBAirportLayout.RUNWAYS:
            sp=self._sp(*rwy['p1']); ep=self._sp(*rwy['p2'])
            w=max(3,self._m(rwy['w']))
            p.setPen(QPen(self.COL_RUNWAY,w,Qt.SolidLine,Qt.FlatCap)); p.drawLine(sp,ep)
            p.setPen(QPen(QColor(220,220,180,160),max(1,w*0.04),Qt.DashLine,Qt.FlatCap))
            p.drawLine(sp,ep)
            mid=QPointF((sp.x()+ep.x())/2,(sp.y()+ep.y())/2)
            p.setPen(QPen(QColor(240,240,180),1))
            p.setFont(QFont("Consolas",max(8,int(self._zoom*9)),QFont.Bold))
            p.drawText(mid+QPointF(5,-5),rwy['name'])

    def _taxiways(self,p):
        p.setPen(QPen(QColor(90,90,100),max(2,self._m(18)),
                      Qt.SolidLine,Qt.RoundCap,Qt.RoundJoin))
        for twy in JNBAirportLayout.TAXIWAYS:
            pts=twy['pts']
            for i in range(len(pts)-1):
                p.drawLine(self._sp(*pts[i]),self._sp(*pts[i+1]))
        p.setFont(QFont("Consolas",max(7,int(self._zoom*7))))
        p.setPen(QPen(QColor(180,180,220),1))
        for twy in JNBAirportLayout.TAXIWAYS:
            mp=self._sp(*twy['pts'][len(twy['pts'])//2])
            p.drawText(mp+QPointF(4,-4),twy['name'])

    def _hotspots(self,p):
        for hs in JNBAirportLayout.HOTSPOTS:
            ctr=self._sp(hs['x'],hs['y']); r=self._m(hs['r'])
            p.setPen(QPen(QColor(255,180,0,180),2,Qt.DashLine))
            p.setBrush(QBrush(self.COL_HOTSPOT)); p.drawEllipse(ctr,r,r)
            p.setPen(QPen(QColor(255,200,0),1))
            p.setFont(QFont("Consolas",max(7,int(self._zoom*7)),QFont.Bold))
            p.drawText(ctr+QPointF(r+2,4),f"HS:{hs['name']}")

    def _gates(self,p):
        p.setPen(QPen(self.COL_GATE,1))
        p.setFont(QFont("Consolas",max(6,int(self._zoom*6))))
        for name,gx,gy in JNBAirportLayout.GATES:
            sc=self._sp(gx,gy); r=max(3,self._m(15))
            p.setBrush(QBrush(QColor(60,120,60)))
            p.drawRect(QRectF(sc.x()-r,sc.y()-r,r*2,r*2))
            p.setPen(QPen(QColor(180,255,180),1))
            p.drawText(sc+QPointF(r+2,4),name)
            p.setPen(QPen(self.COL_GATE,1))

    def _holds(self,p):
        p.setFont(QFont("Consolas",max(7,int(self._zoom*7)),QFont.Bold))
        for name,hx,hy in JNBAirportLayout.HOLDING_POINTS:
            sc=self._sp(hx,hy); r=max(4,self._m(20))
            p.setPen(QPen(self.COL_HOLD,2)); p.setBrush(Qt.NoBrush)
            p.drawRect(QRectF(sc.x()-r,sc.y()-r*0.4,r*2,r*0.8))
            p.setPen(QPen(QColor(255,230,0),1))
            p.drawText(sc+QPointF(r+3,4),name)

    def _targets(self,p):
        for t in self.targets.values():
            self._draw_t(p,t)

    def _draw_t(self,p,t):
        sc=self._sp(t.x,t.y)
        col = {
            GroundTarget.STATUS_OK: (
                self.COL_AIRCRAFT if t.target_type==GroundTarget.TYPE_AIRCRAFT
                else self.COL_VEHICLE),
            IncursionAlert.SEV_CAUTION:  self.COL_CAUTION,
            IncursionAlert.SEV_WARNING:  self.COL_WARNING,
            IncursionAlert.SEV_CRITICAL: self.COL_CRITICAL,
        }.get(t.status, self.COL_AIRCRAFT)
        if len(t.taxi_route)>1:
            p.setPen(QPen(self.COL_ROUTE,1,Qt.DotLine)); prev=sc
            for wx,wy in t.taxi_route[:8]:
                nxt=self._sp(wx,wy); p.drawLine(prev,nxt); prev=nxt
        if len(t.track_hist)>1:
            for i in range(1,len(t.track_hist)):
                alpha=int(120*i/len(t.track_hist))
                tc=QColor(col); tc.setAlpha(alpha); p.setPen(QPen(tc,1))
                pa=self._sp(*t.track_hist[i-1]); pb=self._sp(*t.track_hist[i])
                p.drawLine(pa,pb)
        sym_r=max(5,self._m(30)); hdg_r=math.radians(t.heading)
        p.setPen(QPen(col,2)); p.setBrush(QBrush(col))
        if t.target_type==GroundTarget.TYPE_AIRCRAFT:
            tri=QPolygonF([
                sc+QPointF( sym_r*math.sin(hdg_r),        -sym_r*math.cos(hdg_r)),
                sc+QPointF( sym_r*0.5*math.sin(hdg_r+2.4),-sym_r*0.5*math.cos(hdg_r+2.4)),
                sc+QPointF( sym_r*0.5*math.sin(hdg_r-2.4),-sym_r*0.5*math.cos(hdg_r-2.4)),
            ]); p.drawPolygon(tri)
        else:
            r=sym_r*0.8
            dia=QPolygonF([sc+QPointF(0,-r),sc+QPointF(r,0),
                           sc+QPointF(0,r), sc+QPointF(-r,0)])
            p.drawPolygon(dia)
        vlen=sym_r*2.5; p.setPen(QPen(col,1))
        p.drawLine(sc,sc+QPointF(vlen*math.sin(hdg_r),-vlen*math.cos(hdg_r)))
        if t.status!=GroundTarget.STATUS_OK:
            p.setPen(QPen(col,2,Qt.DashLine)); p.setBrush(Qt.NoBrush)
            p.drawEllipse(sc,sym_r*1.8,sym_r*1.8)
        bx=sc.x()+sym_r+4; by=sc.y()-30
        if bx+110>self.width(): bx=sc.x()-120
        if by<0: by=sc.y()+10
        p.setPen(Qt.NoPen); p.setBrush(QBrush(QColor(10,25,15,210)))
        p.drawRoundedRect(QRectF(bx-2,by-2,114,58),3,3)
        p.setPen(QPen(col,1))
        p.setFont(QFont("Consolas",max(7,int(self._zoom*8)),QFont.Bold))
        p.drawText(QPointF(bx,by+10),t.callsign)
        p.setFont(QFont("Consolas",max(6,int(self._zoom*7))))
        p.setPen(QPen(QColor(180,255,200),1))
        for li,ln in enumerate([
            f"HDG {t.heading:05.1f}\u00b0  {t.speed:.0f}kt",
            f"SQ:{t.squawk}  {t.target_type}",
            f"{t.status}",
        ]):
            col2 = (self.COL_CRITICAL
                    if (li==2 and t.status!=GroundTarget.STATUS_OK)
                    else QColor(180,255,200))
            p.setPen(QPen(col2,1))
            p.drawText(QPointF(bx,by+22+li*12),ln)

    def _overlay(self,p):
        p.setPen(QPen(QColor(180,220,180),1))
        p.setFont(QFont("Consolas",10,QFont.Bold))
        p.drawText(QPointF(8,18),"ASRACS \u2014 OR Tambo International (FAOR)")
        bar=self._m(200); bx,by=12,self.height()-18
        p.setPen(QPen(QColor(200,200,200),2))
        p.drawLine(QPointF(bx,by),QPointF(bx+bar,by))
        p.setFont(QFont("Consolas",8)); p.drawText(QPointF(bx,by-3),"200 m")
        nx,ny=self.width()-28,38
        p.setPen(QPen(QColor(255,100,100),2))
        p.drawLine(QPointF(nx,ny+14),QPointF(nx,ny-14))
        p.drawText(QPointF(nx-4,ny-16),"N")

    def mousePressEvent(self,e):
        if e.button()==Qt.LeftButton: self._drag=e.pos()
    def mouseMoveEvent(self,e):
        if self._drag and e.buttons()&Qt.LeftButton:
            d=e.pos()-self._drag
            self._offset+=QPointF(d.x(),d.y()); self._drag=e.pos(); self.update()
    def mouseReleaseEvent(self,e): self._drag=None
    def wheelEvent(self,e):
        self._zoom=max(0.3,min(8.0,self._zoom*(1.12 if e.angleDelta().y()>0 else 0.89)))
        self.update()
    def reset_view(self): self._zoom=1.0; self._offset=QPointF(0,0); self.update()


class ASRACSAlertPanel(QWidget):
    _SC = {
        IncursionAlert.SEV_CAUTION:  QColor(255,210,0),
        IncursionAlert.SEV_WARNING:  QColor(255,140,0),
        IncursionAlert.SEV_CRITICAL: QColor(255,50,50),
    }
    def __init__(self):
        super().__init__(); lo=QVBoxLayout(self); lo.setContentsMargins(0,0,0,0)
        self.table=QTableWidget(); self.table.setColumnCount(5)
        self.table.setHorizontalHeaderLabels(["Time","Severity","Target","Location","Message"])
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.table.setAlternatingRowColors(True)
        self.table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.table.setMaximumHeight(180); lo.addWidget(self.table)
        clr=QPushButton("Clear Alerts"); clr.clicked.connect(self.clear); lo.addWidget(clr)
    def update_alerts(self,alerts):
        self.table.setRowCount(0)
        for a in reversed(alerts[-50:]):
            r=self.table.rowCount(); self.table.insertRow(r)
            col=self._SC.get(a.severity,QColor(200,200,200))
            for c,v in enumerate([a.timestamp.strftime('%H:%M:%S'),a.severity,
                                   a.target_id,a.location,a.message]):
                item=QTableWidgetItem(v); item.setForeground(col)
                self.table.setItem(r,c,item)
    def clear(self): self.table.setRowCount(0)


# ===========================================================================
# SAAF Overview Map
# ===========================================================================
class SAAFOverviewMap(QWidget):
    base_selected = pyqtSignal(str)
    _SA = [
        (-34.8,18.5),(-34.4,20.0),(-34.0,22.0),(-33.8,25.0),(-33.9,26.9),
        (-33.5,27.5),(-32.8,28.1),(-31.5,29.5),(-30.7,30.5),(-29.9,31.0),
        (-28.3,32.3),(-27.0,33.0),(-26.4,32.9),(-25.7,33.0),(-24.0,32.0),
        (-22.3,31.0),(-22.0,29.5),(-22.2,28.0),(-22.9,27.2),(-23.0,26.0),
        (-23.6,25.5),(-24.0,24.0),(-24.5,22.5),(-25.5,21.0),(-26.5,19.5),
        (-28.0,18.0),(-28.5,17.0),(-29.4,17.0),(-30.4,17.0),(-31.5,17.5),
        (-32.5,18.0),(-33.5,18.0),(-34.8,18.5),
    ]
    _LLAT,_HLAT,_LLON,_HLON = -35.5,-21.5,16.0,33.5
    _COL = {
        'FAWK':QColor(0,180,255),'FALM':QColor(255,80,80),'FAHS':QColor(0,255,120),
        'FALW':QColor(255,200,0),'FAOB':QColor(200,80,255),'FASK':QColor(180,180,180),
        'FABL':QColor(0,180,255),'FAYP':QColor(80,220,255),'FADN':QColor(80,220,255),
        'FAPE':QColor(255,160,80),
    }
    def __init__(self):
        super().__init__(); self.setMinimumSize(360,300)
        self.setStyleSheet("background:#0a140a;"); self._sel=None
        self.setCursor(Qt.PointingHandCursor)
    def _p(self,lat,lon):
        w,h=self.width(),self.height(); mg=30
        x=mg+(lon-self._LLON)/(self._HLON-self._LLON)*(w-2*mg)
        y=h-(mg+(lat-self._LLAT)/(self._HLAT-self._LLAT)*(h-2*mg))
        return QPointF(x,y)
    def paintEvent(self,event):
        p=QPainter(self); p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(),QColor(10,30,50))
        outline=[self._p(la,lo) for la,lo in self._SA]
        poly=QPolygonF(outline)
        p.setPen(QPen(QColor(60,100,60),1)); p.setBrush(QBrush(QColor(25,55,25)))
        p.drawPolygon(poly)
        p.setPen(QPen(QColor(40,60,40),1,Qt.DotLine))
        for lat in range(-34,-21,2):
            a=self._p(lat,self._LLON); b=self._p(lat,self._HLON); p.drawLine(a,b)
        for lon in range(16,34,2):
            a=self._p(self._LLAT,lon); b=self._p(self._HLAT,lon); p.drawLine(a,b)
        for icao,base in SAAF_BASES.items():
            pt=self._p(base['lat'],base['lon'])
            col=self._COL.get(icao,QColor(200,200,200))
            is_sel=(icao==self._sel); r=10 if is_sel else 7
            p.setPen(QPen(QColor(255,255,255) if is_sel else col.darker(150),
                          2 if is_sel else 1))
            p.setBrush(QBrush(col)); p.drawEllipse(pt,r,r)
            p.setPen(QPen(QColor(220,255,220) if is_sel else QColor(180,220,180),1))
            p.setFont(QFont("Consolas",8 if not is_sel else 9,
                            QFont.Bold if is_sel else QFont.Normal))
            p.drawText(pt+QPointF(r+3,4),icao)
            p.setFont(QFont("Consolas",7))
            p.setPen(QPen(QColor(150,200,150),1))
            p.drawText(pt+QPointF(r+3,14),
                       base['name'].replace('AFB ','').replace('AFS ',''))
        p.setPen(QPen(QColor(200,255,200),1))
        p.setFont(QFont("Consolas",9,QFont.Bold))
        p.drawText(QPointF(8,16),"SAAF Bases \u2014 South Africa")
        p.end()
    def mousePressEvent(self,e):
        if e.button()!=Qt.LeftButton: return
        click=QPointF(e.pos())
        for icao,base in SAAF_BASES.items():
            pt=self._p(base['lat'],base['lon'])
            if (click-pt).manhattanLength()<16:
                self._sel=icao; self.update(); self.base_selected.emit(icao); return


# ===========================================================================
# SAAF Base Map Widget
# ===========================================================================
class SAAFBaseMapWidget(QWidget):
    def __init__(self,parent=None):
        super().__init__(parent); self.setMinimumSize(460,380)
        self._base=None; self._zoom=1.0; self._offset=QPointF(0,0)
        self._drag=None; self.setMouseTracking(True)
    def set_base(self,b): self._base=b; self._zoom=1.0; self._offset=QPointF(0,0); self.update()
    def _cx(self): return self.width()/2+self._offset.x()
    def _cy(self): return self.height()/2+self._offset.y()
    def _ll(self,lat,lon,rlat,rlon):
        s=25000*self._zoom
        dx=(lon-rlon)*s*math.cos(math.radians(rlat)); dy=-(lat-rlat)*s
        return QPointF(self._cx()+dx,self._cy()+dy)
    def _m(self,metres): return metres/111320*25000*self._zoom
    def paintEvent(self,event):
        p=QPainter(self); p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(),QColor(22,38,22))
        if not self._base:
            p.setPen(QPen(QColor(120,120,120))); p.setFont(QFont("Consolas",13))
            p.drawText(self.rect(),Qt.AlignCenter,"Select a base"); p.end(); return
        b=self._base; rlat=b['lat']; rlon=b['lon']
        self._draw_aprons(p,b,rlat,rlon); self._draw_taxiways(p,b,rlat,rlon)
        self._draw_runways(p,b,rlat,rlon); self._draw_ils(p,b,rlat,rlon)
        self._draw_compass(p); self._draw_footer(p); p.end()
    def _draw_aprons(self,p,b,rlat,rlon):
        offs=[(0.002,0.004),(0.002,-0.004),(-0.003,0.0),(0.004,0.0)]
        p.setPen(QPen(QColor(80,90,80),1)); p.setBrush(QBrush(QColor(35,40,35)))
        for i,ap in enumerate(b.get('aprons',[])):
            if i>=len(offs): break
            dlat,dlon=offs[i]; ctr=self._ll(rlat+dlat,rlon+dlon,rlat,rlon)
            w2=self._m(300); h2=self._m(150)
            p.drawRect(QRectF(ctr.x()-w2,ctr.y()-h2,w2*2,h2*2))
            p.setPen(QPen(QColor(140,180,140),1))
            p.setFont(QFont("Consolas",max(6,int(6*self._zoom))))
            p.drawText(ctr+QPointF(4,4),ap)
            p.setPen(QPen(QColor(80,90,80),1))
    def _draw_taxiways(self,p,b,rlat,rlon):
        p.setPen(QPen(QColor(55,55,65),max(2,self._m(18)),Qt.SolidLine,Qt.RoundCap))
        for rwy in b['runways']:
            lo=self._ll(rwy['thr_lo_lat'],rwy['thr_lo_lon'],rlat,rlon)
            hi=self._ll(rwy['thr_hi_lat'],rwy['thr_hi_lon'],rlat,rlon)
            dx=hi.x()-lo.x(); dy=hi.y()-lo.y()
            length=math.sqrt(dx*dx+dy*dy)
            if length<1: continue
            nx,ny=-dy/length,dx/length; off=self._m(150)
            tlo=QPointF(lo.x()+nx*off,lo.y()+ny*off)
            thi=QPointF(hi.x()+nx*off,hi.y()+ny*off)
            p.drawLine(tlo,thi)
            mid=QPointF((tlo.x()+thi.x())/2,(tlo.y()+thi.y())/2)
            p.setPen(QPen(QColor(160,160,200),1))
            p.setFont(QFont("Consolas",max(7,int(7*self._zoom))))
            p.drawText(mid+QPointF(4,-4),"TWY")
            p.setPen(QPen(QColor(55,55,65),max(2,self._m(18)),Qt.SolidLine,Qt.RoundCap))
    def _draw_runways(self,p,b,rlat,rlon):
        for rwy in b['runways']:
            lo=self._ll(rwy['thr_lo_lat'],rwy['thr_lo_lon'],rlat,rlon)
            hi=self._ll(rwy['thr_hi_lat'],rwy['thr_hi_lon'],rlat,rlon)
            rw=max(4,self._m(rwy['width_m']))
            p.setPen(QPen(QColor(60,60,65),rw,Qt.SolidLine,Qt.FlatCap)); p.drawLine(lo,hi)
            p.setPen(QPen(QColor(220,220,160,180),max(1,rw*0.06),Qt.DashLine,Qt.FlatCap))
            p.drawLine(lo,hi)
            dx=hi.x()-lo.x(); dy=hi.y()-lo.y()
            length=math.sqrt(dx*dx+dy*dy)
            if length<1: continue
            ux,uy=dx/length,dy/length; nx,ny=-uy,ux; hw=rw*0.5
            for tpt in [lo,hi]:
                p.setPen(QPen(QColor(255,255,255),max(2,rw*0.15)))
                p.drawLine(QPointF(tpt.x()+nx*hw,tpt.y()+ny*hw),
                           QPointF(tpt.x()-nx*hw,tpt.y()-ny*hw))
            desigs=rwy['designation'].split('/')
            for pt,des,sign in [(lo,desigs[0],1),(hi,desigs[1],-1)]:
                ox=ux*sign*(rw*0.7+self._m(80)); oy=uy*sign*(rw*0.7+self._m(80))
                lp=QPointF(pt.x()+ox,pt.y()+oy)
                p.setPen(QPen(QColor(200,255,200),1))
                p.setFont(QFont("Consolas",max(8,int(9*self._zoom)),QFont.Bold))
                p.drawText(lp+QPointF(-12,6),des)
            mid=QPointF((lo.x()+hi.x())/2,(lo.y()+hi.y())/2)
            p.setPen(QPen(QColor(255,230,100),1))
            p.setFont(QFont("Consolas",max(7,int(7*self._zoom))))
            p.drawText(mid+QPointF(rw+4,0),f"{rwy['length_m']}m  {rwy['surface']}")
    def _draw_ils(self,p,b,rlat,rlon):
        for rwy in b['runways']:
            if not rwy.get('ils') or rwy['ils']=='None': continue
            for ils_end in str(rwy['ils']).split('&'):
                ils_end=ils_end.strip(); desigs=rwy['designation'].split('/')
                if ils_end==desigs[0]:
                    apex=self._ll(rwy['thr_lo_lat'],rwy['thr_lo_lon'],rlat,rlon)
                    far=self._ll(rwy['thr_hi_lat'],rwy['thr_hi_lon'],rlat,rlon)
                elif ils_end==desigs[1]:
                    apex=self._ll(rwy['thr_hi_lat'],rwy['thr_hi_lon'],rlat,rlon)
                    far=self._ll(rwy['thr_lo_lat'],rwy['thr_lo_lon'],rlat,rlon)
                else: continue
                dx=apex.x()-far.x(); dy=apex.y()-far.y()
                length=math.sqrt(dx*dx+dy*dy)
                if length<1: continue
                ux,uy=dx/length,dy/length; nx,ny=-uy,ux
                cl=self._m(4000); hw=cl*math.tan(math.radians(2.5))
                tip=QPointF(apex.x()+ux*cl,apex.y()+uy*cl)
                left=QPointF(tip.x()+nx*hw,tip.y()+ny*hw)
                right=QPointF(tip.x()-nx*hw,tip.y()-ny*hw)
                poly=QPolygonF([apex,left,right])
                p.setPen(QPen(QColor(0,200,255,160),1))
                p.setBrush(QBrush(QColor(0,200,255,30))); p.drawPolygon(poly)
                p.setFont(QFont("Consolas",max(7,int(7*self._zoom))))
                p.setPen(QPen(QColor(0,200,255,160),1))
                p.drawText(tip+QPointF(4,0),f"ILS {ils_end}")
    def _draw_compass(self,p):
        cx,cy=50,50; r=24
        p.setPen(QPen(QColor(180,220,180),1))
        p.setBrush(QBrush(QColor(0,0,0,100))); p.drawEllipse(cx-r,cy-r,r*2,r*2)
        p.setPen(QPen(QColor(255,80,80),2)); p.drawLine(cx,cy,cx,cy-r+4)
        p.setPen(QPen(QColor(180,180,180),1))
        p.drawLine(cx,cy,cx,cy+r-4); p.drawLine(cx,cy,cx+r-4,cy); p.drawLine(cx,cy,cx-r+4,cy)
        p.setFont(QFont("Consolas",8,QFont.Bold))
        p.setPen(QPen(QColor(255,80,80),1)); p.drawText(cx-4,cy-r-4,"N")
        p.setPen(QPen(QColor(180,220,180),1))
        p.drawText(cx-4,cy+r+12,"S"); p.drawText(cx+r+4,cy+4,"E"); p.drawText(cx-r-12,cy+4,"W")
    def _draw_footer(self,p):
        p.setPen(QPen(QColor(180,255,180),1)); p.setFont(QFont("Consolas",8))
        p.drawText(QPointF(8,self.height()-8),"Scroll=zoom  Drag=pan")
    def mousePressEvent(self,e):
        if e.button()==Qt.LeftButton: self._drag=e.pos()
    def mouseMoveEvent(self,e):
        if self._drag and e.buttons()&Qt.LeftButton:
            d=e.pos()-self._drag; self._offset+=QPointF(d.x(),d.y())
            self._drag=e.pos(); self.update()
    def mouseReleaseEvent(self,e): self._drag=None
    def wheelEvent(self,e):
        self._zoom=max(0.3,min(10.0,self._zoom*(1.12 if e.angleDelta().y()>0 else 0.89)))
        self.update()


# ===========================================================================
# SAAF Base Info Widget
# ===========================================================================
class SAAFBaseInfoWidget(QWidget):
    def __init__(self):
        super().__init__(); lo=QVBoxLayout(self); lo.setContentsMargins(4,4,4,4)
        self.name_lbl=QLabel("Select a base")
        self.name_lbl.setStyleSheet("font-size:13px;font-weight:bold;color:#aaffaa;padding:4px")
        lo.addWidget(self.name_lbl)
        self.sub_lbl=QLabel("")
        self.sub_lbl.setStyleSheet("font-size:10px;color:#88cc88;padding:2px 4px")
        lo.addWidget(self.sub_lbl)
        gg=QGroupBox("General"); gl=QGridLayout(gg); self._gl={}
        for r,(k,lbl) in enumerate([
            ('icao','ICAO'),('province','Province'),('elevation','Elevation'),
            ('mag_var','Mag Var'),('role','Role'),('freq','Frequencies'),
        ]):
            gl.addWidget(QLabel(lbl+':'),r,0)
            v=QLabel("\u2014"); v.setWordWrap(True); v.setStyleSheet("color:#ccffcc")
            gl.addWidget(v,r,1); self._gl[k]=v
        lo.addWidget(gg)
        rg=QGroupBox("Runway Matrix"); rl=QVBoxLayout(rg)
        self.rwy_table=QTableWidget(); self.rwy_table.setColumnCount(8)
        self.rwy_table.setHorizontalHeaderLabels(
            ["Desig","Hdg Lo","Hdg Hi","Length(m)","Width(m)","Surface","ILS","PCN"])
        self.rwy_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        self.rwy_table.setAlternatingRowColors(True); self.rwy_table.setMaximumHeight(160)
        self.rwy_table.setEditTriggers(QTableWidget.NoEditTriggers)
        rl.addWidget(self.rwy_table); lo.addWidget(rg)
        sg=QGroupBox("Squadrons"); sl=QVBoxLayout(sg)
        self.sq_list=QListWidget(); self.sq_list.setMaximumHeight(90)
        sl.addWidget(self.sq_list); lo.addWidget(sg)
        ng=QGroupBox("Navaids"); nl=QVBoxLayout(ng)
        self.nav_text=QTextEdit(); self.nav_text.setReadOnly(True); self.nav_text.setMaximumHeight(70)
        nl.addWidget(self.nav_text); lo.addWidget(ng)
        rmg=QGroupBox("Remarks"); rml=QVBoxLayout(rmg)
        self.rem_text=QTextEdit(); self.rem_text.setReadOnly(True); self.rem_text.setMaximumHeight(65)
        rml.addWidget(self.rem_text); lo.addWidget(rmg); lo.addStretch()

    def show_base(self,icao,base):
        self.name_lbl.setText(f"{base['name']}  [{icao}]")
        self.sub_lbl.setText(
            f"{base['city']}  \u00b7  {base['province']}  \u00b7  "
            f"Elev {base['elevation']}m AMSL")
        freq=base.get('frequencies',{}); fs="  ".join(f"{k}:{v}" for k,v in freq.items())
        self._gl['icao'].setText(icao)
        self._gl['province'].setText(base['province'])
        self._gl['elevation'].setText(
            f"{base['elevation']} m  ({int(base['elevation']*3.28084)} ft)")
        self._gl['mag_var'].setText(
            f"{abs(base.get('magnetic_var',0))}\u00b0"
            f"{'W' if base.get('magnetic_var',0)<0 else 'E'}")
        self._gl['role'].setText(base['role'])
        self._gl['freq'].setText(fs)
        self.rwy_table.setRowCount(0)
        for rwy in base['runways']:
            r=self.rwy_table.rowCount(); self.rwy_table.insertRow(r)
            for c,v in enumerate([
                rwy['designation'],
                f"{rwy['true_hdg_lo']:03d}\u00b0",
                f"{rwy['true_hdg_hi']:03d}\u00b0",
                str(rwy['length_m']), str(rwy['width_m']),
                rwy['surface'], rwy.get('ils','\u2014'), rwy.get('pcn','\u2014'),
            ]):
                item=QTableWidgetItem(v)
                if c==3:
                    l=int(rwy['length_m'])
                    col=(QColor(0,255,120) if l>=3000
                         else QColor(255,220,80) if l>=2000
                         else QColor(255,140,80))
                    item.setForeground(col)
                self.rwy_table.setItem(r,c,item)
        self.sq_list.clear()
        for sq in base.get('squadrons',[]):
            item=QListWidgetItem(sq); item.setForeground(QColor(180,255,180))
            self.sq_list.addItem(item)
        self.nav_text.setPlainText('\n'.join(_build_base_navaids(base)))
        self.rem_text.setPlainText(_build_base_remarks(base))


# ===========================================================================
# Mock VOR TCP Server
# ===========================================================================
class MockVORTCPServer:
    def __init__(self,host='127.0.0.1',port=5000):
        self.host=host; self.port=port
        self._sock=None; self._running=False; self._thread=None
    def start(self):
        if self._running: return True
        try:
            self._sock=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
            self._sock.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
            self._sock.bind((self.host,self.port))
            self._sock.listen(5); self._sock.settimeout(1.0)
            self._running=True
            self._thread=Thread(target=self._accept,daemon=True); self._thread.start()
            logger.info(f"[MockVOR] {self.host}:{self.port}"); return True
        except OSError as e:
            logger.error(f"[MockVOR] {e}"); self._running=False
            if self._sock: self._sock.close(); self._sock=None
            return False
    def stop(self):
        self._running=False
        if self._sock:
            try: self._sock.close()
            except: pass
            self._sock=None
        if self._thread: self._thread.join(timeout=3)
    @property
    def is_running(self): return self._running
    def _accept(self):
        while self._running:
            try:
                conn,_=self._sock.accept()
                Thread(target=self._serve,args=(conn,),daemon=True).start()
            except socket.timeout: continue
            except OSError: break
    def _serve(self,conn):
        try:
            while self._running:
                data=conn.recv(1024)
                if not data: break
                cmd=data.decode(errors='replace').strip().upper()
                conn.sendall((self._resp(cmd)+'\r\n').encode())
        except: pass
        finally: conn.close()
    @staticmethod
    def _resp(cmd):
        if 'STATUS' in cmd:
            return (f"{round(random.uniform(108,117.9),1)},"
                    f"{round(random.uniform(0,360),1)},"
                    f"{round(random.uniform(-2.5,2.5),2)},"
                    f"{round(random.uniform(-80,-20),1)},"
                    f"{random.choices(['OK','OK','OK','WARN'],[70,10,10,10])[0]}")
        if 'IDENT'     in cmd: return "JNB,114.9,ATIS-ACTIVE"
        if 'HEALTH'    in cmd: return f"HEALTH:{random.randint(85,100)}%,TX:OK,MON:OK,GPS:OK"
        if 'CALIBRATE' in cmd: return "CALIBRATION:STARTED,ETA:30SEC"
        return "ERROR:UNKNOWN_COMMAND"


# ===========================================================================
# VOR / Aircraft Data Structures
# ===========================================================================
class VORData:
    def __init__(self):
        self.timestamp=datetime.now()
        self.frequency=self.bearing=self.deviation=self.signal_strength=0.0
        self.status='OK'; self.health=100; self.ident=''; self.to_from='TO'

class AircraftData:
    def __init__(self,acid):
        self.aircraft_id=acid
        self.latitude=self.longitude=self.altitude=self.heading=self.speed=0.0
        self.timestamp=datetime.now()
        self.vor_bearing=self.distance_from_vor=0.0
        self.flight_phase='CRUISE'; self.waypoints=[]; self.track_history=[]


# ===========================================================================
# Connection Handler
# ===========================================================================
class VORConnectionHandler:
    def __init__(self):
        self.serial_conn=None; self.tcp_conn=None
        self.is_connected=False; self._lock=Lock()
    def connect_serial(self,port,baudrate=9600,timeout=1):
        try:
            self.serial_conn=serial.Serial(port,baudrate=baudrate,timeout=timeout)
            self.is_connected=True; return True
        except Exception as e: logger.error(f"Serial:{e}"); return False
    def connect_tcp(self,host,port=5000,timeout=5):
        try:
            self.tcp_conn=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
            self.tcp_conn.settimeout(timeout); self.tcp_conn.connect((host,port))
            self.is_connected=True; return True
        except Exception as e: logger.error(f"TCP:{e}"); return False
    def send_command(self,cmd):
        try:
            with self._lock:
                if self.serial_conn and self.serial_conn.is_open:
                    self.serial_conn.write(cmd.encode()+b'\r\n')
                    return self.serial_conn.readline().decode().strip()
                if self.tcp_conn:
                    self.tcp_conn.send(cmd.encode()+b'\r\n')
                    return self.tcp_conn.recv(1024).decode().strip()
        except Exception as e: logger.error(f"Cmd:{e}")
        return ''
    def disconnect(self):
        try:
            if self.serial_conn and self.serial_conn.is_open: self.serial_conn.close()
            if self.tcp_conn: self.tcp_conn.close()
        except: pass
        self.is_connected=False
    @staticmethod
    def list_ports():
        return [dict(device=p.device, description=p.description or p.device)
                for p in serial.tools.list_ports.comports()]


# ===========================================================================
# VOR Data Processor
# ===========================================================================
class VORDataProcessor:
    def __init__(self): self._buf=deque(maxlen=1000); self._lock=Lock()
    def process(self,raw):
        try:
            parts=raw.split(',')
            if len(parts)>=5:
                d=VORData()
                d.frequency=float(parts[0]); d.bearing=float(parts[1])
                d.deviation=float(parts[2]); d.signal_strength=float(parts[3])
                d.status=parts[4]
                with self._lock: self._buf.append(d)
                return d
        except Exception as e: logger.error(f"VOR parse:{e}")
        return None
    def latest(self):
        with self._lock: return self._buf[-1] if self._buf else None


# ===========================================================================
# Aircraft Tracker
# ===========================================================================
class AircraftTracker:
    def __init__(self): self._ac={}; self._lock=Lock(); self.vor_pos=(0.0,0.0)
    def set_vor_position(self,lat,lon): self.vor_pos=(lat,lon)
    def update(self,acid,lat,lon,alt,hdg,spd,waypoints=None):
        with self._lock:
            if acid not in self._ac: self._ac[acid]=AircraftData(acid)
            ac=self._ac[acid]
            ac.latitude=lat; ac.longitude=lon; ac.altitude=alt
            ac.heading=hdg; ac.speed=spd; ac.timestamp=datetime.now()
            ac.vor_bearing,ac.distance_from_vor=self._rel(lat,lon)
            ac.track_history.append((lat,lon))
            if len(ac.track_history)>120: ac.track_history.pop(0)
            if waypoints is not None: ac.waypoints=waypoints
    def _rel(self,lat,lon):
        R=6371; vla,vlo=self.vor_pos
        la1,lo1=math.radians(vla),math.radians(vlo)
        la2,lo2=math.radians(lat),math.radians(lon)
        dlat,dlon=la2-la1,lo2-lo1
        a=math.sin(dlat/2)**2+math.cos(la1)*math.cos(la2)*math.sin(dlon/2)**2
        dist=R*2*math.asin(math.sqrt(max(0,a)))
        y=math.sin(dlon)*math.cos(la2)
        x=math.cos(la1)*math.sin(la2)-math.sin(la1)*math.cos(la2)*math.cos(dlon)
        return (math.degrees(math.atan2(y,x))+360)%360, dist
    def get_all(self):
        with self._lock: return list(self._ac.values())
    def clear(self):
        with self._lock: self._ac.clear()


# ===========================================================================
# En-route Simulation Engine
# ===========================================================================
class SimulationEngine:
    _FLEET_TEMPLATE = [
        dict(id='SAA101',init_brg=45, init_dist=90, alt=32000,spd=480,hdg=225,
             waypoints=[('ARNEM',80),('LERKO',60),('GOMTU',40),('NORMA',20),('FINAL',5)]),
        dict(id='BAW202',init_brg=135,init_dist=75, alt=28000,spd=450,hdg=315,
             waypoints=[('GAVEL',65),('TIVDO',45),('ABLET',30),('KUXAM',15),('FINAL',4)]),
        dict(id='MSR303',init_brg=220,init_dist=60, alt=24000,spd=420,hdg=40,
             waypoints=[('DUXOM',50),('NEVLU',35),('POLTU',25),('XAMBI',12),('FINAL',3)]),
        dict(id='DAL404',init_brg=310,init_dist=85, alt=35000,spd=510,hdg=130,
             waypoints=[('TIXNO',70),('BOLUM',50),('GEXIT',35),('DUVLA',18),('FINAL',6)]),
        dict(id='ETH505',init_brg=0,  init_dist=50, alt=20000,spd=390,hdg=180,
             waypoints=[('KOVUN',42),('PELTU',30),('RIMDO',20),('VATEX',10),('FINAL',2)]),
    ]
    VOR_LAT = -25.5967; VOR_LON = 28.2394

    def __init__(self): self._ac={}; self._reset()

    def set_target(self, lat: float, lon: float):
        """Redirect approaches to a different VOR / SAAF base."""
        self.VOR_LAT = lat; self.VOR_LON = lon; self._reset()

    def _reset(self):
        self._ac={}
        for f in self._FLEET_TEMPLATE:
            brg=math.radians(f['init_brg']); d=f['init_dist']
            lat=self.VOR_LAT+(d/111)*math.cos(brg)
            lon=self.VOR_LON+(d/111)*math.sin(brg)/math.cos(math.radians(self.VOR_LAT))
            self._ac[f['id']]=dict(
                lat=lat,lon=lon,alt=f['alt'],spd=f['spd'],hdg=f['hdg'],
                target_brg=f['init_brg'],dist=d,id=f['id'],
                waypoints=[dict(name=w[0],dist=w[1]) for w in f['waypoints']])

    def tick(self, tracker: AircraftTracker):
        for acid,s in self._ac.items():
            s['dist']=max(1.0,s['dist']-s['spd']/3600)
            btv=(s['target_brg']+180)%360; herr=((btv-s['hdg']+540)%360)-180
            s['hdg']=(s['hdg']+max(-3,min(3,herr*0.1)))%360
            if s['dist']<30: s['alt']=max(1000,s['alt']-300)
            brad=math.radians((s['target_brg']+180)%360)
            s['lat']=self.VOR_LAT+(s['dist']/111)*math.cos(brad)
            s['lon']=self.VOR_LON+(s['dist']/111)*math.sin(brad)/math.cos(math.radians(self.VOR_LAT))
            wps=[]
            for wp in s['waypoints']:
                wd=wp['dist']
                wlat=self.VOR_LAT+(wd/111)*math.cos(brad)
                wlon=self.VOR_LON+(wd/111)*math.sin(brad)/math.cos(math.radians(self.VOR_LAT))
                wps.append(dict(name=wp['name'],lat=wlat,lon=wlon))
            tracker.update(acid,s['lat'],s['lon'],s['alt'],s['hdg'],s['spd'],wps)
            if s['dist']<=1.5: self._reset(); break

    def reset(self): self._reset()


# ===========================================================================
# Data Acquisition Thread
# ===========================================================================
class DataAcquisitionThread(QThread):
    data_updated  = pyqtSignal(dict)
    error_occurred= pyqtSignal(str)
    def __init__(self,conn):
        super().__init__(); self.conn=conn; self.running=False
        self.processor=VORDataProcessor()
    def run(self):
        self.running=True
        while self.running:
            try:
                resp=self.conn.send_command("STATUS")
                if resp:
                    vd=self.processor.process(resp)
                    if vd: self.data_updated.emit(dict(type='vor_data',data=vd.__dict__))
                time.sleep(1)
            except Exception as e:
                logger.error(f"DAQ:{e}"); self.error_occurred.emit(str(e))
    def stop(self): self.running=False


# ===========================================================================
# 3-D Terrain Widget  (slope shading)
# ===========================================================================
class Terrain3DWidget(QOpenGLWidget):
    GRID=64; SIZE=40.0
    VOR_LAT=-25.5967; VOR_LON=28.2394

    def __init__(self,parent=None):
        super().__init__(parent); self.setMinimumSize(500,380)
        self._rx=35.0; self._ry=-45.0; self._zoom=1.0; self._last=QPoint()
        self._heights=None; self._slopes=None; self._aircraft=[]; self._q=None

    def set_vor_pos(self,lat,lon): self.VOR_LAT=lat; self.VOR_LON=lon
    def terrain_profile(self):
        if self._heights is None:
            return None
        return [float(v) for v in self._heights[self.GRID // 2]]

    def initializeGL(self):
        glEnable(GL_DEPTH_TEST); glEnable(GL_LIGHTING); glEnable(GL_LIGHT0)
        glEnable(GL_COLOR_MATERIAL)
        glColorMaterial(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE)
        glLightfv(GL_LIGHT0,GL_POSITION,[1.5,3.0,2.0,0.0])
        glLightfv(GL_LIGHT0,GL_DIFFUSE, [1.0,1.0,0.95,1.0])
        glLightfv(GL_LIGHT0,GL_AMBIENT, [0.25,0.25,0.25,1.0])
        glClearColor(0.12,0.18,0.32,1.0)
        self._q=gluNewQuadric(); self._gen()

    def resizeGL(self,w,h):
        glViewport(0,0,w,h if h else 1)
        glMatrixMode(GL_PROJECTION); glLoadIdentity()
        gluPerspective(45.0,w/(h if h else 1),0.1,2000.0)
        glMatrixMode(GL_MODELVIEW)

    def paintGL(self):
        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT); glLoadIdentity()
        eye=120.0/self._zoom
        gluLookAt(eye,eye*0.8,eye,self.SIZE/2,0,self.SIZE/2,0,1,0)
        glRotatef(self._rx,1,0,0); glRotatef(self._ry,0,1,0)
        self._terrain(); self._aircraft_gl()

    def _gen(self):
        g=self.GRID
        x=np.linspace(0,self.SIZE,g+1); z=np.linspace(0,self.SIZE,g+1)
        xx,zz=np.meshgrid(x,z)
        h=(4.5*np.sin(xx/4.5)*np.cos(zz/4.5)+
           2.0*np.sin(xx/2.2+1.0)*np.sin(zz/2.8)+
           1.2*np.cos(xx/6.5+0.5)*np.cos(zz/3.8)+
           0.6*np.sin(xx/1.5+2.1)*np.cos(zz/1.8))
        cx,cz=g//2,g//2
        for i in range(-5,6):
            for j in range(-5,6):
                ri,rj=cx+i,cz+j
                if 0<=ri<=g and 0<=rj<=g: h[ri,rj]=0.0
        self._heights=h.astype(np.float32)
        sl=np.zeros_like(self._heights)
        for i in range(1,g):
            for j in range(1,g):
                dzdx=float(h[i+1,j]-h[i-1,j]) if 1<=i<g else 0
                dzdy=float(h[i,j+1]-h[i,j-1]) if 1<=j<g else 0
                sl[i,j]=math.sqrt(dzdx*dzdx+dzdy*dzdy)
        self._slopes=sl

    def _hcol(self,h,hmin,hmax,slope=0.0):
        t=max(0.0,min(1.0,(h-hmin)/(hmax-hmin+1e-6)))
        s=min(1.0,slope*0.4)
        if   t<0.15: r,g,b=0.20+t,      0.45+t*0.3,  0.18
        elif t<0.40: r,g,b=0.22+t*0.5,  0.52+t*0.2,  0.18
        elif t<0.65: r,g,b=0.55+t*0.25, 0.40,        0.22
        elif t<0.85: r,g,b=0.65+t*0.2,  0.55+t*0.1,  0.40
        else:        r,g,b=0.88,         0.90,        0.95
        dark=1.0-s*0.5
        return r*dark, g*dark, b*dark

    def _terrain(self):
        if self._heights is None: return
        h=self._heights; sl=self._slopes; g=self.GRID
        s=self.SIZE/g; hmin,hmax=h.min(),h.max()
        glBegin(GL_QUADS)
        for i in range(g):
            for j in range(g):
                for di,dj in [(0,0),(1,0),(1,1),(0,1)]:
                    hv=float(h[i+di,j+dj]); sv=float(sl[i+di,j+dj])
                    glColor3f(*self._hcol(hv,hmin,hmax,sv))
                    ni=min(i+di+1,g); pi=max(i+di-1,0)
                    nj=min(j+dj+1,g); pj=max(j+dj-1,0)
                    nx=float(h[pi,j+dj]-h[ni,j+dj])
                    nz=float(h[i+di,pj]-h[i+di,nj])
                    nn=math.sqrt(nx*nx+4+nz*nz)
                    glNormal3f(nx/nn,2/nn,nz/nn)
                    glVertex3f((i+di)*s,hv,(j+dj)*s)
        glEnd()
        cx=self.SIZE/2; glDisable(GL_LIGHTING); glColor3f(0.25,0.25,0.25)
        glBegin(GL_QUADS)
        glVertex3f(cx-1.5,0.02,4); glVertex3f(cx+1.5,0.02,4)
        glVertex3f(cx+1.5,0.02,self.SIZE-4); glVertex3f(cx-1.5,0.02,self.SIZE-4)
        glEnd(); glEnable(GL_LIGHTING)

    def _aircraft_gl(self):
        if not self._aircraft or self._q is None: return
        cx,cz=self.SIZE/2,self.SIZE/2; sc=self.SIZE/2.0
        COLS=[(1,0.2,0.2),(0.2,0.8,1),(1,0.8,0),(0.4,1,0.4),(1,0.4,1)]
        glDisable(GL_LIGHTING)
        for idx,ac in enumerate(self._aircraft):
            col=COLS[idx%len(COLS)]
            dlat=(ac.latitude  - self.VOR_LAT)*sc*3
            dlon=(ac.longitude - self.VOR_LON)*sc*3
            wx=cx+dlon; wz=cz-dlat; wy=max(0.3,ac.altitude/35000.0*12.0)
            glColor3f(*col)
            glPushMatrix(); glTranslatef(wx,wy,wz); gluSphere(self._q,0.6,12,12); glPopMatrix()
            glBegin(GL_LINES); glVertex3f(wx,wy,wz); glVertex3f(wx,0,wz); glEnd()
            for wp in ac.waypoints:
                wdlat=(wp['lat']-self.VOR_LAT)*sc*3
                wdlon=(wp['lon']-self.VOR_LON)*sc*3
                glColor3f(1,0.9,0)
                glPushMatrix(); glTranslatef(cx+wdlon,0.3,cz-wdlat)
                gluSphere(self._q,0.25,8,8); glPopMatrix()
        glEnable(GL_LIGHTING)

    def set_aircraft(self,lst): self._aircraft=lst; self.update()
    def reset_view(self): self._rx,self._ry,self._zoom=35.0,-45.0,1.0; self.update()
    def mousePressEvent(self,e): self._last=e.pos()
    def mouseMoveEvent(self,e):
        dx=e.x()-self._last.x(); dy=e.y()-self._last.y()
        if e.buttons()&Qt.LeftButton:
            self._ry+=dx*0.5; self._rx+=dy*0.5; self.update()
        self._last=e.pos()
    def wheelEvent(self,e):
        self._zoom=max(0.2,min(5.0,self._zoom+e.angleDelta().y()/1200)); self.update()


# ===========================================================================
# Radar Display  — realtime animated aircraft plots
# ===========================================================================
class RadarDisplay(QWidget):
    RANGE_NM = 120
    VOR_LAT  = -25.5967
    VOR_LON  =  28.2394

    def __init__(self):
        super().__init__(); self.aircraft=[]
        self.setMinimumSize(500,500); self.setStyleSheet("background:#040d08;")
        self._pal=[QColor(0,255,120),QColor(0,200,255),QColor(255,220,0),
                   QColor(180,255,80),QColor(255,100,220)]
        self._sweep_angle=0.0
        self._sweep_timer=QTimer()
        self._sweep_timer.timeout.connect(self._advance_sweep)
        self._sweep_timer.start(50)

    def set_vor_pos(self,lat,lon): self.VOR_LAT=lat; self.VOR_LON=lon
    def _advance_sweep(self): self._sweep_angle=(self._sweep_angle+2.0)%360.0; self.update()
    def set_aircraft(self,lst): self.aircraft=lst; self.update()

    def paintEvent(self,event):
        p=QPainter(self); p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(),QColor(4,13,8))
        w,h=self.width(),self.height(); cx,cy=w//2,h//2; R=min(w,h)//2-30

        # Range rings
        for frac,nm in [(0.25,30),(0.5,60),(0.75,90),(1.0,120)]:
            r=int(R*frac)
            p.setPen(QPen(QColor(0,80,40),1,Qt.DashLine))
            p.drawEllipse(cx-r,cy-r,2*r,2*r)
            p.setPen(QPen(QColor(0,150,80),1)); p.setFont(QFont("Consolas",8))
            p.drawText(cx+r+3,cy-3,f"{nm}nm")

        # Radial lines
        p.setPen(QPen(QColor(0,60,30),1))
        for deg in range(0,360,30):
            rad=math.radians(deg)
            p.drawLine(cx,cy,int(cx+R*math.sin(rad)),int(cy-R*math.cos(rad)))

        # Animated sweep
        for i in range(30):
            angle=(self._sweep_angle-i*1.5)%360
            sweep_col=QColor(0,255,80,max(0,35-i))
            p.setPen(QPen(sweep_col,1)); rad=math.radians(angle)
            p.drawLine(cx,cy,int(cx+R*math.sin(rad)),int(cy-R*math.cos(rad)))

        # Cardinals
        p.setPen(QPen(QColor(0,200,100),1)); p.setFont(QFont("Consolas",10,QFont.Bold))
        p.drawText(cx-6,cy-R-8,"N"); p.drawText(cx+R+6,cy+5,"E")
        p.drawText(cx-6,cy+R+18,"S"); p.drawText(cx-R-20,cy+5,"W")

        # VOR dot
        p.setPen(Qt.NoPen); p.setBrush(QBrush(QColor(255,255,0)))
        p.drawEllipse(cx-5,cy-5,10,10)
        p.setPen(QPen(QColor(255,255,0),1)); p.setFont(QFont("Consolas",8))
        p.drawText(cx+7,cy-8,"VOR")

        # Aircraft
        for idx,ac in enumerate(self.aircraft):
            col=self._pal[idx%len(self._pal)]
            if ac.distance_from_vor>self.RANGE_NM: continue
            brg=math.radians(ac.vor_bearing)
            d=(ac.distance_from_vor/self.RANGE_NM)*R
            ax=int(cx+d*math.sin(brg)); ay=int(cy-d*math.cos(brg))

            # Track trail
            if len(ac.track_history)>1:
                for ti in range(1,len(ac.track_history)):
                    pv=ac.track_history[ti-1]; cr=ac.track_history[ti]
                    _,pd=self._rel(pv[0],pv[1]); pb=math.radians(self._brg(pv[0],pv[1]))
                    _,cd=self._rel(cr[0],cr[1]); cb=math.radians(self._brg(cr[0],cr[1]))
                    alpha=int(80*(ti/len(ac.track_history)))
                    tc=QColor(col); tc.setAlpha(alpha); p.setPen(QPen(tc,2))
                    px=int(cx+(pd/self.RANGE_NM)*R*math.sin(pb))
                    py=int(cy-(pd/self.RANGE_NM)*R*math.cos(pb))
                    qx=int(cx+(cd/self.RANGE_NM)*R*math.sin(cb))
                    qy=int(cy-(cd/self.RANGE_NM)*R*math.cos(cb))
                    p.drawLine(px,py,qx,qy)
                    if ti%5==0:
                        p.setPen(Qt.NoPen)
                        tc2=QColor(col); tc2.setAlpha(alpha//2)
                        p.setBrush(QBrush(tc2)); p.drawEllipse(qx-2,qy-2,4,4)

            # Waypoints
            for wp in ac.waypoints:
                _,wd=self._rel(wp['lat'],wp['lon'])
                if wd>self.RANGE_NM: continue
                wb=math.radians(self._brg(wp['lat'],wp['lon']))
                wx2=int(cx+(wd/self.RANGE_NM)*R*math.sin(wb))
                wy2=int(cy-(wd/self.RANGE_NM)*R*math.cos(wb))
                wc=QColor(col); wc.setAlpha(160)
                p.setPen(QPen(wc,1,Qt.DotLine)); p.setBrush(Qt.NoBrush)
                p.drawEllipse(wx2-4,wy2-4,8,8)
                p.setPen(QPen(wc,1)); p.setFont(QFont("Consolas",7))
                p.drawText(wx2+6,wy2+4,wp['name'])

            # Aircraft triangle
            hdg_r=math.radians(ac.heading)
            p.setPen(QPen(col,2)); p.setBrush(QBrush(col))
            tri=QPolygon([
                QPoint(ax+int(10*math.sin(hdg_r)),  ay-int(10*math.cos(hdg_r))),
                QPoint(ax+int(5*math.sin(hdg_r+2.4)),ay-int(5*math.cos(hdg_r+2.4))),
                QPoint(ax+int(5*math.sin(hdg_r-2.4)),ay-int(5*math.cos(hdg_r-2.4))),
            ]); p.drawPolygon(tri)

            # Heading vector
            p.setPen(QPen(col,1))
            p.drawLine(ax,ay,int(ax+32*math.sin(hdg_r)),int(ay-32*math.cos(hdg_r)))

            # Velocity leader (projected position)
            v_scale=0.008
            future_d=max(0,(ac.distance_from_vor/self.RANGE_NM
                            -ac.speed*v_scale/self.RANGE_NM)*R)
            fx=int(cx+future_d*math.sin(brg)); fy=int(cy-future_d*math.cos(brg))
            vc=QColor(col); vc.setAlpha(120)
            p.setPen(QPen(vc,1,Qt.DashLine)); p.drawLine(ax,ay,fx,fy)

            # Data callout box
            bx=ax+14; by=ay-56
            if bx+116>w: bx=ax-130
            if by<0: by=ay+8
            p.setPen(Qt.NoPen); p.setBrush(QBrush(QColor(8,24,16,215)))
            p.drawRoundedRect(bx-2,by-2,120,78,4,4)
            p.setPen(QPen(col,1)); p.setFont(QFont("Consolas",8,QFont.Bold))
            p.drawText(bx,by+11,ac.aircraft_id)
            p.setFont(QFont("Consolas",7)); p.setPen(QPen(QColor(200,255,200),1))
            for li,ln in enumerate([
                f"ALT  {int(ac.altitude):>6} ft",
                f"HDG  {ac.heading:>6.1f}\u00b0",
                f"SPD  {int(ac.speed):>6} kt",
                f"DST  {ac.distance_from_vor:>5.1f} nm",
                f"BRG  {ac.vor_bearing:>5.1f}\u00b0",
                f"PHS  {ac.flight_phase}",
            ]):
                p.drawText(bx,by+23+li*11,ln)
        p.end()

    def _rel(self,lat,lon):
        R=6371
        la1,lo1=math.radians(self.VOR_LAT),math.radians(self.VOR_LON)
        la2,lo2=math.radians(lat),math.radians(lon)
        dlat,dlon=la2-la1,lo2-lo1
        a=math.sin(dlat/2)**2+math.cos(la1)*math.cos(la2)*math.sin(dlon/2)**2
        return 0,R*2*math.asin(math.sqrt(max(0,a)))

    def _brg(self,lat,lon):
        la1,lo1=math.radians(self.VOR_LAT),math.radians(self.VOR_LON)
        la2,lo2=math.radians(lat),math.radians(lon); dlon=lo2-lo1
        y=math.sin(dlon)*math.cos(la2)
        x=math.cos(la1)*math.sin(la2)-math.sin(la1)*math.cos(la2)*math.cos(dlon)
        return (math.degrees(math.atan2(y,x))+360)%360


# ===========================================================================
# CDI
# ===========================================================================
class CDIDisplay(QWidget):
    def __init__(self): super().__init__(); self.deviation=0.0; self.setMinimumHeight(150)
    def set_deviation(self,v): self.deviation=max(-2.5,min(2.5,v)); self.update()
    def paintEvent(self,event):
        p=QPainter(self); p.setRenderHint(QPainter.Antialiasing)
        w,h=self.width(),self.height(); cx,cy=w//2,h//2
        p.fillRect(self.rect(),QColor(240,240,240))
        p.drawEllipse(cx-80,cy-80,160,160)
        p.setPen(QPen(Qt.black,2))
        p.drawLine(cx-70,cy,cx+70,cy); p.drawLine(cx,cy-70,cx,cy+70)
        for i in range(-5,6):
            x=cx+i*14
            p.drawLine(x,cy-75 if i%2==0 else cy-70,x,cy-65)
        nx=int(cx+self.deviation*28)
        p.setPen(QPen(Qt.red,3)); p.drawLine(nx,cy-60,nx,cy+60)
        p.setPen(QPen(Qt.black,1)); p.setFont(QFont("Arial",11))
        p.drawText(10,20,f"Deviation: {self.deviation:+.2f} dots")


# ===========================================================================
# Approach Guidance
# ===========================================================================
class ApproachGuidanceDisplay(QWidget):
    def __init__(self):
        super().__init__()
        self.aircraft = None
        self.guidance = {}
        self.setMinimumHeight(280)
    def set_aircraft(self,ac): self.aircraft=ac; self.update()
    def set_guidance(self,info): self.guidance=info or {}; self.update()
    def paintEvent(self,event):
        p=QPainter(self); p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(),QColor(180,210,240))
        w,h=self.width(),self.height(); ry=h-60; rw=w//3; rx=(w-rw)//2
        p.setPen(QPen(Qt.black,3)); p.drawRect(rx,ry,rw,40)
        p.setPen(QPen(Qt.white,2))
        for i in range(0,rw,40): p.drawLine(rx+i,ry+20,rx+i+20,ry+20)
        p.setPen(QPen(QColor(0,120,255),2,Qt.DashLine))
        p.drawLine(rx+rw//2,ry,rx+rw//2,int(ry-APPROACH_GUIDE_PIXELS))
        if self.aircraft:
            pnm=(h-ry-60)/10; sx=rx+rw//2; sy=ry
            p.setPen(QPen(QColor(0,140,0),2)); p.drawLine(sx,sy,sx,int(sy-5*pnm))
            ay=int(sy-(self.aircraft.distance_from_vor/5)*pnm)
            on_gs=self.guidance.get('on_profile', False)
            acol=QColor(0,180,0) if on_gs else QColor(220,60,40)
            p.setPen(QPen(acol,3)); p.setBrush(QBrush(acol))
            p.drawEllipse(sx-8,ay-8,16,16)
            p.setPen(QPen(Qt.black,1)); p.setFont(QFont("Arial",9))
            for li,ln in enumerate([
                f"ID:  {self.aircraft.aircraft_id}",
                f"ALT: {int(self.aircraft.altitude)} ft",
                f"DST: {self.aircraft.distance_from_vor:.1f} nm",
                f"HDG: {self.aircraft.heading:.1f}\u00b0",
                f"SPD: {int(self.aircraft.speed)} kt",
            ]):
                p.drawText(10,18+li*16,ln)
        if self.guidance:
            p.setPen(QPen(QColor(10,10,10),1)); p.setFont(QFont("Arial",9,QFont.Bold))
            p.drawText(10,h-38,self.guidance.get('runway',''))
            p.drawText(10,h-22,self.guidance.get('ils_text',''))
            p.drawText(10,h-6,self.guidance.get('glide_text',''))


# ===========================================================================
# Main Application Window
# ===========================================================================
class VORAirportMonitorApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle(
            "VOR / ASRACS / SAAF \u2014 Airport Monitoring System v5.2")
        self.setGeometry(50,50,1540,980)

        self.config      = VORAirportConfig()
        self.connection  = VORConnectionHandler()
        self.tracker     = AircraftTracker()
        self.processor   = VORDataProcessor()
        self.mock_server = None
        self.data_thread = None

        self.sim_engine  = SimulationEngine()
        self._sim_active = False
        self._sim_timer  = QTimer(); self._sim_timer.timeout.connect(self._sim_tick)

        self.asracs_engine  = ASRACSSimEngine()
        self._asracs_active = False
        self._asracs_timer  = QTimer(); self._asracs_timer.timeout.connect(self._asracs_tick)

        self._port_timer = QTimer(); self._port_timer.timeout.connect(self._auto_refresh_ports)
        self._last_ports: List[str] = []
        self._current_vor_key = 'JNB_VOR'
        self._current_base_icao = ''
        self.glide_slope_detector = GlideSlopeDetector()
        self.surface_slope_analyzer = SurfaceSlopeAnalyzer()

        # Set initial VOR position
        v = self.config.vor_stations.get('JNB_VOR', {})
        self.tracker.set_vor_position(
            v.get('latitude', -25.5967),
            v.get('longitude', 28.2394))

        # Build UI first, THEN populate VOR combo (Fix 2+3)
        self._build_ui()
        self._build_menus()
        self._build_toolbar()

        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self._update_displays)
        self.update_timer.start(1000)
        self._port_timer.start(2000)
        logger.info("VOR/ASRACS/SAAF v5.1 started")

    # ── Toolbar ───────────────────────────────────────────────────────────
    def _build_toolbar(self):
        tb = QToolBar("Main"); tb.setMovable(False)
        self.addToolBar(Qt.TopToolBarArea, tb)
        self.sim_action = QAction("\u25b6 En-Route Sim", self)
        self.sim_action.triggered.connect(self._toggle_simulation)
        tb.addAction(self.sim_action)
        tb.addSeparator()
        self.asracs_action = QAction("\u25b6 ASRACS Sim", self)
        self.asracs_action.triggered.connect(self._toggle_asracs)
        tb.addAction(self.asracs_action)
        tb.addSeparator()
        ra = QAction("\u21ba Reset 3D", self)
        ra.triggered.connect(lambda: self.terrain3d.reset_view())
        tb.addAction(ra)
        tb.addSeparator()
        da = QAction("\u23cf Disconnect", self)
        da.triggered.connect(self.disconnect)
        tb.addAction(da)

    # ── UI  ── FIX 2+3: _populate_vor_combo called here, AFTER all tabs ──
    def _build_ui(self):
        cw = QWidget(); self.setCentralWidget(cw)
        ml = QVBoxLayout(cw)
        self.tabs = QTabWidget()

        # Build every tab (creates self.radar, self.terrain3d, etc.)
        self.tabs.addTab(self._tab_vor(),         "VOR Monitor")
        self.tabs.addTab(self._tab_radar(),       "Aircraft Tracking")
        self.tabs.addTab(self._tab_approach(),    "Approach Guidance")
        self.tabs.addTab(self._tab_terrain3d(),   "Terrain 3-D")
        self.tabs.addTab(self._tab_asracs(),      "ASRACS Surface")
        self.tabs.addTab(self._tab_saaf_bases(),  "SAAF Bases")
        self.tabs.addTab(self._tab_connection(),  "Connection")
        self.tabs.addTab(self._tab_diagnostics(), "Diagnostics")

        ml.addWidget(self.tabs)
        self.statusBar().showMessage("Ready  |  ASRACS: OFF  |  Sim: OFF")

        # ── FIX 3 ── populate AFTER all tabs (and their widgets) are built
        self._populate_vor_combo("All stations")

    # ── VOR Monitor Tab ───────────────────────────────────────────────────
    def _tab_vor(self):
        w = QWidget(); lo = QVBoxLayout(w)

        # Station selection
        sg = QGroupBox("VOR / VORTAC / NDB Station Selection")
        sl = QGridLayout(sg)
        sl.addWidget(QLabel("Station:"), 0, 0)
        self.vor_combo = QComboBox(); self.vor_combo.setMinimumWidth(440)
        sl.addWidget(self.vor_combo, 0, 1, 1, 3)
        sl.addWidget(QLabel("Filter:"), 1, 0)
        self.vor_filter = QComboBox()
        self.vor_filter.addItems([
            "All stations", "Civil only", "SAAF bases only",
            "VOR/DME", "VORTAC", "NDB",
        ])
        self.vor_filter.currentTextChanged.connect(self._on_vor_filter_changed)
        sl.addWidget(self.vor_filter, 1, 1)
        lo.addWidget(sg)

        # Navaid data display
        dg = QGroupBox("Navaid Data"); dl = QGridLayout(dg)
        def lbl(r, c, txt, val, style=''):
            dl.addWidget(QLabel(txt), r, c)
            lab = QLabel(val)
            if style: lab.setStyleSheet(style)
            dl.addWidget(lab, r, c+1)
            return lab
        self.freq_lbl    = lbl(0, 0, "Frequency:",  "--- MHz")
        self.ident_lbl   = lbl(0, 2, "Ident:",       "---")
        self.gs_info_lbl = lbl(0, 4, "Glide Slope:", "---")
        self.type_lbl    = lbl(1, 0, "Type:",         "---")
        self.ndb_lbl     = lbl(1, 2, "NDB:",          "--- kHz")
        self.ils_info_lbl= lbl(1, 4, "ILS:",          "---")
        self.bearing_lbl = lbl(2, 0, "Bearing:",     "--- \u00b0")
        self.dev_lbl     = lbl(2, 2, "Deviation:",   "--- dots")
        self.coverage_lbl= lbl(2, 4, "Coverage:",     "---")
        self.sig_lbl     = lbl(3, 0, "Signal:",      "--- dBm")
        self.health_lbl  = lbl(3, 2, "Health:",      "0%")
        self.status_lbl  = lbl(4, 0, "Status:",      "OFFLINE",
                               "color:red;font-weight:bold")
        self.remarks_lbl = lbl(4, 2, "Remarks:",     "")
        self.remarks_lbl.setWordWrap(True)
        lo.addWidget(dg)

        # Per-base simulation shortcut
        sim_grp = QGroupBox("Simulate Approach to Selected Base")
        sim_lo  = QHBoxLayout(sim_grp)
        self.base_sim_btn = QPushButton(
            "\u25b6 Simulate Selected Base Approach")
        self.base_sim_btn.setStyleSheet(
            "background:#1a4a8a;color:white;font-weight:bold;padding:6px 18px")
        self.base_sim_btn.clicked.connect(self._simulate_selected_base)
        sim_lo.addWidget(self.base_sim_btn)
        self.base_sim_lbl = QLabel(
            "Select any SAAF base in the dropdown above, then click Simulate")
        self.base_sim_lbl.setStyleSheet("color:#88aacc;font-size:10px")
        sim_lo.addWidget(self.base_sim_lbl); sim_lo.addStretch()
        lo.addWidget(sim_grp)

        cg = QGroupBox("CDI"); cl = QHBoxLayout(cg)
        self.cdi = CDIDisplay(); cl.addWidget(self.cdi)
        lo.addWidget(cg)

        chg = QGroupBox("Signal Trend"); chl = QVBoxLayout(chg)
        self.sig_chart = self._make_chart("Signal (dBm)", -80, -20)
        chl.addWidget(self.sig_chart)
        lo.addWidget(chg); lo.addStretch()

        # ── FIX 2 ── Do NOT call _populate_vor_combo here.
        #             It is called at the end of _build_ui() instead.
        return w

    # ── VOR combo helpers ─────────────────────────────────────────────────
    def _populate_vor_combo(self, filter_text: str = "All stations"):
        """
        Rebuild the VOR station combo with grouped, labelled entries.
        Safe to call at any time — guards all cross-widget accesses.
        """
        # Disconnect existing signal to prevent duplicate connections
        try:
            self.vor_combo.currentIndexChanged.disconnect()
        except Exception:
            pass

        self.vor_combo.blockSignals(True)
        prev_key = self._current_vor_key
        self.vor_combo.clear()

        civil_keys = ['JNB_VOR', 'CPT_VOR', 'DUR_VOR']
        saaf_keys  = [k for k in self.config.vor_stations
                      if k not in civil_keys]

        def _matches(st):
            t       = st.get('type', '')
            airport = st.get('airport', '')
            civil   = {'JNB', 'CPT', 'DUR'}
            if filter_text == "All stations":     return True
            if filter_text == "Civil only":       return airport in civil
            if filter_text == "SAAF bases only":  return airport not in civil
            if filter_text == "VOR/DME":          return 'VOR/DME' in t
            if filter_text == "VORTAC":           return 'VORTAC' in t
            if filter_text == "NDB":              return bool(st.get('ndb_freq'))
            return True

        def _sep(text, color_hex):
            self.vor_combo.addItem(text, userData="__sep__")
            idx  = self.vor_combo.count() - 1
            item = self.vor_combo.model().item(idx)
            if item:
                item.setFlags(item.flags() & ~Qt.ItemIsEnabled)
                item.setForeground(QColor(color_hex))

        def _add_section(keys):
            for k in keys:
                st = self.config.vor_stations.get(k)
                if not st or not _matches(st):
                    continue
                name   = st.get('name', '') or k
                ident  = st.get('ident',  '---')
                freq   = st.get('frequency', 0.0)
                vtype  = st.get('type', 'VOR')
                ndb    = st.get('ndb_freq')
                ndb_id = st.get('ndb_ident', '')
                ndb_s  = f"  NDB {ndb_id} {ndb}kHz" if ndb else ""
                label  = (f"  {ident}  {freq:.2f}MHz  [{vtype}]{ndb_s}"
                          f"  \u2014  {name}")
                self.vor_combo.addItem(label, userData=k)

        show_civil = filter_text in (
            "All stations", "Civil only", "VOR/DME", "VORTAC", "NDB")
        show_saaf  = filter_text in (
            "All stations", "SAAF bases only", "VOR/DME", "VORTAC", "NDB")

        if show_civil:
            _sep("\u2500\u2500 Civil VOR / DME "
                 "\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500"
                 "\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500", "#78b878")
            _add_section(civil_keys)
        if show_saaf:
            _sep("\u2500\u2500 SAAF Air Force Bases "
                 "\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500"
                 "\u2500\u2500\u2500\u2500", "#6496d2")
            _add_section(saaf_keys)

        # Restore previous selection or pick first selectable item
        restored = False
        for i in range(self.vor_combo.count()):
            if self.vor_combo.itemData(i) == prev_key:
                self.vor_combo.setCurrentIndex(i)
                restored = True
                break
        if not restored:
            for i in range(self.vor_combo.count()):
                if self.vor_combo.itemData(i) not in (None, "__sep__"):
                    self.vor_combo.setCurrentIndex(i)
                    break

        self.vor_combo.blockSignals(False)
        # Connect signal ONCE after fully populating
        self.vor_combo.currentIndexChanged.connect(self._on_vor_index_changed)
        # Trigger update for the currently selected station
        self._on_vor_index_changed(self.vor_combo.currentIndex())

    def _on_vor_filter_changed(self, filter_text: str):
        self._populate_vor_combo(filter_text)

    # ── FIX 1: every cross-tab widget access is guarded with hasattr() ────
    def _on_vor_index_changed(self, idx: int):
        if idx < 0:
            return
        key = self.vor_combo.itemData(idx)
        if not key or key == "__sep__":
            return
        st = self.config.vor_stations.get(key)
        if not st:
            return
        self._current_vor_key = key

        # VOR data labels — always safe (created in _tab_vor before combo)
        self.freq_lbl.setText(f"{st.get('frequency', 0):.2f} MHz")
        self.ident_lbl.setText(st.get('ident', '---'))
        self.type_lbl.setText(st.get('type', 'VOR'))
        ndb    = st.get('ndb_freq')
        ndb_id = st.get('ndb_ident', '')
        self.ndb_lbl.setText(f"{ndb_id}  {ndb} kHz" if ndb else "N/A")
        ils_runways = st.get('ils_runways', [])
        self.gs_info_lbl.setText(
            f"{STANDARD_GLIDE_SLOPE_DEG:.1f}°" if st.get('ils_available') else "N/A")
        self.ils_info_lbl.setText(_format_ils_runways(ils_runways))
        self.coverage_lbl.setText(
            f"VOR {'Yes' if st.get('vor_available', True) else 'No'}  /  "
            f"ILS {'Yes' if st.get('ils_available') else 'No'}")
        self.remarks_lbl.setText(_build_station_remarks(st))

        lat = st.get('latitude',  0.0)
        lon = st.get('longitude', 0.0)

        # Tracker — always available
        self.tracker.set_vor_position(lat, lon)

        # ── FIX 1 ── guard widgets built in later tabs ────────────────────
        if hasattr(self, 'radar'):
            self.radar.set_vor_pos(lat, lon)
        if hasattr(self, 'terrain3d'):
            self.terrain3d.set_vor_pos(lat, lon)
        if hasattr(self, 'sim_engine'):
            self.sim_engine.VOR_LAT = lat
            self.sim_engine.VOR_LON = lon
        if hasattr(self, 'rwy_combo'):
            self._sync_approach_base(st.get('airport', ''))

        # Simulation label — may not exist yet during first build pass
        if hasattr(self, 'base_sim_lbl'):
            airport = st.get('airport', '')
            if airport in SAAF_BASES:
                self.base_sim_lbl.setText(
                    f"Will simulate approaches to  "
                    f"{SAAF_BASES[airport]['name']}  "
                    f"({st.get('frequency', 0):.2f} MHz)"
                )
            else:
                self.base_sim_lbl.setText(
                    "Select a SAAF base station to simulate approaches"
                )

        logger.info(
            f"VOR selected: {key}  {st.get('name', '')}  "
            f"{st.get('frequency', 0):.2f} MHz"
        )

    def _simulate_selected_base(self):
        """Start simulation targeting the currently selected VOR airport."""
        key = self._current_vor_key
        st  = self.config.vor_stations.get(key, {})
        lat = st.get('latitude',  -25.5967)
        lon = st.get('longitude',  28.2394)
        self.sim_engine.set_target(lat, lon)
        self.tracker.clear()
        if not self._sim_active:
            self._toggle_simulation()
        else:
            self.sim_engine.reset()
        name = st.get('name', 'Selected base')
        self.statusBar().showMessage(
            f"Simulating approaches to  {name}  "
            f"{st.get('frequency', 0):.2f} MHz"
        )

    @staticmethod
    def _distance_and_bearing_nm(lat1, lon1, lat2, lon2):
        r_km = 6371.0
        la1, lo1 = math.radians(lat1), math.radians(lon1)
        la2, lo2 = math.radians(lat2), math.radians(lon2)
        dlat, dlon = la2 - la1, lo2 - lo1
        a = (math.sin(dlat/2)**2 +
             math.cos(la1) * math.cos(la2) * math.sin(dlon/2)**2)
        dist_nm = r_km * 2 * math.asin(math.sqrt(max(0, a))) / KM_PER_NM
        y = math.sin(dlon) * math.cos(la2)
        x = math.cos(la1) * math.sin(la2) - math.sin(la1) * math.cos(la2) * math.cos(dlon)
        brg = (math.degrees(math.atan2(y, x)) + 360) % 360
        return dist_nm, brg

    @staticmethod
    def _course_delta_deg(course, bearing):
        return ((bearing - course + 540) % 360) - 180

    def _selected_runway(self):
        idx = self.rwy_combo.currentIndex()
        return self.rwy_combo.itemData(idx) if idx >= 0 else None

    def _sync_approach_base(self, airport):
        self._current_base_icao = airport if airport in SAAF_BASES else ''
        if not hasattr(self, 'rwy_combo'):
            return
        self.rwy_combo.blockSignals(True)
        self.rwy_combo.clear()
        base = SAAF_BASES.get(self._current_base_icao)
        if base:
            ils_by_end = {rw.get('runway'): rw for rw in base.get('ils_runways', [])}
            for rwy in base.get('runways', []):
                ils = [end for end in rwy['designation'].split('/') if end in ils_by_end]
                if ils:
                    for ils_end in ils:
                        data = {**rwy, '_ils_end': ils_end}
                        self.rwy_combo.addItem(f"{rwy['designation']}  ·  ILS {ils_end}",
                                               userData=data)
                else:
                    self.rwy_combo.addItem(rwy['designation'], userData=dict(rwy))
        if self.rwy_combo.count() == 0:
            self.rwy_combo.addItem("---", userData=None)
        self.rwy_combo.blockSignals(False)
        self._on_runway_changed(self.rwy_combo.currentIndex())

    def _on_runway_changed(self, idx):
        rwy = self._selected_runway()
        if not rwy:
            self.rwy_hdg_lbl.setText("---")
            self._update_approach_guidance()
            return
        desigs = rwy['designation'].split('/')
        ils_end = rwy.get('_ils_end') or desigs[0]
        inbound = rwy['true_hdg_lo'] if ils_end == desigs[0] else rwy['true_hdg_hi']
        self.rwy_hdg_lbl.setText(f"{int(inbound):03d}°")
        self._update_approach_guidance()

    def _terrain_profile(self):
        if not hasattr(self, 'terrain3d'):
            return None
        row = self.terrain3d.terrain_profile()
        if row is None:
            return None
        if not self.terrain3d.GRID:
            return None
        spacing = (self.terrain3d.SIZE / self.terrain3d.GRID) * M_PER_KM
        return self.surface_slope_analyzer.analyze_profile(row, spacing)

    def _update_approach_guidance(self):
        if not hasattr(self, 'approach'):
            return
        rwy = self._selected_runway()
        ac_list = self.tracker.get_all()
        ac = ac_list[0] if ac_list else None
        if not rwy or not self._current_base_icao or not ac:
            self.gs_lbl.setText(f"{STANDARD_GLIDE_SLOPE_DEG:.1f}°")
            self.loc_lbl.setText("--- dots")
            self.drwy_lbl.setText("--- nm")
            if hasattr(self, 'slope_lbl'):
                self.slope_lbl.setText("---")
            self.approach.set_guidance({})
            return
        base = SAAF_BASES[self._current_base_icao]
        desigs = rwy['designation'].split('/')
        ils_end = rwy.get('_ils_end') or desigs[0]
        runway_label = ils_end
        use_lo = ils_end == desigs[0]
        thr_lat = rwy['thr_lo_lat'] if use_lo else rwy['thr_hi_lat']
        thr_lon = rwy['thr_lo_lon'] if use_lo else rwy['thr_hi_lon']
        inbound = rwy['true_hdg_lo'] if use_lo else rwy['true_hdg_hi']
        dist_nm, bearing = self._distance_and_bearing_nm(ac.latitude, ac.longitude, thr_lat, thr_lon)
        loc_deg = self._course_delta_deg(inbound, bearing)
        loc_scale = FULL_SCALE_LOCALIZER_DEG / MAX_LOCALIZER_DOTS
        loc_dots = max(-MAX_LOCALIZER_DOTS, min(MAX_LOCALIZER_DOTS, loc_deg / loc_scale))
        runway_elev_ft = base['elevation'] * 3.28084
        gs_error = self.glide_slope_detector.calculate_glide_slope_error(
            ac.altitude, dist_nm, runway_elev_ft)
        gs_state = "ON GS" if self.glide_slope_detector.is_on_profile(
            ac.altitude, dist_nm, runway_elev_ft) else ("HIGH" if gs_error > 0 else "LOW")
        ils_match = next((rw for rw in base.get('ils_runways', [])
                          if rw.get('runway') == ils_end), {})
        self.gs_lbl.setText(f"{STANDARD_GLIDE_SLOPE_DEG:.1f}°  ({gs_state} {gs_error:+.0f} ft)")
        self.loc_lbl.setText(f"{loc_dots:+.2f} dots")
        self.drwy_lbl.setText(f"{dist_nm:.1f} nm")
        if hasattr(self, 'slope_lbl'):
            slope = self._terrain_profile()
            self.slope_lbl.setText(
                f"{slope.get('max_slope', 0.0):.1f}% max"
                if isinstance(slope, dict) else "---")
        self.approach.set_guidance(dict(
            runway=f"RWY {runway_label}  CRS {int(inbound):03d}°",
            ils_text=("No ILS" if not ils_match else
                      f"ILS {ils_match.get('frequency', 0.0):.2f} MHz"),
            glide_text=f"GS {STANDARD_GLIDE_SLOPE_DEG:.1f}°  {gs_state} {gs_error:+.0f} ft",
            on_profile=gs_state == "ON GS",
        ))

    # ── Aircraft Tracking Tab ─────────────────────────────────────────────
    def _tab_radar(self):
        w = QWidget(); lo = QVBoxLayout(w)
        rg = QGroupBox("En-Route Radar \u2014 Realtime Aircraft Tracking")
        rl = QVBoxLayout(rg)
        self.radar = RadarDisplay(); rl.addWidget(self.radar)
        lo.addWidget(rg, 2)
        tg = QGroupBox("Tracked Aircraft"); tl = QVBoxLayout(tg)
        self.ac_table = QTableWidget(); self.ac_table.setColumnCount(9)
        self.ac_table.setHorizontalHeaderLabels([
            "ID","Lat","Lon","Alt (ft)","Hdg","Spd (kt)","Dist (nm)","Bearing","Phase"
        ])
        self.ac_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.ac_table.setAlternatingRowColors(True)
        tl.addWidget(self.ac_table); lo.addWidget(tg, 1)
        return w

    # ── Approach Guidance Tab ─────────────────────────────────────────────
    def _tab_approach(self):
        w = QWidget(); lo = QVBoxLayout(w)
        rg = QGroupBox("Runway"); rl = QGridLayout(rg)
        rl.addWidget(QLabel("Active Runway:"), 0, 0)
        self.rwy_combo = QComboBox()
        self.rwy_combo.currentIndexChanged.connect(self._on_runway_changed)
        rl.addWidget(self.rwy_combo, 0, 1)
        rl.addWidget(QLabel("Heading:"), 0, 2)
        self.rwy_hdg_lbl = QLabel("---"); rl.addWidget(self.rwy_hdg_lbl, 0, 3)
        lo.addWidget(rg)
        gg = QGroupBox("Approach Guidance"); gl = QVBoxLayout(gg)
        self.approach = ApproachGuidanceDisplay(); gl.addWidget(self.approach)
        lo.addWidget(gg, 2)
        pg = QGroupBox("Guidance Parameters"); pl = QGridLayout(pg)
        pl.addWidget(QLabel("Glide Slope:"), 0, 0)
        self.gs_lbl = QLabel("3.0\u00b0"); pl.addWidget(self.gs_lbl, 0, 1)
        pl.addWidget(QLabel("Localizer:"), 1, 0)
        self.loc_lbl = QLabel("0 dots"); pl.addWidget(self.loc_lbl, 1, 1)
        pl.addWidget(QLabel("Dist to RWY:"), 0, 2)
        self.drwy_lbl = QLabel("--- nm"); pl.addWidget(self.drwy_lbl, 0, 3)
        pl.addWidget(QLabel("Terrain Slope:"), 1, 2)
        self.slope_lbl = QLabel("---"); pl.addWidget(self.slope_lbl, 1, 3)
        lo.addWidget(pg); lo.addStretch()
        return w

    # ── 3-D Terrain Tab ───────────────────────────────────────────────────
    def _tab_terrain3d(self):
        w = QWidget(); lo = QVBoxLayout(w)
        ctrl = QHBoxLayout()
        self.sim_btn_t = QPushButton("\u25b6 En-Route Sim")
        self.sim_btn_t.setStyleSheet(
            "background:#1a7a3a;color:white;font-weight:bold;padding:5px 14px")
        self.sim_btn_t.clicked.connect(self._toggle_simulation)
        ctrl.addWidget(self.sim_btn_t)
        rb = QPushButton("\u21ba Reset Camera")
        rb.clicked.connect(lambda: self.terrain3d.reset_view())
        rb.setStyleSheet("padding:5px 14px"); ctrl.addWidget(rb)
        ctrl.addWidget(QLabel("  Left-drag=rotate  Scroll=zoom  Slope shading ON"))
        ctrl.addStretch(); lo.addLayout(ctrl)
        tg = QGroupBox("3-D Terrain with Slope Shading"); tl = QVBoxLayout(tg)
        self.terrain3d = Terrain3DWidget(); tl.addWidget(self.terrain3d)
        lo.addWidget(tg, 3)
        og = QGroupBox("Nearby Obstacles"); ol = QVBoxLayout(og)
        self.obs_table = QTableWidget(); self.obs_table.setColumnCount(4)
        self.obs_table.setHorizontalHeaderLabels(
            ["Type","Distance (nm)","Height (ft)","Bearing"])
        self.obs_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        for r,(t,d,h,b) in enumerate([
            ("Tower",    "8.2",  "1450", "042\u00b0"),
            ("Antenna",  "12.5", "980",  "117\u00b0"),
            ("Hill",     "18.0", "4800", "255\u00b0"),
            ("Building", "5.1",  "650",  "330\u00b0"),
        ]):
            self.obs_table.insertRow(r)
            for c,v in enumerate([t,d,h,b]):
                self.obs_table.setItem(r, c, QTableWidgetItem(v))
        ol.addWidget(self.obs_table); lo.addWidget(og, 1)
        return w

    # ── ASRACS Surface Tab ────────────────────────────────────────────────
    def _tab_asracs(self):
        w = QWidget(); lo = QVBoxLayout(w)
        ctrl = QHBoxLayout()
        self.asracs_btn = QPushButton("\u25b6 Start ASRACS Simulation")
        self.asracs_btn.setStyleSheet(
            "background:#1a3a7a;color:white;font-weight:bold;padding:5px 16px;font-size:11px")
        self.asracs_btn.clicked.connect(self._toggle_asracs)
        ctrl.addWidget(self.asracs_btn)
        rv = QPushButton("\u21ba Reset View"); rv.setStyleSheet("padding:5px 12px")
        rv.clicked.connect(lambda: self.asracs_display.reset_view())
        ctrl.addWidget(rv)
        ctrl.addWidget(QLabel("  Left-drag=pan  Scroll=zoom"))
        self.asracs_count_lbl = QLabel("Targets: 0  |  Alerts: 0")
        self.asracs_count_lbl.setStyleSheet("color:#aaffaa;font-weight:bold")
        ctrl.addWidget(self.asracs_count_lbl); ctrl.addStretch()
        lo.addLayout(ctrl)
        splitter = QSplitter(Qt.Horizontal)
        surf_grp = QGroupBox("Surface Movement \u2014 FAOR")
        surf_lo  = QVBoxLayout(surf_grp)
        self.asracs_display = ASRACSDisplay(); surf_lo.addWidget(self.asracs_display)
        splitter.addWidget(surf_grp)
        right = QWidget(); right_lo = QVBoxLayout(right); right_lo.setContentsMargins(0,0,0,0)
        tg = QGroupBox("Ground Targets"); tl = QVBoxLayout(tg)
        self.asracs_table = QTableWidget(); self.asracs_table.setColumnCount(8)
        self.asracs_table.setHorizontalHeaderLabels(
            ["ID","Type","X(m)","Y(m)","Hdg","Spd(kt)","Squawk","Status"])
        self.asracs_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.asracs_table.setAlternatingRowColors(True)
        self.asracs_table.setEditTriggers(QTableWidget.NoEditTriggers)
        tl.addWidget(self.asracs_table); right_lo.addWidget(tg, 2)
        ag = QGroupBox("Runway Incursion Alerts"); al = QVBoxLayout(ag)
        self.alert_panel = ASRACSAlertPanel(); al.addWidget(self.alert_panel)
        right_lo.addWidget(ag, 1)
        splitter.addWidget(right); splitter.setSizes([700,400])
        lo.addWidget(splitter, 1)
        self.asracs_status = QLabel("ASRACS OFF")
        self.asracs_status.setStyleSheet(
            "background:#111;color:#888;padding:3px 6px;font-size:10px")
        lo.addWidget(self.asracs_status)
        return w

    # ── SAAF Bases Tab ────────────────────────────────────────────────────
    def _tab_saaf_bases(self):
        w = QWidget(); lo = QVBoxLayout(w)
        hdr = QLabel(
            "South African Air Force Bases \u2014 Runway Matrices & Layout Maps")
        hdr.setStyleSheet(
            "font-size:13px;font-weight:bold;color:#aaffaa;padding:6px;background:#0a1a0a")
        lo.addWidget(hdr)
        main_sp = QSplitter(Qt.Horizontal)
        left = QWidget(); left_lo = QVBoxLayout(left); left_lo.setContentsMargins(0,0,0,0)
        mg = QGroupBox("National Overview (click a base)"); ml = QVBoxLayout(mg)
        self.saaf_overview = SAAFOverviewMap()
        self.saaf_overview.base_selected.connect(self._on_saaf_base_selected)
        ml.addWidget(self.saaf_overview); left_lo.addWidget(mg, 2)
        lg = QGroupBox("Base Directory"); ll = QVBoxLayout(lg)
        self.saaf_list = QListWidget()
        for icao,base in SAAF_BASES.items():
            vs = next((v for v in self.config.vor_stations.values()
                       if v.get('airport') == icao), None)
            freq_str = f"  {vs['frequency']:.2f}MHz" if vs else ""
            item = QListWidgetItem(
                f"  {icao}{freq_str}  \u2014  {base['name']}  ({base['province']})")
            item.setData(Qt.UserRole, icao)
            item.setForeground(QColor(180,255,180))
            self.saaf_list.addItem(item)
        self.saaf_list.currentItemChanged.connect(self._on_saaf_list_changed)
        ll.addWidget(self.saaf_list); left_lo.addWidget(lg, 1)
        main_sp.addWidget(left)
        right = QWidget(); right_lo = QVBoxLayout(right); right_lo.setContentsMargins(0,0,0,0)
        right_sp = QSplitter(Qt.Vertical)
        mg2 = QGroupBox("Base Layout Map (scroll=zoom, drag=pan)")
        ml2 = QVBoxLayout(mg2)
        self.saaf_base_map = SAAFBaseMapWidget(); ml2.addWidget(self.saaf_base_map)
        right_sp.addWidget(mg2)
        ig = QGroupBox("Base Data & Runway Matrix"); il = QVBoxLayout(ig)
        scroll = QScrollArea(); scroll.setWidgetResizable(True)
        self.saaf_base_info = SAAFBaseInfoWidget(); scroll.setWidget(self.saaf_base_info)
        il.addWidget(scroll); right_sp.addWidget(ig)
        right_sp.setSizes([420,380])
        right_lo.addWidget(right_sp); main_sp.addWidget(right)
        main_sp.setSizes([380,900]); lo.addWidget(main_sp, 1)
        if self.saaf_list.count() > 0:
            self.saaf_list.setCurrentRow(0)
        return w

    def _on_saaf_base_selected(self, icao):
        for i in range(self.saaf_list.count()):
            item = self.saaf_list.item(i)
            if item.data(Qt.UserRole) == icao:
                self.saaf_list.setCurrentItem(item); break

    def _on_saaf_list_changed(self, current, previous):
        if not current: return
        icao = current.data(Qt.UserRole)
        if icao and icao in SAAF_BASES:
            base = SAAF_BASES[icao]
            self.saaf_base_map.set_base(base)
            self.saaf_base_info.show_base(icao, base)
            self.saaf_overview._sel = icao; self.saaf_overview.update()

    # ── Connection Tab ────────────────────────────────────────────────────
    def _tab_connection(self):
        w = QWidget(); lo = QVBoxLayout(w)
        mg = QGroupBox("Built-in Mock VOR Server")
        mg.setStyleSheet("QGroupBox{font-weight:bold;color:#005a9e}")
        ml2 = QGridLayout(mg)
        ml2.addWidget(QLabel("Bind:"), 0, 0)
        self.mock_host = QLineEdit("127.0.0.1"); self.mock_host.setFixedWidth(120)
        ml2.addWidget(self.mock_host, 0, 1)
        ml2.addWidget(QLabel("Port:"), 0, 2)
        self.mock_port = QLineEdit("5000"); self.mock_port.setFixedWidth(60)
        ml2.addWidget(self.mock_port, 0, 3)
        self.mock_start = QPushButton("\u25b6 Start")
        self.mock_start.setStyleSheet("background:#28a745;color:white;font-weight:bold")
        self.mock_start.clicked.connect(self._start_mock); ml2.addWidget(self.mock_start, 0, 4)
        self.mock_stop = QPushButton("\u25a0 Stop")
        self.mock_stop.setStyleSheet("background:#dc3545;color:white;font-weight:bold")
        self.mock_stop.clicked.connect(self._stop_mock); self.mock_stop.setEnabled(False)
        ml2.addWidget(self.mock_stop, 0, 5)
        self.mock_status = QLabel("\u25cf Server not running")
        self.mock_status.setStyleSheet("color:gray;font-style:italic")
        ml2.addWidget(self.mock_status, 1, 0, 1, 6); lo.addWidget(mg)

        sim_g = QGroupBox("Simulations"); sim_lo = QHBoxLayout(sim_g)
        self.sim_btn_c = QPushButton("\u25b6 En-Route Sim")
        self.sim_btn_c.setStyleSheet(
            "background:#1a7a3a;color:white;font-weight:bold;padding:6px 14px")
        self.sim_btn_c.clicked.connect(self._toggle_simulation)
        sim_lo.addWidget(self.sim_btn_c)
        self.asracs_btn_c = QPushButton("\u25b6 ASRACS Sim")
        self.asracs_btn_c.setStyleSheet(
            "background:#1a3a7a;color:white;font-weight:bold;padding:6px 14px")
        self.asracs_btn_c.clicked.connect(self._toggle_asracs)
        sim_lo.addWidget(self.asracs_btn_c); sim_lo.addStretch()
        lo.addWidget(sim_g)

        sg = QGroupBox("Serial Connection (RS-232)"); sl = QGridLayout(sg)
        sl.addWidget(QLabel("Port:"), 0, 0)
        self.serial_combo = QComboBox(); sl.addWidget(self.serial_combo, 0, 1)
        self.port_desc_lbl = QLabel("")
        self.port_desc_lbl.setStyleSheet("color:gray;font-size:10px")
        sl.addWidget(self.port_desc_lbl, 0, 2, 1, 2)
        self.refresh_btn = QPushButton("\u27f3 Refresh")
        self.refresh_btn.clicked.connect(lambda: self._refresh_ports(True))
        sl.addWidget(self.refresh_btn, 0, 4)
        sl.addWidget(QLabel("Baud:"), 1, 0)
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["9600","19200","38400","115200"])
        sl.addWidget(self.baud_combo, 1, 1)
        self.serial_dot = QLabel("\u25cf")
        self.serial_dot.setStyleSheet("color:gray;font-size:18px")
        sl.addWidget(self.serial_dot, 1, 2)
        self.conn_serial_btn = QPushButton("Connect (Serial)")
        self.conn_serial_btn.clicked.connect(self._connect_serial)
        sl.addWidget(self.conn_serial_btn, 1, 3); lo.addWidget(sg)

        tg = QGroupBox("TCP/IP Connection"); tl = QGridLayout(tg)
        tl.addWidget(QLabel("Host:"), 0, 0)
        self.tcp_host = QLineEdit("127.0.0.1"); tl.addWidget(self.tcp_host, 0, 1)
        tl.addWidget(QLabel("Port:"), 0, 2)
        self.tcp_port = QLineEdit("5000"); tl.addWidget(self.tcp_port, 0, 3)
        self.conn_tcp_btn = QPushButton("Connect (TCP)")
        self.conn_tcp_btn.clicked.connect(self._connect_tcp)
        tl.addWidget(self.conn_tcp_btn, 0, 4); lo.addWidget(tg)

        sg2 = QGroupBox("Connection Status"); sl2 = QHBoxLayout(sg2)
        self.conn_status_lbl = QLabel("DISCONNECTED")
        self.conn_status_lbl.setStyleSheet(
            "color:red;font-weight:bold;font-size:14px")
        sl2.addWidget(self.conn_status_lbl)
        self.disc_btn = QPushButton("Disconnect")
        self.disc_btn.clicked.connect(self.disconnect); self.disc_btn.setEnabled(False)
        sl2.addWidget(self.disc_btn); lo.addWidget(sg2)

        teg = QGroupBox("Test Commands"); tel = QVBoxLayout(teg); br = QHBoxLayout()
        for lb2,cb in [("Query Status",self._cmd_status),
                       ("Query Ident", self._cmd_ident),
                       ("Query Health",self._cmd_health)]:
            b = QPushButton(lb2); b.clicked.connect(cb); br.addWidget(b)
        tel.addLayout(br)
        self.test_out = QTextEdit(); self.test_out.setReadOnly(True)
        self.test_out.setMaximumHeight(160); tel.addWidget(self.test_out)
        lo.addWidget(teg); lo.addStretch()
        self._refresh_ports(True)
        return w

    # ── Diagnostics Tab ───────────────────────────────────────────────────
    def _tab_diagnostics(self):
        w = QWidget(); lo = QVBoxLayout(w)
        sg = QGroupBox("System Status"); sl = QGridLayout(sg)
        self.gps_lbl   = QLabel("No Fix"); self.gps_lbl.setStyleSheet("color:red")
        self.tsync_lbl = QLabel("Synced"); self.tsync_lbl.setStyleSheet("color:green")
        self.mem_lbl   = QLabel("--- MB"); self.cpu_lbl = QLabel("--- %")
        for r,c,lab,wid in [(0,0,"GPS:",self.gps_lbl),(1,0,"Time Sync:",self.tsync_lbl),
                             (0,2,"Memory:",self.mem_lbl),(1,2,"CPU:",self.cpu_lbl)]:
            sl.addWidget(QLabel(lab),r,c); sl.addWidget(wid,r,c+1)
        lo.addWidget(sg)
        mg = QGroupBox("Performance"); mlo = QVBoxLayout(mg)
        self.metrics_chart = self._make_chart("Load (%)",0,100)
        mlo.addWidget(self.metrics_chart); lo.addWidget(mg)
        lg = QGroupBox("Event Log"); ll = QVBoxLayout(lg)
        self.event_log = QTextEdit(); self.event_log.setReadOnly(True)
        ll.addWidget(self.event_log); lo.addWidget(lg)
        return w

    def _make_chart(self,title,ymin,ymax):
        s = QLineSeries(); s.setName(title)
        for i in range(30): s.append(i,random.uniform(ymin*0.7,ymax*0.9))
        ch = QChart(); ch.addSeries(s); ch.setTitle(title)
        ch.createDefaultAxes(); ch.setAnimationOptions(QChart.NoAnimation)
        v = QChartView(ch); v.setRenderHint(QPainter.Antialiasing)
        v.setMaximumHeight(180); return v

    # ── Menus ─────────────────────────────────────────────────────────────
    def _build_menus(self):
        mb = self.menuBar()
        fm = mb.addMenu("File")
        for lb,cb in [("Export Data",self._export),("Import Config",self._import)]:
            a = QAction(lb,self); a.triggered.connect(cb); fm.addAction(a)
        fm.addSeparator()
        ex = QAction("Exit",self); ex.triggered.connect(self.close); fm.addAction(ex)
        tm = mb.addMenu("Tools")
        for lb,cb in [("Settings",self._settings),("Calibrate VOR",self._calibrate)]:
            a = QAction(lb,self); a.triggered.connect(cb); tm.addAction(a)
        hm = mb.addMenu("Help")
        ab = QAction("About",self); ab.triggered.connect(self._about); hm.addAction(ab)

    # ── COM Port ──────────────────────────────────────────────────────────
    def _auto_refresh_ports(self): self._refresh_ports(False)
    def _refresh_ports(self,force=True):
        ports = VORConnectionHandler.list_ports()
        keys  = [p['device'] for p in ports]
        if not force and keys == self._last_ports: return
        self._last_ports = keys
        prev = self.serial_combo.currentData()
        self.serial_combo.blockSignals(True); self.serial_combo.clear()
        if ports:
            for p in ports:
                self.serial_combo.addItem(
                    f"{p['device']}  \u2014  {p['description']}",
                    userData=p['device'])
            for i in range(self.serial_combo.count()):
                if self.serial_combo.itemData(i) == prev:
                    self.serial_combo.setCurrentIndex(i); break
            self.serial_combo.setStyleSheet("color:white")
            self.serial_dot.setStyleSheet("color:#28a745;font-size:18px")
            self.serial_dot.setToolTip(f"{len(ports)} port(s)")
        else:
            self.serial_combo.addItem("No serial ports detected", userData="")
            self.serial_combo.setStyleSheet("color:gray")
            self.serial_dot.setStyleSheet("color:gray;font-size:18px")
            self.serial_dot.setToolTip("No ports")
        self.serial_combo.blockSignals(False)
        idx = self.serial_combo.currentIndex()
        if idx >= 0:
            dev   = self.serial_combo.itemData(idx) or ""
            ports2= VORConnectionHandler.list_ports()
            desc  = next((p['description'] for p in ports2
                          if p['device'] == dev), "")
            self.port_desc_lbl.setText(desc if desc != dev else "")

    # ── Simulations ───────────────────────────────────────────────────────
    def _toggle_simulation(self):
        self._sim_active = not self._sim_active
        if self._sim_active:
            self.sim_engine.reset(); self._sim_timer.start(1000)
            lbl = "\u25a0 Stop En-Route Sim"
            sty = "background:#c0392b;color:white;font-weight:bold;padding:5px 14px"
            self.statusBar().showMessage("En-Route Sim RUNNING")
        else:
            self._sim_timer.stop()
            lbl = "\u25b6 En-Route Sim"
            sty = "background:#1a7a3a;color:white;font-weight:bold;padding:5px 14px"
            self.statusBar().showMessage("En-Route Sim STOPPED")
        self.sim_action.setText(lbl)
        self.sim_btn_t.setText(lbl); self.sim_btn_t.setStyleSheet(sty)
        self.sim_btn_c.setText(lbl); self.sim_btn_c.setStyleSheet(sty)

    def _sim_tick(self):
        self.sim_engine.tick(self.tracker)
        self._update_aircraft_displays()
        self.terrain3d.set_aircraft(self.tracker.get_all())

    def _toggle_asracs(self):
        self._asracs_active = not self._asracs_active
        if self._asracs_active:
            self.asracs_engine.reset(); self._asracs_timer.start(1000)
            lbl = "\u25a0 Stop ASRACS Sim"
            sty = "background:#8b0000;color:white;font-weight:bold;padding:5px 16px;font-size:11px"
            self.asracs_status.setText("ASRACS RUNNING \u2014 FAOR Surface Simulation Active")
            self.asracs_status.setStyleSheet(
                "background:#001a00;color:#00ff80;padding:3px 6px;font-size:10px")
        else:
            self._asracs_timer.stop()
            lbl = "\u25b6 Start ASRACS Simulation"
            sty = "background:#1a3a7a;color:white;font-weight:bold;padding:5px 16px;font-size:11px"
            self.asracs_status.setText("ASRACS OFF")
            self.asracs_status.setStyleSheet(
                "background:#111;color:#888;padding:3px 6px;font-size:10px")
        self.asracs_btn.setText(lbl); self.asracs_btn.setStyleSheet(sty)
        self.asracs_action.setText(lbl); self.asracs_btn_c.setText(lbl)

    def _asracs_tick(self):
        targets = self.asracs_engine.tick()
        self.asracs_display.set_targets(targets)
        self._update_asracs_table(targets)
        self.alert_panel.update_alerts(self.asracs_engine.alerts)
        n = len([a for a in self.asracs_engine.alerts if not a.acknowledged])
        self.asracs_count_lbl.setText(f"Targets: {len(targets)}  |  Alerts: {n}")
        self.asracs_count_lbl.setStyleSheet(
            "color:#ff4444;font-weight:bold" if n > 0
            else "color:#aaffaa;font-weight:bold")

    def _update_asracs_table(self,targets):
        self.asracs_table.setRowCount(0)
        SC = {
            GroundTarget.STATUS_OK:       QColor(180,255,200),
            IncursionAlert.SEV_CAUTION:   QColor(255,220,80),
            IncursionAlert.SEV_WARNING:   QColor(255,140,0),
            IncursionAlert.SEV_CRITICAL:  QColor(255,50,50),
        }
        for t in targets.values():
            r = self.asracs_table.rowCount(); self.asracs_table.insertRow(r)
            col = SC.get(t.status, QColor(180,255,200))
            for c,v in enumerate([
                t.target_id, t.target_type,
                f"{t.x:.0f}", f"{t.y:.0f}",
                f"{t.heading:.1f}\u00b0", f"{t.speed:.1f}",
                t.squawk, t.status,
            ]):
                item = QTableWidgetItem(v); item.setForeground(col)
                self.asracs_table.setItem(r,c,item)

    # ── Mock Server ───────────────────────────────────────────────────────
    def _start_mock(self):
        host = self.mock_host.text().strip() or '127.0.0.1'
        try: port = int(self.mock_port.text())
        except ValueError:
            QMessageBox.warning(self,"Invalid Port","Enter a valid port number.")
            return
        self.mock_server = MockVORTCPServer(host, port)
        if self.mock_server.start():
            self.mock_status.setText(f"\u25cf Running on {host}:{port}")
            self.mock_status.setStyleSheet("color:#28a745;font-weight:bold")
            self.mock_start.setEnabled(False); self.mock_stop.setEnabled(True)
            self.mock_host.setEnabled(False);  self.mock_port.setEnabled(False)
            self.tcp_host.setText('127.0.0.1'); self.tcp_port.setText(str(port))
            self.test_out.append(f"[{_ts()}] Mock server started on {host}:{port}")
        else:
            self.mock_server = None
            QMessageBox.critical(self,"Mock Server Error",
                                 f"Cannot bind to {host}:{port}.")

    def _stop_mock(self):
        if self.mock_server: self.mock_server.stop(); self.mock_server = None
        self.mock_status.setText("\u25cf Server not running")
        self.mock_status.setStyleSheet("color:gray;font-style:italic")
        self.mock_start.setEnabled(True); self.mock_stop.setEnabled(False)
        self.mock_host.setEnabled(True);  self.mock_port.setEnabled(True)
        self.test_out.append(f"[{_ts()}] Mock server stopped.")

    # ── Connection ────────────────────────────────────────────────────────
    def _connect_serial(self):
        idx = self.serial_combo.currentIndex()
        dev = self.serial_combo.itemData(idx) if idx >= 0 else ""
        if not dev:
            QMessageBox.warning(self,"No Port","No valid serial port selected.")
            return
        baud = int(self.baud_combo.currentText())
        if self.connection.connect_serial(dev, baud):
            self._set_connected(True,"SERIAL"); self._start_daq()
        else:
            QMessageBox.critical(self,"Error",f"Cannot open {dev}")

    def _connect_tcp(self):
        host = self.tcp_host.text().strip()
        if not host:
            QMessageBox.warning(self,"No Host","Enter a host address."); return
        try: port = int(self.tcp_port.text())
        except ValueError:
            QMessageBox.warning(self,"Invalid Port","Enter a valid port number.")
            return
        if self.connection.connect_tcp(host, port):
            self._set_connected(True,"TCP"); self._start_daq()
        else:
            hint = ("Mock server not running.\nClick '\u25b6 Start' first."
                    if host in ('localhost','127.0.0.1')
                    else f"Cannot reach {host}:{port}.")
            QMessageBox.critical(self,"Connection Refused",
                                 f"TCP {host}:{port} refused.\n\n{hint}")

    def disconnect(self):
        if self.data_thread:
            self.data_thread.stop(); self.data_thread.wait()
        self.connection.disconnect(); self._set_connected(False)

    def _set_connected(self,connected,ctype=None):
        if connected:
            self.conn_status_lbl.setText(f"CONNECTED ({ctype})")
            self.conn_status_lbl.setStyleSheet(
                "color:green;font-weight:bold;font-size:14px")
            self.disc_btn.setEnabled(True)
            self.conn_serial_btn.setEnabled(False)
            self.conn_tcp_btn.setEnabled(False)
        else:
            self.conn_status_lbl.setText("DISCONNECTED")
            self.conn_status_lbl.setStyleSheet(
                "color:red;font-weight:bold;font-size:14px")
            self.disc_btn.setEnabled(False)
            self.conn_serial_btn.setEnabled(True)
            self.conn_tcp_btn.setEnabled(True)
            self.status_lbl.setText("OFFLINE")
            self.status_lbl.setStyleSheet("color:red;font-weight:bold")

    def _start_daq(self):
        if not self.data_thread or not self.data_thread.isRunning():
            self.data_thread = DataAcquisitionThread(self.connection)
            self.data_thread.data_updated.connect(self._on_data)
            self.data_thread.error_occurred.connect(self._on_err)
            self.data_thread.start()

    # ── Data Handlers ─────────────────────────────────────────────────────
    def _on_data(self,data):
        try:
            if data['type'] == 'vor_data':
                vd = data['data']
                self.bearing_lbl.setText(f"{vd.get('bearing',0):.1f} \u00b0")
                self.dev_lbl.setText(f"{vd.get('deviation',0):.2f} dots")
                self.sig_lbl.setText(f"{vd.get('signal_strength',0):.1f} dBm")
                st = vd.get('status','OFFLINE'); self.status_lbl.setText(st)
                self.status_lbl.setStyleSheet(
                    "color:green;font-weight:bold" if st=="OK"
                    else "color:red;font-weight:bold")
                self.health_lbl.setText(f"{vd.get('health',0)}%")
                self.cdi.set_deviation(vd.get('deviation',0))
        except Exception as e: logger.error(f"Data:{e}")

    def _on_err(self,msg):
        logger.error(f"DAQ:{msg}"); QMessageBox.warning(self,"DAQ Error",msg)

    def _update_aircraft_displays(self):
        ac_list = self.tracker.get_all()
        self.radar.set_aircraft(ac_list)
        self.terrain3d.set_aircraft(ac_list)
        self.ac_table.setRowCount(len(ac_list))
        for r,ac in enumerate(ac_list):
            for c,v in enumerate([
                ac.aircraft_id, f"{ac.latitude:.4f}", f"{ac.longitude:.4f}",
                f"{int(ac.altitude)}", f"{ac.heading:.1f}\u00b0", f"{int(ac.speed)}",
                f"{ac.distance_from_vor:.1f}", f"{ac.vor_bearing:.1f}\u00b0",
                ac.flight_phase,
            ]):
                self.ac_table.setItem(r,c,QTableWidgetItem(v))
        if ac_list: self.approach.set_aircraft(ac_list[0])
        self._update_approach_guidance()

    def _update_displays(self):
        try:
            self.radar.update(); self.terrain3d.update()
            if not self._sim_active:
                self._update_aircraft_displays()
        except Exception as e: logger.error(f"Update:{e}")

    # ── Test Commands ─────────────────────────────────────────────────────
    def _cmd_status(self):
        r=self.connection.send_command("STATUS")
        self.test_out.append(f"[{_ts()}] STATUS: {r}")
    def _cmd_ident(self):
        r=self.connection.send_command("IDENT")
        self.test_out.append(f"[{_ts()}] IDENT: {r}")
    def _cmd_health(self):
        r=self.connection.send_command("HEALTH")
        self.test_out.append(f"[{_ts()}] HEALTH: {r}")

    # ── File / Menu ───────────────────────────────────────────────────────
    def _export(self):
        fn,_ = QFileDialog.getSaveFileName(self,"Export","","CSV Files (*.csv)")
        if fn:
            try:
                with open(fn,'w',newline='') as f:
                    wrt = csv.writer(f)
                    wrt.writerow(['Timestamp','ID','Lat','Lon','Alt','Hdg','Spd'])
                    for ac in self.tracker.get_all():
                        wrt.writerow([ac.timestamp.isoformat(),ac.aircraft_id,
                                      ac.latitude,ac.longitude,
                                      ac.altitude,ac.heading,ac.speed])
                QMessageBox.information(self,"Exported",f"Saved to {fn}")
            except Exception as e:
                QMessageBox.critical(self,"Error",str(e))

    def _import(self):
        fn,_ = QFileDialog.getOpenFileName(
            self,"Import Config","","YAML (*.yaml)")
        if fn:
            try:
                with open(fn) as f: cfg = yaml.safe_load(f)
                self.config.airports      = cfg.get('airports', {})
                self.config.vor_stations  = cfg.get('vor_stations', {})
                try: self.vor_combo.currentIndexChanged.disconnect()
                except: pass
                self._populate_vor_combo("All stations")
                QMessageBox.information(self,"Imported","Configuration loaded.")
            except Exception as e:
                QMessageBox.critical(self,"Error",str(e))

    def _settings(self):
        d = QDialog(self); d.setWindowTitle("VOR Stations")
        d.setGeometry(200,200,700,420); lo = QVBoxLayout(d)
        tbl = QTableWidget(); tbl.setColumnCount(5)
        tbl.setHorizontalHeaderLabels(["Key","Ident","Freq (MHz)","Type","Name"])
        tbl.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        for key,v in self.config.vor_stations.items():
            r = tbl.rowCount(); tbl.insertRow(r)
            for c,val in enumerate([
                key, v.get('ident',''),
                f"{v.get('frequency',0):.2f}",
                v.get('type','VOR'), v.get('name',''),
            ]):
                tbl.setItem(r,c,QTableWidgetItem(val))
        lo.addWidget(tbl)
        btn = QPushButton("Close"); btn.clicked.connect(d.accept); lo.addWidget(btn)
        d.exec_()

    def _calibrate(self):
        r = self.connection.send_command("CALIBRATE")
        (QMessageBox.information if r else QMessageBox.warning)(
            self,"Calibration",r or "No response")

    def _about(self):
        QMessageBox.about(self,"About",
            "VOR / ASRACS / SAAF Airport Monitoring System v5.2\n\n"
            "\u2022 All 10 SAAF bases include VOR / ILS metadata\n"
            "\u2022 Glide slope detection in approach guidance\n"
            "\u2022 Surface slope analysis tied to 3-D terrain\n"
            "\u2022 Realtime radar: sweep, trails, velocity leader, callouts\n"
            "\u2022 Per-base SAAF approach simulation\n"
            "\u2022 Terrain slope shading in 3-D OpenGL\n"
            "\u2022 ASRACS surface movement (FAOR)\n\n"
            "\u00a9 2026 Aviation Systems")

    def closeEvent(self,event):
        self._sim_timer.stop(); self._asracs_timer.stop(); self._port_timer.stop()
        if self.data_thread and self.data_thread.isRunning():
            self.data_thread.stop(); self.data_thread.wait()
        self.connection.disconnect()
        if self.mock_server and self.mock_server.is_running:
            self.mock_server.stop()
        logger.info("System closed"); event.accept()


# ===========================================================================
# Utility
# ===========================================================================
def _ts(): return datetime.now().strftime('%H:%M:%S')


# ===========================================================================
# Entry Point
# ===========================================================================
def main():
    app = QApplication(sys.argv); app.setStyle('Fusion')
    pal = QPalette()
    pal.setColor(QPalette.Window,          QColor(45,45,48))
    pal.setColor(QPalette.WindowText,      QColor(220,220,220))
    pal.setColor(QPalette.Base,            QColor(30,30,32))
    pal.setColor(QPalette.AlternateBase,   QColor(53,53,56))
    pal.setColor(QPalette.Text,            QColor(220,220,220))
    pal.setColor(QPalette.Button,          QColor(60,63,65))
    pal.setColor(QPalette.ButtonText,      QColor(220,220,220))
    pal.setColor(QPalette.Highlight,       QColor(0,120,215))
    pal.setColor(QPalette.HighlightedText, QColor(255,255,255))
    app.setPalette(pal)
    w = VORAirportMonitorApp(); w.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()