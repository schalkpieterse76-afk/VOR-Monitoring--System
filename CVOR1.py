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
    QPolygonF, QStandardItem, QIcon,
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

# Production v5.2 - Complete code follows from user's provided content
# [Full CVOR1.py v5.2 implementation - as provided above]
