================================================================================
  VOR / ASRACS / SAAF Airport Monitoring System v5.2
  South African Air Force (SAAF) Aviation Monitoring Platform
================================================================================

OVERVIEW
--------
This is a comprehensive aviation monitoring system designed for South African
Air Force (SAAF) bases and civilian airports. It provides real-time aircraft
tracking, approach guidance, surface movement monitoring, and VOR/ILS navaid
management across 10 SAAF bases plus 3 major civilian airports.

KEY FEATURES
------------

1. VOR MONITORING & ILS GUIDANCE
   - Real-time VOR station tracking for all 13 monitored airports
   - ILS approach guidance with glide slope detection
   - Automatic glide slope error calculation
   - Coverage: 8 bases with ILS, 2 with VOR-only, 3 civilian terminals

2. EN-ROUTE AIRCRAFT TRACKING
   - Animated radar display with sweep line
   - Aircraft track history with velocity vectors
   - Real-time position/altitude/heading display
   - Multi-aircraft monitoring (5+ simultaneous)

3. APPROACH GUIDANCE
   - Runway selection per base
   - Localizer/glide slope display
   - Terrain slope analysis
   - Distance-to-runway calculation
   - Standard 3.0° glide slope reference

4. 3-D TERRAIN VISUALIZATION
   - OpenGL-based slope-shaded terrain
   - Aircraft position overlay
   - Waypoint markers
   - Interactive pan/zoom/rotate controls

5. ASRACS SURFACE MOVEMENT
   - OR Tambo (FAOR) surface simulation
   - Runway incursion detection
   - Ground target tracking (5 aircraft + 5 vehicles)
   - Alert severity levels: CAUTION, WARNING, CRITICAL

6. SAAF BASE MANAGEMENT
   - Detailed runway matrices for 10 bases
   - Squadron/unit assignments
   - Navaid frequencies and ILS data
   - Interactive base layout maps

7. CONNECTION MANAGEMENT
   - Serial RS-232 VOR receiver support
   - TCP/IP mock VOR server (for testing)
   - Real-time data acquisition thread
   - Health status monitoring

SAAF BASES MONITORED
--------------------

WITH ILS (8 bases):
  ✓ FAWK - Waterkloof (WKV 116.90 MHz) - Strategic transport HQ
  ✓ FALM - Makhado (LTV 115.00 MHz) - Fighter operations
  ✓ FAHS - Hoedspruit (HSV 114.00 MHz) - Helicopter/Rooivalk
  ✓ FAOB - Overberg (OBV 115.40 MHz) - Test & Evaluation
  ✓ FABL - Bloemspruit (BLV 114.10 MHz) - Shared civil/military
  ✓ FADN - Durban (DNV 112.50 MHz) - Maritime patrol
  ✓ FAPE - Port Elizabeth (PEV 113.40 MHz) - Liaison operations

VOR-ONLY (2 bases):
  ○ FALW - Langebaanweg (LWV 117.00 MHz) - Pilot training
  ○ FAYP - Ysterplaat (CTV shared) - Maritime patrol/SAR

CIVIL AIRPORTS (3):
  • JNB - OR Tambo International (JHB 114.90 MHz)
  • CPT - Cape Town International (CTV 115.70 MHz)
  • DUR - King Shaka International (DNV 112.50 MHz)

SYSTEM REQUIREMENTS
-------------------

HARDWARE:
  - Windows 7 SP1 or later (7, 8, 8.1, 10, 11)
  - Processor: Intel/AMD 2.0 GHz or faster
  - RAM: 4 GB minimum (8 GB recommended)
  - Disk: 500 MB free space (including Python runtime)
  - GPU: OpenGL 3.0+ capable (for 3-D terrain rendering)
  - Serial Port: RS-232 for VOR receiver (optional)

NETWORK (Optional):
  - Ethernet for TCP/IP VOR server mode
  - Localhost (127.0.0.1) for testing

GRAPHICS:
  - Minimum 1024x768 display
  - Recommended 1920x1080 for full visibility

INSTALLATION
------------

1. Run CVOR1_Setup_v5.2.exe
2. Follow the installer wizard
3. Choose installation directory (default: C:\Program Files\...)
4. Create desktop shortcut (optional)
5. Click "Finish"

FIRST RUN
---------

1. Launch from:
   - Start Menu > VOR / ASRACS / SAAF Monitoring System
   - Desktop shortcut (if created)
   - C:\Program Files\...\CVOR1.exe

2. On first launch:
   - Default VOR: OR Tambo (JNB)
   - en-route simulation ready to start
   - Mock server available for testing

3. To test without hardware:
   - Connection tab > Mock Server > Start
   - Connection tab > TCP/IP > Host: 127.0.0.1, Port: 5000
   - Connection tab > Connect (TCP)
   - Terrain tab > En-Route Sim > Start

USER INTERFACE GUIDE
--------------------

TABS:

1. VOR MONITOR
   - Select VOR station from dropdown
   - View frequency, bearing, deviation, signal strength
   - CDI (Course Deviation Indicator) display
   - Signal trend chart

2. AIRCRAFT TRACKING
   - Animated radar display (120nm range)
   - Aircraft table with position/altitude/speed
   - Track history trails
   - Waypoint markers

3. APPROACH GUIDANCE
   - Runway selector per base
   - Glide slope display with error
   - Localizer deviation (±2.5 dots)
   - Distance to runway
   - Terrain slope analysis

4. TERRAIN 3-D
   - Interactive 3-D terrain view
   - Aircraft position overlays
   - Obstacle database
   - Left-drag to rotate, scroll to zoom

5. ASRACS SURFACE
   - OR Tambo surface layout
   - Ground target tracking
   - Runway incursion alerts
   - Alert severity color coding

6. SAAF BASES
   - National overview map
   - Base directory with frequencies
   - Runway matrices with ILS data
   - Squadron assignments
   - Navaid information

7. CONNECTION
   - Serial port RS-232 setup
   - Mock VOR server (testing)
   - TCP/IP connection
   - Test commands (Status, Ident, Health)

8. DIAGNOSTICS
   - System status (GPS, Time sync, Memory, CPU)
   - Performance metrics chart
   - Event log viewer

CONFIGURATION FILES
-------------------

vor_config.yaml
  - Stores VOR stations and airport data
  - Auto-created on first run
  - Location: C:\Users\[username]\AppData\Roaming\CVOR1\ (or app directory)

vor_monitor.log
  - Application event log
  - Contains errors, warnings, system messages
  - Location: Application directory

TROUBLESHOOTING
---------------

PROBLEM: "AttributeError: 'VORAirportMonitorApp' has no attribute 'radar'"
SOLUTION: This was a v5.1 bug, fixed in v5.2. Update to latest version.

PROBLEM: 3-D terrain not rendering (black window)
SOLUTION:
  1. Check GPU drivers (OpenGL 3.0+)
  2. Update graphics drivers from manufacturer
  3. Fall back to 2-D radar display if needed

PROBLEM: Serial port connection fails
SOLUTION:
  1. Check VOR receiver is powered and connected
  2. Verify COM port in Device Manager
  3. Try mock server for testing (no hardware needed)
  4. Use TCP mode if receiver supports network

PROBLEM: Application crashes on startup
SOLUTION:
  1. Delete vor_config.yaml and restart
  2. Check Python dependencies: pip list
  3. Reinstall from CVOR1_Setup_v5.2.exe
  4. Check vor_monitor.log for error details

PROBLEM: VOR frequencies not updating
SOLUTION:
  1. Verify connection status (should show CONNECTED)
  2. Try Test Commands to verify receiver
  3. Check mock server is running (for testing)
  4. Verify serial port baud rate (usually 9600)

SUPPORT & DOCUMENTATION
-----------------------

Repository:
  https://github.com/schalkpieterse76-afk/VOR-Monitoring--System

Issues & Bug Reports:
  https://github.com/schalkpieterse76-afk/VOR-Monitoring--System/issues

Releases & Updates:
  https://github.com/schalkpieterse76-afk/VOR-Monitoring--System/releases

DEVELOPMENT & SOURCE CODE
-------------------------

Main File:
  CVOR1.py (v5.2 - 3000+ lines)

Dependencies:
  - PyQt5 (GUI framework)
  - PyQtChart (charting)
  - PyOpenGL (3-D graphics)
  - pyserial (RS-232 communication)
  - numpy (numerical arrays)
  - PyYAML (configuration files)

Build from Source:
  1. pip install PyQt5 PyQtChart PyOpenGL PyOpenGL_accelerate pyserial numpy PyYAML PyInstaller
  2. python CVOR1.py (to run directly)
  3. build_exe.bat (to create EXE)
  4. build_installer.bat (to create MSI installer)

VERSION HISTORY
----------------

v5.2 (Current)
  - SAAF VOR/ILS metadata for all 10 bases
  - Glide slope detection & terrain analysis
  - Enhanced approach guidance tab
  - Production-ready build system

v5.1
  - Fixed widget initialization order (AttributeError)
  - Cross-tab guard checks (hasattr)
  - Radar sweep animation

v5.0
  - Initial ASRACS surface movement
  - 3-D terrain rendering
  - En-route simulation

LICENSE
-------

Copyright 2026 Aviation Systems. All rights reserved.
See LICENSE.txt for full terms.

================================================================================
