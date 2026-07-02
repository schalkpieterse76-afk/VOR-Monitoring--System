# VOR / ASRACS / SAAF Airport Monitoring System v5.2

![VOR Monitoring System](CVOR1.ico)

## Overview

A comprehensive **real-time airport monitoring and navigation system** for South African aviation operations. Integrates VOR (VHF Omnidirectional Range) navigation, ASRACS surface movement detection, and SAAF (South African Air Force) base monitoring with advanced approach guidance.

### ✅ Features

#### Navigation & VOR Monitoring
- **10 SAAF Bases**: Complete VOR/ILS coverage mapping
- **3 Civil Airports**: JNB, CPT, DUR with approach aids
- **Real-time VOR Data**: Frequency, bearing, deviation, signal strength
- **ILS Runway Mapping**: Glide slope detection (3.0° standard ICAO)
- **NDB Support**: Secondary navigation integration

#### Advanced Approach Guidance
- **Glide Slope Detection**: Real-time vertical deviation (±200 ft tolerance)
- **Localizer Guidance**: Course deviation indicator (CDI display)
- **Terrain Analysis**: Surface slope gradient detection
- **Descent Rate Calculation**: Dynamic FPM requirement
- **Runway Threshold Display**: Active runway and heading info

#### ASRACS Surface Movement (FAOR)
- **Real-time Ground Tracking**: Aircraft & vehicle monitoring
- **Runway Incursion Alerts**: Critical/warning/caution thresholds
- **Hot Spot Identification**: Known conflict areas
- **Taxi Route Animation**: Visual surface representation
- **Gate-to-Gate Simulation**: Full airport operations

#### En-Route Radar
- **Animated Sweep**: 360° radar display
- **Aircraft Tracking**: Trail history with decay
- **Velocity Leader**: Projected aircraft position
- **Waypoint Display**: Approach fix identification
- **Range Rings**: 30/60/90/120 nm coverage zones

#### Terrain & Visualization
- **3-D Terrain Rendering**: OpenGL with slope shading
- **Obstacle Database**: Towers, antennas, hills, buildings
- **Airport Layouts**: Complete SAAF base infrastructure
- **Runway Profiles**: Threshold coordinates & dimensions

#### Data Acquisition
- **Serial Port Support**: RS-232/COM port connectivity
- **TCP/IP Connection**: Network VOR equipment
- **Mock VOR Server**: Built-in test/training mode
- **Real-time Updates**: 1Hz acquisition rate

#### SAAF Bases Coverage

| Base | ICAO | VOR Ident | Frequency | ILS | Status |
|------|------|-----------|-----------|-----|--------|
| Waterkloof | FAWK | WKV | 116.90 MHz | RWY 01 | ✅ Active |
| Makhado | FALM | LTV | 115.00 MHz | RWY 10/28 | ✅ Active |
| Hoedspruit | FAHS | HSV | 114.00 MHz | RWY 09 | ✅ Active |
| Langebaanweg | FALW | LWV | 117.00 MHz | No ILS | ✅ Active |
| Overberg | FAOB | OBV | 115.40 MHz | RWY 35 | ✅ Active |
| Swartkop | FASK | WKV* | 116.90 MHz | No ILS | ✅ Active |
| Bloemspruit | FABL | BLV | 114.10 MHz | RWY 20 | ✅ Active |
| Ysterplaat | FAYP | CTV* | 115.70 MHz | No ILS | ✅ Active |
| Durban | FADN | DNV | 112.50 MHz | RWY 06 | ✅ Active |
| Port Elizabeth | FAPE | PEV | 113.40 MHz | RWY 08 | ✅ Active |

*Shared VOR from nearby facility

---

## Installation

### Option 1: Pre-built Executable (Recommended)

1. Download `dist/CVOR1.exe`
2. Double-click to run (no dependencies required)
3. Application launches immediately

### Option 2: From Source

**Requirements:**
- Python 3.8 or later
- Windows 7 / 8 / 10 / 11
- 500 MB free disk space
- OpenGL-capable graphics card

**Installation Steps:**

```bash
# Clone repository
git clone https://github.com/schalkpieterse76-afk/VOR-Monitoring--System.git
cd VOR-Monitoring--System

# Install dependencies
pip install -r requirements.txt

# Run application
python CVOR1.py
```

or simply:

```bash
run.bat
```

---

## Dependencies

```
PyQt5>=5.15.0
PyQtChart>=5.15.0
PyOpenGL>=3.1.5
PyOpenGL_accelerate>=3.1.5
pyserial>=3.5
numpy>=1.21.0
PyYAML>=5.4.0
```

Automatically installed via `build.bat` or `pip install -r requirements.txt`

---

## Quick Start

### 1. Launch Application
```bash
.\dist\CVOR1.exe
```
or
```bash
python CVOR1.py
```

### 2. Select VOR Station
- **VOR Monitor Tab** → Select from dropdown
- Filter by: "SAAF bases only", "Civil only", "VOR/DME", "VORTAC"
- View real-time data: frequency, bearing, deviation, signal strength

### 3. Start Simulations
- **En-Route Sim** (5 aircraft approaching VOR)
- **ASRACS Sim** (surface movement at FAOR)
- **Mock VOR Server** (127.0.0.1:5000)

### 4. Connect to Hardware
**Connection Tab:**
- Serial: COM port + baud rate (9600-115200)
- TCP/IP: Host + port (e.g., 127.0.0.1:5000)
- Test: Query Status, Ident, Health

### 5. Monitor Approaches
- **Approach Guidance Tab**
  - Select active runway
  - View glide slope: ON GS / HIGH / LOW
  - Monitor localizer deviation ±2.5 dots
  - Track terrain slope gradient

---

## Tabs & Features

### 🎯 VOR Monitor
- Live VOR station data
- CDI (Course Deviation Indicator) display
- Signal trend chart
- Per-base approach simulation

### 📡 Aircraft Tracking (Radar)
- 120 nm range display
- Animated sweep pattern
- Aircraft trails with decay
- Waypoint identification
- Velocity leader projection

### 🛬 Approach Guidance
- Active runway selection
- Glide slope status (ON GS/HIGH/LOW)
- Localizer error display (±2.5 dots)
- Distance to runway
- Terrain slope analysis
- Runway heading & designation

### 🗻 Terrain 3-D
- OpenGL-rendered terrain
- Slope shading visualization
- Aircraft position overlay
- Waypoint markers
- Obstacle reference database
- Interactive camera control (drag/scroll)

### 🛬 ASRACS Surface
- Real-time airport surface tracking
- Aircraft & vehicle simulation
- Runway incursion alerts
- Hot spot highlighting
- Gate assignments
- Taxi route visualization

### 📍 SAAF Bases
- National overview map
- Base directory with VOR frequencies
- Airport layout maps
- Runway matrix details
- Squadron assignments
- Navaid information

### 🔌 Connection
- Mock VOR server (built-in)
- Serial port configuration
- TCP/IP connection
- Test commands (Status/Ident/Health)
- Real-time connection monitoring

### 📊 Diagnostics
- System status (GPS, time sync)
- Performance metrics
- Memory & CPU usage
- Event logging

---

## Building Executable

### Automatic Build (Recommended)

```bash
build.bat
```

This will:
1. Check Python installation
2. Install/update all dependencies
3. Clean old builds
4. Compile with PyInstaller
5. Generate `dist/CVOR1.exe`

### Manual Build

```bash
pyinstaller CVOR1.spec --clean
```

### Build Output
```
dist/
├── CVOR1.exe (main executable)
├── vor_config.yaml (configuration)
├── LICENSE.txt
└── README.md
```

---

## Configuration

### VOR Station Database

**File:** `vor_config.yaml`

```yaml
airports:
  FAWK:
    name: AFB Waterkloof
    latitude: -25.8300
    longitude: 28.2225
    elevation: 1506

vor_stations:
  FAWK_VOR:
    name: Waterkloof VORTAC
    ident: WKV
    frequency: 116.90
    type: VORTAC
    airport: FAWK
    ils_available: true
    ils_runways:
      - runway: '01'
        frequency: 111.50
        glide_slope_deg: 3.0
```

Modify to add custom stations or update frequency data.

---

## Hardware Integration

### Serial VOR Equipment

**Expected Input Format (RS-232):**
```
STATUS: frequency,bearing,deviation,signal_strength,status
114.90,045.5,-1.25,-45.2,OK
```

**Command Format:**
```
STATUS       → Query current VOR data
IDENT        → Station identification
HEALTH       → System health status
CALIBRATE    → Start calibration sequence
```

### TCP/IP VOR Server

Connect via network on port 5000 (configurable):
```bash
tcpconnect 127.0.0.1:5000
STATUS
114.90,045.5,-1.25,-45.2,OK
```

### Mock Server (Testing)

1. **Connection Tab** → Mock VOR Server
2. Click "▶ Start"
3. Automatically binds to 127.0.0.1:5000
4. Returns simulated VOR data
5. Suitable for training & demonstrations

---

## Troubleshooting

### Application Won't Start

**Problem:** `ModuleNotFoundError: No module named 'PyQt5'`

**Solution:**
```bash
pip install PyQt5 PyQtChart PyOpenGL PyOpenGL_accelerate pyserial numpy PyYAML
```

### OpenGL Rendering Issues

**Problem:** 3-D terrain displays black or errors

**Solution:**
- Update graphics drivers
- Ensure OpenGL 3.0+ support
- Reinstall PyOpenGL:
  ```bash
  pip install --upgrade PyOpenGL PyOpenGL_accelerate
  ```

### Serial Port Not Detected

**Problem:** COM port list is empty

**Solution:**
1. Check device manager (Windows + X → Device Manager)
2. Verify COM port number (e.g., COM3)
3. Ensure driver is installed
4. Click "Refresh" button in Connection tab

### Connection Refused (TCP)

**Problem:** `Connection Refused 127.0.0.1:5000`

**Solution:**
1. Start Mock VOR Server first
2. Verify correct host/port
3. Check firewall settings
4. Ensure remote VOR server is running

### Performance / Lag

**Problem:** Application runs slowly

**Solution:**
- Close other applications
- Reduce simulation update rate (↓ fps)
- Disable 3-D terrain visualization
- Upgrade GPU drivers

### Log File Location

Application logs saved to: `vor_monitor.log`

View for detailed error information:
```bash
type vor_monitor.log
```

---

## Development

### Project Structure

```
VOR-Monitoring--System/
├── CVOR1.py              # Main application (v5.2)
├── CVOR1.spec            # PyInstaller specification
├── build.bat             # Windows build script
├── run.bat               # Windows run script
├── requirements.txt      # Python dependencies
├── LICENSE.txt           # GNU GPL v3.0
├── README.md             # This file
├── CVOR1.ico             # Application icon
├── vor_config.yaml       # VOR station database
└── dist/
    └── CVOR1.exe         # Compiled executable
```

### Key Classes

- **GlideSlopeDetector**: Vertical guidance calculations
- **SurfaceSlopeAnalyzer**: Terrain gradient analysis
- **VORAirportConfig**: Station database management
- **ASRACSSimEngine**: Surface movement simulation
- **RadarDisplay**: En-route radar visualization
- **Terrain3DWidget**: OpenGL 3-D terrain rendering
- **VORAirportMonitorApp**: Main application window

### Adding Custom VOR Stations

1. Edit `vor_config.yaml`
2. Add entry to `vor_stations` section:
   ```yaml
   CUSTOM_VOR:
     name: Custom VOR/DME
     ident: CUS
     frequency: 115.50
     type: VOR/DME
     airport: CUSTOM
     latitude: -25.0000
     longitude: 28.0000
     ils_available: false
   ```
3. Restart application

---

## Testing

### Unit Tests

```bash
python -m pytest tests/
```

### Integration Testing

1. Start Mock VOR Server
2. Test Connection → Query Status
3. Verify signal in VOR Monitor tab
4. Launch En-Route Sim
5. Confirm radar display
6. Test ASRACS Sim
7. Verify surface alerts

---

## Performance Specifications

| Metric | Value |
|--------|-------|
| Update Rate | 1 Hz (configurable) |
| Radar Range | 120 nm |
| Tracked Aircraft | 5-10 (configurable) |
| Surface Targets | 50+ (simulated) |
| Memory Usage | 150-250 MB |
| GPU Memory | 100-200 MB |
| CPU Usage | 10-30% (idle), 30-50% (active) |
| Supported Platforms | Windows 7/8/10/11 (64-bit) |

---

## License

GNU General Public License v3.0 - See LICENSE.txt for details

**Copyright © 2026 South African Air Force (SAAF) / Aviation Systems**

---

## Support & Documentation

### Official Repository
https://github.com/schalkpieterse76-afk/VOR-Monitoring--System

### Documentation
- In-app Help: Menu → Help → About
- Configuration: vor_config.yaml
- Logs: vor_monitor.log

### Reporting Issues

Create GitHub issue with:
1. Application version (Help → About)
2. Error message / stack trace
3. vor_monitor.log excerpt
4. System info (Windows version, Python version)
5. Steps to reproduce

---

## Roadmap (v5.3+)

- [ ] Multi-base simultaneous monitoring
- [ ] Real-time weather integration
- [ ] ATIS/VOLMET text-to-speech
- [ ] Aircraft performance calculations
- [ ] Fuel computation & range rings
- [ ] Standard instrument departure (SID) overlays
- [ ] Vector radar guidance
- [ ] Conflict detection & avoidance alerts
- [ ] Flight data recording (FDR)
- [ ] Multi-language support
- [ ] Linux/macOS ports

---

## Contributors

- **System Architecture**: SAAF Navigation Systems
- **VOR/ILS Database**: South African Aviation Authority
- **ASRACS Integration**: Surface Movement Specialists
- **Development**: Schalk Pieterse (schalkpieterse76@gmail.com)

---

**Version 5.2 | Built 2026 | All Rights Reserved**
