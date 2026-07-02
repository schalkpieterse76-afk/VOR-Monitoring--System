# Installation Guide: VOR/ASRACS/SAAF Monitoring System v5.2

## Quick Install (5 minutes)

### Windows Users (Easiest)

#### Method 1: Pre-built Executable
1. Download `CVOR1.exe` from releases
2. Double-click to run
3. Done! No installation needed

#### Method 2: Automated Script
```bash
build.bat
```
This creates `dist/CVOR1.exe` automatically.

---

## Detailed Installation

### Prerequisites

**System Requirements:**
- Windows 7/8/10/11 (64-bit recommended)
- 500 MB free disk space
- OpenGL-capable graphics card
- COM port or network connection (optional, for real hardware)

**Software Requirements:**
- Python 3.8 or later
- Git (optional, for cloning repository)

---

## Step 1: Install Python

### Option A: From python.org (Recommended)

1. Visit https://www.python.org/downloads/
2. Click "Download Python 3.11" (or latest)
3. Run installer
4. **IMPORTANT:** Check "Add Python to PATH"
5. Click "Install Now"
6. Wait for completion
7. Close installer

### Option B: Microsoft Store

1. Open Microsoft Store
2. Search "Python 3.11"
3. Click "Install"
4. Wait for completion

### Verify Installation

Open Command Prompt (Windows + R, type `cmd`) and run:
```bash
python --version
```

You should see: `Python 3.x.x`

---

## Step 2: Get the Source Code

### Option A: Download ZIP

1. Visit https://github.com/schalkpieterse76-afk/VOR-Monitoring--System
2. Click green "Code" button
3. Click "Download ZIP"
4. Extract to desired folder
5. Open Command Prompt in that folder

### Option B: Clone with Git

1. Install Git from https://git-scm.com/download/win
2. Open Command Prompt
3. Run:
   ```bash
   git clone https://github.com/schalkpieterse76-afk/VOR-Monitoring--System.git
   cd VOR-Monitoring--System
   ```

---

## Step 3: Install Dependencies

In Command Prompt (in project folder), run:

```bash
pip install -r requirements.txt
```

This installs:
- PyQt5 (GUI framework)
- PyOpenGL (3D graphics)
- NumPy (numerical computing)
- pyserial (serial port communication)
- PyYAML (configuration files)

---

## Step 4: Run Application

### Option A: Quick Run

Double-click `run.bat`

### Option B: Command Line

In Command Prompt:
```bash
python CVOR1.py
```

### Option C: Built Executable

Run `build.bat` first, then:
```bash
.\dist\CVOR1.exe
```

---

## Troubleshooting

### "Python not found" error

**Solution:** Python not in PATH
1. Reinstall Python
2. **IMPORTANT:** Check "Add Python to PATH" during install
3. Restart Command Prompt
4. Try again

### "No module named 'PyQt5'"

**Solution:** Dependencies not installed
```bash
pip install --upgrade pip
pip install -r requirements.txt
```

### "Failed to load OpenGL"

**Solution:** Graphics driver issue
1. Update GPU drivers:
   - NVIDIA: https://www.nvidia.com/Download/driverDetails.aspx
   - AMD: https://www.amd.com/en/technologies/radeon-drivers
   - Intel: https://downloadcenter.intel.com/
2. Restart computer
3. Try again

### "Permission denied" on build.bat

**Solution:** Run as Administrator
1. Right-click `build.bat`
2. Select "Run as administrator"
3. Click "Yes" when prompted

---

## Building Standalone Executable

### Automatic (Recommended)

```bash
build.bat
```

Output: `dist/CVOR1.exe` (~150 MB)

### Manual

```bash
pip install PyInstaller
pyinstaller CVOR1.spec --clean
```

---

## Verification

### Test Installation

1. Run application
2. Check "VOR Monitor" tab
3. See "Mock VOR Server" in Connection tab
4. Click "▶ Start" on Mock Server
5. Click "Connect (TCP)" → "Query Status"
6. Verify output in test log

If all works: ✅ Installation successful!

---

## Uninstall

### Remove Application

1. Delete application folder
2. Delete `vor_config.yaml` if desired
3. Delete `vor_monitor.log` if desired

### Remove Python (Optional)

1. Open Settings (Windows + I)
2. Apps → Installed apps
3. Find "Python 3.x"
4. Click "Uninstall"
5. Confirm

---

## Advanced Setup

### Virtual Environment (Optional)

For development/isolation:

```bash
python -m venv venv
venv\Scripts\activate
pip install -r requirements.txt
python CVOR1.py
```

Deactivate:
```bash
venv\Scripts\deactivate
```

### Custom Installation Path

```bash
pip install --target C:\CustomPath -r requirements.txt
```

---

## Support

For issues:
1. Check `vor_monitor.log`
2. Review troubleshooting above
3. Post issue on GitHub: https://github.com/schalkpieterse76-afk/VOR-Monitoring--System/issues
4. Include: Python version, Windows version, error message, log excerpt

---

**Installation Complete!** 🎉

Start using the VOR Monitoring System:
1. Launch application
2. Select VOR station
3. Start simulation or connect hardware
4. Monitor approaches in real-time
