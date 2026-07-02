# -*- mode: python ; coding: utf-8 -*-
# PyInstaller spec file for VOR/ASRACS/SAAF Airport Monitoring System v5.2
# Build with: pyinstaller CVOR1.spec

block_cipher = None

a = Analysis(
    ['CVOR1.py'],
    pathex=[],
    binaries=[],
    datas=[
        ('vor_config.yaml', '.'),
        ('LICENSE.txt', '.'),
        ('README.md', '.'),
    ],
    hiddenimports=[
        'PyQt5',
        'PyQt5.QtCore',
        'PyQt5.QtGui',
        'PyQt5.QtWidgets',
        'PyQt5.QtChart',
        'serial',
        'serial.tools.list_ports',
        'OpenGL.GL',
        'OpenGL.GLU',
        'numpy',
        'yaml',
    ],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludedimports=[],
    win_no_prefer_redirects=False,
    win_private_assemblies=False,
    cipher=block_cipher,
    noarchive=False,
)

pyz = PYZ(a.pure, a.zipped_data, cipher=block_cipher)

exe = EXE(
    pyz,
    a.scripts,
    a.binaries,
    a.zipfiles,
    a.datas,
    [],
    name='CVOR1',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    upx_exclude=[],
    runtime_tmpdir=None,
    console=False,
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
    icon='CVOR1.ico',
)
