# -*- mode: python ; coding: utf-8 -*-
# PyInstaller spec file for CVOR1.py
# Generated for VOR / ASRACS / SAAF Monitoring System v5.2

block_cipher = None

a = Analysis(
    ['CVOR1.py'],
    pathex=[],
    binaries=[],
    datas=[
        ('CVOR1.ico', '.'),
        ('CVOR1.png', '.'),
    ],
    hiddenimports=[
        'PyQt5.sip',
        'serial',
        'OpenGL.GL',
        'OpenGL.GLU',
        'PyQtChart',
        'numpy',
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
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
    icon='CVOR1.ico',
)
