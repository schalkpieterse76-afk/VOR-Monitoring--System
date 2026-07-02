# -*- mode: python ; coding: utf-8 -*-
# PyInstaller spec file for CVOR1.py v5.2
# Build command: pyinstaller CVOR1.spec

from PyInstaller.utils.hooks import collect_submodules, collect_data_files
import os

block_cipher = None

a = Analysis(
    ['CVOR1.py'],
    pathex=[],
    binaries=[],
    datas=[
        ('icon.ico', '.'),
        ('LICENSE.txt', '.'),
        ('README.md', '.'),
    ],
    hiddenimports=[
        'PyQt5.QtChart',
        'PyOpenGL',
        'PyOpenGL_accelerate',
        'serial',
        'yaml',
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
    icon='icon.ico',
)

coll = COLLECT(
    exe,
    a.binaries,
    a.zipfiles,
    a.datas,
    strip=False,
    upx=True,
    upx_exclude=[],
    name='CVOR1',
)
