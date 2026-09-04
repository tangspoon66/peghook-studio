# PyInstaller specification for PegHook Studio.
from pathlib import Path
from PyInstaller.utils.hooks import collect_data_files, collect_dynamic_libs, collect_submodules

project_root = Path(SPECPATH).parent
datas = [(str(project_root / "preset_hooks"), "preset_hooks")]
datas += collect_data_files("pyvista")
datas += collect_data_files("pyvistaqt")
datas += collect_data_files("OCP")

hiddenimports = []
for package in ("pyvista", "pyvistaqt", "vtk", "trimesh", "OCP"):
    hiddenimports += collect_submodules(package)

binaries = collect_dynamic_libs("OCP")

excludes = [
    "PySide6.Qt3DAnimation", "PySide6.Qt3DCore", "PySide6.Qt3DExtras",
    "PySide6.Qt3DInput", "PySide6.Qt3DLogic", "PySide6.Qt3DRender",
    "PySide6.QtCharts", "PySide6.QtDataVisualization", "PySide6.QtWebEngine",
    "PySide6.QtWebEngineCore", "PySide6.QtWebEngineWidgets", "PySide6.QtQuick",
    "PySide6.QtQuick3D", "PySide6.QtMultimedia", "PySide6.QtPdf",
]

a = Analysis(
    [str(project_root / "pegboard_converter.py")],
    pathex=[str(project_root)],
    binaries=binaries,
    datas=datas,
    hiddenimports=hiddenimports,
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=excludes,
    noarchive=False,
)
pyz = PYZ(a.pure)
exe = EXE(
    pyz,
    a.scripts,
    [],
    name="PegHookStudio",
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=False,
    console=False,
    exclude_binaries=True,
)

coll = COLLECT(
    exe,
    a.binaries,
    a.datas,
    strip=False,
    upx=False,
    name="PegHookStudio",
)

if __import__("platform").system() == "Darwin":
    app = BUNDLE(coll, name="PegHookStudio.app", icon=None)
