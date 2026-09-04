# PegHook Studio

![PegHook Studio logo](assets/peghook-studio-logo.svg)

PegHook Studio is a desktop workbench for adapting 3D-printed accessories to IKEA-style pegboards and other perforated panels. It addresses a common problem: a useful model has a good main body, but its rear hook is made for a different board thickness. For example, a model may use a 5 mm hook while the user's board requires a 1.2 mm hook.

## Why it exists

Without a dedicated tool, users must open a CAD application, remove the old hook, repair or cap the cut surface, import a replacement hook, align it by hand, and repeat the process for every model. That workflow is slow and easy to misalign.

PegHook Studio keeps the conversion in one focused 3D workspace: select the mounting face, remove the outside hook, load a replacement, attach it to the selected face, make precise adjustments, and export the result.

The application includes 1.2 mm and 5 mm replacement hooks. Users can import custom STL or STEP hooks with other thicknesses or shapes.

## Interface preview

These images are from the current development build and show the workbench, transform gizmo, and numeric transform panel. The interface may continue to evolve.

![PegHook Studio main workbench](docs/screenshots/peghook-studio-main.png)

## Features

- Import a body in STL, STEP, or 3MF format.
- Use built-in 1.2 mm or 5 mm hooks, or import a custom STL, STEP, or 3MF hook.
- Select a mounting face on the body.
- Cut the outside hook using a selected plane and depth.
- Automatically attach a replacement hook to the selected face.
- Directly select, move, rotate, copy, delete, group, and ungroup hooks.
- Use X/Y/Z colored axis controls and precise numeric input.
- Multi-select with Shift or Cmd and transform a group together.
- Undo and redo with an operation history.
- Use standard views and the orientation cube.
- Export the combined STL.

## Supported formats

STL is the primary and most stable format. STEP files are triangulated through OpenCascade and then processed as meshes. 3MF files are read through `trimesh` and `lxml`; multi-part 3MF scenes are combined into one editable mesh. Complex or unusual STEP/3MF topology may affect results.

## Typical workflow

1. Import the body STL.
2. Choose **Select mounting face**, then click the body face where the new hook should attach.
3. Enter a cutting depth if the old hook must be removed, then choose **Cut outside hook**.
4. Select the built-in 1.2 mm or 5 mm hook, or import a custom hook.
5. Choose **Attach hook**.
6. Click a hook to move or rotate it. Use the numeric panel for exact offsets, or hold Shift/Cmd to select multiple hooks.
7. Choose **Export model** and save the combined STL.

## Run from source

Python 3.11 is recommended because the current PySide6, VTK, and scientific-computing dependency combination has been exercised most thoroughly there. Python 3.12 and 3.13 may work after a clean dependency installation. Python 3.14 is newer and some dependencies may not yet publish compatible wheels; if installation fails on 3.14, use Python 3.11 instead of mixing packages between environments.

### macOS

```bash
python3.11 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
python pegboard_converter.py
```

### Windows PowerShell

```powershell
py -3.11 -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
python pegboard_converter.py
```

STEP import additionally requires:

```bash
python -m pip install cadquery-ocp
```

## Troubleshooting with AI

Doubao, DeepSeek, ChatGPT, and other coding assistants can help diagnose setup problems. Include your operating system, CPU architecture, Python version, command, complete error log, and expected result. Do not send only “it does not run”.

Use this template:

```text
I am deploying PegHook Studio.
Operating system and CPU:
Python version:
Command:
Complete error log:
Expected result:

First determine whether this is a dependency, Qt platform plugin, VTK rendering,
resource path, or STL/STEP processing problem. Then propose the smallest fix.
Do not rewrite the whole project or remove existing features.
```

Common cases include missing Python modules, Qt platform plugin errors, blank VTK windows, missing `preset_hooks` resources, STEP triangulation failures, and PyInstaller resource paths. Ask the AI to explain the cause first, then change only the relevant files and keep a backup.

## Packaging

The repository does not include a virtual environment. PyInstaller builds are produced on native runners for macOS arm64, macOS x86_64, and Windows x64. Intel and AMD Windows CPUs use the same x64 package. The project provides build scripts and a GitHub Actions workflow; one-folder builds are preferred initially because Qt/VTK plugins and resources are easier to diagnose.

## Status and limitations

This is an actively evolving desktop tool. The current focus is STL mesh conversion and hook attachment. Broken or non-manifold meshes may require repair in another mesh editor. Cutting uses a user-selected plane and geometric computation; it does not claim to semantically identify every hook in every model.

## License

MIT License. Third-party dependencies retain their own licenses; distributed applications should include the corresponding notices for Qt/PySide6, VTK, NumPy, SciPy, trimesh, and other bundled libraries.
