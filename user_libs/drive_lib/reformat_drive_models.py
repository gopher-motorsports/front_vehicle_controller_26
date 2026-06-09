from pathlib import Path
import shutil
import re

script_dir = Path(__file__).resolve().parent

# user_libs/drive_lib -> user_libs -> project root
project_root = script_dir.parents[1]

src_dir = project_root / "Core" / "Src"
inc_dir = project_root / "Core" / "Inc"

src_dir.mkdir(parents=True, exist_ok=True)
inc_dir.mkdir(parents=True, exist_ok=True)

models = [
    {
        "name": "open_differential_no_PID",
        "build_dir": project_root / "open_differential_no_PID_ert_rtw",
        "c_file": "open_differential_no_PID.c",
        "h_file": "open_differential_no_PID.h",
        "prefix": "simulink_OD_No_PID",
    },
    {
        "name": "open_differential",
        "build_dir": project_root / "open_differential_ert_rtw",
        "c_file": "open_differential.c",
        "h_file": "open_differential.h",
        "prefix": "simulink_OD",
    },
]

def copy_and_patch_file(source_file: Path, destination_file: Path, rename_map: dict):
    if not source_file.exists():
        print(f"Missing source file: {source_file}")
        return False

    if destination_file.exists():
        print(f"Replacing existing file: {destination_file}")

    shutil.copy2(source_file, destination_file)
    print(f"Copied {source_file} -> {destination_file}")

    if destination_file.suffix in [".c", ".h"]:
        text = destination_file.read_text()

        # Longer names first so rtPrevZCX does not get partially affected by rt*
        for old_name, new_name in sorted(rename_map.items(), key=lambda x: len(x[0]), reverse=True):
            text = re.sub(rf"\b{old_name}\b", new_name, text)

        destination_file.write_text(text)
        print(f"Patched symbols in {destination_file}")

    return True

for model in models:
    build_dir = model["build_dir"]
    prefix = model["prefix"]

    if not build_dir.exists():
        print(f"Skipping {model['name']}: build folder not found: {build_dir}")
        continue

    print(f"\nProcessing model: {model['name']}")

    rename_map = {
        "rtU": f"{prefix}_inports",
        "rtY": f"{prefix}_outports",
        "rtDW": f"{prefix}_rtDW",
        "rtPrevZCX": f"{prefix}_rtPrevZCX",
    }

    files_to_copy = {
        model["c_file"]: src_dir,
        model["h_file"]: inc_dir,
        "rtwtypes.h": inc_dir,
        "zero_crossing_types.h": inc_dir,
    }

    for filename, destination_dir in files_to_copy.items():
        source_file = build_dir / filename
        destination_file = destination_dir / filename
        copy_and_patch_file(source_file, destination_file, rename_map)

# Delete Simulink cache/autosave files in project root
for pattern in ["*.slxc", "*.autosave"]:
    for file_path in project_root.glob(pattern):
        file_path.unlink()
        print(f"Deleted {file_path}")

# Delete generated code folders after copying
for model in models:
    build_dir = model["build_dir"]

    if build_dir.exists():
        shutil.rmtree(build_dir)
        print(f"Deleted folder {build_dir}")

# Delete slprj folder
slprj_dir = project_root / "slprj"

if slprj_dir.exists():
    shutil.rmtree(slprj_dir)
    print(f"Deleted folder {slprj_dir}")

print("\nDone.")