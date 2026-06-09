from pathlib import Path
import shutil
import re

script_dir = Path(__file__).resolve().parent

# user_libs/drive_lib -> user_libs -> project root
project_root = script_dir.parents[1]

simulink_build_dir = project_root / "open_differential_no_PID_ert_rtw"

src_dir = project_root / "Core" / "Src"
inc_dir = project_root / "Core" / "Inc"

files_to_copy = {
    "open_differential_no_PID.c": src_dir,
    "open_differential_no_PID.h": inc_dir,
    "rtwtypes.h": inc_dir,
}

rename_map = {
    "rtU": "simulink_OD_No_PID_inports",
    "rtY": "simulink_OD_No_PID_outports",
    "rtDW": "simulink_OD_No_PID_rtDW",
}

src_dir.mkdir(parents=True, exist_ok=True)
inc_dir.mkdir(parents=True, exist_ok=True)

# Copy generated files into STM32 project
for filename, destination_dir in files_to_copy.items():
    source_file = simulink_build_dir / filename
    destination_file = destination_dir / filename

    if not source_file.exists():
        print(f"Missing source file: {source_file}")
        continue

    if destination_file.exists():
        print(f"Replacing existing file: {destination_file}")

    shutil.copy2(source_file, destination_file)
    print(f"Copied {source_file} -> {destination_file}")

    if destination_file.suffix in [".c", ".h"]:
        text = destination_file.read_text()

        for old_name, new_name in rename_map.items():
            text = re.sub(rf"\b{old_name}\b", new_name, text)

        destination_file.write_text(text)
        print(f"Renamed rtU/rtY/rtDW symbols in {destination_file}")

# Delete Simulink cache/autosave files
for pattern in ["*.slxc", "*.autosave"]:
    for file_path in project_root.glob(pattern):
        file_path.unlink()
        print(f"Deleted {file_path}")

# Delete generated code folder
if simulink_build_dir.exists():
    shutil.rmtree(simulink_build_dir)
    print(f"Deleted folder {simulink_build_dir}")

# Delete slprj folder
slprj_dir = project_root / "slprj"

if slprj_dir.exists():
    shutil.rmtree(slprj_dir)
    print(f"Deleted folder {slprj_dir}")

print("Done.")