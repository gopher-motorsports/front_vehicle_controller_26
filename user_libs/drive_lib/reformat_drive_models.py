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


def patch_text_file(file_path: Path, rename_map: dict):
    text = file_path.read_text(encoding="utf-8", errors="ignore")

    # Longer names first so rtPrevZCX / rtM_ do not get partially patched by rt* / rtM
    for old_name, new_name in sorted(rename_map.items(), key=lambda x: len(x[0]), reverse=True):
        text = re.sub(rf"\b{re.escape(old_name)}\b", new_name, text)

    file_path.write_text(text, encoding="utf-8")
    print(f"Patched symbols in {file_path}")


def copy_and_patch_file(source_file: Path, destination_file: Path, rename_map: dict):
    if not source_file.exists():
        print(f"Missing source file: {source_file}")
        return False

    if destination_file.exists():
        print(f"Replacing existing file: {destination_file}")

    shutil.copy2(source_file, destination_file)
    print(f"Copied {source_file} -> {destination_file}")

    if destination_file.suffix in [".c", ".h"]:
        patch_text_file(destination_file, rename_map)

    return True


def find_support_file(filename: str, preferred_build_dir: Path):
    """
    First check the generated model folder.
    Then search the project tree.

    Useful for files like solver_zc.h, which may appear in generated support folders
    depending on Simulink settings.
    """
    direct_path = preferred_build_dir / filename
    if direct_path.exists():
        return direct_path

    matches = list(project_root.rglob(filename))

    # Do not use files already copied into Core/Inc or Core/Src as the source
    filtered_matches = []
    for match in matches:
        if inc_dir in match.parents:
            continue
        if src_dir in match.parents:
            continue
        filtered_matches.append(match)

    if filtered_matches:
        return filtered_matches[0]

    return None


def process_model(model: dict):
    build_dir = model["build_dir"]
    prefix = model["prefix"]
    model_name = model["name"]

    if not build_dir.exists():
        print(f"\nSkipping {model_name}: build folder not found: {build_dir}")
        return

    print(f"\nProcessing model: {model_name}")

    rename_map = {
        # Generated Simulink external globals
        "rtU": f"{prefix}_inports",
        "rtY": f"{prefix}_outports",
        "rtDW": f"{prefix}_rtDW",
        "rtPrevZCX": f"{prefix}_rtPrevZCX",

        # Real-time model globals
        "rtM_": f"{prefix}_rtM_",
        "rtM": f"{prefix}_rtM",
    }

    files_to_copy = {
        # Main generated model files
        model["c_file"]: src_dir,
        model["h_file"]: inc_dir,

        # Common generated support headers
        "rtwtypes.h": inc_dir,
        "zero_crossing_types.h": inc_dir,
        "solver_zc.h": inc_dir,

        # Model-specific generated headers
        f"{model_name}_private.h": inc_dir,
        f"{model_name}_types.h": inc_dir,

        # Sometimes generated depending on model/config
        f"{model_name}_data.c": src_dir,
    }

    for filename, destination_dir in files_to_copy.items():
        source_file = find_support_file(filename, build_dir)

        if source_file is None:
            print(f"Optional/missing file skipped: {filename}")
            continue

        destination_file = destination_dir / filename
        copy_and_patch_file(source_file, destination_file, rename_map)


for model in models:
    process_model(model)


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