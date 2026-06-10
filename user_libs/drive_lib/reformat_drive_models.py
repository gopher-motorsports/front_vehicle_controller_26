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
        "suffix": "OD_No_PID",
    },
    {
        "name": "open_differential",
        "build_dir": project_root / "open_differential_ert_rtw",
        "c_file": "open_differential.c",
        "h_file": "open_differential.h",
        "suffix": "OD",
    },
    {
        "name": "torque_vectoring",
        "build_dir": project_root / "torque_vectoring_ert_rtw",
        "c_file": "torque_vectoring.c",
        "h_file": "torque_vectoring.h",
        "suffix": "TV",
    },
]


COMMON_SUPPORT_FILES = {
    "rtwtypes.h",
    "zero_crossing_types.h",
    "solver_zc.h",
}


def patch_text_file(file_path: Path, rename_map: dict):
    text = file_path.read_text(encoding="utf-8", errors="ignore")

    # Longer names first so rtPrevZCX / rtM_ do not get partially patched by rtM
    for old_name, new_name in sorted(rename_map.items(), key=lambda x: len(x[0]), reverse=True):
        text = re.sub(rf"\b{re.escape(old_name)}\b", new_name, text)

    file_path.write_text(text, encoding="utf-8")
    print(f"Patched symbols in {file_path}")


def copy_and_patch_file(source_file: Path, destination_file: Path, rename_map: dict, should_patch: bool):
    if not source_file.exists():
        print(f"Missing source file: {source_file}")
        return False

    if destination_file.exists():
        print(f"Replacing existing file: {destination_file}")

    shutil.copy2(source_file, destination_file)
    print(f"Copied {source_file} -> {destination_file}")

    if should_patch and destination_file.suffix in [".c", ".h"]:
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


def make_rename_map(suffix: str):
    """
    Rename both generated globals and generated typedefs.

    Example for suffix OD:
      rtDW      -> simulink_OD_rtDW
      rtPrevZCX -> simulink_OD_rtPrevZCX
      rtM_      -> simulink_OD_rtM_
      rtM       -> simulink_OD_rtM
      rtU       -> simulink_OD_inports
      rtY       -> simulink_OD_outports

      DW        -> DW_OD
      ExtU      -> ExtU_OD
      ExtY      -> ExtY_OD
      RT_MODEL  -> RT_MODEL_OD
      tag_RTM   -> tag_RTM_OD
    """
    return {
        # Generated Simulink external globals
        "rtDW": f"simulink_{suffix}_rtDW",
        "rtPrevZCX": f"simulink_{suffix}_rtPrevZCX",

        # Real-time model globals
        "rtM_": f"simulink_{suffix}_rtM_",
        "rtM": f"simulink_{suffix}_rtM",

        # Inports/outports last in the variable name
        "rtU": f"simulink_{suffix}_inports",
        "rtY": f"simulink_{suffix}_outports",

        # Generated typedefs that collide between models
        "DW": f"DW_{suffix}",
        "ExtU": f"ExtU_{suffix}",
        "ExtY": f"ExtY_{suffix}",
        "RT_MODEL": f"RT_MODEL_{suffix}",

        # Generated RT model struct tag that can also collide
        "tag_RTM": f"tag_RTM_{suffix}",
    }


def process_model(model: dict):
    build_dir = model["build_dir"]
    model_name = model["name"]
    suffix = model["suffix"]

    if not build_dir.exists():
        print(f"\nSkipping {model_name}: build folder not found: {build_dir}")
        return

    print(f"\nProcessing model: {model_name} with suffix: {suffix}")

    rename_map = make_rename_map(suffix)

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

        # Important:
        # Do NOT patch common support headers like rtwtypes.h, solver_zc.h,
        # or zero_crossing_types.h. They are shared support files and should
        # stay generic.
        #
        # Patch only model-specific generated files, because those contain
        # rtU/rtY/rtDW/rtM/DW/ExtU/ExtY/RT_MODEL collisions.
        should_patch = filename not in COMMON_SUPPORT_FILES

        copy_and_patch_file(
            source_file=source_file,
            destination_file=destination_file,
            rename_map=rename_map,
            should_patch=should_patch,
        )


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