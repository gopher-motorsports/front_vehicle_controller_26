import re
import subprocess
from pathlib import Path

OUTPUT_FILE = Path("../../Core/Inc/git_info.h")


def run_git_command(args):
    try:
        return subprocess.check_output(
            ["git"] + args,
            text=True,
            stderr=subprocess.DEVNULL
        ).strip()
    except (subprocess.CalledProcessError, FileNotFoundError):
        return None


def get_all_branches():
    output = run_git_command(["branch", "-a", "--format=%(refname:short)"])
    if not output:
        return []

    branches = []
    seen = set()

    for line in output.splitlines():
        branch = line.strip()
        if not branch:
            continue

        if branch == "origin":
            continue
        
        if "->" in branch:
            continue

        if branch.startswith("origin/"):
            branch = branch[len("origin/"):]

        if branch not in seen:
            seen.add(branch)
            branches.append(branch)

    return branches


def get_current_branch():
    branch = run_git_command(["branch", "--show-current"])
    if branch:
        return branch

    branch = run_git_command(["rev-parse", "--abbrev-ref", "HEAD"])
    if not branch or branch == "HEAD":
        return None

    if branch.startswith("origin/"):
        branch = branch[len("origin/"):]

    return branch


def get_git_hash_short():
    return run_git_command(["rev-parse", "--short", "HEAD"])


def git_hash_to_int(hash_str):
    if not hash_str:
        return 0
    try:
        return int(hash_str, 16)
    except ValueError:
        return 0


def branch_sort_key(branch):
    lower = branch.lower()

    if lower == "main":
        return (0, lower)

    if lower.startswith("feature-") or lower.startswith("feature/"):
        return (1, lower)

    return (2, lower)


def sanitize_enum_name(branch):
    name = branch.upper()

    if name.startswith("FEATURE-"):
        name = "FEATURE_" + name[len("FEATURE-"):]
    elif name.startswith("FEATURE/"):
        name = "FEATURE_" + name[len("FEATURE/"):]

    name = re.sub(r"[^A-Z0-9]", "_", name)
    name = re.sub(r"_+", "_", name).strip("_")

    if not name:
        name = "UNKNOWN"

    if name[0].isdigit():
        name = "_" + name

    return name


def generate_header(output_path):
    branches = sorted(get_all_branches(), key=branch_sort_key)
    current_branch = get_current_branch()
    git_hash_short = get_git_hash_short()

    git_hash_dec = git_hash_to_int(git_hash_short)
    git_hash_hex = f"0x{git_hash_dec:X}"

    enum_entries = []
    used_names = set()
    branch_to_enum = {}

    for branch in branches:
        enum_name = sanitize_enum_name(branch)

        base_name = enum_name
        suffix = 2
        while enum_name in used_names:
            enum_name = f"{base_name}_{suffix}"
            suffix += 1

        used_names.add(enum_name)
        branch_to_enum[branch] = enum_name
        enum_entries.append(enum_name)

    if not enum_entries:
        enum_entries.append("UNKNOWN")
        branch_to_enum["UNKNOWN"] = "UNKNOWN"

    current_enum = branch_to_enum.get(current_branch, "UNKNOWN")

    lines = []
    lines.append("#ifndef GIT_INFO_H")
    lines.append("#define GIT_INFO_H")
    lines.append("")
    lines.append("typedef enum {")

    for i, enum_name in enumerate(enum_entries):
        comma = "," if i < len(enum_entries) - 1 else ""
        lines.append(f"    {enum_name} = {i}{comma}")

    lines.append("} branch_t;")
    lines.append("")
    lines.append(f"#define CURRENT_BRANCH {current_enum}")
    lines.append(f"#define BUILD_GIT_HASH_HEX {git_hash_hex}")
    lines.append(f"#define BUILD_GIT_HASH_DEC {git_hash_dec}")
    lines.append("")
    lines.append("#endif")

    output_path.write_text("\n".join(lines) + "\n")
    print(f"Generated {output_path}")


if __name__ == "__main__":
    generate_header(OUTPUT_FILE)