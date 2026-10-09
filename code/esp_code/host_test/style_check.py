#!/usr/bin/env python3
"""
Style test of the project C code (no ESP-IDF and no hardware).

    make style                                  check components/ and main/
    python3 style_check.py path/a.c path/dir    check other files or folders

Rules:
    indent  - 4 spaces per level, no tabs. The line after a "{" goes 4 spaces
              more and the "}" goes at the same column as the line that opened
              it. Lines that continue a sentence (inside "( )" or after an
              operator) can be aligned freely.
    brace   - the "{" goes at the end of the line of the function, if, for,
              struct... never alone in the next line.
    naming  - names with "_" (snake_case or UPPER_CASE), no camelCase,
              PascalCase or mixes like Acc_lin_X. Names of external APIs
              (FreeRTOS, ESP-IDF, micro-ROS) are skipped: they are taken
              from the mocks of host_test/mocks/.
    define  - the names of the #define in UPPER_CASE.
    space   - a space between if/for/while/switch and the "(": "if (".
    pointer - the "*" next to the name: "float *x", not "float* x".
    else    - "} else {" in the same line, never "else" at the start of a line.
    trailing- no spaces at the end of the lines.
    header  - every file starts with the "/** Made by ..." comment.
    length  - (warning) lines of more than 90 characters.

Exit code 1 if there is any error. The warnings do not fail.
"""
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, ".."))
DEFAULT_PATHS = [os.path.join(ROOT, "components"), os.path.join(ROOT, "main")]
EXCLUDED_DIRS = {"micro_ros_espidf_component", "build", "unity", ".git"}

INDENT = 4
MAX_LEN = 90
RULES = ("indent", "brace", "naming", "define", "space", "pointer", "else", "trailing", "header")
IDENT_RE = re.compile(r"\b[A-Za-z_][A-Za-z0-9_]*\b")


# ============================================================
#                         Helpers
# ============================================================
def find_files(paths):
    files = []
    for p in paths:
        if os.path.isfile(p):
            files.append(p)
            continue
        for dirpath, dirnames, filenames in os.walk(p):
            dirnames[:] = sorted(d for d in dirnames if d not in EXCLUDED_DIRS)
            for f in sorted(filenames):
                if f.endswith((".c", ".h")):
                    files.append(os.path.join(dirpath, f))
    return files


def strip_code(lines):
    """Returns the lines with comments and strings replaced by spaces
    (same length, so the columns do not change)."""
    out = []
    in_block = False
    for line in lines:
        res = []
        i = 0
        n = len(line)
        while i < n:
            c = line[i]
            if in_block:
                if line.startswith("*/", i):
                    in_block = False
                    res.append("  ")
                    i += 2
                else:
                    res.append(" ")
                    i += 1
            elif line.startswith("/*", i):
                in_block = True
                res.append("  ")
                i += 2
            elif line.startswith("//", i):
                res.append(" " * (n - i))
                break
            elif c in "\"'":
                # Keep the quotes, blank the content
                j = i + 1
                while j < n and line[j] != c:
                    j += 2 if line[j] == "\\" else 1
                j = min(j, n - 1)
                res.append(c + " " * (j - i - 1) + (line[j] if j > i else ""))
                i = j + 1
            else:
                res.append(c)
                i += 1
        out.append("".join(res))
    return out


def external_names():
    """Names of the external APIs, taken from the mocks."""
    names = set()
    for f in find_files([os.path.join(HERE, "mocks")]):
        with open(f, encoding="utf-8", errors="replace") as fh:
            code = "\n".join(strip_code(fh.read().split("\n")))
        names.update(IDENT_RE.findall(code))
    return names


def is_bad_name(name):
    """True if the name mixes upper and lower case letters."""
    return re.search(r"[a-z]", name) is not None and re.search(r"[A-Z]", name) is not None


# ============================================================
#                         Checker
# ============================================================
def check_file(path, external, errors, warnings):
    with open(path, encoding="utf-8", errors="replace") as fh:
        lines = fh.read().split("\n")
    code = strip_code(lines)

    stack = []          # indent of the line that opened each "{"
    saved = []          # stacks saved at each #if (the #else branches restart from there)
    paren = 0           # open "(" and "[" that continue in the next lines
    prev_end = ";"      # last char of the previous code line
    prev_code = ""      # previous code line
    stmt_indent = 0     # indent of the line where the current sentence started
    after_open = None   # indent of the "{" opened in the previous code line
    pp_between = False  # a #if was between that "{" and this line
    in_macro = False    # inside a #define with "\"
    seen_names = set()

    def error(num, rule, msg):
        errors.append((path, num, rule, msg))

    # --- Header "/** Made by ..." ---
    first = next((i for i, l in enumerate(lines) if l.strip()), None)
    if first is None or not lines[first].strip().startswith("/**") \
            or not any("Made by" in l for l in lines[first:first + 5]):
        error((first or 0) + 1, "header", "the file must start with the '/** Made by ...' comment")

    for num, (raw, src) in enumerate(zip(lines, code), 1):
        stripped = src.strip()
        lead = raw[: len(raw) - len(raw.lstrip(" \t"))]
        indent = len(lead)

        if raw != raw.rstrip():
            error(num, "trailing", "spaces at the end of the line")
        if len(raw) > MAX_LEN:
            warnings.append((path, num, "length", f"{len(raw)} characters (max {MAX_LEN})"))

        bad_indent = "\t" in lead  # only one indent error for each line
        if bad_indent:
            error(num, "indent", "tab in the indentation")

        # --- Preprocessor ---
        if in_macro or stripped.startswith("#"):
            directive = stripped.lstrip("#").strip().split(" ")[0] if not in_macro else ""
            if directive in ("if", "ifdef", "ifndef"):
                saved.append(list(stack))
                pp_between = True
            elif directive in ("else", "elif") and saved:
                stack = list(saved[-1])
                pp_between = True
            elif directive == "endif" and saved:
                saved.pop()
            if directive == "define":
                m = re.match(r"#\s*define\s+([A-Za-z_]\w*)", stripped)
                if m and re.search(r"[a-z]", m.group(1)):
                    error(num, "define", f"'{m.group(1)}' must be in UPPER_CASE")
            if directive == "define" or in_macro:
                for name in IDENT_RE.findall(src):
                    if is_bad_name(name) and name not in external and name not in seen_names:
                        seen_names.add(name)
                        error(num, "naming", f"'{name}' mixes upper and lower case")
            in_macro = raw.rstrip().endswith("\\")
            continue

        if not stripped:
            continue

        # --- Naming ---
        # The ROS type names (ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu)) are external
        for name in IDENT_RE.findall(re.sub(r"ROSIDL_\w+\([^)]*\)", "", src)):
            if is_bad_name(name) and name not in external and name not in seen_names:
                seen_names.add(name)
                error(num, "naming", f"'{name}' mixes upper and lower case, use '_'")

        # --- Space after if/for/while/switch ---
        for m in re.finditer(r"\b(if|for|while|switch)\(", src):
            error(num, "space", f"'{m.group(1)}(' without space, use '{m.group(1)} ('")

        # --- "*" next to the name: "float *x" ---
        for m in re.finditer(r"\b([A-Za-z_]\w*)\*+\s+([A-Za-z_]\w*)", src):
            error(num, "pointer", f"'{m.group(1)}* {m.group(2)}', use '{m.group(1)} *{m.group(2)}'")

        # --- "} else" in the same line ---
        if re.match(r"else\b", stripped):
            error(num, "else", "'else' at the start of the line, use '} else {'")

        # --- Brace alone in its line ---
        if stripped.startswith("{") and prev_end not in ";{},":
            error(num, "brace", "'{' alone in its line, put it at the end of the previous line")

        # --- Indentation ---
        continuation = paren > 0 or not (
            prev_end in ";{},):" or re.search(r"\b(else|do)$", prev_code)
        ) if num > 1 else False

        if not continuation:
            stmt_indent = indent
            if not bad_indent and indent % INDENT:
                bad_indent = True
                error(num, "indent", f"{indent} spaces, it must be a multiple of {INDENT}")

            if not bad_indent and after_open is not None:
                expected = [after_open] if stripped.startswith("}") else [after_open + INDENT]
                if pp_between and not stripped.startswith("}"):
                    expected.append(after_open + 2 * INDENT)
                if indent not in expected:
                    bad_indent = True
                    error(num, "indent",
                          f"{indent} spaces after '{{', expected {expected[0]}")

        # --- Braces of this line ---
        # The content of extern "C" { } is not indented
        extern_c = stripped.startswith("extern") and '"' in stripped
        after_open = None
        line_start = True
        for ch in src:
            if ch == "{":
                stack.append(stmt_indent)
                after_open = stmt_indent - INDENT if extern_c else stmt_indent
                line_start = False
            elif ch == "}":
                if stack:
                    opened = stack.pop()
                    if line_start and indent != opened and not continuation and not bad_indent:
                        error(num, "indent",
                              f"'}}' at {indent} spaces, the '{{' was at {opened}")
                after_open = None
                line_start = False
            elif ch == "(" or ch == "[":
                paren += 1
                line_start = False
            elif ch == ")" or ch == "]":
                paren = max(0, paren - 1)
                line_start = False
            elif not ch.isspace():
                line_start = False
        if after_open is not None:
            pp_between = False

        prev_code = stripped
        prev_end = stripped[-1]


# ============================================================
#                           Main
# ============================================================
def main():
    paths = sys.argv[1:] or DEFAULT_PATHS
    files = find_files(paths)
    external = external_names()

    errors = []
    warnings = []
    for f in files:
        check_file(f, external, errors, warnings)

    for kind, items in (("warning", warnings), ("error", errors)):
        for path, num, rule, msg in sorted(items, key=lambda e: (e[0], e[1])):
            shown = os.path.relpath(path) if os.path.abspath(path).startswith(ROOT) else path
            print(f"{shown}:{num}: {kind} [{rule}] {msg}")

    counts = ", ".join(f"{r}: {sum(1 for e in errors if e[2] == r)}" for r in RULES)
    print()
    print(f"Files: {len(files)}  Errors: {len(errors)}  Warnings: {len(warnings)}")
    print(f"  errors   ({counts})")
    print(f"  warnings (length > {MAX_LEN}: {len(warnings)})")
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
