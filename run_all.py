from __future__ import annotations

import json
import sys
from pathlib import Path

from engine.flow import ROOT, write_csv, write_text
from main import run_case


CASE_ORDER = ["OPA4", "OPA9", "OPA16", "ODL6", "BBA8A", "BBA8B", "BBA14"]
EXCLUDED_BY_DEFAULT = {"toy_case"}


def flatten_summary(summary: dict[str, object]) -> dict[str, object]:
    lut = summary.pop("lut", {}) or {}
    summary["lut_accepted"] = bool(lut.get("accepted", False))
    summary["lut_reason"] = str(lut.get("reason", ""))
    return summary


def discover_cases() -> list[Path]:
    case_dir = ROOT / "cases"
    by_name = {path.parent.name: path for path in case_dir.glob("*/path.yml")}
    for name in EXCLUDED_BY_DEFAULT:
        by_name.pop(name, None)
    ordered = [by_name.pop(name) for name in CASE_ORDER if name in by_name]
    ordered.extend(by_name[name] for name in sorted(by_name))
    return ordered


def main() -> None:
    paths = [Path(arg).resolve() for arg in sys.argv[1:]] if len(sys.argv) > 1 else discover_cases()
    summaries = []
    for path in paths:
        print(f"running {path.parent.name} ...", flush=True)
        summaries.append(flatten_summary(run_case(path)))
    outputs = ROOT / "outputs"
    write_csv(outputs / "summary.csv", summaries)
    write_text(outputs / "summary.json", json.dumps(summaries, indent=2) + "\n")
    print(f"wrote {outputs / 'summary.csv'}")


if __name__ == "__main__":
    main()
