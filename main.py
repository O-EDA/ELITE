from __future__ import annotations

import json
import shutil
import sys
from pathlib import Path

from engine import flow


def prepare_output_dir(path_yml: Path, case: dict[str, object]) -> Path:
    case_name = str(case.get("case", path_yml.parent.name))
    out_dir = flow.ROOT / "outputs" / case_name
    if out_dir.exists():
        shutil.rmtree(out_dir)
    flow.mkdir(out_dir)
    flow.write_text(out_dir / "path.yml", path_yml.read_text(encoding="utf-8"))
    return out_dir


def run_matching_stage(
    out_dir: Path,
    case: dict[str, object],
    matching_nets: list[int],
    *,
    nested_output: bool,
) -> tuple[dict[int, list[flow.Point]], dict[str, object], list[dict[str, object]]]:
    stage_dir = flow.mkdir(out_dir / "matching") if nested_output else out_dir
    return flow.run_matching_pipeline(stage_dir, case, matching_nets)


def run_general_stage(
    out_dir: Path,
    case: dict[str, object],
    matching_paths: dict[int, list[flow.Point]],
    matching_nets: list[int],
    general_nets: list[int],
) -> dict[int, list[flow.Point]]:
    return flow.run_general_routing(
        out_dir,
        case,
        general_nets,
        [matching_paths[idx] for idx in matching_nets],
        matching_nets,
    )


def merge_final_paths(
    case: dict[str, object],
    matching_paths: dict[int, list[flow.Point]],
    general_paths: dict[int, list[flow.Point]],
) -> flow.PathData:
    final_paths: flow.PathData = []
    for idx in range(len(case["endpoints"])):
        if idx in matching_paths:
            final_paths.append(matching_paths[idx])
        elif idx in general_paths:
            final_paths.append(general_paths[idx])
        else:
            raise ValueError(f"No final path for net {idx}")
    return final_paths


def write_results(
    out_dir: Path,
    case: dict[str, object],
    final_paths: flow.PathData,
    rows: list[dict[str, object]],
    lut_gate: dict[str, object],
    matching_nets: list[int],
    general_nets: list[int],
) -> dict[str, object]:
    flow.write_csv(out_dir / "metrics.csv", rows)
    summary = flow.summarize(case, final_paths, preserve_order=bool(general_nets))
    summary["lut"] = lut_gate
    if general_nets:
        summary["general_nets"] = list(general_nets)
    flow.write_text(out_dir / "metrics.json", json.dumps(summary, indent=2) + "\n")
    flow.write_case_literal(out_dir / "final_paths.txt", final_paths)
    return summary


def run_case(path_yml: Path) -> dict[str, object]:
    case = flow.load_case(path_yml)
    matching_nets = flow.matching_net_indices(case)
    general_nets = flow.general_net_indices(case)
    out_dir = prepare_output_dir(path_yml, case)

    matching_paths, lut_gate, rows = run_matching_stage(
        out_dir,
        case,
        matching_nets,
        nested_output=bool(general_nets),
    )

    general_paths: dict[int, list[flow.Point]] = {}
    if general_nets:
        general_paths = run_general_stage(out_dir, case, matching_paths, matching_nets, general_nets)
        rows += flow.stage_metrics(
            "general_astar",
            [general_paths[idx] for idx in general_nets],
            flow.subset_case(case, general_nets, matching_only=False),
            preserve_order=True,
        )

    final_paths = merge_final_paths(case, matching_paths, general_paths)
    if general_nets:
        rows += flow.stage_metrics("final", final_paths, case, preserve_order=True)
    return write_results(out_dir, case, final_paths, rows, lut_gate, matching_nets, general_nets)


def main() -> None:
    if len(sys.argv) != 2:
        print("Usage: python main.py <path.yml>", file=sys.stderr)
        raise SystemExit(2)
    summary = run_case(Path(sys.argv[1]).resolve())
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
