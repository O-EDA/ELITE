from __future__ import annotations

import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from engine.flow import ROOT, PathData, load_case_literal, manhattan_length, bend_count, sort_routes


STAGES = [
    ("A*", "01_astar/astar_paths.txt"),
    ("Detour", "02_detour/detour_paths.txt"),
    ("LUT", "03_lut/lut_paths.txt"),
    ("Final", "04_final/final_paths.txt"),
]

MIXED_STAGES = [
    ("Match A*", "matching/01_astar/astar_paths.txt"),
    ("Match Detour", "matching/02_detour/detour_paths.txt"),
    ("Match LUT", "matching/03_lut/lut_paths.txt"),
    ("General", "05_general_astar/general_paths.txt"),
    ("Final", "final_paths.txt"),
]


def existing_case_outputs(args: list[str]) -> list[Path]:
    if args:
        return [Path(arg).resolve() for arg in args]
    case_names = {path.parent.name for path in (ROOT / "cases").glob("*/path.yml")}
    outputs = ROOT / "outputs"
    case_dirs = [
        outputs / name
        for name in case_names
        if (outputs / name / "metrics.json").exists()
    ]
    order = {name: idx for idx, name in enumerate(["OPA4", "OPA9", "OPA16", "ODL6", "WCA8", "BBA8A", "BBA8B", "BBA14"])}
    return sorted(case_dirs, key=lambda path: (order.get(path.name, 999), path.name))


def read_stage(case_output: Path, rel_path: str) -> PathData:
    path = case_output / rel_path
    if not path.exists():
        return []
    return load_case_literal(path)


def case_stages(case_output: Path) -> list[tuple[str, str]]:
    if (case_output / "matching").exists() and (case_output / "05_general_astar").exists():
        return MIXED_STAGES
    return STAGES


def plot_paths(ax, paths: PathData, title: str, show_metrics: bool = True) -> None:
    ax.set_aspect("equal", adjustable="box")
    ax.set_title(title, fontsize=9)
    ax.tick_params(labelsize=6, length=2)
    ax.grid(True, linewidth=0.25, alpha=0.35)
    if not paths:
        ax.text(0.5, 0.5, "missing", transform=ax.transAxes, ha="center", va="center", fontsize=8)
        return
    colors = plt.cm.tab20.colors
    for idx, route in enumerate(paths):
        xs = [point[0] for point in route]
        ys = [point[1] for point in route]
        color = colors[idx % len(colors)]
        ax.plot(xs, ys, color=color, linewidth=1.4)
        ax.scatter([xs[0], xs[-1]], [ys[0], ys[-1]], color=color, s=10)
    all_x = [point[0] for route in paths for point in route]
    all_y = [point[1] for route in paths for point in route]
    pad_x = max(2, int((max(all_x) - min(all_x)) * 0.04))
    pad_y = max(2, int((max(all_y) - min(all_y)) * 0.04))
    ax.set_xlim(min(all_x) - pad_x, max(all_x) + pad_x)
    ax.set_ylim(min(all_y) - pad_y, max(all_y) + pad_y)
    if show_metrics:
        lengths = [manhattan_length(route) for route in paths]
        bends = [bend_count(route) for route in paths]
        ax.text(
            0.02,
            0.02,
            f"L {min(lengths)}..{max(lengths)}  B {min(bends)}..{max(bends)}",
            transform=ax.transAxes,
            fontsize=7,
            ha="left",
            va="bottom",
            bbox={"facecolor": "white", "edgecolor": "none", "alpha": 0.75, "pad": 1.5},
        )


def write_case_flow(case_output: Path) -> Path:
    stages = case_stages(case_output)
    fig, axes = plt.subplots(1, len(stages), figsize=(3.0 * len(stages), 3.2), constrained_layout=True)
    if len(stages) == 1:
        axes = [axes]
    fig.suptitle(case_output.name, fontsize=12)
    for ax, (stage_name, rel_path) in zip(axes, stages):
        plot_paths(ax, read_stage(case_output, rel_path), stage_name)
    out = case_output / "flow_compare.png"
    fig.savefig(out, dpi=220)
    plt.close(fig)
    return out


def write_overview(case_outputs: list[Path]) -> Path:
    max_cols = max(len(case_stages(case_output)) for case_output in case_outputs)
    fig, axes = plt.subplots(
        len(case_outputs),
        max_cols,
        figsize=(3.0 * max_cols, max(2.4, 2.15 * len(case_outputs))),
        constrained_layout=True,
    )
    if len(case_outputs) == 1:
        axes = [axes]
    for row, case_output in enumerate(case_outputs):
        stages = case_stages(case_output)
        for col in range(max_cols):
            ax = axes[row][col]
            if col >= len(stages):
                ax.axis("off")
                continue
            stage_name, rel_path = stages[col]
            title = stage_name if row == 0 else stage_name if len(stages) != len(STAGES) else ""
            plot_paths(ax, read_stage(case_output, rel_path), title, show_metrics=False)
            if col == 0:
                ax.set_ylabel(case_output.name, fontsize=8)
    out = ROOT / "outputs" / "all_cases_flow_overview.png"
    fig.savefig(out, dpi=220)
    plt.close(fig)
    return out


def main() -> None:
    case_outputs = existing_case_outputs(sys.argv[1:])
    if not case_outputs:
        raise SystemExit("No case outputs found. Run run_all.py or main.py first.")
    for case_output in case_outputs:
        out = write_case_flow(case_output)
        print(f"wrote {out}")
    overview = write_overview(case_outputs)
    print(f"wrote {overview}")


if __name__ == "__main__":
    main()
