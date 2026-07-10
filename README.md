# ELITE

Implementation of the ELITE routing flow proposed in ISEDA 2026.

## Citation

```bibtex
@INPROCEEDINGS{Yu2026ISEDA,
  author={Yu, Xiaofei and Wu, Yuchao and Yan, Haopeng and Tong, Yeyu and Ma, Yuzhe},
  booktitle={International Symposium of EDA (ISEDA)},
  title={ELITE: Efficient Lookup Table-Assisted Routing Engine for Photonic Integrated Circuits},
  year={2026},
  address={Singapore},
  month={May},
  pages={},
}
```

## Quick Start

Build and install the in-process C++ extension (Windows or Linux):

```bash
python -m pip install -e .
```

Windows requires Visual Studio 2022 Build Tools with the C++ workload. Linux
requires a C++17 compiler and Python development headers.

Run one case:

```bash
python main.py cases/BBA8A/path.yml
```

Run all cases and refresh `outputs/summary.csv`:

```bash
python run_all.py
```

Run the toy case:

```bash
python main.py cases/toy_case/path.yml
```

Regenerate visual checks:

```bash
python visualize_outputs.py
```

The Python flow calls the C++ A* and LUT kernels directly through `pybind11`.
No executable is launched and route data stays in memory. The text and CSV
files under `outputs/` are diagnostics only.

## Flow

```text
path.yml
  -> endpoints + obstacles + bbox
  -> matching-stage C++ A*
  -> detour-region detection
  -> diffuse / detour
  -> LUT bend reduction
  -> final bend insertion / U-bend length padding
  -> matched metrics
  -> congestion-aware C++ A* (general_groups, if any)
  -> RRR on general nets (if any)
  -> merged final paths + metrics
```

## Case Format

Each case is a `path.yml`.

Required geometry:

- `case`: case name used for the output directory
- `bbox`: `[xmin, xmax, ymin, ymax]` in grid coordinates
- `target_length`: scalar or per-net length target in grid units
- `obstacles`: static blocked rectangles, inclusive `[xmin, xmax, ymin, ymax]`
- `matching_groups[].name`, `matching_groups[].endpoints`
- `general_groups[].name`, `general_groups[].endpoints`: optional non-matching nets routed after matching
- `*.endpoints[].source`, `*.endpoints[].target`
- `*.endpoints[].source_direction`, `*.endpoints[].target_direction`

Loss and technology:

- `pitch_um`
- `waveguide_width_um`
- `general.spacing_um`, with top-level `spacing_um` as a fallback for general routing
- `loss.propagation_loss_per_um`
- `loss.crossing_loss`
- `loss.min_bend_radius_um`
- `loss.bend_loss_by_angle`, for allowed bend angles up to 90 degrees

## Outputs

Each case run writes:

```text
outputs/<case>/
  path.yml
  metrics.csv
  metrics.json
  final_paths.txt
```

`run_all.py` writes:

```text
outputs/summary.csv
outputs/summary.json
```

`visualize_outputs.py` writes:

```text
outputs/<case>/flow_compare.png
outputs/all_cases_flow_overview.png
```

## Repository Layout

- `main.py`: CLI wrapper
- `engine/flow.py`: Python flow orchestration, metrics, and routing stages
- `engine/detour_adapter.py`: contour detour and diffuse adapter
- `engine/bend_insertion.py`: final bend insertion operators
- `engine/astar/src/`: reusable C++ A* core
- `engine/lut/source/`: LUT optimizer and optional table generator
- `engine/lut/optimizer_lut_60_60_100_20_final`: canonical LUT table

## License

BSD 3-Clause License. See `LICENSE`.
