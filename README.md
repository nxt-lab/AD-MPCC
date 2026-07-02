<h1 align="center">AD-MPCC: Adaptive Differentiable Model Predictive Contouring Control for Autonomous Racing</h1>

<p align="center">
  <a href="https://2026.ieee-iros.org/"><img src="https://img.shields.io/badge/IROS-2026-blue.svg" alt="IROS 2026"/></a>
  <a href="https://arxiv.org/abs/2607.00141"><img src="https://img.shields.io/badge/arXiv-2607.00141-b31b1b.svg" alt="arXiv"/></a>
  <a href="https://github.com/nxt-lab/AD-MPCC"><img src="https://img.shields.io/badge/GitHub-AD--MPCC-black?logo=github" alt="GitHub"/></a>
  <a href="https://github.com/f1tenth/f1tenth_gym"><img src="https://img.shields.io/badge/F1TENTH_Gym-black?logo=github" alt="F1TENTH Gym"/></a>
</p>

<p align="center">
  <b>Nam T. Nguyen</b><sup>1,†</sup>&emsp;
  <b>Binh Nguyen</b><sup>1,†</sup>&emsp;
  <b>Ahmad Amine</b><sup>2</sup>&emsp;
  <b>Thanh Vo-Duy</b><sup>3</sup>&emsp;
  <b>Rahul Mangharam</b><sup>2</sup>&emsp;
  <b>Truong X. Nghiem</b><sup>1</sup>
</p>

<p align="center">
  <sup>1</sup>University of Central Florida &emsp;
  <sup>2</sup>University of Pennsylvania &emsp;
  <sup>3</sup>Hanoi University of Science and Technology<br>
  <sup>†</sup>Equal contribution
</p>

<p align="left">
  AD-MPCC integrates differentiable MPCC with online Pacejka tire parameter estimation to handle varying road-surface conditions in autonomous racing. AD-MPCC achieves faster lap times than baseline controllers on single-surface tracks and is the <b>only controller capable of completing multi-surface races</b>.
</p>

---

## Method Overview

<p align="center">
  <img src="showcase/AD_MPCC_architechture.png" width="600"/>
</p>

AD-MPCC combines two components operating at each time step:

- **Online Parameter Estimation (Sec. III-A):** A prior-regularized moving-horizon estimator (MHE) with exponentially decaying weights rapidly updates Pacejka tire parameters to capture surface transitions in real time.
- **Differentiable MPCC (Sec. III-B)**: Compute the sensitivity of the MPCC solution w.r.t. objective weights via the implicit function theorem (IFT) to find optimal MPCC weights.
- **PaIML (Sec. III–C):** A Pacejka-informed machine learning model (PaIML) is trained offline to approximate these optimal weights from a 5-dimensional input, enabling real-time weight adaptation.

---

## Simulation Results

<!-- <p align="center">
  <a href="https://www.youtube.com/watch?v=6dc-kbvjNFE">
    <img src="https://img.shields.io/badge/-%20-FF0000?logo=youtube&logoColor=white&style=flat-square" height="45" alt="Watch on YouTube"/>
  </a>
</p> -->

<!-- <p align="center">
  <a href="https://www.youtube.com/watch?v=6dc-kbvjNFE">
    <img src="https://img.youtube.com/vi/6dc-kbvjNFE/maxresdefault.jpg" width="720" alt="AD-MPCC Demo"/>
  </a>
</p> -->


### Single-Surface Scenario

All four controllers are evaluated on a uniform road surface with $μ_{max}$ = 1.2 for 10 laps (Sec. IV-B). Diff-MPCC and AD-MPCC achieve approximately 11 s faster lap times than the baseline MPCC by adapting the objective weights online, while AD-MPCC further maintains the smallest mean lateral offset (0.926 m), demonstrating a better safety–speed trade-off.

<p align="center">
  <img src="showcase/IROS2026-single.gif" width="720" alt="Single-Surface Scenario"/>
</p>

<div align="center">

| Controller | Avg. Lap Time (s) | Avg. v_x (m/s) | Avg. Lateral Offset (m) | Avg. Comp. Time (ms) |
|:---:|:---:|:---:|:---:|:---:|
| MPCC | 75.57 | 11.38 | 1.953 | 20.45 |
| A-MPCC | 75.62 | 11.41 | 1.815 | 21.21 |
| Diff-MPCC | **64.08** | **13.59** | 1.040 | 23.56 |
| AD-MPCC | 64.89 | 13.50 | **0.926** | 24.18 |

</div>

### Multi-Surface Scenario

The track surface varies from $μ_{max}$ = 0.7 to 1.2 across different sections (Sec. IV-C). Only AD-MPCC successfully completes the lap by jointly updating Pacejka parameters and MPCC weights in real time, while MPCC, A-MPCC, and Diff-MPCC all crash at the first slippery segment ($μ_{max}$ = 0.75) due to insufficient adaptation.

<p align="center">
  <img src="showcase/IROS2026-multiple.gif" width="720" alt="Multi-Surface Scenario"/>
</p>

<div align="center">

| Controller | Avg. Lap Time (s) | Avg. v_x (m/s) | Avg. Lateral Offset (m) |
|:---:|:---:|:---:|:---:|
| MPCC | crashed | — | — |
| A-MPCC | crashed | — | — |
| Diff-MPCC | crashed | — | — |
| **AD-MPCC** | **74.9** | **11.65** | **1.069** |

</div>

---

## Repository Structure

```
ADMPCC/
├── main/
│   ├── MPCC.py              # Baseline MPCC (fixed weights, fixed Pacejka)
│   ├── A_MPCC.py            # A-MPCC  (online MHE only)
│   ├── Diff_MPCC.py         # Diff-MPCC (PaIML weight adaptation only)
│   ├── AD_MPCC.py           # AD-MPCC  (MHE + PaIML, full method)
│   ├── common.py            # Shared config, utilities, and helpers
│   ├── scenarios.py         # Simulation scenario parameters
│   ├── diff_look.py         # KDTree-based PaIML lookup (used at runtime)
│   ├── diff_look_xgb.py     # XGBoost-based PaIML lookup (alternative)
│   ├── look_up.py           # PaIML training data loader
│   └── scale0.25_Oschersleben_waypoints.csv
├── requirements.txt
├── configs/
│   └── config_Oschersleben.yaml
├── maps/
│   └── Oschersleben/        # Map image and raceline waypoints
├── regulators/
│   ├── path_follow_mpcc_casadi.py       # Fixed-weight MPCC solver
│   ├── path_follow_diff_mpcc_casadi.py  # Differentiable MPCC solver
│   ├── get_look_table.py                # Spline lookup table for track
│   └── pure_pursuit.py                  # Waypoint rendering helper
├── PAIL_MPCC/
│   ├── dynamic.py           # Dynamic bicycle model (Numba JIT)
│   ├── closest_point.py     # Nearest point on trajectory
│   └── optimize_Pacejka.py  # MHE Pacejka parameter estimator
├── diff_data/               # Pre-generated PaIML training data (μ=0.5–1.2)
├── differientiable_MPCC_technique/   # Algorithm 1: IFT-based weight optimization (offline)
│   ├── casadi_outer_sensitivity.py   # Core IFT sensitivity computation (KKT sensitivity)
│   ├── MPCCsolver.py                 # MPCC NLP solver used during differentiation
│   ├── main_casadi_sensitivity.py    # Runner script — call with a friction value
│   ├── plot_new_weights.py           # Visualization of optimized weights
│   ├── slurm-script_diff.slurm      # HPC job script for cluster runs
│   └── scale0.25_TK20_log_*         # Input simulation logs (one per friction level)
├── collect_data_for_diff/   # Scripts to collect input simulation logs
├── showcase/                # Architecture figure and demo videos
└── results/                 # Simulation logs saved here at runtime
```

---

## Setup

> **Two separate Python environments are required:**
> - **Python 3.7.1** — for the four main controllers (`MPCC.py`, `A_MPCC.py`, `Diff_MPCC.py`, `AD_MPCC.py`) with F1TENTH Gym
> - **Python 3.13.1** — for the offline IFT weight optimization in `differientiable_MPCC_technique/`

### Environment 1 — Main Controllers (Python 3.7.1)

#### 1. Install F1TENTH Gym (multibody branch)

```bash
git clone https://github.com/atomyks/f1tenth_gym.git
cd f1tenth_gym
git checkout multibody
pip install -e .
```

#### 2. Install Dependencies

```bash
pip install -r requirements.txt
```

#### 3. Run Controllers

Run from the repository root:

```bash
python main/MPCC.py       # Baseline MPCC
python main/A_MPCC.py     # Adaptive MPCC  (online MHE)
python main/Diff_MPCC.py  # Differentiable MPCC (PaIML weights)
python main/AD_MPCC.py    # Full AD-MPCC (MHE + PaIML)
```

### Environment 2 — IFT Weight Optimization (Python 3.13.1)

#### 1. Install Dependencies

```bash
cd differientiable_MPCC_technique
pip install -r requirements_diff.txt
```

### 4. Log Output

After each run, a JSON log file is saved to the `results/` folder containing per-step records of:

| Field | Description |
|---|---|
| `time`, `x`, `y` | Simulation time and vehicle position |
| `vx`, `vy` | Longitudinal and lateral velocity |
| `yaw`, `yaw_rate`, `steer_angle` | Orientation and steering state |
| `theta` | Track progress variable |
| `acce`, `steering_rate` | Applied control inputs |
| `tracking_error` | Lateral offset from centerline |
| `BR/CR/DR/BF/CF/DF/CM` | Pacejka tire parameters (estimated or fixed) |
| `mu_x`, `mu_y` | Actual road friction coefficients |
| `time_compute` | Computation time per step |
| `lap_n` | Lap counter |

#### 2. Regenerating PaIML Training Data (Optional)

The pre-generated training data in `diff_data/` is sufficient to run all four controllers. If you wish to regenerate it from scratch, use the scripts in `differientiable_MPCC_technique/`, which implement Algorithm 1 (IFT-based sensitivity weight optimization, Sec. III-B of the paper).

```bash
cd differientiable_MPCC_technique
python main_casadi_sensitivity.py 0.5
python main_casadi_sensitivity.py 0.6
# ... repeat for 0.7, 0.8, 0.9, 1.0, 1.1, 1.2
```

Each run reads the corresponding input simulation log (`scale0.25_TK20_log_Oschersleben_full_Vinit_8.0friction<μ>`) and writes an optimized weight file (`friction<μ>_outer_steps100_pg_iters10_lr0.1`) that can be used to rebuild `diff_data/`.

---

## Citation

```bibtex
@misc{nguyen2026admpccadaptivedifferentiablemodel,
      title={AD-MPCC: Adaptive Differentiable Model Predictive Contouring Control for Autonomous Racing}, 
      author={Nam T. Nguyen and Binh Nguyen and Ahmad Amine and Thanh Vo-Duy and Rahul Mangharam and Truong X. Nghiem},
      year={2026},
      eprint={2607.00141},
      archivePrefix={arXiv},
      primaryClass={cs.RO},
      url={https://arxiv.org/abs/2607.00141}, 
}
```

---

## Contact

**Nam T. Nguyen** (Corresponding Author)<br>
Email: [nam.nguyen2@ucf.edu](mailto:nam.nguyen2@ucf.edu)<br>
Website: [namnguyenee2.github.io](https://namnguyenee2.github.io/)
