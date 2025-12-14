# DRL-Guided Constraint Funneling System

A hierarchical multi-agent control architecture combining **Deep Reinforcement Learning (DRL)** with **Model Predictive Control (MPC)** for coordinated robot navigation.

![Architecture](https://img.shields.io/badge/Architecture-Hierarchical_DRL_+_MPC-blue)
![Agents](https://img.shields.io/badge/Agents-4_Robots-green)
![Simulator](https://img.shields.io/badge/Simulator-F1TENTH_Gym-orange)

---

## 🏗️ Architecture Overview

```
┌──────────────────────────────────────────────────────────────────────────┐
│                    LAYER I: STRATEGIC (DRL Policy)                       │
│                         Frequency: ~5 Hz                                 │
├──────────────────────────────────────────────────────────────────────────┤
│  Inputs:                           │  Outputs:                           │
│  • Swarm centroid (x, y)           │  • Shape(t): a, b, θ                │
│  • Swarm spread                    │  • Velocity(t): vx, vy              │
│  • Goal direction/distance         │                                     │
│  • Corridor width (LIDAR)          │  Center = Team Centroid (fixed)     │
│  • SE-MPC feasibility rate ←───────┤  ← KEY FEEDBACK LOOP!               │
└──────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌──────────────────────────────────────────────────────────────────────────┐
│                 LAYER II: TACTICAL (SE-MPC Solvers)                      │
│                 CasADi/IPOPT - Frequency: ≥20 Hz                         │
├──────────────────────────────────────────────────────────────────────────┤
│  Per-Agent Optimization:                                                 │
│                                                                          │
│  min  J = w_vel·|V_agent - V_patch|² + w_center·|X - P_patch|²          │
│   U                                                                      │
│                                                                          │
│  HARD CONSTRAINTS:                                                       │
│  ├─ G_Containment: X(k) ∈ Ellipsoid(k)   ← Stay in patch!               │
│  ├─ G_Safety:      dist(i,j) ≥ ε_min     ← No inter-collision           │
│  └─ G_Feasibility: Ackermann dynamics    ← Car-like constraints         │
│                                                                          │
│  If MPC fails → DRL is PENALIZED → Learns to "wait" for agents!         │
└──────────────────────────────────────────────────────────────────────────┘
```

---

## 🎯 Key Innovation

**The Patch Learns to Wait!**

Traditional approaches penalize agents for leaving the formation. Our approach flips this:

1. **DRL outputs patch velocity** (`vx`, `vy`)
2. **SE-MPC tries to track** with hard containment constraint
3. **If MPC fails** (agents can't keep up) → **DRL gets negative reward**
4. **DRL learns** to output velocities that agents CAN achieve
5. **Result**: Optimal team speed that respects physical constraints!

---

## 📁 File Structure

```
your-repo/
├── scripts/
│   └── drl_patch_funnel.py      # Main script
├── requirements.txt              # Dependencies
├── README.md                     # This file
└── f1tenth_gym/                  # F1TENTH simulator (submodule or copy)
```

---

## 🚀 Installation

### Option 1: Add to F1TENTH Examples (Recommended)

```bash
# Clone F1TENTH gym
git clone https://github.com/f1tenth/f1tenth_gym.git
cd f1tenth_gym

# Install F1TENTH gym
pip install -e .

# Copy the script to examples
cp /path/to/drl_patch_funnel.py examples/

# Run from f1tenth_gym directory
python examples/drl_patch_funnel.py --mode demo
```

### Option 2: Standalone (Requires F1TENTH installed)

```bash
# Install F1TENTH gym first
pip install f1tenth-gym

# Or from source
git clone https://github.com/f1tenth/f1tenth_gym.git
cd f1tenth_gym && pip install -e .

# Run from anywhere (F1TENTH must be importable)
python drl_patch_funnel.py --mode demo
```

### Install Dependencies

```bash
pip install -r requirements.txt
```

---

## 🎮 Usage

### Demo Mode (Heuristic Policy)
```bash
python drl_patch_funnel.py --mode demo
```

Watch the system in action with a hand-crafted policy. Good for testing setup.

### Training Mode
```bash
python drl_patch_funnel.py --mode train --timesteps 200000
```

Train the DRL policy. Checkpoints saved automatically.

### Resume Training
```bash
python drl_patch_funnel.py --mode train --resume patch_funnel_models/run_xxx/best_model
```

### Custom Training Parameters
```bash
python drl_patch_funnel.py --mode train \
    --timesteps 500000 \
    --checkpoint-freq 20000
```

---

## 📊 What You'll See

### Visualization Window

- **Cyan Ellipse**: The deformable patch (control funnel)
- **Blue Arrow**: `V_patch` - patch velocity from DRL
- **Colored Dots**: Agents (R0, R1, R2, R3)
- **✓**: MPC solved successfully
- **✗**: MPC failed (patch too aggressive)
- **Gold Star**: Team goal

### Console Output

```
[Episode 10] Avg Reward: 245.3, MPC Feasibility: 92.5%
   🏆 New best model! (reward: 245.3)

💾 Checkpoint saved: patch_funnel_models/run_xxx/checkpoint_10000
```

---

## 🔧 Configuration

### DRL Action Space (5 dimensions)

| Parameter | Range | Description |
|-----------|-------|-------------|
| `a` | [1.2, 4.0] | Semi-major axis (meters) |
| `b` | [1.2, 4.0] | Semi-minor axis (meters) |
| `θ` | [0, π] | Ellipsoid orientation (radians) |
| `vx` | [-5, 8] | Patch velocity X (m/s) |
| `vy` | [-5, 8] | Patch velocity Y (m/s) |

### SE-MPC Hard Constraints

| Constraint | Type | Description |
|------------|------|-------------|
| `G_Containment` | HARD | Trajectory must stay inside ellipsoid |
| `G_Safety` | HARD | Min distance to neighbors ≥ 0.6m |
| `G_Feasibility` | HARD | Ackermann dynamics, v ∈ [0.5, 10] m/s |

### Reward Structure

| Component | Weight | Description |
|-----------|--------|-------------|
| Progress | +5.0 | Team closer to goal |
| V_toward_goal | +2.0 | Patch velocity toward goal |
| Wall collision | -30.0 | Patch larger than safe size |
| **MPC Failure** | **-100.0** | **DRL penalized if agents can't keep up!** |
| Agent inside | +2.0 | Per agent inside patch |
| All inside | +15.0 | Bonus if all 4 inside |

---

## 📈 Training Tips

1. **Start with demo mode** to verify setup
2. **Monitor MPC feasibility** - should increase over training
3. **Check TensorBoard** for learning curves:
   ```bash
   tensorboard --logdir=patch_funnel_models/tensorboard
   ```
4. **Resume from checkpoints** if training is interrupted
5. **Expect ~100k-200k timesteps** for reasonable behavior

---

## 🧪 Technical Details

### Dependencies

- **CasADi**: Symbolic optimization (MPC solver)
- **IPOPT**: Interior point optimizer (via CasADi)
- **Stable-Baselines3**: PPO implementation
- **F1TENTH Gym**: Multi-agent racing simulator
- **PyTorch**: Neural network backend

### Dynamics Model

Ackermann (Bicycle) model:
```
ẋ = v · cos(θ)
ẏ = v · sin(θ)
θ̇ = (v/L) · tan(δ)
v̇ = a
```

Where `L = 0.33m` (wheelbase), `δ` is steering angle, `a` is acceleration.

---

## 📝 Citation

If you use this work, please cite:

```bibtex
@misc{drl_patch_funnel,
  title={DRL-Guided Constraint Funneling for Multi-Agent Coordination},
  author={Your Name},
  year={2024},
  url={https://github.com/your-repo}
}
```

---

## 📄 License

MIT License - See LICENSE file for details.

---

## 🤝 Acknowledgments

- [F1TENTH](https://f1tenth.org/) - Autonomous racing platform
- [CasADi](https://web.casadi.org/) - Symbolic optimization framework
- [Stable-Baselines3](https://stable-baselines3.readthedocs.io/) - RL algorithms

