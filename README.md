# 16714_extra_credit
## Code and Video Resources

### MATLAB Scripts
The `.m` codes are available in `/scripts`.

### Video Examples
- **LQR**: [https://youtu.be/emXJfF-Ou2w](https://youtu.be/emXJfF-Ou2w)
- **MPC**: [https://youtu.be/GZaHwXqBMV8](https://youtu.be/GZaHwXqBMV8)
- **LQG**: [https://youtu.be/-kz9NUTAU_8](https://youtu.be/-kz9NUTAU_8)

---

## Questions

### 1.1
- **Q** and **R** are cost terms for states and control.
- Larger **R** results in less aggressive control.
- Larger **Q_xx** makes the end-effector reach the goal position faster (reduces \((x-x_T)\) cost).
- Larger **Q_vv** slows down the system (reduces \((v-v_T)\) cost).

### 1.2
- **Control constraint** is applied to acceleration, causing the trajectory to have a larger radius.

### 1.3
- **Velocity constraint** slows down the system.

### 1.4
- **Q** and **R** are the same as in the LQR problem.
- **W** is the covariance of Gaussian white noise acting on the dynamics, affecting reliability.
- **V** is the covariance of Gaussian white noise acting on the measurement.
  - Larger **V** or **W** makes the system trajectory more random and less accurate in following the original LQR trajectory.
