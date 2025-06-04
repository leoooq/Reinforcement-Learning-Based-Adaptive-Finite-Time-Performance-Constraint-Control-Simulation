## Paper: Reinforcement Learning-Based Adaptive  Finite-Time Performance Constraint Control  for Nonlinear Systems
## Overview

This project aims to reproduce and simulate a Reinforcement Learning-based Adaptive Finite-Time Performance Constraint Control method for a second-order nonlinear system using MATLAB. The control strategy combines techniques from Reinforcement Learning (RL), adaptive control, finite-time control theory, and Prescribed Performance Control (PPC). This ensures that the system state errors converge to a predefined, time-varying boundary within a finite time while optimizing a certain performance index.

The code primarily consists of:
1.  **Main Script (e.g., `main_simulation.m`)**: Sets simulation parameters, initial conditions, runs the ODE solver, processes results, and generates plots.
2.  **System Dynamics Function (`system_dynamics.m`)**: Defines the differential equations of the controlled system, the controller structure, neural network update laws, and adaptive laws.
3.  **RBF Neural Network Evaluation Function (`rbf_eval.m`)**: Calculates the output of the Radial Basis Function (RBF) neural network.
4.  **Signed Power Function (`signed_power.m`)**: A helper function to handle non-integer power operations that may arise in finite-time control.

## System Model

The second-order nonlinear system simulated in this project is described as:

*   **State Equations**:
    ```
    dx1(t)/dt = x2
    dx2(t)/dt = -4.9 * sin(x1) - x2 + u
    y(t) = x1
    ```
*   **Initial States**:
    ```
    x1(0) = 0.2
    x2(0) = -0.5
    ```
*   **Reference Trajectory (Desired Output `y_d`, denoted as `x_r`)**:
    ```
    x_r(t) = -0.4 * exp(-0.3*t) + 0.6 * sin(0.5*t) + 0.5 * cos(0.7*t)
    ```

## Controller Characteristics

*   **Adaptive Laws**: Used to estimate unknown system dynamics (or their bounds).
*   **Actor-Critic Neural Networks**: Employed to approximate the optimal control policy and the value function. Radial Basis Function (RBF) neural networks are used in this implementation.
*   **Finite-Time Convergence**: Ensures error convergence in finite time through specific controller design and Lyapunov stability analysis.
*   **Prescribed Performance Control (PPC)**: Guarantees that the tracking error evolves within predefined, time-varying bounds `|e(t)| < kappa(t)` using error transformation and a performance function `kappa(t)`.

## File Structure

*   `main_simulation.m` (or your chosen main script name): Main simulation program.
*   (If `system_dynamics`, `rbf_eval`, `signed_power` are in separate files, list them here. The current code embeds these within the main script.)

## How to Run

1.  Ensure you have MATLAB installed (no special toolboxes are typically required beyond the standard ODE solvers).
2.  Open the main script file (e.g., `main_simulation.m`) in the MATLAB editor.
3.  Click the "Run" button in the MATLAB editor or type the script name in the MATLAB command window and press Enter.
4.  After the simulation completes, several figures will be displayed, showing time histories of system states, tracking errors, control input, neural network weights, etc.

## Parameter Tuning

The parameters in the code significantly affect the simulation results. Key parameters include:

*   **Performance Function `kappa_i(t)` Parameters**:
    *   `k01, k02`: Initial width of the error funnel.
    *   `k_inf1, k_inf2`: Steady-state width of the error funnel.
    *   `l1, l2`: Convergence rate of the funnel. **Increasing `l1, l2` can speed up error convergence.**
*   **Controller Gains**:
    *   `eta_bar1, eta_bar2`: Gains for finite-time terms.
    *   `tau_1, tau_2`: Gains for finite-time terms.
    *   `zeta_1, zeta_2`: Weights for control energy in the cost function. **Decreasing `zeta` may allow larger control inputs for faster convergence.**
*   **Learning Rates and Adaptive Rates**:
    *   `gamma_a1, gamma_a2` (Actor learning rates)
    *   `gamma_c1, gamma_c2` (Critic learning rates)
    *   `gamma_1, gamma_2` (Update rates for adaptive parameters `U_hat`)
    *   `sigma_1, sigma_2` (Decay terms for adaptive parameters `U_hat`)
*   **RBF Neural Network Parameters**:
    *   `num_nodes`: Number of RBF nodes.
    *   `phi_width_sq`: RBF width parameter.
    *   `mu_centers`: RBF center locations.
*   **Finite-Time Exponents**: `delta_1, delta_2, delta_3, pi_const` (These are usually derived from theory and should generally not be changed arbitrarily).

**Debugging Tips**:
*   Start by tuning the performance function parameters `l1, l2` to achieve the desired error convergence rate according to the performance funnel.
*   If the control signal is too large or oscillatory, try increasing `zeta_1, zeta_2` or decreasing the learning rates.
*   Monitor the neural network weights and adaptive parameters `U_hat` to ensure they are converging and not exhibiting unstable behavior.

## Notes

*   This implementation may involve simplifications of certain complex terms (e.g., derivatives of virtual controls in `Tau_i` and `Nu_i` terms) from the original research paper, which could lead to deviations from the original paper's results.
*   The input to the RBF neural networks is assumed to be the scalar transformed error `xi_i`.
*   Numerical stability: Saturation of `xi_i` and protection for `(1-xi_i)` denominator terms have been implemented, but issues might still arise with extreme parameter settings.
*   Careful selection and tuning of parameters are crucial for achieving good performance.

## Future Work (Optional)

*   Implement the full, non-simplified versions of complex controller terms.
*   Conduct a more systematic optimization and sensitivity analysis of the parameters.
*   Apply this control methodology to other, more complex systems.

