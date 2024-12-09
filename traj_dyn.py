"""
    Tracé de différents points avec gains dynamiques
    """

import numpy as np
import matplotlib.pyplot as plt

# Constants
L = 20e-3     # Length between the wheels (20 mm)
T = 0.01      # Time step (10 ms)
epsilon = 1e-3  # Convergence threshold
max_iterations = 10000  # Maximum number of iterations to avoid infinite loops

def dynamic_gains(rho, alpha, beta):
    if rho > 1.0:
        return 2.0, 20.0, -10.0
    elif rho > 0.5:
        return 1.5, 15.0, -8.0
    elif abs(alpha) < 0.1 and abs(beta) < 0.1:  # Straight-line condition
            return 1.0, 0.0, 0.0
    else:
        return 1.0, 10.0, -5.0

def run_simulation_with_colored_segments(x0, y0, theta0, x_goal, y_goal, theta_goal):
    X, Y, THETA = [x0], [y0], [theta0]
    gains_segments = []
    iteration = 0

    while iteration < max_iterations:
        x, y, theta = X[-1], Y[-1], THETA[-1]
        dx, dy = x_goal - x, y_goal - y
        rho = np.hypot(dx, dy)
        alpha = np.arctan2(dy, dx) - theta
        beta = -theta_goal - alpha

        k_rho, k_alpha, k_beta = dynamic_gains(rho, alpha, beta)
        gains_segments.append((x, y, k_rho, k_alpha, k_beta))

        sens = 1 if -np.pi / 2 <= alpha <= np.pi / 2 else -1
        v = k_rho * rho * sens
        omega = k_alpha * alpha + k_beta * beta
        gamma = np.arctan2(L * omega, abs(v)) * sens

        theta_dot = v / L * np.tan(gamma)
        x += T * v * np.cos(theta)
        y += T * v * np.sin(theta)
        theta += T * theta_dot

        X.append(x)
        Y.append(y)
        THETA.append(theta)

        if rho < epsilon and abs(theta - theta_goal) < epsilon:
            break

        iteration += 1

    return X, Y, THETA, gains_segments

def run_simulation_with_params(x0, y0, theta0, x_goal, y_goal, theta_goal, k_rho, k_alpha, k_beta):
    X, Y, THETA = [x0], [y0], [theta0]
    iteration = 0

    while iteration < max_iterations:
        x, y, theta = X[-1], Y[-1], THETA[-1]
        dx, dy = x_goal - x, y_goal - y
        rho = np.hypot(dx, dy)
        alpha = np.arctan2(dy, dx) - theta
        beta = -theta_goal - alpha

        sens = 1 if -np.pi / 2 <= alpha <= np.pi / 2 else -1
        v = k_rho * rho * sens
        omega = k_alpha * alpha + k_beta * beta
        gamma = np.arctan2(L * omega, abs(v)) * sens

        theta_dot = v / L * np.tan(gamma)
        x += T * v * np.cos(theta)
        y += T * v * np.sin(theta)
        theta += T * theta_dot

        X.append(x)
        Y.append(y)
        THETA.append(theta)

        if rho < epsilon and abs(theta - theta_goal) < epsilon:
            break

        iteration += 1

    return X, Y, THETA

# Simulating for multiple goal points
x0, y0, theta0 = 0, 0, 0
goal_points = [
    (5, 5, 0),
    (-5, 5, 0),
    (10, -10, 0),
    (-10, -10, np.pi / 2),
    (0, 15, np.pi / 4)
]

plt.figure(figsize=(12, 10))

for idx, (x_goal, y_goal, theta_goal) in enumerate(goal_points):
    X_dynamic, Y_dynamic, THETA_dynamic, gains_segments = run_simulation_with_colored_segments(
        x0, y0, theta0, x_goal, y_goal, theta_goal
    )

    fixed_k_rho, fixed_k_alpha, fixed_k_beta = 1.5, 15.0, -10.0
    X_fixed, Y_fixed, THETA_fixed = run_simulation_with_params(
        x0, y0, theta0, x_goal, y_goal, theta_goal, fixed_k_rho, fixed_k_alpha, fixed_k_beta
    )

    for i in range(len(gains_segments) - 1):
        x_start, y_start, k_rho, k_alpha, k_beta = gains_segments[i]
        x_end, y_end, _, _, _ = gains_segments[i + 1]
        if k_rho == 2.0:
            color = 'green'
        elif k_rho == 1.5:
            color = 'orange'
        else:
            color = 'red'
        plt.plot([x_start, x_end], [y_start, y_end], color=color, linewidth=2)

    plt.plot(X_fixed, Y_fixed, label=f'Fixed Gains (Goal {idx+1})', linestyle='--')
    plt.scatter(x_goal, y_goal, label=f'Goal {idx+1}', marker='o', s=100)

plt.scatter(x0, y0, color='blue', label='Start', s=100)
plt.title("Dynamic vs Fixed Gains for Multiple Goal Points")
plt.xlabel("x (m)")
plt.ylabel("y (m)")
plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
plt.grid()
plt.tight_layout()
plt.show()

"""
    import numpy as np
import matplotlib.pyplot as plt

# Constants
L = 20e-3     # Length between the wheels (20 mm)
T = 0.01      # Time step (10 ms)
epsilon = 1e-3  # Convergence threshold
max_iterations = 10000  # Maximum number of iterations to avoid infinite loops

def dynamic_gains(rho, alpha, beta):

    if rho > 1.0:  # Far from the goal
        return 2.0, 20.0, -10.0
    elif rho > 0.5:  # Moderately close to the goal
        return 1.5, 15.0, -8.0
    elif abs(alpha) < 0.1 and abs(beta) < 0.1:  # Straight-line condition
        return 1.0, 0.0, 0.0  # Focus on moving straight
    else:  # Close to the goal with angular correction
        return 1.0, 10.0, -5.0

def detect_minimal_progress(alpha_history, beta_history, threshold=0.05, window=4):
    if len(alpha_history) < window or len(beta_history) < window:
        return False

    recent_alpha = alpha_history[-window:]
    recent_beta = beta_history[-window:]

    return all(abs(a) < threshold for a in recent_alpha) and all(abs(b) < threshold for b in recent_beta)

def run_simulation_with_progress_detection(x0, y0, theta0, x_goal, y_goal, theta_goal):
    X, Y, THETA = [x0], [y0], [theta0]
    alpha_history, beta_history = [], []
    gains_segments = []
    iteration = 0

    while iteration < max_iterations:
        x, y, theta = X[-1], Y[-1], THETA[-1]
        dx, dy = x_goal - x, y_goal - y
        rho = np.hypot(dx, dy)
        alpha = np.arctan2(dy, dx) - theta
        beta = -theta_goal - alpha

        alpha_history.append(alpha)
        beta_history.append(beta)

        # Detect minimal progress
        if detect_minimal_progress(alpha_history, beta_history):
            k_rho, k_alpha, k_beta = 1.0, 0.0, 0.0  # Force straight-line motion
        else:
            k_rho, k_alpha, k_beta = dynamic_gains(rho, alpha, beta)

        gains_segments.append((x, y, k_rho, k_alpha, k_beta))

        sens = 1 if -np.pi / 2 <= alpha <= np.pi / 2 else -1
        v = k_rho * rho * sens
        omega = k_alpha * alpha + k_beta * beta
        gamma = np.arctan2(L * omega, abs(v)) * sens

        # Update state
        theta_dot = v / L * np.tan(gamma)
        x += T * v * np.cos(theta)
        y += T * v * np.sin(theta)
        theta += T * theta_dot

        X.append(x)
        Y.append(y)
        THETA.append(theta)

        # Check convergence
        if rho < epsilon and abs(theta - theta_goal) < epsilon:
            break

        iteration += 1

    return X, Y, THETA, gains_segments

# Simulation for (-10, -10)
x0, y0, theta0 = 0, 0, 0
x_goal, y_goal, theta_goal = -10, -10, np.pi / 2

X_progress, Y_progress, THETA_progress, gains_segments = run_simulation_with_progress_detection(
    x0, y0, theta0, x_goal, y_goal, theta_goal
)

# Plot the trajectory
plt.figure(figsize=(10, 8))
for i in range(len(gains_segments) - 1):
    x_start, y_start, k_rho, k_alpha, k_beta = gains_segments[i]
    x_end, y_end, _, _, _ = gains_segments[i + 1]
    if k_alpha == 0.0 and k_beta == 0.0:  # Minimal progress detected
        color = 'blue'
    elif k_rho == 2.0:
        color = 'green'
    elif k_rho == 1.5:
        color = 'orange'
    else:
        color = 'red'
    plt.plot([x_start, x_end], [y_start, y_end], color=color, linewidth=2)

# Plot the goal point
plt.scatter(x_goal, y_goal, color='purple', label='Goal', s=100)
plt.scatter(x0, y0, color='blue', label='Start', s=100)
plt.title("Trajectory for (-10, -10) with Minimal Progress Detection")
plt.xlabel("x (m)")
plt.ylabel("y (m)")
plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
plt.grid()
plt.tight_layout()
plt.show()

    """