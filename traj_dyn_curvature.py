import matplotlib.patches as mpatches
import numpy as np
import matplotlib.pyplot as plt

# Constants
L = 20e-3  # Length between the wheels (20 mm)
T = 0.01  # Time step (10 ms)
epsilon = 1e-3  # Convergence threshold
max_iterations = 1000  # Maximum iterations to avoid infinite loops

def dynamic_gains_with_curvature(rho, alpha, beta, prev_alpha):
    delta_alpha=alpha - prev_alpha
    curvature = abs(delta_alpha) / T  # Approximate curvature as angular change
    
    # Initial alignment condition
    if rho > 6 and abs(alpha) > 0.5:  # Far from the goal and significant misalignment
        k_rho = 0.5  # Reduce linear speed to prioritize alignment
        k_alpha = 25.0  # Strong angular correction
        k_beta = -5.0
        level = "Initial Alignment"
        return k_rho, k_alpha, k_beta, level

    if rho > 6:  # Far from the goal
        k_rho = 3.5
    elif rho > 2:  # Moderately close to the goal
        k_rho = 1.5
    else:  # Close to the goal
        k_rho = 1.0

    # Adjust angular gains based on curvature
    if curvature > 10.0:  # High curvature
        k_alpha = 20.0
        k_beta = -10.0
        level = "High Curvature"
    elif curvature > 5.0:  # Moderate curvature
        k_alpha = 15.0
        k_beta = -8.0
        level = "Medium Curvature"
    else:  # Low curvature (straight line)
        k_alpha = 10.0
        k_beta = -5.0
        level = "Low Curvature"
    
    # Handle oscillations when close to the goal
    if rho < 2 and curvature > 10:
        # Reduce angular corrections to focus on straight-line motion
        k_alpha = -1  # Small angular gain to minimize oscillation
        k_beta = 0   # No angular adjustment based on orientation error
        k_rho=2
        level = "Straight-Line Correction"
        
    if delta_alpha>np.pi/2:
        k_alpha = -1
        k_beta = 10
        k_rho=3
        
    return k_rho, k_alpha, k_beta, level

def run_simulation_with_path_complexity(x0, y0, theta0, x_goal, y_goal, theta_goal):
    X, Y, THETA = [x0], [y0], [theta0]
    gains_segments = []
    prev_alpha = 0.0
    iteration = 0

    while iteration < max_iterations:
        x, y, theta = X[-1], Y[-1], THETA[-1]
        dx, dy = x_goal - x, y_goal - y
        rho = np.hypot(dx, dy)
        alpha = np.arctan2(dy, dx) - theta
        beta = -theta_goal - alpha

        k_rho, k_alpha, k_beta, level = dynamic_gains_with_curvature(rho, alpha, beta, prev_alpha)
        gains_segments.append((x, y, k_rho, k_alpha, k_beta, level))

        # Print alpha and remaining distance (rho) at each iteration
        print(f"Iteration {iteration}: alpha = {alpha:.4f}, remaining distance (rho) = {rho:.4f}")

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

        prev_alpha = alpha
        iteration += 1

    return X, Y, THETA, gains_segments

# Simulate for a goal point
x0, y0, theta0 = 0, 0, 0
x_goal, y_goal, theta_goal = -10, -10, np.pi/4

X, Y, THETA, gains_segments = run_simulation_with_path_complexity(
    x0, y0, theta0, x_goal, y_goal, theta_goal
)

# Plot the trajectory with gain adjustments
plt.figure(figsize=(10, 8))

for i in range(len(gains_segments) - 1):
    x_start, y_start, k_rho, k_alpha, k_beta, level = gains_segments[i]
    x_end, y_end, _, _, _, _ = gains_segments[i + 1]

    if level == "High Curvature":
        color = 'red'
    elif level == "Medium Curvature":
        color = 'orange'
    elif level == "Straight-Line Correction":
        color = 'blue'
    else:
        color = 'green'

    plt.plot([x_start, x_end], [y_start, y_end], color=color, linewidth=2)
    plt.scatter(x_start, y_start, color=color, s=10)  # Show each point

# Add legend with details
high_patch = mpatches.Patch(color='red', label='High Curvature')
medium_patch = mpatches.Patch(color='orange', label='Medium Curvature')
low_patch = mpatches.Patch(color='green', label='Low Curvature')
straight_patch = mpatches.Patch(color='blue', label='Straight-Line Correction ')
plt.legend(handles=[high_patch, medium_patch, low_patch, straight_patch], bbox_to_anchor=(1.05, 1), loc='upper left')

# Plot goal and start points
plt.scatter(x_goal, y_goal, color='purple', label='Goal', s=100)
plt.scatter(x0, y0, color='blue', label='Start', s=100)
plt.title("Trajectory with Path Complexity-Based Gain Adjustment and Iteration Points")
plt.xlabel("x (m)")
plt.ylabel("y (m)")
plt.grid()
plt.tight_layout()
plt.show()
