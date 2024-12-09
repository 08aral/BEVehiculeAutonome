import numpy as np
import matplotlib.pyplot as plt

# ctes
k_rho = 2     
k_alpha = 20 
k_beta = -5  
L = 20     # Longueur entre les roues (20 cm)
T = 0.01      # Pas de temps (10 ms)
epsilon = 1e-3  # Seuil de convergence
max_iterations = 10000  

def loi_cmd(x, y, theta, x_star, y_star, theta_star):
   
    dx, dy = x_star - x, y_star - y   
    rho = np.hypot(dx, dy)            # Distance euclidienne à la cible
    alpha = np.arctan2(dy, dx) - theta  # Angle entre la direction actuelle et la cible
    beta = -theta_star - alpha        # Orientation finale relative

    # Détermine si le robot doit avancer ou reculer
    sens = 1 if -np.pi / 2 <= alpha <= np.pi / 2 else -1

    # Commandes
    v = k_rho * rho * sens            
    omega = k_alpha * alpha + k_beta * beta  
    gamma = np.arctan2(L * omega, abs(v)) * sens  # Angle de braquage
    return v, gamma

def obs(x, y, theta, v, gamma):
   
    theta_dot = v / L * np.tan(gamma)  # Calcul de la vitesse angulaire
    x += T * v * np.cos(theta)        
    y += T * v * np.sin(theta)        
    theta += T * theta_dot            
    return x, y, theta

def sim(x0, y0, theta0, x_goal, y_goal, theta_goal):
    X, Y, THETA = [x0], [y0], [theta0]
    for _ in range(max_iterations):  
        
        x, y, theta = X[-1], Y[-1], THETA[-1]

        # Calcul des commandes
        v, gamma = loi_cmd(x, y, theta, x_goal, y_goal, theta_goal)

        # Mise à jour de l'état
        x, y, theta = obs(x, y, theta, v, gamma)

        X.append(x)
        Y.append(y)
        THETA.append(theta)

        # Critère d'arrêt basé sur la distance totale
        if np.hypot(x - x_goal, y - y_goal) < epsilon and abs(theta - theta_goal) < epsilon:
            print("Convergence atteinte")
            break

    return X, Y, THETA

# Simulation
x0, y0, theta0 = 0, 0, 0  # État initial
x_goal, y_goal, theta_goal = -10, -10, np.pi/4  # État cible


X, Y, THETA = sim(x0, y0, theta0, x_goal, y_goal, theta_goal)

plt.figure(figsize=(8, 6))
plt.plot(X, Y, label='Trajectoire')
plt.scatter(x0, y0, color='blue', label='Départ')
plt.scatter(x_goal, y_goal, color='red', label='Cible')
plt.title("Simulation de la trajectoire corrigée")
plt.xlabel("x (m)")
plt.ylabel("y (m)")
plt.legend()
plt.grid()
plt.show()
