import numpy as np
from scipy.optimize import minimize

class ModelPredictiveControl:
    def __init__(self):
        self.horizon = 10
        self.dt = 0.2
    
    def car_model(self, car_state, dt, steering, throttle):
        # model of car
        l_f = 2
        x_1 = x + v*np.cos(psi) * self.dt
        y_1 = y + v*np.sin(psi) * self.dt
        a = throttle
        v_1 = v + a * self.dt - v/10
        delta = steering
        
        psi_1 = psi + (v/l_f) * delta * self.dt
            # Position cost
            # Angle cost
            # Acceleration cost
            # Steering Input cost
        #cte_1 = error 
        #epsi_1 = error angle

        # INPUT DATA
        # x, y -> position
        # psi -> orientation
        # v -> velocity
        # cte -> cross-track error (the difference between the
        # trajectory defined by the waypoints and the current vehicle
        # position y in the coordinate space of the vehicle)
        # epsi -> orientation error

        # ACTUATOR CONSTRAINTS
        # a -> acceleration is in the range [-1, 1] -> [full brake, full throttle]
        # delta -> steering angle is in the range -> [-25, 25]

    def cost_function(self, u):
        cost = 0
        state = car_state
        for k in range(0, self.horizon):
            # cross-track error
            # orientation error# Position cost
            # Angle cost
            # Acceleration cost
            # Steering Input cost
            state = self.car_model(state, self.dt, u[k*2], u[k*2+1])
            cost += abs(cte)**2
            cost += abs(epsi)**2
            v_des = 20
            cost += abs(v_des - v_t)**2
            # Position cost
            # Angle cost
            # Acceleration cost
            cost += abs(epsi)**2
            # Steering Input cost


 

mpc = ModelPredictiveControl()
# bounds of output params
bounds = []
num_inputs = 2
u = np.zeros(mpc.horizon*num_inputs)
for i in range(mpc.horizon):
    bounds += [[-25, 25]]
    bounds += [[-1, 1]]

# Inputs
# u -> Inputs
# Non-linear optimization
u_solution = minimize(mpc.cost_function,
                     x0=u,
                     method='SLSQP',
                     bounds=bounds,
                     tol = 1e-8)  
print(u_solution)
# publish to  
                
