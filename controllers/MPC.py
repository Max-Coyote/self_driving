import numpy as np
import threading
from scipy.optimize import minimize
import math


class ModelPredictiveControl:
    """This is MPC class for vehicle control"""
    def __init__(self):
        self.horizon = 5
        self.dt = 0.2
        self.current_x = 0
        self.current_y = 0
        self.current_orientation = 0
        self.current_speed = 0

        self.parking_place_x = 10
        self.parking_place_y = 10    
        self.bounds = []
        self.num_inputs = 2
        self.u = np.zeros(self.horizon*self.num_inputs)
        for i in range(self.horizon):
          self.bounds += [[-36, 36]]
          self.bounds += [[0, 1]]

    def car_model(self, x, y, orientation, v, steering, throttle): 
        l_f = 1.7
        psi = np.deg2rad(orientation)
        a = throttle 
        delta = np.deg2rad(steering)
        psi_1 = psi + (v/l_f) * delta * self.dt
        x_1 = x + v*np.cos(psi_1) * self.dt
        y_1 = y + v*np.sin(psi_1) * self.dt
        v_1 = v + a*20* self.dt - v/10
        psi_1 = np.rad2deg(psi_1)
  
        error = math.sqrt((self.parking_place_x - x)**2 + (self.parking_place_y - y)**2)       
        return x_1, y_1, v_1, psi_1, error


    def cost_function(self, u):
        cost = 0
        x, y, orientation = self.current_x, self.current_y, self.current_orientation
        speed = self.current_speed      

        for k in range(0, self.horizon):
            dt = self.dt * k            
            x, y, v, psi, error = self.car_model(x, y, orientation, speed, u[k*2], u[k*2+1])
            
            if error > 1:
              cost += abs(10-v)**2
            else:
                cost += 0
            cost += abs(error)**2
           
        return cost
      

    # counting thread
    def count_mpc(self):
        
        u_solution = minimize(self.cost_function,
                     x0=self.u,
                     method='SLSQP',
                     bounds=self.bounds,
                     tol = 1e-2)


        speed = self.current_speed
        x = self.current_x
        y = self.current_y
        orientation = self.current_orientation
        points = []
        points.append([x, y])
        for k in range(0, self.horizon):
            dt = self.dt * k
            x, y, v, psi, error= self.car_model(x, y, speed, orientation, u_solution.x[k*2], u_solution.x[k*2+1])    
            points.append([x, y])
            print(error)
        return u_solution.x[0], u_solution.x[1], points
  

import matplotlib.pyplot as plt
def main():
  mpc_test = ModelPredictiveControl()
  mpc_test.current_orientation = 30
  mpc_test.current_speed = 10
  steering, throttle, points = mpc_test.count_mpc()
  print('steering_angle', steering)    
  print('throttle', throttle)
  print("position in i * dt {}  :{}".format(mpc_test.dt, points))

if __name__ == '__main__':
    main()
