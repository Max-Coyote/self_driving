
import numpy as np
import scipy.linalg


class LQR_controller(object):
    """This is LQR regulator class for vehicle control"""
    def __init__(self, A, B, Q, R):
      """ Continuous time lqr controller.
 
      x_dot = A x + B u
 
      cost = integral x.T*Q*x + u.T*R*u
      """
        
      # Input and Dynamics matrices
      self.A = A
      self.B = B
      # Input and State weight matrices
      self.Q = Q
      self.R = R   

      # koeff
      self.K = 0 

    def calculate_coeff(self):
       # Ricatti equation
        X = np.matrix(scipy.linalg.solve_continuous_are(self.A, self.B, self.Q, self.R))
 
        K = np.matrix(scipy.linalg.inv(self.R)*(self.B.T*X))
        self.K = K

    def calculate_angle(self, cte, head_error):
        x = np.matrix([[cte], [head_error]])
        steer_angle = self.K * x
        return steer_angle

def main():

  A1 = np.matrix([[0.5, 1], [1, 8]])
  B1 = np.matrix([[0.5], [0.5]])
  Q1 = np.matrix([[0.5, 1], [1, 0.5]])
  R1 = np.matrix([[20]])

  LQR = LQR_controller(A1, B1, Q1, R1)
  LQR.calculate_coeff()
  
  #cross-track error  
  cte = -0.5
  #heading error 
  head_error = 0.2
  
  print('steering_angle', LQR.calculate_angle(cte, head_error)[0,0])
  

if __name__ == '__main__':
    main()
