import math
import numpy as np

class pure_pursuit(object):
    """This is pure pursuit class for vehicle control"""
    def __init__(self, len_ax, angle_min, angle_max): # car model dist between axes, angle_min, angle_max -> degrees
        # car_model
        self.angle_min = angle_min
        self.angle_max = angle_max
        self.len_ax = len_ax    

       

    def count_course_angle (self, car_angle, x_car, y_car, x_desired, y_desired):
        
        # car_pose -> Vector 3D
        car_pose_mat = np.matrix([[x_car], [y_car], [0]] )
        goal_pose_mat = np.matrix([[x_desired], [y_desired], [0]])
        path_vector_mat2 =  goal_pose_mat - car_pose_mat

        # ld -> Vector 3D
       # print('car_angle', car_angle)
        #print('cos_car', math.cos(car_angle))
        car2_pose_mat = np.matrix([[x_car + math.cos(car_angle)], [y_car + math.sin(car_angle)], [0]])
        pose_vector_mat =  car2_pose_mat - car_pose_mat
        
        

        # Count angle
        scalar_comp = pose_vector_mat[0][0] * path_vector_mat2[0][0] + pose_vector_mat[1][0] * path_vector_mat2[1][0] + pose_vector_mat[2][0] * path_vector_mat2[2][0]
        comp_of_scalar = math.sqrt(pose_vector_mat[0][0]**2 + pose_vector_mat[1][0]**2 + pose_vector_mat[2][0]**2) * math.sqrt(path_vector_mat2[0][0]**2 + path_vector_mat2[1][0]**2 + path_vector_mat2[2][0]**2)
        cos_alpha = (float)(scalar_comp/comp_of_scalar)
        scalar_comp2 = pose_vector_mat[0][0] * path_vector_mat2[1][0] - pose_vector_mat[1][0] * path_vector_mat2[0][0]   
        sin_alpha = (float)(scalar_comp2/comp_of_scalar)    
        
        alpha = math.acos(round(cos_alpha,3))
        alpha2 = math.asin(round(sin_alpha,3))    
        #print('cos_alpha', sin_alpha)
    
        if alpha2 < 0:
            return alpha
        else:
            return -alpha 


    def calculate_angle(self, car_angle, x_car, y_car, x_desired, y_desired): # car current param, position, direction, desired point position
             
        dist_to_point =  math.sqrt((x_desired - x_car)**2 + (y_desired - y_car)**2)
        angle_error = self.count_course_angle(car_angle, x_car, y_car, x_desired, y_desired)
        angle = math.atan2((2*self.len_ax * math.sin(angle_error)) , dist_to_point)
        #angle = math.degrees(angle)
        #print('angle', math.degrees(angle_error))
        if angle > self.angle_max:
             angle = self.angle_max
             print('ERROR POINT IS TOO FAR')
        elif angle < self.angle_min:
             angle = self.angle_min             
             print('ERROR POINT IS TOO FAR')
        if math.isnan(angle):
            angle = 0
        return angle

  

if __name__ == "__main__":
    global pure_pursuit_controller
    pure_pursuit_controller = pure_pursuit(1.5, -36, 36)
    
    car_angle = 0.52
    x_car = 0    
    y_car = 0
    x_desired = 2
    y_desired = 0

    print('steering_angle', pure_pursuit_controller.calculate_angle(car_angle, x_car, y_car, x_desired, y_desired)) 
    