
import math
import numpy as np

class stanley_controller(object):
    """This is stanley class for vehicle control"""
    def __init__(self, len_ax, angle_min, angle_max, koeff ): # car model dist between axes, angle_min, angle_max -> degrees
        # car_model
        self.angle_min = angle_min
        self.angle_max = angle_max
        self.len_ax = len_ax  
        self.koeff = koeff #koeff for gain parameter

   
    def calculate_angle(self, cte, head_error, speed): # cross track error; heading error; current speed
        #counted angle in rad
        angle = head_error + math.atan2(self.koeff * cte, 1 + speed)    
        angle = math.degrees(angle)    
        if angle > self.angle_max:
             angle = self.angle_max
             #print('ANGLE IS MORE THAN MATH MODEL')
        elif angle < self.angle_min:
             angle = self.angle_min             
             #print('ANGLE IS LESS THAN MAT MODEL')
        if math.isnan(angle):
            angle = 0
       
        
        return -angle

   
# test of programm
if __name__ == "__main__":
    stanley = stanley_controller(1.5, -36, 36, 1)
    speed = 20
    cte = -0.5
    head_error = 0.2
    print('steering_angle', stanley.calculate_angle(cte, head_error, speed))
    
   