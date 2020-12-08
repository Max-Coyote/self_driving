from datetime import timedelta, datetime

class PID(object):
    """This is PID regulator class for vehicle control"""
    def __init__(self, Kp, Ki, Kd):
        #main error
        self.error = 0

        #koefficients
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        #time interval
        self.delta_time = 0

        #buf error
        self.last_error = 0
        self.P = 0
        self.I = 0
        self.D = 0
        
        #total error
        self.total_error = 0

    def Proportional(self):
        self.P = self.error * self.Kp

    def Integral(self):
        self.total_error += self.error
        self.I = self.total_error * self.delta_time * self.Ki

    def Derivative(self):
        self.D = (self.Kd / self.delta_time) * (self.error - self.last_error)
        self.last_error = self.error

    def update_measurements(self, error, delta_time):
        self.error = error
        self.delta_time = delta_time
        self.Proportional()
        self.Integral()
        self.Derivative()
        self.PID = -self.P - self.I - self.D

if __name__ == "__main__":
    pid_regulator = PID(2, 1, 1)
    now = datetime.now()
    while True:
        error = float(input())
        last_time = now
        now = datetime.now()
        delta_time_format = now - last_time
        delta_time = delta_time_format.total_seconds()
        print(delta_time)        
        pid_regulator.update_measurements(error, delta_time)
        print(pid_regulator.PID)
