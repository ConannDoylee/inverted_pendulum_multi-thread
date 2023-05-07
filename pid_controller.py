import numpy as np
from matplotlib import pyplot as plt
import os
import yaml
from base_element import BaseElement
ROOT = os.path.abspath(os.path.dirname(__file__))

class PID(BaseElement):
    def __init__(self):
        self.name = "pid"
        self.conf_file = ROOT + "/conf/pid_controller.yaml"
        super(PID, self).__init__(self.name,file=self.conf_file)
        self.integ_error = np.zeros(1)
        self.error_pre = np.zeros(1)
        self.input_cmd = 0
        self.input_sts = 0
        return

    def compute_d_error(self,error):
        d_error = (error - self.error_pre) / self.T_step
        return d_error

    def run_once(self):
        if self.input is None:
            return 0
        input_cmd = self.input[0][1]
        input_sts = self.input[1][1]
        error = input_cmd - input_sts
        KP = self.model_conf['kp']
        KI = self.model_conf['ki']
        KD = self.model_conf['kd']
        d_error = self.compute_d_error(error)
        u = -(KP * error + KI * self.integ_error + KD * d_error)

        self.integ_error += error * self.T_step
        self.error_pre = error
        
        self.output = [[self.name,float(u)]]
        return u

    def test(self):
        for i in np.arange(12):
            self.update_input(1)
            u = self.run_once()
            print(u,self.integ_error)
        return

def main(root):
    pid = PID()
    pid.test()
    plt.show()

# if __name__ == '__main__':
#     main(".")