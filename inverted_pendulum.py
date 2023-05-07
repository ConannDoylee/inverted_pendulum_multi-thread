import numpy as np
import copy
import os
from base_element import BaseElement

# Inverted Pendulem 
# 1. Parameters
# cart: 
# (1) mass(kg): M
# (2) length(m): 2l
# pole:
# (1) mass(kg): m
# 2. States
# cart translation(m): y
# pole rotation(rad): theta, clockwise is positive
# 3. Inputs
# cart translation force(N): u
# 4. Else
# resistance coefficient: K
ROOT = os.path.abspath(os.path.dirname(__file__))
class InvertedPendulum(BaseElement):
    def __init__(self):
        self.name = 'InvertedPendulum'
        self.conf_file = ROOT + "/conf/inverted_pendulum.yaml"
        super(InvertedPendulum, self).__init__(self.name,file=self.conf_file)
        # sim params
        self.N = self.model_conf['N']
        # initial state
        self.X = np.zeros(4)
        self.X[0] = self.model_conf['x1_init']
        self.X[1] = self.model_conf['x2_init']
        self.X[2] = self.model_conf['y1_init']
        self.X[3] = self.model_conf['y2_init']
        self.y1_max = self.model_conf['y1_max']
        self.y1_min = self.model_conf['y1_min']
        return

    def run_once(self):
        self.odeRK4()
        self.output = [[self.name+"_theta",copy.copy(self.X[0])],
                       [self.name+"_dtheta",copy.copy(self.X[1])],
                       [self.name+"_y",copy.copy(self.X[2])],
                       [self.name+"_dy",copy.copy(self.X[3])]]
        return
    
    def f(self,X_k,u):
        F = np.zeros(4)
        x1 = X_k[0]
        x2 = X_k[1]
        y1 = X_k[2]
        y2 = X_k[3]
        # model params
        m = self.model_conf['m']
        M = self.model_conf['M']
        l = self.model_conf['l']
        a = 1 / (m + M)
        g = 9.81
        K = self.model_conf['K']

        # y1 saturation situation
        if u < 0 and y1 <= self.y1_min or u > 0 and y1 >= self.y1_max:
            F[0] = x2
            F[1] = (g*np.sin(x1)-l*x1) / (4/3*l)
            F[2] = 0
            F[3] = 0
            return F

        F[0] = x2
        F[1] = (g*np.sin(x1)-l*x1-a*m*l*x2*x2*np.sin(2*x1)/2-a*np.cos(x1)*u) / (4/3*l-a*m*l*np.cos(x1)*np.cos(x1))
        F[2] = y2
        F[3] = a*(u+m*l*np.sin(x1)*x2*x2-m*l*np.cos(x1)*F[1]-K*y2)

        return F
    
    def odeRK4(self):
        if self.input is None:
            return
        u = self.input[0][1]
        dt = self.T_step / self.N
        for i in np.arange(self.N):
            K1 = self.f(self.X,u)
            K2 = self.f(self.X+K1*dt/2,u)
            K3 = self.f(self.X+K2*dt/2,u)
            K4 = self.f(self.X+K3*dt,u)
            self.X += dt/6.0*(K1+2.0*K2+2.0*K3+K4)
            # y1 saturation
            if self.X[2] <= self.y1_min:
                self.X[2] = self.y1_min
                self.X[3] = 0
            if self.X[2] >= self.y1_max:
                self.X[2] = self.y1_max
                self.X[3] = 0
        return

    def state(self):
        return copy.copy(self.X)

    def test(self):

        return

def main():
    model = InvertedPendulum()
    model.test()
    # plt.show()

if __name__ == '__main__':
    main()