from pid_controller import PID
from command import Command
from inverted_pendulum import InvertedPendulum
from matplotlib import pyplot as plt
import time
import numpy as np
import threading
import traceback
import signal
import os

modules_dict = {
                "PID":PID(),
                "command":Command(),
                "InvertedPendulum": InvertedPendulum(),
                }

# "current_module": ["deps_input_module",""]
modules_inputs_deps = {
    "InvertedPendulum": ["PID"],
    "PID": ["command","InvertedPendulum"],
}

modules_plot = ["PID","InvertedPendulum"]

WORKBENCH_LENGTH = 10
CART_LENGTH = 1
POLE_LENGTH = 2
X_BOUND = WORKBENCH_LENGTH / 2 + CART_LENGTH
Y_BOUND = 1.5*POLE_LENGTH
WORBENCH_BOUND = WORKBENCH_LENGTH / 2 + CART_LENGTH / 2

class Simulation(object):
    def __init__(self):
        self.is_stop = False
        signal.signal(signal.SIGINT,self.quit)
        return
    
    def quit(self,signum,frame):
        os._exit(0)

    def run_once(self):
        for module in modules_inputs_deps:
            outputs = []
            for input_module in modules_inputs_deps[module]:
                output = modules_dict[input_module].get_output()
                if output is not None:
                    outputs.extend(output)
            if output:
                modules_dict[module].update_input(outputs)
        return
    
    def start_modules(self):
        print("start_modules...")
        for key in modules_dict:
            print(key)
            modules_dict[key].thread_start()
        return

    def stop_modules(self):
        print("stop_modules...")
        for key in modules_dict:
            print(key)
            modules_dict[key].stop_thread()
        self.is_stop = True
        return
    
    def simulate(self,count,T_s=0.01):
        self.start_modules()
        start_time = time.time()
        for i in np.arange(count):
            self.run_once()
            time.sleep(T_s)
        end_time = time.time()
        print("total_time:",end_time-start_time)
        self.stop_modules()
        return

    def plot_modules(self):
        for module in modules_plot:
            modules_dict[module].plot_data()
        return

    def plot_simulation(self):
        plt.figure('simulation')
        commands = modules_dict['command'].output_list_dict['command']
        states = modules_dict['InvertedPendulum'].output_list_dict['InvertedPendulum_theta']
        plt.plot(np.array(commands)[:,0],np.array(commands)[:,1],label='command')
        plt.plot(np.array(states)[:,0],np.array(states)[:,1],label='state')
        plt.legend()
        return

def plot_ax(ax1,ax2,ax3,ax4,ax5,ax6):
    try:
        theta = modules_dict['InvertedPendulum'].output_list_dict['InvertedPendulum_theta'][-1:]
        dtheta = modules_dict['InvertedPendulum'].output_list_dict['InvertedPendulum_dtheta'][-1:]
        y = modules_dict['InvertedPendulum'].output_list_dict['InvertedPendulum_y'][-1:]
        dy = modules_dict['InvertedPendulum'].output_list_dict['InvertedPendulum_dy'][-1:]
        u = modules_dict['PID'].output_list_dict['pid'][-1:]
        ax1.plot(np.array(theta)[:,0],np.array(theta)[:,1],'.')
        ax2.plot(np.array(dtheta)[:,0],np.array(dtheta)[:,1],'.')
        ax3.plot(np.array(y)[:,0],np.array(y)[:,1],'.')
        ax4.plot(np.array(dy)[:,0],np.array(dy)[:,1],'.')
        ax5.plot(np.array(u)[:,0],np.array(u)[:,1],'.')
        ax6.set_ylim(-Y_BOUND,Y_BOUND)
        ax6.set_xlim(-X_BOUND,X_BOUND)
        ax6.plot([-WORBENCH_BOUND,-WORBENCH_BOUND,WORBENCH_BOUND,WORBENCH_BOUND],[0.2,0,0,0.2],'k',lw=2)
        ax6.plot([y[0][1]-CART_LENGTH/2,y[0][1]+CART_LENGTH/2],[0,0],lw=4)
        ax6.plot([y[0][1],y[0][1]+POLE_LENGTH*np.sin(theta[0][1])],[0,POLE_LENGTH*np.cos(theta[0][1])],'g',lw=2)
        plt.pause(0.05)
        ax6.cla()
    except:
        traceback.print_exc()
        return
    
def main():
    fig = plt.figure()
    ax1 = plt.subplot(2,3,1)
    ax2 = plt.subplot(2,3,2)
    ax3 = plt.subplot(2,3,3)
    ax4 = plt.subplot(2,3,4)
    ax5 = plt.subplot(2,3,5)
    ax6 = plt.subplot(2,3,6)
    ax1.set_title("theta")
    ax2.set_title("dtheta")
    ax3.set_title("y")
    ax4.set_title("dy")
    ax5.set_title("u")

    plt.pause(0.1)

    simu = Simulation()
    t = threading.Thread(target=simu.simulate,name='simulation',args=(3000,))
    t.start()

    while not simu.is_stop:
        plot_ax(ax1,ax2,ax3,ax4,ax5,ax6)
        pass

    simu.plot_simulation()
    plt.show()

if __name__ == '__main__':
    main()