import numpy as np
import yaml
import os
import threading
import time
from matplotlib import pyplot as plt

class BaseElement(object):
    def __init__(self,name,T_step=None,file=None):
        self.name = name
        if file is not None:
            self.load_conf(file)

        if T_step is not None:
            self.T_step = T_step
        else:
            self.T_step = self.model_conf['T_step']
        # log
        self.time_list = []
        self.input_list_dict = {}
        self.output_list_dict = {}
        # 
        self.input = None
        self.output = None
        return
    
    def load_conf(self,file):
        self.model_conf = dict()
        with open(file) as f:
            data = yaml.load_all(f)
            for d in data:
                self.model_conf.update(d)
        print(self.name,self.model_conf)
    
    def update_input(self,input):
        self.input = input
        return

    def get_output(self):
        return self.output
    
    def loop_thread(self):
        while self.is_running:
            self.run_once()
            self.log()
            time.sleep(self.T_step)
        return
    
    def run_once(self):
        pass
            
    def thread_start(self):
        self.is_running = True
        t = threading.Thread(target=self.loop_thread,name=self.name)
        t.daemon = True
        t.start()
        return

    def stop_thread(self):
        self.is_running = False
        return
    
    def add2dict(self,dict,name_value):
        name = name_value[0]
        value = name_value[1]
        t = time.time()
        if name in dict:
            dict[name].append([t,value])
        else:
            dict[name] = [[t,value]]
        return

    def log(self):
        if self.input is None or self.output is None:
            return
        for input in self.input:
            self.add2dict(self.input_list_dict,input)
        for output in self.output:
            self.add2dict(self.output_list_dict,output)
        return

    def plot_data(self):
        plt.figure()
        plt.title(self.name)
        for input in self.input_list_dict:
            plt.plot(np.array(self.input_list_dict[input])[:,0],np.array(self.input_list_dict[input])[:,1],label="input_"+input)
        for output in self.output_list_dict:
            plt.plot(np.array(self.output_list_dict[output])[:,0],np.array(self.output_list_dict[output])[:,1],label="output_"+output)
        plt.grid()
        plt.legend()
        return