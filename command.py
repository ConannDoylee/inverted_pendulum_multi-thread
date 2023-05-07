import numpy as np
from base_element import BaseElement

class Command(BaseElement):
    def __init__(self):
        self.name = 'command'
        super(Command, self).__init__(self.name,0.1)
        self.input = [[self.name,0]]
        return
    
    def run_once(self):
        self.output = [[self.name,0]]
        return
