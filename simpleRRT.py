import random
import matplotlib.pyplot as plt
import collections
import numpy as py

class RRT:
    def __init__(self, domain_w=100, domain_h=100, delta=1, init_pos=(50,50), k=500):
        self.domain = [[0 for _ in range(domain_w)] for _ in range(domain_h)]
        self.delta = 1
        self.init_pos = (50,50)
        # let's experiment with a adjecency list first
        self.tree = defaultdict(list)
    
    def build_tree(self):
        pass
    
    def find_closest_vertex(self, vertex):
        pass
    
    def pair_vertices(self, vertex1, vertex2):
        pass