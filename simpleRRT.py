import random
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
import collections
import numpy as py
import math
import networkx as nx
import pylab as pl

class GraphVisualization:
    def __init__(self):
        self.visual = []
    def addEdge(self,a,b):
        temp = [a,b]
        self.visual.append(temp)
    def visualize(self):
        G = nx.Graph()
        G.add_edges_from(self.visual)
        nx.draw_networkx(G)
        plt.show()

class RRT:
    def __init__(self, domain_w=100, domain_h=100, delta=1, init_pos=(50.0,50.0), k=5000):
        self.domain = [[0 for _ in range(domain_w)] for _ in range(domain_h)]
        self.domain_w = domain_w
        self.domain_h = domain_h
        self.delta = 1
        self.init_pos = init_pos
        self.k = k
        # let's experiment with an adjecency list first
        self.tree = collections.defaultdict(list)
        self.tree[self.init_pos].append(None)
    
    def build_tree(self):
        for i in range(self.k):
            x0 = random.randint(0,self.domain_w-1)
            y0 = random.randint(0,self.domain_h-1)
            closest = self.find_closest_vertex((x0,y0))
            self.extend_vertex_towards(closest, (x0,y0))
    
    def find_closest_vertex(self, vertex):
        min_distance = float("inf")
        closest_vertex = self.init_pos
        for existing_vertex in self.tree:
            distance = self.find_distance(vertex, existing_vertex)
            if distance < min_distance:
                min_distance = distance
                closest_vertex = existing_vertex
        return closest_vertex
    
    def find_distance(self, vertex1, vertex2):
        return math.sqrt((vertex2[0]-vertex1[0])**2+(vertex2[1]-vertex1[1])**2)

    def extend_vertex_towards(self, root, random_vertex):
        vector_mag = self.find_distance(root, random_vertex)
        new_vertex = ((random_vertex[0]-root[0])/vector_mag + root[0],(random_vertex[1]-root[1])/vector_mag + root[1])
        self.tree[root].append(new_vertex)
        self.tree[new_vertex].append(root)
    
    def visualize_tree(self):
        lines = []
        # print(self.tree)
        for vertex in self.tree:
            for neighbour in self.tree[vertex]:
                if neighbour:
                    lines.append([vertex, neighbour])
        # length = [len(e) for e in lines]
        print(lines[0])
        #print(lines)
        lc = LineCollection(lines)
        fig,ax = pl.subplots()
        ax.add_collection(lc)
        ax.autoscale()
        ax.margins(0.1)
        plt.show()
        
my_RRT = RRT()
my_RRT.build_tree()
my_RRT.visualize_tree()