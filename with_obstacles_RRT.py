import random
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
import collections
import numpy as py
import math
import networkx as nx
import pylab as pl

class RRT_with_obstacle:
    def __init__(self, domain_w=100, domain_h=100, delta=1, init_pos=(50.0,50.0), k=5000, obstacle_amount = 20):
        # self.domain = [[0 for _ in range(domain_w)] for _ in range(domain_h)]
        self.domain_w = domain_w
        self.domain_h = domain_h
        self.delta = 1
        self.init_pos = init_pos
        self.k = k
        # let's experiment with an adjecency list first
        self.tree = collections.defaultdict(list)
        self.tree[self.init_pos].append(None)
        self.obstacles = self.generate_obstacles(obstacle_amount)
        start_finished = False
        while not start_finished:
            self.start = (random.randint(0, self.domain_w), random.randint(0,self.domain_h))
            start_finished = not self.collision_detection_dot(self.start)
        goal_finished = False
        while not goal_finished:
            self.goal = (random.randint(0, self.domain_w), random.randint(0,self.domain_h))
            goal_finished = not self.collision_detection_dot(self.goal)
    
    def build_tree(self):
        for i in range(self.k):
            x0 = random.randint(0,self.domain_w-1)
            y0 = random.randint(0,self.domain_h-1)
            closest = self.find_closest_vertex((x0,y0))
            new_vertex = self.extend_vertex_towards(closest, (x0,y0))
            if self.check_goal(new_vertex):
                '''
                FIND PATH and color code it, traverse back from goal to start location
                '''
                break
    
    def generate_obstacles(self, amount):
        max_radius = math.ceil(min(self.domain_w, self.domain_h)/20)
        min_radius = math.ceil(min(self.domain_w, self.domain_h)/100)
        obstacles = []
        for i in range(amount):
            obstacle = [(random.randint(0, self.domain_w), random.randint(0,self.domain_h)), random.randint(min_radius, max_radius)]
            obstacles.append(obstacle)
        return obstacles
    
    def collision_detection_dot(self, vertex):
        pass
    
    def collition_detection_line(self, vertex1, vertex2):
        pass

    def recursive_path_finder(self, curr_node):
        '''
        recursively find path from goal back to start
        '''
        pass
    
    def check_goal(self, vertex):
        # check if there is a collision free path from vertex to goal
        # if so, terminate
        return False

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
        '''
        TODO: COLLESION CHECKING
        '''
        vector_mag = self.find_distance(root, random_vertex)
        new_vertex = ((random_vertex[0]-root[0])/vector_mag + root[0],(random_vertex[1]-root[1])/vector_mag + root[1])
        self.tree[root].append(new_vertex)
        self.tree[new_vertex].append(root)
        return new_vertex
    
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
        
my_RRT = RRT_with_obstacle()
my_RRT.build_tree()
my_RRT.visualize_tree()