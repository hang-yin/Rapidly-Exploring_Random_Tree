import random
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
import collections
import numpy as py
import math
import networkx as nx
import pylab as pl

class RRT_with_obstacle:
    def __init__(self, domain_w=100, domain_h=100, delta=1, init_pos=(50.0,50.0), k=5000, obstacle_amount = 40):
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
            if new_vertex and self.check_goal(new_vertex):
                '''
                FIND PATH and color code it, traverse back from goal to start location
                '''
                break
    
    def generate_obstacles(self, amount):
        max_radius = math.ceil(min(self.domain_w, self.domain_h)/10)
        min_radius = math.ceil(min(self.domain_w, self.domain_h)/100)
        obstacles = []
        for i in range(amount):
            obstacle = [(random.randint(0, self.domain_w), random.randint(0,self.domain_h)), random.randint(min_radius, max_radius)]
            obstacles.append(obstacle)
        return obstacles
    
    def collision_detection_dot(self, vertex):
        for center, radius in self.obstacles:
            if self.find_distance(center, vertex) < radius:
                return True
        return False
    
    def collition_detection_line(self, vertex1, vertex2):
        for center, radius in self.obstacles:
            if self.min_distance(vertex1,vertex2,center)<radius:
                return True
        return False

    def recursive_path_finder(self, curr_node):
        '''
        recursively find path from goal back to start
        '''
        pass
    
    def check_goal(self, vertex):
        # check if there is a collision free path from vertex to goal
        # if so, terminate
        return not self.collition_detection_line(vertex, self.goal)

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
        if not self.collition_detection_line(root, new_vertex):
            return new_vertex
        else:
            return None
    
    def minDistance(A, B, E):
        # vector AB
        AB = [None, None]
        AB[0] = B[0] - A[0]
        AB[1] = B[1] - A[1]
        # vector BP
        BE = [None, None]
        BE[0] = E[0] - B[0]
        BE[1] = E[1] - B[1]
        # vector AP
        AE = [None, None]
        AE[0] = E[0] - A[0]
        AE[1] = E[1] - A[1]
        # Variables to store dot product
        # Calculating the dot product
        AB_BE = AB[0] * BE[0] + AB[1] * BE[1]
        AB_AE = AB[0] * AE[0] + AB[1] * AE[1]
        # Minimum distance from
        # point E to the line segment
        reqAns = 0
        # Case 1
        if (AB_BE > 0) :
    
            # Finding the magnitude
            y = E[1] - B[1]
            x = E[0] - B[0]
            reqAns = math.sqrt(x * x + y * y)
        # Case 2
        elif (AB_AE < 0) :
            y = E[1] - A[1]
            x = E[0] - A[0]
            reqAns = math.sqrt(x * x + y * y)
        # Case 3
        else:
            # Finding the perpendicular distance
            x1 = AB[0]
            y1 = AB[1]
            x2 = AE[0]
            y2 = AE[1]
            mod = math.sqrt(x1 * x1 + y1 * y1)
            reqAns = abs(x1 * y2 - y1 * x2) / mod
        return reqAns

    def visualize_tree(self):
        lines = []
        for vertex in self.tree:
            for neighbour in self.tree[vertex]:
                if neighbour:
                    lines.append([vertex, neighbour])
        lc = LineCollection(lines)
        fig,ax = pl.subplots()
        '''
        TODO: mark start, goal, and path with different colors
        '''

        for obstacle in self.obstacles:
            circle = plt.Circle(obstacle[0], obstacle[1], color='k')
            ax.add_patch(circle)
        ax.add_collection(lc)
        ax.autoscale()
        ax.margins(0.1)
        plt.show()
        
my_RRT = RRT_with_obstacle()
my_RRT.build_tree()
my_RRT.visualize_tree()