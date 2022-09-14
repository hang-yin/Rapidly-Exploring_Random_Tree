import random
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
import collections
import numpy as np
import math
import pylab as pl

class RRT_with_image:
    def __init__(self, file_name, delta=1, k=5000):
        self.image = self.read_image(file_name)
        self.image = np.rot90(self.image)
        where_0 = np.where(self.image == 0)
        where_1 = np.where(self.image == 1)
        self.image[where_0] = 1
        self.image[where_1] = 0
        # self.image = np.flipud(self.image)
        shape = self.image.shape
        self.domain_w, self.domain_h = shape
        self.delta = 1
        self.k = k
        self.tree = collections.defaultdict(list)
        
        start_finished = False
        while not start_finished:
            self.start = (random.randint(0, self.domain_w-1), random.randint(0,self.domain_h-1))
            start_finished = not self.collision_detection_dot(self.start)
        self.tree[self.start].append(None)
        goal_finished = False
        while not goal_finished:
            self.goal = (random.randint(0, self.domain_w-1), random.randint(0,self.domain_h-1))
            goal_finished = not self.collision_detection_dot(self.goal)
        
        self.finish_point = None
        self.path = []
        self.visited = set()
    
    def read_image(self, file_name):
        image = plt.imread(file_name)
        return image
    
    def build_tree(self):
        for i in range(self.k):
            x0 = random.randint(0,self.domain_w-1)
            y0 = random.randint(0,self.domain_h-1)
            closest = self.find_closest_vertex((x0,y0))
            new_vertex = self.extend_vertex_towards(closest, (x0,y0))
            if new_vertex and self.check_goal(new_vertex):
                self.finish_point = new_vertex
                self.recursive_path_finder(new_vertex, [])
                self.path.append((self.finish_point, self.goal))
                '''
                FIND PATH and color code it, traverse back from goal to start location
                '''
                break
    
    def collision_detection_dot(self, vertex):
        # print(self.image[vertex[0]][vertex[1]])
        return self.image[vertex[0]][vertex[1]]

    def in_range_horizontal(self, num):
        return num >= 0 and num < self.domain_w
    
    def in_range_vertical(self, num):
        return num >= 0 and num < self.domain_h

    def collision_detection_line(self, vertex1, vertex2):
        '''
        TODO
        '''
        if vertex2[0] < vertex1[0]:
            vertex1, vertex2 = vertex2, vertex1
        x1,y1 = vertex1
        x2,y2 = vertex2
        if (x2-x1)==0:
            return True
        m = (y2-y1)/(x2-x1)
        c = y1 - m*x1
        for x in np.arange(x1, x2+1, 0.1):
            y = m*x + c
            if x>=0 and x<self.domain_w and y>=0 and y<self.domain_h:
                if self.in_range_horizontal(math.ceil(x)) and self.in_range_vertical(math.ceil(y)) and self.image[math.ceil(x)][math.ceil(y)] == 1:
                    return True
                if self.in_range_horizontal(math.floor(x)) and self.in_range_vertical(math.floor(y)) and self.image[math.floor(x)][math.floor(y)] == 1:
                    return True
                if self.in_range_horizontal(math.ceil(x)) and self.in_range_vertical(math.floor(y)) and self.image[math.ceil(x)][math.floor(y)] == 1:
                    return True
                if self.in_range_horizontal(math.floor(x)) and self.in_range_vertical(math.ceil(y)) and self.image[math.floor(x)][math.ceil(y)] == 1:
                    return True
        if y2 < y1:
            y1,y2 = y2,y1
        for y in np.arange(y1, y2+1, 0.1):
            x = (y-c)/m
            if x>=0 and x<self.domain_w and y>=0 and y<self.domain_h:
                if self.in_range_horizontal(math.ceil(x)) and self.in_range_vertical(math.ceil(y)) and self.image[math.ceil(x)][math.ceil(y)] == 1:
                    return True
                if self.in_range_horizontal(math.floor(x)) and self.in_range_vertical(math.floor(y)) and self.image[math.floor(x)][math.floor(y)] == 1:
                    return True
                if self.in_range_horizontal(math.ceil(x)) and self.in_range_vertical(math.floor(y)) and self.image[math.ceil(x)][math.floor(y)] == 1:
                    return True
                if self.in_range_horizontal(math.floor(x)) and self.in_range_vertical(math.ceil(y)) and self.image[math.floor(x)][math.ceil(y)] == 1:
                    return True
        return False
    
    def recursive_path_finder(self, curr_node, path):
        '''
        recursively find path from goal back to start
        '''
        if curr_node:
            path = path[:]
            # path.append(curr_node)
            if curr_node == self.start:
                self.path = path
                return
            self.visited.add(curr_node)
            for neighbour in self.tree[curr_node]:
                if neighbour not in self.visited:
                    path.append((curr_node, neighbour))
                    self.recursive_path_finder(neighbour, path)


    
    def check_goal(self, vertex):
        # check if there is a collision free path from vertex to goal
        # if so, terminate
        return not self.collision_detection_line(vertex, self.goal)

    def find_closest_vertex(self, vertex):
        min_distance = float("inf")
        closest_vertex = (0,0)
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
        if not self.collision_detection_line(root, new_vertex):
            self.tree[root].append(new_vertex)
            self.tree[new_vertex].append(root)
            return new_vertex
        else:
            return None

    def visualize_tree(self):
        print(self.image)
        lines = []
        for vertex in self.tree:
            for neighbour in self.tree[vertex]:
                if neighbour:
                    lines.append([vertex, neighbour])
        lc = LineCollection(lines)

        path_lc = LineCollection(self.path, color=(1,0,0,1))

        fig,ax = pl.subplots()
        # mark start and goal
        plt.plot(self.start[0], self.start[1], "o")
        plt.plot(self.goal[0], self.goal[1], "x")

        '''
        for i in range(self.domain_w):
            for j in range(self.domain_h):
                if self.image[i][j]==1:
                    plt.plot(i, j, "s")
        '''
        plt.imshow(self.image.T, origin="lower")

        ax.add_collection(lc)
        ax.add_collection(path_lc)
        # ax.autoscale()
        ax.set_xlim(0,self.domain_w)
        ax.set_ylim(0,self.domain_h)
        ax.margins(0.1)
        plt.show()
        
my_RRT = RRT_with_image(file_name="greyscale.png")
my_RRT.build_tree()
my_RRT.visualize_tree()