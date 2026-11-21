
import cv2
import numpy as np

from path_planning import *
from path_planning.a_star_planner import AStarPlanner


class AStarImplementation(AStarPlanner):
    def preloop(self):
        print('Start:', self.start_node.coordinates, 'Goal:', self.goal_node.coordinates)
        print('grid_size:', self.grid_size, 'goal_threshold:', self.goal_threshold)
        
        self.queue:set[PathNode] = set() #frontier 
        self.g:dict[PathNode, float] = {}
        self.h:dict[PathNode, float] = {}
        self.f:dict[PathNode, float] = {}
        self.visited_nodes:set[PathNode] = set()
        self.visited_nodes.add(self.start_node)
        self.queue.add(self.start_node)
        self.g[self.start_node] = 0.0 # g value (Dijkstra cost) of start node is 0
        self.h[self.start_node] = calculate_node_distance(
            self.start_node, 
            self.goal_node
        ) #calculate_node_distance returns the euclidean distance between two nodes

        self.f[self.start_node] = self.g[self.start_node] + self.h[self.start_node]

        # print('Initial g:', self.g)
        # print('Initial h:', self.h)
        # print('Initial f:', self.f)
        self.sampled_nodes[self.start_node.coordinates] = self.start_node
        self.sampled_nodes[self.goal_node.coordinates] = self.goal_node
        # print('\nInitial queue:', self.queue)#[node.coordinates for node in self.queue])
        # print('\nInitial sampled nodes:', self.sampled_nodes)#[node.coordinates for node in self.sampled_nodes.values()])
        # print('\nInitial visited nodes:', self.visited_nodes)#[node.coordinates for node in self.visited_nodes])
        self.goal_found = False

        self.step_counter=0

    def step(self):
        # self.step_counter+=1
        # if self.step_counter<=2:
        #     print(f'\n===== Step {self.step_counter}=====')
        if not self.queue: # if frontier is empty, stop planning. (we have explored all reachable nodes without finding the goal)
            self.is_done.set()
            return
        if self.grid_size <= 0:
            raise ValueError("grid_size must be positive")

        # get the node with the lowest f value to be the current node
        current_node = min(
            self.queue, 
            key=lambda node: self.f.get(node, float("inf"))
        ) 
        #.get() returns the value for the given node, if node is not present, return inf instead of raising KeyError
        
        
        # if self.step_counter<=2:
        #     print('Queue:', [(node.coordinates,node.cost) for node in self.queue])
        #     print('self.f:', {node.coordinates:self.f[node] for node in self.queue})
        #     print('Current node (node with lowest f value (g+h)):', current_node.coordinates)
        #     print('self.g:', self.g[current_node])
        #     #print('Current h:', self.h[current_node])
        #     #print('Current f:', self.f[current_node])


        self.queue.remove(current_node)
        self.visited_nodes.add(current_node)

        distance_to_goal = calculate_node_distance(current_node, self.goal_node)

        # if current node is within the goal threshold, we consider the goal reached
        if distance_to_goal <= self.goal_threshold: 
            if current_node != self.goal_node:
                if check_collision_free(
                    occupancy_map=self.occupancy_map, 
                    source_node=current_node, 
                    target_node=self.goal_node
                ):
                    self.goal_node.parent = current_node
                    self.g[self.goal_node] = self.g[current_node] + distance_to_goal
                    self.h[self.goal_node] = 0.0 # h value (BFS node cost) of goal node is 0
                    self.f[self.goal_node] = self.g[self.goal_node]
                    self.visited_nodes.add(self.goal_node)
                    self.goal_found = True
                    self.is_done.set()
                    return
            else:
                self.g[self.goal_node] = self.g[current_node]
                self.h[self.goal_node] = 0.0 # h value (BFS node cost) of goal node is 0
                self.f[self.goal_node] = self.g[self.goal_node]
                self.visited_nodes.add(self.goal_node)
                self.goal_found = True
                self.is_done.set()
                return

        # get_neighbor_nodes uses self.grid_size internally for sampling offsets
        neighbor_nodes = self.get_neighbor_nodes(current_node)

        for neighbor_node in neighbor_nodes:
            ## Dijkstra part (we only consider g value here, without heuristic h)
            edge_cost = calculate_node_distance(current_node, neighbor_node)
            tentative_g = self.g[current_node] + edge_cost # g(v): calculated path cost from start point to v (Dijkstra) 
            
            # compares one neighbor at a time (tentative_g or g(v)) against the best known cost for that same neighbor (self.g.get(neighbor_node, float("inf"))).
            # skip this neighbor if the new path is not better.
            if tentative_g >= self.g.get(neighbor_node, float("inf")):
                continue
            
            # update g(v) because we found a better path    
            neighbor_node.parent = current_node
            neighbor_node.cost = tentative_g
            self.g[neighbor_node] = tentative_g

            ## Best First Search part
            # h(v): heuristic estimated cost from v to goal (Euclidean distance)
            self.h[neighbor_node] = calculate_node_distance(
                neighbor_node, 
                self.goal_node
            )

            ## f(v) = g(v) + h(v)
            self.f[neighbor_node] = self.g[neighbor_node] + self.h[neighbor_node]
            self.queue.add(neighbor_node)
            self.visited_nodes.add(neighbor_node)

    def postloop(self):
        if not self.goal_found:
            return [], self.visited_nodes
        return collect_path(self.goal_node), self.visited_nodes
        
