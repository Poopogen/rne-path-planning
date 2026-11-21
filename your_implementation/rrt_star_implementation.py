
import cv2
import numpy as np

from path_planning import *
from path_planning.rrt_star_planner import RRTStarPlanner


class RRTStarImplementation(RRTStarPlanner):
    def preloop(self):

        self.start_node.parent = None
        self.start_node.cost = 0.0
        self.goal_node.parent = None
        self.goal_node.cost = float("inf")
        self.goal_found = False

        self.visited_nodes:set[PathNode] = set()
        self.visited_nodes.add(self.start_node)

        self.graph:set[PathNode] = set() #graph is the set of all nodes in the RRT* tree
        self.graph.add(self.start_node)        

        self.node_map:dict[PixelCoordinates, PathNode] = {} #map of all nodes in the RRT* tree
        self.node_map[self.start_node.coordinates] = self.start_node

    def step(self):
        if self.step_size <= 0:
            raise ValueError("step_size must be positive")
        if self.search_radius <= 0:
            raise ValueError("search_radius must be positive")
        
        if not self.graph:
            self.is_done.set()
            return

        # sample the point in 2D space 
        random_node = self.sample_random_node()

        # find the nearest node to the sampled point in graph
        nearest_node = min(
            self.graph, 
            key=lambda node: calculate_node_distance(node, random_node)
        )
        nearest_distance = calculate_node_distance(nearest_node, random_node)
        
        if nearest_distance == 0:
            return

        nearest_coordinates = nearest_node.coordinates
        random_coordinates = random_node.coordinates

        # new node is build between nearest node and random node, with distance of step_size from nearest node towards random node
        ratio = min(1.0, self.step_size / nearest_distance)
        new_x = nearest_coordinates.x + (random_coordinates.x - nearest_coordinates.x) * ratio
        new_y = nearest_coordinates.y + (random_coordinates.y - nearest_coordinates.y) * ratio
        new_node = PathNode(coordinates=PixelCoordinates(new_x, new_y))

        #existing node check, inside map check and collision check
        if new_node.coordinates in self.node_map: 
            return
        if not check_inside_map(self.occupancy_map, new_node):
            return
        if not check_collision_free(self.occupancy_map, nearest_node, new_node):
            return

        nearby_nodes:list[PathNode] = []

        #find nearby nodes within search radius
        for node in self.graph:
            if calculate_node_distance(node, new_node) > self.search_radius:
                continue
            if not check_collision_free(self.occupancy_map, node, new_node):
                continue
            nearby_nodes.append(node)

        if nearest_node not in nearby_nodes:
            nearby_nodes.append(nearest_node)
        
        # Re-parenting: find the best parent for the new node among nearby nodes (including nearest node)
        best_parent = nearest_node
        best_cost = nearest_node.cost + calculate_node_distance(nearest_node, new_node) #cost(start->nearest_node) + cost(nearest_node->new_node)==>cost(start->new_node through nearest_node)
        for node in nearby_nodes:
            candidate_cost = node.cost + calculate_node_distance(node, new_node)#cost(start->node) + cost(node->new_node) ==> cost(start->new_node through node)
            if candidate_cost < best_cost:
                best_parent = node
                best_cost = candidate_cost

        new_node.parent = best_parent #update 
        new_node.cost = best_cost
        self.graph.add(new_node)
        self.visited_nodes.add(new_node)
        self.node_map[new_node.coordinates] = new_node


        # Re-wiring: update the cost and parent of the nearby nodes
        for node in nearby_nodes:
            if node == best_parent or node == self.start_node:
                continue
            rewired_cost = new_node.cost + calculate_node_distance(new_node, node) #cost(start->new_node) + cost(new_node->node) ==> cost(start->node through new_node)
            if rewired_cost >= node.cost:
                continue
            if not check_collision_free(self.occupancy_map, new_node, node):
                continue
            node.parent = new_node #update 
            node.cost = rewired_cost


        # Goal check
        if (
            calculate_node_distance(new_node, self.goal_node) <= self.goal_threshold and
            check_collision_free(self.occupancy_map, new_node, self.goal_node)
        ):
            self.goal_node.parent = new_node
            self.goal_node.cost = (
                new_node.cost + calculate_node_distance(new_node, self.goal_node)
            )
            self.visited_nodes.add(self.goal_node)
            self.goal_found = True
            self.is_done.set()
    
    def postloop(self):
        if not self.goal_found:
            return ([], self.visited_nodes)
        return (
            collect_path(self.goal_node), 
            self.visited_nodes
        )
