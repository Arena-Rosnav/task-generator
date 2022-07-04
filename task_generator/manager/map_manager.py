import numpy as np
import random
import math
from nav_msgs.msg import OccupancyGrid
from task_generator.constants import Constants


class MapManager:
    def __init__(self, map: OccupancyGrid):
        self.update_map(map)

    def update_map(self, map: OccupancyGrid):
        self.map = map

        self._set_freespace_indices()
    
    def get_random_pos_on_map(self, safe_dist: float, forbidden_zones: list = None):
        """
            Args:
                indices_y_x(tuple): a 2 elementary tuple stores the indices of the non-occupied cells, the first element is the y-axis indices,
                    the second element is the x-axis indices.
                map (OccupancyGrid): map proviced by the ros map service
                forbidden_zones (list of 3 elementary tuple(x,y,r)): a list of zones which is forbidden
            Returns:
            x_in_meters,y_in_meters,theta
        """
        print("SAFE DIST", safe_dist)

        assert len(self.free_space_indices) == 2 and len(self.free_space_indices[0]) == len(
            self.free_space_indices[1]), "free_space_indices is not correctly setup"

        n_freespace_cells = len(self.free_space_indices[0])
        n_check_failed = 0
        x_in_meters, y_in_meters = None, None

        while n_check_failed < 100:
            idx = random.randint(0, n_freespace_cells - 1)

            # in cells
            y_in_cells, x_in_cells = self.free_space_indices[0][idx], self.free_space_indices[1][idx]
            
            # convert x, y in meters
            y_in_meters = y_in_cells * self.map.info.resolution + self.map.info.origin.position.y
            x_in_meters = x_in_cells * self.map.info.resolution + self.map.info.origin.position.x

            if self._is_pos_valid(x_in_meters, y_in_meters, safe_dist, forbidden_zones):
                break

            n_check_failed += 1

        if n_check_failed >= 100:
            raise Exception(
                "cann't find any no-occupied space please check the map information")

        theta = random.uniform(-math.pi, math.pi)

        return x_in_meters, y_in_meters, theta

    def _is_pos_valid(self, x, y, safe_dist, forbidden_zones):
        return (
            self._check_safe_dist(x, y, safe_dist, forbidden_zones) 
            or self._check_static_map_dist(x, y, int(safe_dist / self.map.info.resolution))
        )

    def _check_safe_dist(self, x, y, safe_dist, forbidden_zones):
        for zone in forbidden_zones:
            if math.sqrt((x - zone[0]) ** 2 + (y - zone[1]) ** 2) < (zone[2] + safe_dist):
                return False
        
        return True

    def _check_static_map_dist(self, x, y, cell_radius):
        # in pixel
        x_index = int((x - self.map.info.origin.position.x) // self.map.info.resolution)
        y_index = int((y - self.map.info.origin.position.y) // self.map.info.resolution)

        # check occupancy around (x_index,y_index) with cell_radius
        # TODO use numpy for checking
        for i in range(x_index - cell_radius, x_index + cell_radius, 1):
            for j in range(y_index - cell_radius, y_index + cell_radius, 1):
                index = j * self.map.info.width + i

                if len(self.map.data) <= index or self.map.data[index]:
                    return False

        return True

    def _set_freespace_indices(self):
        ## generate the indices(represented in a tuple) of the freespace based on the map
        width_in_cell, height_in_cell = self.map.info.width, self.map.info.height

        map_2d = np.reshape(self.map.data, (height_in_cell, width_in_cell))

        self.free_space_indices = np.where(map_2d == 0)