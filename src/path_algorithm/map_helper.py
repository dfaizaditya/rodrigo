#!/usr/bin/env python3
import sys
import rospy
from nav_msgs.msg import OccupancyGrid, GridCells, Path, MapMetaData
from geometry_msgs.msg import Point, PoseWithCovarianceStamped, PoseStamped, PoseArray, Pose


def get_neighbors(index2d, my_map):

    list_of_neighbors = []

    x_index = index2d[0]
    y_index = index2d[1]

    if(is_valid_index2d((x_index, y_index - 1), my_map)):
        neighbor_n = (x_index, y_index - 1)
        list_of_neighbors.append(neighbor_n)

    if(is_valid_index2d((x_index + 1, y_index), my_map)):
        neighbor_e = (x_index + 1, y_index)
        list_of_neighbors.append(neighbor_e)

    if(is_valid_index2d((x_index, y_index + 1), my_map)):
        neighbor_s = (x_index, y_index + 1)
        list_of_neighbors.append(neighbor_s)

    if(is_valid_index2d((x_index - 1, y_index), my_map)):
        neighbor_w = (x_index - 1, y_index)
        list_of_neighbors.append(neighbor_w)

    return list_of_neighbors


def is_valid_index2d(index2d, my_map):
    x_index = index2d[0]
    y_index = index2d[1]

    if(x_index < 0 or x_index > my_map.info.width or y_index < 0 or y_index > my_map.info.height):
        return False

    cell_val = my_map.data[index2d_to_index1d(index2d, my_map)]
    if(cell_val == 0):
        return True
    else:
        return False


def world_to_index2d(loc, my_map):
    """converts points to the grid"""
    #take in a real world xy location, give back a 2d index
    x_point = loc[0]
    y_point = loc[1]

    x_index_offset = my_map.info.origin.position.x  # Get the x position of the map origin
    y_index_offset = my_map.info.origin.position.y  # Get the y position of the map origin
    x_point -= x_index_offset
    y_point -= y_index_offset

    res = my_map.info.resolution
    x_index = int(x_point / res)
    y_index = int(y_point / res)

    return (x_index, y_index)

def index2d_to_world(index2d, my_map):
    """convert a 2d index to a point"""
    x_index = index2d[0]
    y_index = index2d[1]

    res = my_map.info.resolution
    x_point = x_index * res
    y_point = y_index * res

    x_index_offset = my_map.info.origin.position.x
    y_index_offset = my_map.info.origin.position.y
    x_point += x_index_offset + res/2
    y_point += y_index_offset + res/2

    return (x_point, y_point)


def world_to_map(xy, my_map):
    x_offset = my_map.info.origin.position.x
    y_offset = my_map.info.origin.position.y
    new_x = xy[0] + x_offset
    new_y = xy[1] + y_offset
    return (new_x, new_y)


def map_to_world(xy, my_map):
    x_offset = my_map.info.origin.position.x
    y_offset = my_map.info.origin.position.y
    new_x = xy[0] + x_offset
    new_y = xy[1] + y_offset
    return (new_x, new_y)


def to_grid_cells(cells_to_paint, my_map):
    grid = GridCells()
    grid.header.frame_id = "/odom"
    grid.cell_height = my_map.info.resolution
    grid.cell_width = my_map.info.resolution

    grid.cells = []
    # Fill in points
    for index2d in cells_to_paint:
        point = Point()
        # point.x, point.y = index2d_to_point(index2d, my_map)
        point.x = index2d[0]
        point.y = index2d[1]
        grid.cells.append(point)

    return grid


def index2d_to_index1d(index2d, my_map):
    return index2d[1] * my_map.info.width + index2d[0]


