#!/usr/bin/env python3
import rospy
from nav_msgs.msg import OccupancyGrid, GridCells, Path
from geometry_msgs.msg import PointStamped, Pose, PoseStamped, Point
import map_helper
from PriorityQueue import PriorityQueue
import math
from rodrigo.srv import MakePath
import tf
from tf.transformations import euler_from_quaternion


class A_Star:

    def __init__(self):
        # Initialize node
        rospy.init_node("a_star", log_level=rospy.DEBUG)  # start node

        #Setup Map Publishers
        self.waypoints_pub = rospy.Publisher("local_costmap/waypoints", GridCells, queue_size=10)
        self.path_pub = rospy.Publisher("local_costmap/path", Path, queue_size=10)
        self.point_pub = rospy.Publisher("/point_cell", GridCells, queue_size=10)
        self.wavefront_pub = rospy.Publisher("local_costmap/wavefront", GridCells, queue_size=10)

    
        rospy.Subscriber("map", OccupancyGrid, self.dynamic_map_client)
        rospy.Service('make_path', MakePath, self.handle_a_star)
        
        self.map = None
        rospy.logdebug("Initializing A_Star")

        self.goal = PoseStamped()
        self.start = PoseStamped()
        self.pose = Pose()
        self.points = None

        self.rate = rospy.Rate(.5)

        while self.map is None and not rospy.is_shutdown():
            pass

    def handle_a_star(self, req):
        start = req.start
        goal = req.goal
    
        self.paint_point(start, goal)
        path = self.publish_path(self.points)
 
        return path

    def dynamic_map_client(self, new_map):
        rospy.logdebug("Getting map")
        self.map = new_map
        
        # Show map details for debugging
        rospy.logdebug("Resolution is: %s" % new_map.info.resolution)
        x_index_offset = self.map.info.origin.position.x
        y_index_offset = self.map.info.origin.position.y
        rospy.logdebug("Map Origin: x: %s y: %s" % (x_index_offset, y_index_offset))

    def paint_point(self, start_pose, end_pose):
        self.start = start_pose
        start_x = start_pose.pose.position.x
        start_y = start_pose.pose.position.y
        sq = start_pose.pose.orientation
        start_quat = [sq.x, sq.y, sq.z, sq.w]
        start_euler = euler_from_quaternion(start_quat)
        start_ang = start_euler[2]
        

        self.goal = end_pose
        end_x = end_pose.pose.position.x
        end_y = end_pose.pose.position.y
        eq = end_pose.pose.orientation
        end_quat = [eq.x, eq.y, eq.z, eq.w]
        end_euler = euler_from_quaternion(end_quat)
        end_ang = end_euler[2]

        # Print debug information
        rospy.logdebug("Start x, y, ang: %s %s %s" % (start_x, start_y, start_ang))
        rospy.logdebug("Goal x, y, ang: %s %s %s" % (end_x, end_y, end_ang))

        # Generate cell for end position
        painted_cell = map_helper.to_grid_cells([(end_x, end_y)], self.map)

        # Call A*
        self.a_star((start_x, start_y), (end_x, end_y))

        # Publish end point
        self.point_pub.publish(painted_cell)

    def a_star(self, start, goal):
        start = map_helper.world_to_index2d(start, self.map)
        goal = map_helper.world_to_index2d(goal, self.map)

        frontier = PriorityQueue()
        frontier.put(start, 0)
    
        came_from = {}
        cost_so_far = {}

        came_from[start] = None
        cost_so_far[start] = 0

        while not frontier.empty():
            current = frontier.get()
            rospy.logdebug("Current Node %s " % (current, ))

            if current == goal:
                break

            for next in map_helper.get_neighbors(current, self.map):
                rospy.logdebug("Next node %s" % (next,))

                # Find cost
                new_cost = cost_so_far[current] + self.move_cost(current, next)
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    priority = new_cost + self.euclidean_heuristic(goal, next)
                    frontier.put(next, priority)
                    came_from[next] = current

        frontier_list = frontier.get_items()
        frontier_map = []
        frontier_to_display = []
        for point in frontier_list:
            frontier_map.append(map_helper.world_to_index2d(point, self.map))
            frontier_to_display.append(map_helper.index2d_to_world(point, self.map))

        rospy.logdebug("Frontier list: %s " % frontier_list)

        # Generate path
        path = [map_helper.index2d_to_world(goal, self.map)]
        last_node = goal
        while came_from[last_node] is not None:
            next_node = came_from[last_node]
            path.insert(0, map_helper.index2d_to_world(next_node, self.map))
            last_node = next_node

        new_path = self.optimize_path(path)
        rospy.logdebug("Path %s " % new_path)

        self.paint_wavefront(frontier_to_display)

        self.paint_obstacles(new_path)
        self.paint_cells(frontier_list, new_path)
        self.points = new_path

    def euclidean_heuristic(self, point1, point2):
        return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)
    pass

    def move_cost(self, current, next):
        return abs(current[0] - next[0]) + abs(current[1] - next[1])


    def optimize_path(self, path):
        pathOptimized = []

        for idx in range(len(path)):
            if idx == 0 or idx == len(path) - 1:
                pathOptimized.append(path[idx])
            elif not self.redundant_point(path[idx - 1], path[idx], path[idx + 1]):
                pathOptimized.append(path[idx])

        return pathOptimized

    def redundant_point(self, last, curr, next):
        if last[0] == curr[0] == next[0]:
            return True
        if last[1] == curr[1] == next[1]:
            return True
        return False

    def paint_cells(self, frontier, came_from):
        # type: (list, list) -> None
        pass

    def publish_path(self, points):
        path_poses = []
        for point in points:
            # Generate pose
            pose = PoseStamped()
            # Mark frame
            pose.header.frame_id = "/odom"
            # Populate pose
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            path_poses += [pose]
        # Make path message
        path = Path()
        path.header.frame_id = "/odom"
        path.poses = path_poses
        # Publish to rviz
        self.path_pub.publish(path)
        # Return to send back in service reply
        return path

    def draw_circle(self):
        obstacles = [(math.cos(i/3.0), math.sin(i/3.0)) for i in range(0, 20)]
        rospy.logdebug(obstacles)
        cells = map_helper.to_grid_cells(obstacles, self.map)
        self.waypoints_pub.publish(cells)

    def paint_obstacles(self, obstacles):
        rospy.logdebug("Painting Path")
        cells = map_helper.to_grid_cells(obstacles, self.map)
        self.waypoints_pub.publish(cells)

    def paint_wavefront(self, wavefront):
        cells = map_helper.to_grid_cells(wavefront, self.map)
        self.wavefront_pub.publish(cells)


if __name__ == '__main__':
    astar = A_Star()
    rospy.loginfo("Initializing A_Star")

    rate = rospy.Rate(1)
    rate.sleep()
    # Draw the circle
    astar.draw_circle()
    while not rospy.is_shutdown():
        rate.sleep()
    rospy.spin()
    pass
