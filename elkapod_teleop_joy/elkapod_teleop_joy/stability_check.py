import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from elkapod_msgs.msg import TouchSensorArrayStamped, LegPositions
import numpy as np

class StabilityCheck(Node):
    def __init__(self):
        super().__init__('stability_check')

        # Subscribers
        self.touch_sensor_sub = self.create_subscription(
            TouchSensorArrayStamped,
            '/elkapod_touch_sensors',
            self.touch_sensor_callback,
            10
        )

        self.leg_position_sub = self.create_subscription(
            LegPositions,
            '/elkapod_legs_positions',
            self.leg_position_callback,
            10
        )

        # Publishers
        self.bool_publisher = self.create_publisher(Bool, '/stability_boolean', 10)
        self.float_publisher = self.create_publisher(Float32, '/stability_float', 10)

        # Stored Data
        self.touch_sensor_data = None
        self.leg_position_data = None

    def touch_sensor_callback(self, msg: TouchSensorArrayStamped):
        self.touch_sensor_data = msg.sensor_state
        self.process_and_publish()

    def leg_position_callback(self, msg: LegPositions):
        self.leg_position_data = msg.leg_positions
        self.process_and_publish()


    ##############################################################################
    ### DO WYWALENIA ###
    ##############################################################################
    def check_stability(self, touching_points):
        polygon_points = self.convex_polygon_points(touching_points)
        if(len([pnt for pnt in polygon_points if (pnt[0] == 0 and pnt[1] == 0)])):
            return False, 0.0
        minimal_distance = self.calculate_minimal_distance_from_edge([0,0], polygon_points)
        return True, minimal_distance
        
        
    def is_counter_clockwise(self, p1, p2, p3) -> bool:
        if (p3[1]-p1[1])*(p2[0]-p1[0]) >= (p2[1]-p1[1])*(p3[0]-p1[0]):
            return True
        return False


    def convex_polygon_points(self, coordinates) -> list:
        """
        Solution to Convex Hull problem using Jarvis' March algorithm
        """
        n = len(coordinates)
        coordinates = np.array(coordinates)
        polygon_vertices = [None] * n
        most_left_point = np.where(coordinates[:,0] == np.min(coordinates[:,0]))
        new_vertex = coordinates[most_left_point[0][0]]
        i = 0
        while True:
            polygon_vertices[i] = new_vertex
            endpoint = coordinates[0]
            for j in range(1,n):
                if (endpoint[0] == new_vertex[0] and endpoint[1] == new_vertex[1]) or not self.is_counter_clockwise(coordinates[j],polygon_vertices[i],endpoint):
                    endpoint = coordinates[j]
            i = i + 1
            new_vertex = endpoint
            if endpoint[0] == polygon_vertices[0][0] and endpoint[1] == polygon_vertices[0][1]:
                break
        for i in range(n):
            if polygon_vertices[-1] is None:
                del polygon_vertices[-1]
        return polygon_vertices


    def line_from_two_points(self, p1, p2) -> list:
        A = p1[1] - p2[1]
        B = p2[0] - p1[0]
        C = p2[1]*p1[0] - p1[1]*p2[0]
        return [A, B, C]


    def distance_from_line(self, point, abc) -> float:
        A = abc[0]
        B = abc[1]
        C = abc[2]
        nominator = abs(A*point[0] + B*point[1] + C)
        denominator = np.sqrt(A**2 + B**2)
        return nominator/denominator


    def calculate_minimal_distance_from_edge(self, point, vertices) -> float:
        distances = []
        edge = self.line_from_two_points(vertices[-1], vertices[0])
        distances.append(self.distance_from_line(point, edge))
        for i in range(len(vertices)-1):
            edge = self.line_from_two_points(vertices[i],vertices[i+1])
            distances.append(self.distance_from_line(point, edge))
        return min(distances)


    ##############################################################################
    ### KONIEC CZĘŚCI DO WYWALENIA ###
    ##############################################################################

    def process_and_publish(self):
        # Ensuring that ther is data from both topics
        if self.touch_sensor_data is None or self.leg_position_data is None:
            return

# @TODO Legs coordinates to be adjusted to center mass point
        touching_legs = []
        for coun in range(6):
            if self.touch_sensor_data[coun]:
               touching_legs.append(np.array([self.leg_position_data[coun].x, self.leg_position_data[coun].y]))
        touching_legs.append(np.array([0,0]))
        [is_stable, distance_from_center_mass] = self.check_stability(touching_legs)

        # Publishing boolean value
        bool_msg = Bool()
        bool_msg.data = is_stable
        self.bool_publisher.publish(bool_msg)

        # Publishing float value
        float_msg = Float32()
        float_msg.data = distance_from_center_mass
        self.float_publisher.publish(float_msg)


def main(args=None):
    rclpy.init(args=args)
    node = StabilityCheck()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
