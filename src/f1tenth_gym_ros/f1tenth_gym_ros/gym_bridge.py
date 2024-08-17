import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Transform
# from geometry_msgs.msg import Quaternion
from ackermann_msgs.msg import AckermannDriveStamped
from tf2_ros import TransformBroadcaster

import gym
import numpy as np
from transforms3d import euler

from std_msgs.msg import Int8MultiArray, MultiArrayDimension

class GymBridge(Node):
    def __init__(self):
        super().__init__('gym_bridge')

        self.logger = self.get_logger()

        self.declare_parameter('scan_distance_to_base_link')
        self.declare_parameter('scan_fov')
        self.declare_parameter('scan_beams')
        self.declare_parameter('map_path')
        self.declare_parameter('map_img_ext')
        self.declare_parameter('num_agent')
        self.declare_parameter('kb_teleop')

        self.declare_parameter('drive_topic')
        self.declare_parameter('odom_topic')
        self.declare_parameter('scan_topic')

        self.declare_parameter('start_xs')
        self.declare_parameter('start_ys')
        self.declare_parameter('start_thetas')

        # check num_agents
        self.num_agents = self.get_parameter('num_agent').value
        if self.num_agents < 1 or self.num_agents > 2:
            raise ValueError('num_agents should be either 1 or 2.')
        elif type(self.num_agents) != int:
            raise ValueError('num_agents should be an int.')

        # env backend
        self.env = gym.make('f110_gym:f110-v0',
                            map=self.get_parameter('map_path').value,
                            map_ext=self.get_parameter('map_img_ext').value,
                            num_agents=self.num_agents)
        
        start_xs = self.get_parameter('start_xs').value
        start_ys = self.get_parameter('start_ys').value
        start_thetas = self.get_parameter('start_thetas').value

        scan_fov = self.get_parameter('scan_fov').value
        scan_beams = self.get_parameter('scan_beams').value
        self.angle_min = -scan_fov / 2.
        self.angle_max = scan_fov / 2.
        self.angle_inc = scan_fov / scan_beams
        self.scan_distance_to_base_link = self.get_parameter('scan_distance_to_base_link').value

        self.car_pose_list = []
        self.car_speed_list = []
        self.car_requested_speed_list = []
        self.car_steer_list = []
        self.car_collision_list = []
        car_scan_topic_list = []
        car_drive_topic_list = []
        car_odom_topic_list = []
        init_reset_list = []
        for i in range(self.num_agents):
            self.car_pose_list.append([start_xs[i], start_ys[i], start_thetas[i]])
            self.car_speed_list.append([0.0, 0.0, 0.0])
            self.car_requested_speed_list.append(0.0)
            self.car_steer_list.append(0.0)
            self.car_collision_list.append(True)

            car_scan_topic_list.append('/car' + str(i) + self.get_parameter('scan_topic').value)
            car_drive_topic_list.append('/car' + str(i) + self.get_parameter('drive_topic').value)
            car_odom_topic_list.append('/car' + str(i) + self.get_parameter('odom_topic').value)

            init_reset_list.append([start_xs[i], start_ys[i], start_thetas[i]])

        # initialize state
        self.obs, _ , self.done, _ = self.env.reset(np.array(init_reset_list))

        self.car_scan = []
        for i in range(self.num_agents):
            self.car_scan.append(list(self.obs['scans'][i]))
        
        if self.num_agents > 1:
            self.has_car1 = True
            self.car_collision_list[1] = True
        else:
            self.has_car1 = False

        # sim physical step timer
        self.drive_timer = self.create_timer(0.01, self.drive_timer_callback)
        # topic publishing timer
        self.timer = self.create_timer(0.004, self.timer_callback)

        # transform broadcaster
        self.br = TransformBroadcaster(self)

        # publishers
        self.car_scan_pub_list = []
        self.car_odom_pub_list = []
        self.car_drive_published_list = []
        for i in range(self.num_agents):
            self.car_scan_pub_list.append(self.create_publisher(LaserScan, car_scan_topic_list[i], 10))
            self.car_odom_pub_list.append(self.create_publisher(Odometry, car_odom_topic_list[i], 10))
            self.car_drive_published_list.append(False)
        
        self.car_collision_pub = self.create_publisher(Int8MultiArray, '/collisions', 10)

        # subscribers
        self.car_drive_sub_list = []
        for i in range(self.num_agents):
            drive_callback = self.create_car_drive_callback(i)
            self.car_drive_sub_list.append(
                self.create_subscription(
                    AckermannDriveStamped,
                    car_drive_topic_list[i],
                    drive_callback,
                    10
                )
            )
        
        self.car0_reset_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.car0_reset_callback,
            10)
        
        if self.num_agents > 1:
            self.car1_reset_sub = self.create_subscription(
                PoseStamped,
                '/goal_pose',
                self.car1_reset_callback,
                10)

        if self.get_parameter('kb_teleop').value:
            self.teleop_sub = self.create_subscription(
                Twist,
                '/cmd_vel',
                self.teleop_callback,
                10)


    def create_car_drive_callback(self, car_index):
        def car_drive_callback(drive_msg):
            self.car_requested_speed_list[car_index] = drive_msg.drive.speed
            self.car_steer_list[car_index] = drive_msg.drive.steering_angle
            self.car_drive_published_list[car_index] = True
        return car_drive_callback

    def car0_reset_callback(self, pose_msg):
        rx = pose_msg.pose.pose.position.x
        ry = pose_msg.pose.pose.position.y
        rqx = pose_msg.pose.pose.orientation.x
        rqy = pose_msg.pose.pose.orientation.y
        rqz = pose_msg.pose.pose.orientation.z
        rqw = pose_msg.pose.pose.orientation.w
        _, _, rtheta = euler.quat2euler([rqw, rqx, rqy, rqz], axes='sxyz')
        if self.has_car1:
            car1_pose = [self.obs['poses_x'][1], self.obs['poses_y'][1], self.obs['poses_theta'][1]]
            self.obs, _ , self.done, _ = self.env.reset(np.array([[rx, ry, rtheta], car1_pose]))
        else:
            self.obs, _ , self.done, _ = self.env.reset(np.array([[rx, ry, rtheta]]))

    def car1_reset_callback(self, pose_msg):
        if self.has_car1:
            rx = pose_msg.pose.position.x
            ry = pose_msg.pose.position.y
            rqx = pose_msg.pose.orientation.x
            rqy = pose_msg.pose.orientation.y
            rqz = pose_msg.pose.orientation.z
            rqw = pose_msg.pose.orientation.w
            _, _, rtheta = euler.quat2euler([rqw, rqx, rqy, rqz], axes='sxyz')
            self.obs, _ , self.done, _ = self.env.reset(np.array([list(self.car_pose_list[0]), [rx, ry, rtheta]]))


    def teleop_callback(self, twist_msg):
        if not self.car_drive_published_list[0]:
            self.car_drive_published_list[0] = True

        self.car_requested_speed_list[0] = twist_msg.linear.x

        if twist_msg.angular.z > 0.0:
            self.car_steer_list[0] = 0.3
        elif twist_msg.angular.z < 0.0:
            self.car_steer_list[0] = -0.3
        else:
            self.car_steer_list[0] = 0.0

    def drive_timer_callback(self):
        env_command_list = []
        for i in range(self.num_agents):
            if (self.car_drive_published_list[i]):
                env_command_list.append([self.car_steer_list[i], self.car_requested_speed_list[i]])
            else:
                env_command_list.append([0.0, 0.0])
        self.obs, _, self.done, _ = self.env.step(np.array(env_command_list))

        self._update_sim_state()

    def timer_callback(self):
        ts = self.get_clock().now().to_msg()

        for i in range(self.num_agents):
            # pub scans
            scan = LaserScan()
            scan.header.stamp = ts
            scan.header.frame_id = 'car' + str(i) + '/laser'
            scan.angle_min = self.angle_min
            scan.angle_max = self.angle_max
            scan.angle_increment = self.angle_inc
            scan.range_min = 0.
            scan.range_max = 30.
            scan.ranges = self.car_scan[i]
            self.car_scan_pub_list[i].publish(scan)

        # pub tf
        self._publish_odom(ts)
        self._publish_transforms(ts)
        self._publish_laser_transforms(ts)
        self._publish_wheel_transforms(ts)

    def _update_sim_state(self):
        for i in range(self.num_agents):
            self.car_scan[i] = list(self.obs['scans'][i])
            self.car_pose_list[i][0] = self.obs['poses_x'][i]
            self.car_pose_list[i][1] = self.obs['poses_y'][i]
            self.car_pose_list[i][2] = self.obs['poses_theta'][i]
            self.car_speed_list[i][0] = self.obs['linear_vels_x'][i]
            self.car_speed_list[i][1] = self.obs['linear_vels_y'][i]
            self.car_speed_list[i][2] = self.obs['ang_vels_z'][i]

        self._publish_collision()
        
        
    def _publish_collision(self):
        collision_msg = Int8MultiArray()
        collision_array = self.obs['collisions'].flatten().astype(np.int8).tolist()
        collision_msg.data = collision_array

        dim = MultiArrayDimension()
        dim.label = "num_agents"
        dim.size = len(collision_array)
        collision_msg.layout.dim = [dim]

        self.car_collision_pub.publish(collision_msg)

    def _publish_odom(self, ts):
        for i in range(self.num_agents):
            car_odom = Odometry()
            car_odom.header.stamp = ts
            car_odom.header.frame_id = 'map'
            car_odom.child_frame_id = 'car' + str(i) + '/base_link'
            car_odom.pose.pose.position.x = self.car_pose_list[i][0]
            car_odom.pose.pose.position.y = self.car_pose_list[i][1]
            car_quat = euler.euler2quat(0., 0., self.car_pose_list[i][2], axes='sxyz')
            car_odom.pose.pose.orientation.x = car_quat[1]
            car_odom.pose.pose.orientation.y = car_quat[2]
            car_odom.pose.pose.orientation.z = car_quat[3]
            car_odom.pose.pose.orientation.w = car_quat[0]
            car_odom.twist.twist.linear.x = self.car_speed_list[i][0]
            car_odom.twist.twist.linear.y = self.car_speed_list[i][1]
            car_odom.twist.twist.angular.z = self.car_speed_list[i][2]
            self.car_odom_pub_list[i].publish(car_odom)

    def _publish_transforms(self, ts):
        for i in range(self.num_agents):
            car_t = Transform()
            car_t.translation.x = self.car_pose_list[i][0]
            car_t.translation.y = self.car_pose_list[i][1]
            car_t.translation.z = 0.0
            car_quat = euler.euler2quat(0.0, 0.0, self.car_pose_list[i][2], axes='sxyz')
            car_t.rotation.x = car_quat[1]
            car_t.rotation.y = car_quat[2]
            car_t.rotation.z = car_quat[3]
            car_t.rotation.w = car_quat[0]

            car_ts = TransformStamped()
            car_ts.transform = car_t
            car_ts.header.stamp = ts
            car_ts.header.frame_id = 'map'
            car_ts.child_frame_id = 'car' + str(i) + '/base_link'
            self.br.sendTransform(car_ts)

    def _publish_wheel_transforms(self, ts):
        for i in range(self.num_agents):
            car_wheel_ts = TransformStamped()
            car_wheel_quat = euler.euler2quat(0., 0., self.car_steer_list[i], axes='sxyz')
            car_wheel_ts.transform.rotation.x = car_wheel_quat[1]
            car_wheel_ts.transform.rotation.y = car_wheel_quat[2]
            car_wheel_ts.transform.rotation.z = car_wheel_quat[3]
            car_wheel_ts.transform.rotation.w = car_wheel_quat[0]
            car_wheel_ts.header.stamp = ts
            car_wheel_ts.header.frame_id = 'car' + str(i) + '/front_left_hinge'
            car_wheel_ts.child_frame_id = 'car' + str(i) + '/front_left_wheel'
            self.br.sendTransform(car_wheel_ts)
            car_wheel_ts.header.frame_id = 'car' + str(i) + '/front_right_hinge'
            car_wheel_ts.child_frame_id = 'car' + str(i) + '/front_right_wheel'
            self.br.sendTransform(car_wheel_ts)

    def _publish_laser_transforms(self, ts):
        for i in range(self.num_agents):
            car_scan_ts = TransformStamped()
            car_scan_ts.transform.translation.x = self.scan_distance_to_base_link
            # ego_scan_ts.transform.translation.z = 0.04+0.1+0.025
            car_scan_ts.transform.rotation.w = 1.
            car_scan_ts.header.stamp = ts
            car_scan_ts.header.frame_id = 'car' + str(i) + '/base_link'
            car_scan_ts.child_frame_id = 'car' + str(i) + '/laser'
            self.br.sendTransform(car_scan_ts)

def main(args=None):
    rclpy.init(args=args)
    gym_bridge = GymBridge()
    rclpy.spin(gym_bridge)

if __name__ == '__main__':
    main()
