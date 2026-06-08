#!/usr/bin/env python3
from __future__ import annotations
import math
import random
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple
import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from duckietown_msgs.msg import Pose2DStamped
from geometry_msgs.msg import Point, Pose, PoseArray, PoseStamped, Quaternion
from nav_msgs.msg import Path
from sensor_msgs.msg import CompressedImage, Image
from visualization_msgs.msg import Marker, MarkerArray
REQUIRED_TAG_COUNT = 8
TAG_FAMILY_ALIASES = {'tag36h11': 'DICT_APRILTAG_36h11', 'apriltag_36h11': 'DICT_APRILTAG_36h11', '36h11': 'DICT_APRILTAG_36h11', 'dict_6x6_250': 'DICT_6X6_250', 'aruco_6x6_250': 'DICT_6X6_250'}

def resolve_tag_dictionary(tag_family: str):
    key = str(tag_family).strip().lower()
    dict_name = TAG_FAMILY_ALIASES.get(key)
    if dict_name is None:
        raise ValueError(f'''Unsupported tag_family "{tag_family}". Use one of: {', '.join(sorted(TAG_FAMILY_ALIASES))}''')
    dict_id = getattr(cv2.aruco, dict_name, None)
    if dict_id is None:
        raise RuntimeError(f'OpenCV build lacks {dict_name}. Install opencv-contrib-python>=4.7 for AprilTag tag36h11.')
    return (cv2.aruco.getPredefinedDictionary(dict_id), dict_name)

def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q

@dataclass
class Particle:
    x: float
    y: float
    theta: float
    weight: float

@dataclass
class TagObservation:
    range: float
    bearing_world: float

@dataclass
class OdomDelta:
    dx_body: float
    dy_body: float
    dtheta: float

class ParticleFilter:

    def __init__(self, num_particles: int, room_bounds: Tuple[float, float, float, float], tag_positions: Sequence[Tuple[float, float]], odom_noise_trans: float, odom_noise_rot: float, sensor_noise_range: float, sensor_noise_bearing: float, min_resample_neff_ratio: float) -> None:
        self.num_particles = num_particles
        (self.room_min_x, self.room_max_x, self.room_min_y, self.room_max_y) = room_bounds
        self.tag_positions = list(tag_positions)
        self.odom_noise_trans = odom_noise_trans
        self.odom_noise_rot = odom_noise_rot
        self.sensor_noise_range = sensor_noise_range
        self.sensor_noise_bearing = sensor_noise_bearing
        self.min_resample_neff_ratio = min_resample_neff_ratio
        if len(self.tag_positions) != REQUIRED_TAG_COUNT:
            raise ValueError(f'Expected exactly {REQUIRED_TAG_COUNT} tag positions for multi-hypothesis model.')
        self.particles: List[Particle] = []
        self._initialize_particles_uniform()

    def _initialize_particles_uniform(self) -> None:
        weight = 1.0 / self.num_particles
        self.particles = []
        for _ in range(self.num_particles):
            x = random.uniform(self.room_min_x, self.room_max_x)
            y = random.uniform(self.room_min_y, self.room_max_y)
            theta = random.uniform(-math.pi, math.pi)
            self.particles.append(Particle(x, y, theta, weight))

    def predict(self, delta: OdomDelta) -> None:
        for p in self.particles:
            noisy_dx = delta.dx_body + random.gauss(0.0, self.odom_noise_trans)
            noisy_dy = delta.dy_body + random.gauss(0.0, self.odom_noise_trans)
            noisy_dtheta = delta.dtheta + random.gauss(0.0, self.odom_noise_rot)
            p.x += noisy_dx * math.cos(p.theta) - noisy_dy * math.sin(p.theta)
            p.y += noisy_dx * math.sin(p.theta) + noisy_dy * math.cos(p.theta)
            p.theta = normalize_angle(p.theta + noisy_dtheta)
            p.x = min(max(p.x, self.room_min_x), self.room_max_x)
            p.y = min(max(p.y, self.room_min_y), self.room_max_y)

    def reinitialize_gaussian(self, x: float, y: float, theta: float, std_x: float, std_y: float, std_theta: float) -> None:
        weight = 1.0 / self.num_particles
        self.particles = []
        for _ in range(self.num_particles):
            px = random.gauss(x, std_x)
            py = random.gauss(y, std_y)
            ptheta = normalize_angle(random.gauss(theta, std_theta))
            px = min(max(px, self.room_min_x), self.room_max_x)
            py = min(max(py, self.room_min_y), self.room_max_y)
            self.particles.append(Particle(px, py, ptheta, weight))

    def _likelihood_given_tag(self, particle: Particle, observation: TagObservation, tag_x: float, tag_y: float) -> float:
        dx = tag_x - particle.x
        dy = tag_y - particle.y
        expected_range = math.hypot(dx, dy)
        expected_bearing_world = math.atan2(dy, dx)
        range_err = observation.range - expected_range
        bearing_err = normalize_angle(observation.bearing_world - expected_bearing_world)
        range_var = self.sensor_noise_range ** 2
        bearing_var = self.sensor_noise_bearing ** 2
        exponent = -0.5 * (range_err * range_err / range_var + bearing_err * bearing_err / bearing_var)
        return math.exp(exponent)

    def measurement_likelihood(self, particle: Particle, observation: TagObservation) -> float:
        total = 0.0
        for (tag_x, tag_y) in self.tag_positions:
            total += self._likelihood_given_tag(particle, observation, tag_x, tag_y)
        return total

    def update(self, observation: TagObservation) -> None:
        for p in self.particles:
            p.weight *= self.measurement_likelihood(p, observation)
        self._normalize_weights()

    def _normalize_weights(self) -> None:
        total = sum((p.weight for p in self.particles))
        if total <= 0.0:
            uniform = 1.0 / self.num_particles
            for p in self.particles:
                p.weight = uniform
            return
        for p in self.particles:
            p.weight /= total

    def effective_sample_size(self) -> float:
        return 1.0 / sum((p.weight * p.weight for p in self.particles))

    def resample_if_needed(self) -> None:
        neff = self.effective_sample_size()
        if neff >= self.min_resample_neff_ratio * self.num_particles:
            return
        self._low_variance_resample()

    def _low_variance_resample(self) -> None:
        weights = [p.weight for p in self.particles]
        n = self.num_particles
        new_particles: List[Particle] = []
        r = random.uniform(0.0, 1.0 / n)
        c = weights[0]
        i = 0
        for m in range(n):
            u = r + m * (1.0 / n)
            while u > c and i < n - 1:
                i += 1
                c += weights[i]
            sampled = self.particles[i]
            new_particles.append(Particle(sampled.x, sampled.y, sampled.theta, 1.0 / n))
        self.particles = new_particles

    def estimated_pose(self) -> Tuple[float, float, float]:
        x = sum((p.x * p.weight for p in self.particles))
        y = sum((p.y * p.weight for p in self.particles))
        sin_sum = sum((math.sin(p.theta) * p.weight for p in self.particles))
        cos_sum = sum((math.cos(p.theta) * p.weight for p in self.particles))
        theta = math.atan2(sin_sum, cos_sum)
        return (x, y, theta)

class ParticleFilterNodeROS1:

    def __init__(self) -> None:
        rospy.init_node('pf_node', anonymous=False)
        self._load_parameters()
        self.pf = ParticleFilter(num_particles=self.num_particles, room_bounds=(self.room_min_x, self.room_max_x, self.room_min_y, self.room_max_y), tag_positions=list(zip(self.tag_x, self.tag_y)), odom_noise_trans=self.odom_noise_trans, odom_noise_rot=self.odom_noise_rot, sensor_noise_range=self.sensor_noise_range, sensor_noise_bearing=self.sensor_noise_bearing, min_resample_neff_ratio=self.min_resample_neff_ratio)
        self.last_pose: Optional[Pose2DStamped] = None
        self._particles_seeded_from_odom = not self.seed_particles_from_odom
        self.odom_path_poses: List[PoseStamped] = []
        self.pf_path_poses: List[PoseStamped] = []
        self.bridge = CvBridge()
        (self.tag_dict, self.tag_dict_name) = resolve_tag_dictionary(self.tag_family)
        if hasattr(cv2.aruco, 'DetectorParameters'):
            self.tag_detector_params = cv2.aruco.DetectorParameters()
        else:
            self.tag_detector_params = cv2.aruco.DetectorParameters_create()
        self.pose_sub = rospy.Subscriber(self.pose_topic, Pose2DStamped, self.pose_callback, queue_size=10)
        self.image_sub = rospy.Subscriber(self.camera_image_topic, CompressedImage, self.image_callback, queue_size=1)
        self.particles_pub = rospy.Publisher('/particles', PoseArray, queue_size=10)
        self.particle_markers_pub = rospy.Publisher('/particle_markers', MarkerArray, queue_size=10)
        self.tag_markers_pub = rospy.Publisher('/tag_markers', MarkerArray, queue_size=10)
        self.estimated_pose_pub = rospy.Publisher('/estimated_pose', PoseStamped, queue_size=10)
        self.annotated_image_pub = rospy.Publisher('/camera/image_annotated', Image, queue_size=1)
        self.odom_path_pub = rospy.Publisher('/odom_path', Path, queue_size=10)
        self.pf_path_pub = rospy.Publisher('/pf_path', Path, queue_size=10)
        period = rospy.Duration(1.0 / self.publish_rate_hz)
        self.viz_timer = rospy.Timer(period, self.publish_visualization)
        init_mode = 'odometry seed' if self.seed_particles_from_odom else 'uniform (unknown pose)'
        rospy.loginfo('PF real robot ready: veh=%s, N=%d, room=[%.1f,%.1f]x[%.1f,%.1f], init=%s', self.veh, self.num_particles, self.room_min_x, self.room_max_x, self.room_min_y, self.room_max_y, init_mode)
        rospy.loginfo('Subscribing pose: %s', self.pose_topic)
        rospy.loginfo('Subscribing camera: %s', self.camera_image_topic)
        rospy.loginfo('Fiducial detector: %s (%s)', self.tag_family, self.tag_dict_name)

    def _load_parameters(self) -> None:
        self.veh = rospy.get_param('~veh', 'default_robot_name')
        self.num_particles = int(rospy.get_param('~num_particles', 500))
        self.room_min_x = float(rospy.get_param('~room_min_x', 0.0))
        self.room_max_x = float(rospy.get_param('~room_max_x', 5.0))
        self.room_min_y = float(rospy.get_param('~room_min_y', 0.0))
        self.room_max_y = float(rospy.get_param('~room_max_y', 4.0))
        self.tag_size_m = float(rospy.get_param('~tag_size_m', 0.2))
        self.tag_family = str(rospy.get_param('~tag_family', 'tag36h11'))
        self.tag_x = list(rospy.get_param('~tag_x'))
        self.tag_y = list(rospy.get_param('~tag_y'))
        self.tag_yaw = list(rospy.get_param('~tag_yaw'))
        self.odom_noise_trans = float(rospy.get_param('~odom_noise_trans', 0.05))
        self.odom_noise_rot = float(rospy.get_param('~odom_noise_rot', 0.05))
        self.sensor_noise_range = float(rospy.get_param('~sensor_noise_range', 0.25))
        self.sensor_noise_bearing = float(rospy.get_param('~sensor_noise_bearing', 0.12))
        self.min_resample_neff_ratio = float(rospy.get_param('~min_resample_neff_ratio', 0.5))
        self.publish_rate_hz = float(rospy.get_param('~publish_rate_hz', 10.0))
        self.map_frame = str(rospy.get_param('~map_frame', 'odom'))
        self.path_max_poses = int(rospy.get_param('~path_max_poses', 0))
        self.camera_yaw_offset = float(rospy.get_param('~camera_yaw_offset', 0.0))
        self.seed_particles_from_odom = bool(rospy.get_param('~seed_particles_from_odom', False))
        self.init_std_x = float(rospy.get_param('~init_std_x', 0.35))
        self.init_std_y = float(rospy.get_param('~init_std_y', 0.35))
        self.init_std_yaw = float(rospy.get_param('~init_std_yaw', 0.4))
        self.camera_image_width = int(rospy.get_param('~camera_image_width', 640))
        self.camera_image_height = int(rospy.get_param('~camera_image_height', 480))
        self.camera_horizontal_fov = float(rospy.get_param('~camera_horizontal_fov', 1.04))
        cam_param = str(rospy.get_param('~camera_image_topic', '')).strip()
        pose_param = str(rospy.get_param('~pose_topic', '')).strip()
        self.camera_image_topic = cam_param or f'/{self.veh}/camera_node/image/compressed'
        self.pose_topic = pose_param or f'/{self.veh}/kinematics_node/pose'
        if len(self.tag_x) != REQUIRED_TAG_COUNT or len(self.tag_y) != REQUIRED_TAG_COUNT:
            raise ValueError('tag_x and tag_y must each have length 8')
        if len(self.tag_yaw) != REQUIRED_TAG_COUNT:
            raise ValueError('tag_yaw must have length 8')
        half_w = self.camera_image_width * 0.5
        self._camera_fx = half_w / math.tan(self.camera_horizontal_fov * 0.5)
        self._camera_cx = half_w

    def _trim_path_history(self, history: List[PoseStamped]) -> None:
        if self.path_max_poses > 0 and len(history) > self.path_max_poses:
            del history[:-self.path_max_poses]

    def _pose_to_pose_stamped(self, msg: Pose2DStamped) -> PoseStamped:
        pose = PoseStamped()
        pose.header = msg.header
        pose.header.frame_id = self.map_frame
        pose.pose.position.x = msg.x
        pose.pose.position.y = msg.y
        pose.pose.position.z = 0.0
        pose.pose.orientation = yaw_to_quaternion(msg.theta)
        return pose

    def _record_odom_path_pose(self, msg: Pose2DStamped) -> None:
        self.odom_path_poses.append(self._pose_to_pose_stamped(msg))
        self._trim_path_history(self.odom_path_poses)

    @staticmethod
    def _pose_delta(prev: Pose2DStamped, curr: Pose2DStamped) -> OdomDelta:
        dx = curr.x - prev.x
        dy = curr.y - prev.y
        dtheta = normalize_angle(curr.theta - prev.theta)
        prev_yaw = prev.theta
        dx_body = math.cos(prev_yaw) * dx + math.sin(prev_yaw) * dy
        dy_body = -math.sin(prev_yaw) * dx + math.cos(prev_yaw) * dy
        return OdomDelta(dx_body, dy_body, dtheta)

    def pose_callback(self, msg: Pose2DStamped) -> None:
        if not self._particles_seeded_from_odom and self.seed_particles_from_odom:
            self.pf.reinitialize_gaussian(msg.x, msg.y, msg.theta, self.init_std_x, self.init_std_y, self.init_std_yaw)
            self._particles_seeded_from_odom = True
            rospy.loginfo('Seeded %d particles from pose at (%.2f, %.2f, %.2f rad)', self.num_particles, msg.x, msg.y, msg.theta)
        if self.last_pose is not None:
            delta = self._pose_delta(self.last_pose, msg)
            self.pf.predict(delta)
        self._record_odom_path_pose(msg)
        self.last_pose = msg

    @staticmethod
    def _marker_pixel_metrics(corners) -> Tuple[float, float, float]:
        pts = corners[0]
        us = [float(p[0]) for p in pts]
        vs = [float(p[1]) for p in pts]
        width_px = max(us) - min(us)
        return (sum(us) / 4.0, sum(vs) / 4.0, width_px)

    def _range_from_pixel_width(self, pixel_width: float) -> float:
        if pixel_width < 1.0:
            return float('inf')
        return self._camera_fx * self.tag_size_m / pixel_width

    def _bearing_camera_frame(self, center_u: float) -> float:
        return math.atan2(center_u - self._camera_cx, self._camera_fx)

    def _compressed_to_bgr(self, msg: CompressedImage) -> Optional[np.ndarray]:
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if frame is None:
                return None
            return frame
        except Exception as exc:
            rospy.logwarn('Compressed image decode failed: %s', exc)
            return None

    def image_callback(self, msg: CompressedImage) -> None:
        frame = self._compressed_to_bgr(msg)
        if frame is None:
            return
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        (corners, ids, _) = cv2.aruco.detectMarkers(gray, self.tag_dict, parameters=self.tag_detector_params)
        annotated = frame.copy()
        if self.last_pose is None:
            out = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            out.header = msg.header
            self.annotated_image_pub.publish(out)
            return
        odom_yaw = self.last_pose.theta
        if ids is not None and len(corners) > 0:
            cv2.aruco.drawDetectedMarkers(annotated, corners, ids)
            for marker_corners in corners:
                (center_u, _, width_px) = self._marker_pixel_metrics(marker_corners)
                range_m = self._range_from_pixel_width(width_px)
                bearing_cam = self._bearing_camera_frame(center_u)
                bearing_world = normalize_angle(odom_yaw + bearing_cam + self.camera_yaw_offset)
                if not math.isfinite(range_m) or range_m <= 0.0:
                    continue
                self.apply_tag_observation(TagObservation(range_m, bearing_world))
        out = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        out.header = msg.header
        self.annotated_image_pub.publish(out)

    def apply_tag_observation(self, observation: TagObservation) -> None:
        self.pf.update(observation)
        self.pf.resample_if_needed()

    @staticmethod
    def _weight_to_color(weight_norm: float) -> Tuple[float, float, float, float]:
        w = min(max(weight_norm, 0.0), 1.0)
        return (1.0 - w, w, 0.15, 0.25 + 0.75 * w)

    def _make_particle_markers(self, stamp) -> MarkerArray:
        markers = MarkerArray()
        weights = [p.weight for p in self.pf.particles]
        max_w = max(weights) if weights else 1.0
        if max_w <= 0.0:
            max_w = 1.0
        for (idx, p) in enumerate(self.pf.particles):
            w_norm = p.weight / max_w
            (r, g, b, a) = self._weight_to_color(w_norm)
            m = Marker()
            m.header.stamp = stamp
            m.header.frame_id = self.map_frame
            m.ns = 'particles'
            m.id = idx
            m.type = Marker.ARROW
            m.action = Marker.ADD
            m.pose.position.x = p.x
            m.pose.position.y = p.y
            m.pose.position.z = 0.02
            m.pose.orientation = yaw_to_quaternion(p.theta)
            m.scale.x = 0.12 + 0.08 * w_norm
            m.scale.y = 0.04 + 0.02 * w_norm
            m.scale.z = 0.04
            m.color.r = float(r)
            m.color.g = float(g)
            m.color.b = float(b)
            m.color.a = float(a)
            markers.markers.append(m)
        return markers

    def _make_tag_and_room_markers(self, stamp) -> MarkerArray:
        markers = MarkerArray()
        (x0, x1) = (self.room_min_x, self.room_max_x)
        (y0, y1) = (self.room_min_y, self.room_max_y)
        room = Marker()
        room.header.stamp = stamp
        room.header.frame_id = self.map_frame
        room.ns = 'room'
        room.id = 0
        room.type = Marker.LINE_STRIP
        room.action = Marker.ADD
        room.scale.x = 0.03
        room.color.r = 0.6
        room.color.g = 0.6
        room.color.b = 0.6
        room.color.a = 1.0
        room.points = [Point(x=x0, y=y0, z=0.01), Point(x=x1, y=y0, z=0.01), Point(x=x1, y=y1, z=0.01), Point(x=x0, y=y1, z=0.01), Point(x=x0, y=y0, z=0.01)]
        markers.markers.append(room)
        for (idx, (tx, ty)) in enumerate(zip(self.tag_x, self.tag_y)):
            tag = Marker()
            tag.header.stamp = stamp
            tag.header.frame_id = self.map_frame
            tag.ns = 'ar_tags'
            tag.id = idx
            tag.type = Marker.CUBE
            tag.action = Marker.ADD
            tag.pose.position.x = float(tx)
            tag.pose.position.y = float(ty)
            tag.pose.position.z = 0.2
            tag.pose.orientation.w = 1.0
            tag.scale.x = self.tag_size_m
            tag.scale.y = 0.02
            tag.scale.z = self.tag_size_m
            tag.color.r = 0.95
            tag.color.g = 0.85
            tag.color.b = 0.1
            tag.color.a = 0.85
            markers.markers.append(tag)
            label = Marker()
            label.header.stamp = stamp
            label.header.frame_id = self.map_frame
            label.ns = 'ar_tag_labels'
            label.id = idx
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = float(tx)
            label.pose.position.y = float(ty)
            label.pose.position.z = 0.35
            label.scale.z = 0.12
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 0.9
            label.text = 'T%d' % (idx + 1)
            markers.markers.append(label)
        return markers

    def publish_visualization(self, _event) -> None:
        stamp = rospy.Time.now()
        cloud = PoseArray()
        cloud.header.stamp = stamp
        cloud.header.frame_id = self.map_frame
        for p in self.pf.particles:
            pose = Pose()
            pose.position.x = p.x
            pose.position.y = p.y
            pose.position.z = 0.0
            pose.orientation = yaw_to_quaternion(p.theta)
            cloud.poses.append(pose)
        self.particles_pub.publish(cloud)
        self.particle_markers_pub.publish(self._make_particle_markers(stamp))
        self.tag_markers_pub.publish(self._make_tag_and_room_markers(stamp))
        (est_x, est_y, est_theta) = self.pf.estimated_pose()
        estimate = PoseStamped()
        estimate.header.stamp = stamp
        estimate.header.frame_id = self.map_frame
        estimate.pose.position.x = est_x
        estimate.pose.position.y = est_y
        estimate.pose.position.z = 0.0
        estimate.pose.orientation = yaw_to_quaternion(est_theta)
        self.estimated_pose_pub.publish(estimate)
        self.pf_path_poses.append(estimate)
        self._trim_path_history(self.pf_path_poses)
        odom_path = Path()
        odom_path.header.stamp = stamp
        odom_path.header.frame_id = self.map_frame
        odom_path.poses = list(self.odom_path_poses)
        self.odom_path_pub.publish(odom_path)
        pf_path = Path()
        pf_path.header.stamp = stamp
        pf_path.header.frame_id = self.map_frame
        pf_path.poses = list(self.pf_path_poses)
        self.pf_path_pub.publish(pf_path)

def main() -> None:
    try:
        ParticleFilterNodeROS1()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
if __name__ == '__main__':
    main()
