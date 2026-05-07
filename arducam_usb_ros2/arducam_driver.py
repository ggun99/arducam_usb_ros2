#!/usr/bin/env python3
"""
ROS2 node that publishes two ArduCAM streams with CameraInfo.

Publishes:
- <left_image_topic>  (sensor_msgs/Image)
- <right_image_topic> (sensor_msgs/Image)
- <left_info_topic>   (sensor_msgs/CameraInfo)
- <right_info_topic>  (sensor_msgs/CameraInfo)
"""

import argparse
import time
from pathlib import Path
import os
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
from .Arducam import *
from .ImageConvert import *


def load_intrinsics(k_path: str, d_path: str):
    k = np.load(k_path)
    d = np.load(d_path)
    k = np.asarray(k, dtype=np.float64).reshape(3, 3)
    d = np.asarray(d, dtype=np.float64).reshape(-1)
    return k, d


def make_camera_info(k: np.ndarray, d: np.ndarray, width: int, height: int, frame_id: str):
    msg = CameraInfo()
    msg.width = int(width)
    msg.height = int(height)
    msg.distortion_model = "plumb_bob"
    msg.d = d.tolist()

    msg.k = [
        float(k[0, 0]), float(k[0, 1]), float(k[0, 2]),
        float(k[1, 0]), float(k[1, 1]), float(k[1, 2]),
        float(k[2, 0]), float(k[2, 1]), float(k[2, 2]),
    ]

    msg.r = [1.0, 0.0, 0.0,
             0.0, 1.0, 0.0,
             0.0, 0.0, 1.0]

    msg.p = [
        float(k[0, 0]), float(k[0, 1]), float(k[0, 2]), 0.0,
        float(k[1, 0]), float(k[1, 1]), float(k[1, 2]), 0.0,
        float(k[2, 0]), float(k[2, 1]), float(k[2, 2]), 0.0,
    ]

    msg.header.frame_id = frame_id
    return msg


class ArducamStereoPublisher(Node):
    def __init__(self, args):
        super().__init__("arducam_stereo_publisher")

        self.cfg = args.config_file
        self.flip = args.flip
        self.publish_rate = args.rate

        self.left_frame_id = args.left_frame_id
        self.right_frame_id = args.right_frame_id

        self.left_image_topic = args.left_image_topic
        self.right_image_topic = args.right_image_topic
        self.left_info_topic = args.left_info_topic
        self.right_info_topic = args.right_info_topic

        self.left_k, self.left_d = load_intrinsics(args.left_k, args.left_d)
        self.right_k, self.right_d = load_intrinsics(args.right_k, args.right_d)

        self.bridge = CvBridge()
        self.left_pub = self.create_publisher(Image, self.left_image_topic, 10)
        self.right_pub = self.create_publisher(Image, self.right_image_topic, 10)
        self.left_info_pub = self.create_publisher(CameraInfo, self.left_info_topic, 10)
        self.right_info_pub = self.create_publisher(CameraInfo, self.right_info_topic, 10)

        self.camera0 = ArducamCamera()
        self.camera1 = ArducamCamera()

        self.get_logger().info("Opening ArduCAM devices...")
        self.isopen0, _ = self.camera0.openCamera(self.cfg, index=0)
        self.isopen1, _ = self.camera1.openCamera(self.cfg, index=1)
        if not self.isopen0 or not self.isopen1:
            raise RuntimeError("Failed to open one or both cameras.")

        self.camera0.start()
        self.camera1.start()

        self.last_info = 0.0
        self.info_interval = 1.0

        period = 1.0 / max(self.publish_rate, 1e-3)
        self.timer = self.create_timer(period, self._on_timer)

    def _on_timer(self):
        ret0, data0, cfg0 = self.camera0.read()
        ret1, data1, cfg1 = self.camera1.read()

        if not (ret0 and ret1):
            self.get_logger().warn("Read timeout")
            return

        img0 = convert_image(data0, cfg0, self.camera0.color_mode)
        img1 = convert_image(data1, cfg1, self.camera1.color_mode)

        if self.flip:
            img0 = cv2.flip(img0, 1)
            img1 = cv2.flip(img1, 1)

        now = self.get_clock().now().to_msg()

        left_msg = self.bridge.cv2_to_imgmsg(img0, encoding="bgr8")
        right_msg = self.bridge.cv2_to_imgmsg(img1, encoding="bgr8")

        left_msg.header.stamp = now
        right_msg.header.stamp = now
        left_msg.header.frame_id = self.left_frame_id
        right_msg.header.frame_id = self.right_frame_id

        self.left_pub.publish(left_msg)
        self.right_pub.publish(right_msg)

        current = time.time()
        if current - self.last_info >= self.info_interval:
            h0, w0 = img0.shape[:2]
            h1, w1 = img1.shape[:2]

            left_info = make_camera_info(self.left_k, self.left_d, w0, h0, self.left_frame_id)
            right_info = make_camera_info(self.right_k, self.right_d, w1, h1, self.right_frame_id)
            left_info.header.stamp = now
            right_info.header.stamp = now

            self.left_info_pub.publish(left_info)
            self.right_info_pub.publish(right_info)
            self.last_info = current

    def destroy_node(self):
        try:
            self.camera0.stop()
            self.camera1.stop()
            self.camera0.closeCamera()
            self.camera1.closeCamera()
        except Exception:
            pass
        super().destroy_node()

def load_intrinsics(k_path: str, d_path: str):
    # 만약 입력된 경로가 절대 경로가 아니라면, 
    # 현재 작업 디렉토리 또는 패키지 share 디렉토리에서 찾도록 보정
    def get_abs_path(target_path):
        if os.path.isabs(target_path):
            return target_path
        # 1. 현재 터미널 실행 위치 기준 확인
        if os.path.exists(target_path):
            return os.path.abspath(target_path)
        # 2. 패키지 설치 위치(share) 기준 확인 (config 폴더 내에 저장했다고 가정)
        try:
            package_share = get_package_share_directory('arducam_usb_ros2')
            share_path = os.path.join(package_share, 'config', target_path)
            if os.path.exists(share_path):
                return share_path
        except Exception:
            pass
        return target_path # 마지막 수단으로 원래 값 반환

    abs_k = get_abs_path(k_path)
    abs_d = get_abs_path(d_path)

    if not os.path.exists(abs_k) or not os.path.exists(abs_d):
        raise FileNotFoundError(f"Intrinsic files not found: {abs_k} or {abs_d}")

    k = np.load(abs_k)
    d = np.load(abs_d)
    k = np.asarray(k, dtype=np.float64).reshape(3, 3)
    d = np.asarray(d, dtype=np.float64).reshape(-1)
    return k, d

def parse_args():
    parser = argparse.ArgumentParser(description="ArduCAM stereo ROS2 publisher")
    parser.add_argument("-f", "--config-file", required=True, help="ArduCAM config file")

    parser.add_argument("--left-serial", default="0002", help="Left camera serial suffix")
    parser.add_argument("--right-serial", default="0019", help="Right camera serial suffix")

    parser.add_argument("--left-k", default=None, help="Left camera K .npy")
    parser.add_argument("--left-d", default=None, help="Left camera D .npy")
    parser.add_argument("--right-k", default=None, help="Right camera K .npy")
    parser.add_argument("--right-d", default=None, help="Right camera D .npy")

    parser.add_argument("--left-image-topic", default=None)
    parser.add_argument("--right-image-topic", default=None)
    parser.add_argument("--left-info-topic", default=None)
    parser.add_argument("--right-info-topic", default=None)

    parser.add_argument("--left-frame-id", default=None)
    parser.add_argument("--right-frame-id", default=None)

    parser.add_argument("--rate", type=float, default=30.0, help="Publish rate (Hz)")
    parser.add_argument("--flip", action="store_true", help="Flip images horizontally")

    args = parser.parse_args()

    # config 파일도 절대 경로로 변환 시도
    if not os.path.isabs(args.config_file):
        try:
            package_share = get_package_share_directory('arducam_usb_ros2')
            # 만약 config 폴더에 .cfg를 넣어뒀다면
            potential_cfg = os.path.join(package_share, 'config', args.config_file)
            if os.path.exists(potential_cfg):
                args.config_file = potential_cfg
        except Exception:
            pass
    
    if args.left_k is None:
        args.left_k = f"ac_sn_{args.left_serial}_camera_mat.npy"
    if args.left_d is None:
        args.left_d = f"ac_sn_{args.left_serial}_dist_coefs.npy"
    if args.right_k is None:
        args.right_k = f"ac_sn_{args.right_serial}_camera_mat.npy"
    if args.right_d is None:
        args.right_d = f"ac_sn_{args.right_serial}_dist_coefs.npy"

    if args.left_image_topic is None:
        args.left_image_topic = f"/arducam/_{args.left_serial}/image_raw"
    if args.right_image_topic is None:
        args.right_image_topic = f"/arducam/_{args.right_serial}/image_raw"
    if args.left_info_topic is None:
        args.left_info_topic = f"/arducam/_{args.left_serial}/camera_info"
    if args.right_info_topic is None:
        args.right_info_topic = f"/arducam/_{args.right_serial}/camera_info"

    if args.left_frame_id is None:
        args.left_frame_id = f"arducam_{args.left_serial}"
    if args.right_frame_id is None:
        args.right_frame_id = f"arducam_{args.right_serial}"

    return args


def main():
    args = parse_args()
    # [수정된 경로 보정 로직]
    if not os.path.isabs(args.config_file):
        try:
            package_share = get_package_share_directory('arducam_usb_ros2')
            # setup.py 설정에 따라 share/arducam_usb_ros2/config 디렉토리를 확인
            potential_path = os.path.join(package_share, 'config', args.config_file)
            
            if os.path.exists(potential_path):
                args.config_file = potential_path
            else:
                # 찾지 못했을 경우 현재 경로를 출력하여 디버깅 도움
                print(f"[DEBUG] Config not found in: {potential_path}")
        except Exception as e:
            print(f"[DEBUG] Error locating package share: {e}")

    # 최종 경로가 실제로 존재하는지 다시 확인
    if not os.path.exists(args.config_file):
        print(f"[ERROR] Cannot find config file at: {args.config_file}")
    else:
        print(f"[SUCCESS] Loading config from: {args.config_file}")
        
    rclpy.init()

    node = ArducamStereoPublisher(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
