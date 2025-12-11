#!/usr/bin/env python3
"""
ENHANCED OUSTER OS1 BACKEND WITH WEB VISUALIZATION OUTPUT
==========================================================
Supports both REAL sensor and SIMULATION mode
"""

import os
import sys
import time
import json
import csv
import socket
import numpy as np
import open3d as o3d
from collections import deque
from datetime import datetime
from pathlib import Path

# Import from existing pipeline if available
try:
    from pipeline_utils import (
        ransac,
        dbscan,
        get_bounding_boxes,
        check_obb_against_envelope,
        sod_interp
    )
    PIPELINE_AVAILABLE = True
except ImportError:
    print("⚠️  pipeline_utils.py not found - using simplified processing")
    PIPELINE_AVAILABLE = False


# ============================================================================
# CONFIGURATION
# ============================================================================

class Config:
    """Configuration settings"""
    # MODE: 'REAL' or 'SIMULATION'
    MODE = 'SIMULATION'  # Change to 'REAL' when sensor is ready

    # Sensor settings
    SENSOR_IP = "192.168.1.1"
    SENSOR_PORT = 7502
    SENSOR_CONFIG_PORT = 7501

    # Processing settings
    VOXEL_SIZE = 0.05
    RANSAC_DISTANCE = 0.05
    DBSCAN_EPS = 0.25          # tuned for meter-scale data [web:1][web:8]
    DBSCAN_MIN_POINTS = 30

    # RDSO specifications (simplified)
    MMD_WIDTH = 3.25
    MMD_HEIGHT = 4.115
    SOD_CLEARANCE = 0.05
    INTRUSION_MINIMUM_M = 0.02
    PERSISTENCE_FRAMES = 3

    # Output settings
    OUTPUT_DIR = "Data/Output"
    WEB_OUTPUT_DIR = "Data/WebOutput"
    SAVE_POINT_CLOUDS = True
    GENERATE_REPORTS = True

    # Web streaming
    WEB_UPDATE_INTERVAL = 0.1
    MAX_POINTS_WEB = 5000


# ============================================================================
# WEB OUTPUT MANAGER
# ============================================================================

class WebOutputManager:
    """Manages JSON and CSV output for web visualization"""

    def __init__(self, config: Config):
        self.config = config
        self.csv_file = None
        self.csv_writer = None
        self.csv_path = None
        self.frame_count = 0

        os.makedirs(self.config.WEB_OUTPUT_DIR, exist_ok=True)
        self._init_csv()

    def _init_csv(self):
        """Initialize CSV file"""
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_path = os.path.join(
            self.config.WEB_OUTPUT_DIR,
            f'lidar_detections_{timestamp}.csv'
        )

        self.csv_file = open(self.csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)

        self.csv_writer.writerow([
            'Timestamp',
            'Frame',
            'Object_ID',
            'Center_X_m',
            'Center_Y_m',
            'Center_Z_m',
            'Width_mm',
            'Depth_mm',
            'Height_mm',
            'Volume_m3',
            'Point_Count',
            'Distance_from_LiDAR_m',
            'Lateral_Distance_m',
            'Intrudes',
            'Intrusion_Amount_mm',
            'Required_Clearance_m',
            'Actual_Clearance_m',
            'SOD_Height_Level_m',
            'Risk_Level'
        ])

        print(f"📊 CSV initialized: {self.csv_path}")

    def export_frame(self, frame_data):
        """Export frame data to JSON and CSV"""
        self.frame_count += 1
        timestamp = datetime.now().isoformat()

        web_data = {
            'timestamp': timestamp,
            'frame': int(self.frame_count),
            'lidar_position': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'rdso_envelope': {
                'mmd_width': float(self.config.MMD_WIDTH),
                'mmd_height': float(self.config.MMD_HEIGHT),
                'sod_clearance': float(self.config.SOD_CLEARANCE),
                'profile': self._get_envelope_profile()
            },
            'point_cloud': self._downsample_points(frame_data.get('points', [])),
            'detections': []
        }

        for det in frame_data.get('detections', []):
            detection_data = self._format_detection(det, timestamp)
            web_data['detections'].append(detection_data)
            self._write_csv_row(timestamp, detection_data)

        # Write latest.json
        latest_path = os.path.join(self.config.WEB_OUTPUT_DIR, 'latest.json')
        with open(latest_path, 'w') as f:
            json.dump(web_data, f, indent=2)

        # Write frame archive
        json_path = os.path.join(
            self.config.WEB_OUTPUT_DIR,
            f'frame_{self.frame_count:06d}.json'
        )
        with open(json_path, 'w') as f:
            json.dump(web_data, f, indent=2)

        return json_path

    def _get_envelope_profile(self):
        """Get simplified RDSO envelope profile points"""
        return [
            {'height': 0.0,   'clearance': 1.625},
            {'height': 1.0,   'clearance': 1.675},
            {'height': 2.0,   'clearance': 1.75},
            {'height': 3.0,   'clearance': 1.85},
            {'height': 4.115, 'clearance': 2.0}
        ]

    def _downsample_points(self, points):
        """Downsample point cloud for web display"""
        # points is a Python list of lists
        if len(points) <= self.config.MAX_POINTS_WEB:
            return [
                {
                    'x': float(p[0]),
                    'y': float(p[1]),
                    'z': float(p[2])
                } for p in points
            ]

        indices = np.random.choice(
            len(points),
            self.config.MAX_POINTS_WEB,
            replace=False
        )
        sampled = [points[int(i)] for i in indices]

        return [
            {
                'x': float(p[0]),
                'y': float(p[1]),
                'z': float(p[2])
            } for p in sampled
        ]

    def _format_detection(self, det, timestamp):
        """Format detection for web output"""
        center = np.asarray(det['center'], dtype=float)
        extent = np.asarray(det['extent'], dtype=float)

        width = float(extent[0])
        depth = float(extent[1])
        height = float(extent[2])

        volume = float(width * depth * height)
        distance_from_lidar = float(np.linalg.norm(center))
        lateral_distance = abs(float(center[1]))

        intrusion = float(det.get('max_intrusion_m', 0.0))
        intrudes_flag = bool(det.get('intrudes', False))

        if intrusion > 0.1:
            risk_level = 'CRITICAL'
        elif intrusion > 0.05:
            risk_level = 'HIGH'
        elif intrusion > 0.02:
            risk_level = 'MEDIUM'
        elif intrudes_flag:
            risk_level = 'LOW'
        else:
            risk_level = 'SAFE'

        detection_dict = {
            'id': int(det['label']),
            'timestamp': timestamp,
            'center': {
                'x': float(center[0]),
                'y': float(center[1]),
                'z': float(center[2])
            },
            'dimensions': {
                'width': width,
                'depth': depth,
                'height': height,
                'volume': volume
            },
            'bounds': {
                'min': {
                    'x': float(center[0] - width / 2.0),
                    'y': float(center[1] - depth / 2.0),
                    'z': float(center[2] - height / 2.0)
                },
                'max': {
                    'x': float(center[0] + width / 2.0),
                    'y': float(center[1] + depth / 2.0),
                    'z': float(center[2] + height / 2.0)
                }
            },
            'point_count': int(det['num_points']),
            'distance_from_lidar': distance_from_lidar,
            'lateral_distance': lateral_distance,
            'intrusion': {
                'intrudes': intrudes_flag,
                'amount': intrusion,
                'amount_mm': intrusion * 1000.0,
                'required_clearance': lateral_distance + intrusion,
                'actual_clearance': lateral_distance
            },
            'risk_level': risk_level,
            'color': self._get_risk_color(risk_level)
        }
        return detection_dict

    def _get_risk_color(self, risk_level):
        colors = {
            'SAFE': '#00ff00',
            'LOW': '#ffff00',
            'MEDIUM': '#ff9900',
            'HIGH': '#ff3300',
            'CRITICAL': '#ff0000'
        }
        return colors.get(risk_level, '#ffffff')

    def _write_csv_row(self, timestamp, det):
        """Write detection to CSV"""
        self.csv_writer.writerow([
            timestamp,
            int(self.frame_count),
            int(det['id']),
            float(det['center']['x']),
            float(det['center']['y']),
            float(det['center']['z']),
            float(det['dimensions']['width']) * 1000.0,
            float(det['dimensions']['depth']) * 1000.0,
            float(det['dimensions']['height']) * 1000.0,
            float(det['dimensions']['volume']),
            int(det['point_count']),
            float(det['distance_from_lidar']),
            float(det['lateral_distance']),
            'YES' if det['intrusion']['intrudes'] else 'NO',
            float(det['intrusion']['amount_mm']),
            float(det['intrusion']['required_clearance']),
            float(det['intrusion']['actual_clearance']),
            float(det['center']['z']),
            det['risk_level']
        ])
        self.csv_file.flush()

    def close(self):
        if self.csv_file:
            self.csv_file.close()
            print(f"\n✅ CSV saved: {self.csv_path}")


# ============================================================================
# SIMULATION MODE
# ============================================================================

class SimulationSensor:
    """Simulates LiDAR data for testing"""

    def __init__(self, config: Config):
        self.config = config
        self.frame_count = 0
        print("\n🎮 SIMULATION MODE ACTIVE")
        print("Generating synthetic LiDAR data...")

    def generate_frame(self):
        """Generate synthetic point cloud"""
        points = []

        # Ground plane
        for _ in range(1000):
            x = (np.random.rand() - 0.5) * 20.0
            y = (np.random.rand() - 0.5) * 10.0
            z = -0.05 + np.random.rand() * 0.1
            points.append([x, y, z])

        # Random objects
        num_objects = np.random.randint(2, 5)

        for obj_id in range(num_objects):
            center_x = (np.random.rand() - 0.5) * 10.0 + 8.0
            center_y = (np.random.rand() - 0.5) * 4.0
            center_z = 0.5 + np.random.rand() * 2.0

            width = 0.4 + np.random.rand() * 1.0
            depth = 0.4 + np.random.rand() * 1.0
            height = 0.7 + np.random.rand() * 2.5

            for _ in range(350):
                x = center_x + (np.random.rand() - 0.5) * width
                y = center_y + (np.random.rand() - 0.5) * depth
                z = center_z + np.random.rand() * height
                points.append([x, y, z])

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.array(points, dtype=float))

        self.frame_count += 1
        return pcd, time.time()


# ============================================================================
# OUSTER SENSOR
# ============================================================================

class OusterSensor:
    """Handles Ouster OS1 sensor with correct API"""

    def __init__(self, config: Config):
        self.config = config
        self.sensor_ip = None
        self.local_ip = None
        self.udp_socket = None

    def get_local_ip(self):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            local_ip = s.getsockname()[0]
            s.close()
            return local_ip
        except Exception:
            return "192.168.1.100"

    def test_connection(self, ip, port=7501, timeout=2):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(timeout)
        try:
            sock.connect((ip, port))
            sock.close()
            return True
        except Exception:
            return False

    def find_sensor(self):
        print("\n" + "=" * 70)
        print("STEP 1: AUTO-DETECTING OUSTER OS1 SENSOR")
        print("=" * 70)

        common_ips = [self.config.SENSOR_IP, "10.5.5.1", "192.168.0.100"]

        for ip in common_ips:
            print(f"Testing {ip}...", end=" ")
            if self.test_connection(ip):
                print("✅ FOUND!")
                self.sensor_ip = ip
                return True
            print("❌")

        print("\n⚠️  Sensor not found")
        return False

    def configure_sensor(self):
        """Configure sensor using HTTP REST API"""
        print("\n" + "=" * 70)
        print("STEP 2: CONFIGURING SENSOR")
        print("=" * 70)

        self.local_ip = self.get_local_ip()
        print(f"Local IP: {self.local_ip}")

        try:
            import requests

            base_url = f"http://{self.sensor_ip}"

            print(f"Fetching config from {base_url}/api/v1/sensor/config...")
            response = requests.get(f"{base_url}/api/v1/sensor/config", timeout=5)

            if response.status_code == 200:
                print("✅ Connected to sensor")

                config_data = {
                    "udp_ip": self.local_ip,
                    "udp_dest": self.local_ip,
                    "udp_port_lidar": self.config.SENSOR_PORT,
                    "udp_port_imu": 7503
                }

                print(f"Setting UDP destination to {self.local_ip}:{self.config.SENSOR_PORT}")
                response = requests.post(
                    f"{base_url}/api/v1/sensor/config",
                    json=config_data,
                    timeout=5
                )

                if response.status_code == 200:
                    print("✅ Configuration updated")
                    print("Reinitializing sensor...")
                    requests.post(f"{base_url}/api/v1/sensor/cmd/reinitialize", timeout=5)
                    time.sleep(3)
                    return True
                else:
                    print(f"⚠️  Config update returned: {response.status_code}")
                    return False
            else:
                print(f"⚠️  Connection failed: {response.status_code}")
                return False

        except ImportError:
            print("⚠️  'requests' library not found")
            print("Install with: pip install requests")
            print("\nAttempting fallback TCP configuration...")
            return self._configure_tcp_fallback()
        except Exception as e:
            print(f"⚠️  Configuration error: {e}")
            return self._configure_tcp_fallback()

    def _configure_tcp_fallback(self):
        """Fallback TCP configuration"""
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(5)

        try:
            sock.connect((self.sensor_ip, self.config.SENSOR_CONFIG_PORT))

            commands = [
                f"set_udp_dest {self.local_ip}\n",
                f"set_udp_port {self.config.SENSOR_PORT}\n",
                "reinitialize\n"
            ]

            for cmd in commands:
                sock.sendall(cmd.encode())
                time.sleep(0.5)

            print("✅ Fallback configuration sent")
            time.sleep(3)
            return True

        except Exception as e:
            print(f"⚠️  Fallback also failed: {e}")
            print("\nConfigure sensor via browser UI if needed.")
            return False
        finally:
            sock.close()

    def start_udp_stream(self):
        print("\n" + "=" * 70)
        print("STEP 3: STARTING UDP STREAM")
        print("=" * 70)

        for port in range(self.config.SENSOR_PORT, self.config.SENSOR_PORT + 10):
            try:
                self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self.udp_socket.bind(('0.0.0.0', port))
                self.udp_socket.settimeout(1.0)
                print(f"✅ UDP socket bound to port {port}")
                self.config.SENSOR_PORT = port
                return True
            except Exception:
                if self.udp_socket:
                    self.udp_socket.close()
                continue

        print("❌ Could not bind UDP socket")
        return False

    def receive_frame(self):
        """Receive and parse Ouster packet (simplified)"""
        if not self.udp_socket:
            return None, None

        try:
            data, addr = self.udp_socket.recvfrom(65536)

            if len(data) < 100:
                return None, None

            try:
                payload = data[100:]
                num_measurements = min(len(payload) // 12, 1024)

                points = []
                for i in range(num_measurements):
                    offset = i * 12
                    if offset + 12 > len(payload):
                        break

                    range_mm = int.from_bytes(payload[offset:offset + 4], 'little')

                    if range_mm < 100 or range_mm > 100000:
                        continue

                    range_m = range_mm / 1000.0

                    theta = (i % 64) * 2 * np.pi / 64
                    phi = ((i // 64) - 16) * np.pi / 180

                    x = range_m * np.cos(phi) * np.cos(theta)
                    y = range_m * np.cos(phi) * np.sin(theta)
                    z = range_m * np.sin(phi)

                    points.append([x, y, z])

                if len(points) > 10:
                    pcd = o3d.geometry.PointCloud()
                    pcd.points = o3d.utility.Vector3dVector(
                        np.array(points, dtype=float)
                    )
                    return pcd, time.time()

            except Exception:
                pass

            return None, None

        except socket.timeout:
            return None, None
        except Exception:
            return None, None


# ============================================================================
# PROCESSOR
# ============================================================================

class EnhancedProcessor:
    """Process point clouds and generate web output"""

    def __init__(self, config: Config):
        self.config = config
        self.frame_count = 0
        self.web_manager = WebOutputManager(config)

    def process_frame(self, pcd, timestamp):
        """Process frame"""
        if pcd is None or len(pcd.points) == 0:
            return None

        self.frame_count += 1
        raw_points = np.asarray(pcd.points, dtype=float)

        print(f"\n[Frame {self.frame_count}] Processing {len(raw_points)} points")

        # Optional voxel downsampling
        if self.config.VOXEL_SIZE > 0:
            pcd = pcd.voxel_down_sample(self.config.VOXEL_SIZE)
            print(f"  ▶ After voxel downsample: {len(pcd.points)} points")

        # Clustering
        detections = self._simple_clustering(pcd)

        # Fallback debug detection if no clusters
        if len(detections) == 0:
            debug_det = self._debug_single_detection(raw_points)
            if debug_det is not None:
                print("  ⚠️  Using DEBUG detection (non-ground bbox)")
                detections = [debug_det]

        frame_data = {
            'points': raw_points.tolist(),
            'detections': detections
        }

        json_path = self.web_manager.export_frame(frame_data)
        print(f"  ✅ Exported: {json_path}")
        print(f"  📊 Detections: {len(detections)}")

        return detections

    def _simple_clustering(self, pcd):
        """Simple clustering using Open3D DBSCAN"""
        detections = []

        try:
            labels = np.array(pcd.cluster_dbscan(
                eps=self.config.DBSCAN_EPS,
                min_points=self.config.DBSCAN_MIN_POINTS,
                print_progress=False
            ))
            unique_labels = set(labels.tolist())
            unique_labels.discard(-1)

            print(f"  ▶ Unique cluster labels (excluding noise): {unique_labels}")

            for label in unique_labels:
                indices = np.where(labels == label)[0]
                if len(indices) < self.config.DBSCAN_MIN_POINTS:
                    continue

                cluster = pcd.select_by_index(indices)
                obb = cluster.get_oriented_bounding_box()

                center = np.asarray(obb.center, dtype=float)
                extent = np.asarray(obb.extent, dtype=float)

                lateral_dist = abs(float(center[1]))
                required_clearance = 1.7
                intrudes_np = lateral_dist < required_clearance
                intrusion_amt = max(0.0, required_clearance - lateral_dist) if intrudes_np else 0.0

                detections.append({
                    'label': int(label),
                    'center': center.tolist(),
                    'extent': extent.tolist(),
                    'num_points': int(len(indices)),
                    'intrudes': bool(intrudes_np),
                    'max_intrusion_m': float(intrusion_amt)
                })

        except Exception as e:
            print(f"  ⚠️  Clustering error: {e}")

        return detections

    def _debug_single_detection(self, raw_points):
        """Create one bounding box around non-ground points (for visualization)"""
        if raw_points is None or len(raw_points) == 0:
            return None

        pts = np.asarray(raw_points, dtype=float)
        mask = pts[:, 2] > 0.15
        fg = pts[mask]

        if fg.shape[0] < 50:
            return None

        min_xyz = fg.min(axis=0)
        max_xyz = fg.max(axis=0)
        center = (min_xyz + max_xyz) / 2.0
        extent = (max_xyz - min_xyz)

        return {
            'label': 0,
            'center': center.tolist(),
            'extent': extent.tolist(),
            'num_points': int(fg.shape[0]),
            'intrudes': False,          # Python bool
            'max_intrusion_m': 0.0      # Python float
        }

    def close(self):
        self.web_manager.close()


# ============================================================================
# MAIN
# ============================================================================

def main():
    print("""
╔══════════════════════════════════════════════════════════════════╗
║     ENHANCED OUSTER OS1 + WEB VISUALIZATION BACKEND              ║
║     Generates JSON + CSV for real-time web display               ║
╚══════════════════════════════════════════════════════════════════╝
    """)

    config = Config()
    os.makedirs(config.OUTPUT_DIR, exist_ok=True)
    os.makedirs(config.WEB_OUTPUT_DIR, exist_ok=True)

    if config.MODE == 'SIMULATION':
        print("\n🎮 Running in SIMULATION mode")
        print("To use real sensor, set Config.MODE = 'REAL'")
        sensor = SimulationSensor(config)
    else:
        print("\n📡 Running in REAL SENSOR mode")
        sensor = OusterSensor(config)

        if not sensor.find_sensor():
            print("\n❌ Sensor not found. Switching to SIMULATION mode...")
            sensor = SimulationSensor(config)
        else:
            if not sensor.configure_sensor():
                print("\n⚠️  Configuration issues. Data may not stream correctly.")

            if not sensor.start_udp_stream():
                print("\n❌ UDP failed. Switching to SIMULATION mode...")
                sensor = SimulationSensor(config)

    processor = EnhancedProcessor(config)

    print("\n" + "=" * 70)
    print("STEP 4: MONITORING + WEB OUTPUT")
    print("=" * 70)
    print(f"📁 Output directory: {config.WEB_OUTPUT_DIR}")
    print(f"📄 Monitor: {config.WEB_OUTPUT_DIR}/latest.json")
    print(f"📊 CSV file: Creating...")
    print("\nPress Ctrl+C to stop\n")

    frame_count = 0
    last_print = time.time()

    try:
        while True:
            if isinstance(sensor, SimulationSensor):
                pcd, timestamp = sensor.generate_frame()
                time.sleep(0.2)  # ~5 FPS
            else:
                pcd, timestamp = sensor.receive_frame()
                if pcd is None:
                    time.sleep(0.01)
                    continue

            frame_count += 1
            processor.process_frame(pcd, timestamp)

            if time.time() - last_print > 5:
                print(f"\n✅ Processed {frame_count} frames")
                print(f"📄 Latest data: {config.WEB_OUTPUT_DIR}/latest.json")
                last_print = time.time()

    except KeyboardInterrupt:
        print("\n\n⏹️  Stopping...")

    processor.close()
    print(f"\n✅ Processed {frame_count} total frames")
    print(f"📊 CSV file: {processor.web_manager.csv_path}")
    print("\n✅ Complete!\n")


if __name__ == "__main__":
    main()
