"""
Integration tests for config file consistency.
Validates that frame IDs, topics, and parameters are consistent
across ekf.yaml, slam_params.yaml, and nav2_params.yaml.
"""
import os
import pytest
import yaml


CONFIG_DIR = os.path.join(os.path.dirname(__file__), '..', 'config')


def _load_yaml(filename):
    path = os.path.join(CONFIG_DIR, filename)
    with open(path) as f:
        return yaml.safe_load(f)


class TestEkfConfig:
    @pytest.fixture(scope='class')
    def ekf(self):
        return _load_yaml('ekf.yaml')

    def test_local_ekf_exists(self, ekf):
        assert 'ekf_filter_node_odom' in ekf

    def test_local_ekf_frames(self, ekf):
        params = ekf['ekf_filter_node_odom']['ros__parameters']
        assert params['odom_frame'] == 'odom'
        assert params['base_link_frame'] == 'base_footprint'
        assert params['world_frame'] == 'odom'

    def test_local_ekf_2d_mode(self, ekf):
        params = ekf['ekf_filter_node_odom']['ros__parameters']
        assert params['two_d_mode'] is True

    def test_odom_source_configured(self, ekf):
        params = ekf['ekf_filter_node_odom']['ros__parameters']
        assert 'odom0' in params
        assert params['odom0'] == 'odom'

    def test_imu_source_configured(self, ekf):
        params = ekf['ekf_filter_node_odom']['ros__parameters']
        assert 'imu0' in params
        assert params['imu0'] == 'imu/data'


class TestSlamConfig:
    @pytest.fixture(scope='class')
    def slam(self):
        return _load_yaml('slam_params.yaml')

    def test_slam_toolbox_exists(self, slam):
        assert 'slam_toolbox' in slam

    def test_slam_frames_match_ekf(self, slam):
        params = slam['slam_toolbox']['ros__parameters']
        assert params['odom_frame'] == 'odom'
        assert params['map_frame'] == 'map'
        assert params['base_frame'] == 'base_footprint'

    def test_scan_topic(self, slam):
        params = slam['slam_toolbox']['ros__parameters']
        assert params['scan_topic'] == '/scan'

    def test_sim_time_enabled(self, slam):
        params = slam['slam_toolbox']['ros__parameters']
        assert params['use_sim_time'] is True


class TestNav2Config:
    @pytest.fixture(scope='class')
    def nav2(self):
        return _load_yaml('nav2_params.yaml')

    def test_bt_navigator_frames(self, nav2):
        params = nav2['bt_navigator']['ros__parameters']
        assert params['global_frame'] == 'map'
        assert params['robot_base_frame'] == 'base_footprint'

    def test_controller_server_exists(self, nav2):
        assert 'controller_server' in nav2

    def test_rpp_controller_plugin(self, nav2):
        params = nav2['controller_server']['ros__parameters']
        fp = params['FollowPath']
        assert 'RegulatedPurePursuitController' in fp['plugin']

    def test_global_costmap_frame(self, nav2):
        params = nav2['global_costmap']['global_costmap']['ros__parameters']
        assert params['global_frame'] == 'map'
        assert params['robot_base_frame'] == 'base_footprint'

    def test_local_costmap_frame(self, nav2):
        params = nav2['local_costmap']['local_costmap']['ros__parameters']
        assert params['global_frame'] == 'odom'
        assert params['robot_base_frame'] == 'base_footprint'

    def test_behavior_server_frame(self, nav2):
        params = nav2['behavior_server']['ros__parameters']
        assert params['global_frame'] == 'odom'
        assert params['robot_base_frame'] == 'base_footprint'

    def test_scan_topic_in_costmap(self, nav2):
        """Verify the LiDAR topic in costmap matches the bridge/URDF."""
        params = nav2['global_costmap']['global_costmap']['ros__parameters']
        scan_config = params['obstacle_layer']['scan']
        assert scan_config['topic'] == '/scan'


class TestCrossConfigConsistency:
    """Verify frame IDs are consistent across ALL config files."""

    def test_base_frame_consistent(self):
        ekf = _load_yaml('ekf.yaml')
        slam = _load_yaml('slam_params.yaml')
        nav2 = _load_yaml('nav2_params.yaml')

        ekf_base = ekf['ekf_filter_node_odom']['ros__parameters']['base_link_frame']
        slam_base = slam['slam_toolbox']['ros__parameters']['base_frame']
        nav2_base = nav2['bt_navigator']['ros__parameters']['robot_base_frame']

        assert ekf_base == slam_base == nav2_base, \
            f"base_frame mismatch: EKF={ekf_base}, SLAM={slam_base}, Nav2={nav2_base}"

    def test_odom_frame_consistent(self):
        ekf = _load_yaml('ekf.yaml')
        slam = _load_yaml('slam_params.yaml')

        ekf_odom = ekf['ekf_filter_node_odom']['ros__parameters']['odom_frame']
        slam_odom = slam['slam_toolbox']['ros__parameters']['odom_frame']

        assert ekf_odom == slam_odom, \
            f"odom_frame mismatch: EKF={ekf_odom}, SLAM={slam_odom}"

    def test_map_frame_consistent(self):
        slam = _load_yaml('slam_params.yaml')
        nav2 = _load_yaml('nav2_params.yaml')

        slam_map = slam['slam_toolbox']['ros__parameters']['map_frame']
        nav2_map = nav2['bt_navigator']['ros__parameters']['global_frame']

        assert slam_map == nav2_map, \
            f"map_frame mismatch: SLAM={slam_map}, Nav2={nav2_map}"
