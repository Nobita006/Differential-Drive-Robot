"""
Integration tests for the URDF / xacro model.
Parses the xacro and validates links, joints, and plugin configuration.
"""
import os
import pytest
import xml.etree.ElementTree as ET


URDF_PATH = os.path.join(
    os.path.dirname(__file__), '..', 'urdf', 'mower.urdf.xacro'
)


@pytest.fixture(scope='module')
def urdf_xml():
    """Process the xacro file and return the parsed XML tree."""
    import xacro
    doc = xacro.process_file(URDF_PATH)
    return ET.fromstring(doc.toxml())


EXPECTED_LINKS = [
    'base_link', 'base_footprint', 'left_wheel', 'right_wheel',
    'camera_link', 'lidar_link', 'imu_link', 'caster_front_wheel',
]

EXPECTED_JOINTS = {
    'base_footprint_joint': 'fixed',
    'left_wheel_joint': 'continuous',
    'right_wheel_joint': 'continuous',
    'camera_joint': 'fixed',
    'lidar_joint': 'fixed',
    'imu_joint': 'fixed',
    'caster_front_joint': 'fixed',
}


def _get_plugins(urdf_xml):
    """Collect all <plugin> elements from within <gazebo> blocks."""
    plugins = []
    for gz in urdf_xml.findall('.//gazebo'):
        plugins.extend(gz.findall('plugin'))
    return plugins


def _get_sensors(urdf_xml):
    """Collect all <sensor> elements from within <gazebo> blocks."""
    sensors = []
    for gz in urdf_xml.findall('.//gazebo'):
        sensors.extend(gz.findall('.//sensor'))
    return sensors


class TestLinks:
    def test_all_expected_links_exist(self, urdf_xml):
        link_names = [link.get('name') for link in urdf_xml.findall('.//link')]
        for name in EXPECTED_LINKS:
            assert name in link_names, f"Missing link: {name}"

    def test_base_link_has_collision(self, urdf_xml):
        base = [l for l in urdf_xml.findall('.//link') if l.get('name') == 'base_link'][0]
        assert base.find('collision') is not None

    def test_wheels_have_inertia(self, urdf_xml):
        for wheel in ['left_wheel', 'right_wheel']:
            link = [l for l in urdf_xml.findall('.//link') if l.get('name') == wheel][0]
            assert link.find('inertial') is not None
            assert link.find('inertial/mass') is not None


class TestJoints:
    def test_all_expected_joints_exist(self, urdf_xml):
        joint_names = [j.get('name') for j in urdf_xml.findall('.//joint')]
        for name in EXPECTED_JOINTS:
            assert name in joint_names, f"Missing joint: {name}"

    def test_joint_types(self, urdf_xml):
        joints = {j.get('name'): j.get('type') for j in urdf_xml.findall('.//joint')}
        for name, expected_type in EXPECTED_JOINTS.items():
            assert joints.get(name) == expected_type, \
                f"Joint {name}: expected {expected_type}, got {joints.get(name)}"

    def test_wheel_axis_is_y(self, urdf_xml):
        for jname in ['left_wheel_joint', 'right_wheel_joint']:
            joint = [j for j in urdf_xml.findall('.//joint') if j.get('name') == jname][0]
            axis = joint.find('axis')
            assert axis is not None
            xyz = axis.get('xyz')
            assert xyz == '0 1 0', f"{jname} axis should be '0 1 0', got '{xyz}'"


class TestPlugins:
    def test_diff_drive_plugin_present(self, urdf_xml):
        plugins = _get_plugins(urdf_xml)
        dd = [p for p in plugins if 'DiffDrive' in (p.get('name') or '')]
        assert len(dd) == 1, f"DiffDrive plugin not found. Plugins: {[p.get('name') for p in plugins]}"

    def test_diff_drive_topics(self, urdf_xml):
        plugins = _get_plugins(urdf_xml)
        dd = [p for p in plugins if 'DiffDrive' in (p.get('name') or '')][0]
        assert dd.find('topic').text == '/cmd_vel'
        assert dd.find('odom_topic').text == '/odom'
        assert dd.find('frame_id').text == 'odom'
        assert dd.find('child_frame_id').text == 'base_footprint'

    def test_odom_tf_disabled(self, urdf_xml):
        """publish_odom_tf should be false (EKF handles this)."""
        plugins = _get_plugins(urdf_xml)
        dd = [p for p in plugins if 'DiffDrive' in (p.get('name') or '')][0]
        assert dd.find('publish_odom_tf').text == 'false'

    def test_imu_plugin_present(self, urdf_xml):
        plugins = _get_plugins(urdf_xml)
        imu = [p for p in plugins if 'Imu' in (p.get('name') or '')]
        assert len(imu) >= 1

    def test_joint_state_publisher_plugin(self, urdf_xml):
        plugins = _get_plugins(urdf_xml)
        jsp = [p for p in plugins if 'JointStatePublisher' in (p.get('name') or '')]
        assert len(jsp) == 1

    def test_lidar_sensor_present(self, urdf_xml):
        sensors = _get_sensors(urdf_xml)
        lidar = [s for s in sensors if s.get('type') in ('gpu_lidar', 'ray')]
        assert len(lidar) >= 1

    def test_camera_sensor_present(self, urdf_xml):
        sensors = _get_sensors(urdf_xml)
        cam = [s for s in sensors if s.get('type') == 'camera']
        assert len(cam) >= 1
