"""
Integration test for the launch file — verifies it generates a valid
LaunchDescription and all referenced config files exist.
"""
import os
import pytest
from unittest import mock

PKG_DIR = os.path.join(os.path.dirname(__file__), '..')
LAUNCH_FILE = os.path.join(PKG_DIR, 'launch', 'start_simulation.launch.py')


class TestLaunchFile:
    @pytest.fixture(scope='class')
    def launch_desc(self):
        """Import and call generate_launch_description()."""
        import importlib.util
        # Capture the real function BEFORE patching
        from ament_index_python.packages import get_package_share_directory as _real

        def _patched(pkg):
            if pkg == 'lawn_mower_bot':
                return os.path.abspath(PKG_DIR)
            return _real(pkg)

        with mock.patch(
            'ament_index_python.packages.get_package_share_directory',
            side_effect=_patched,
        ):
            spec = importlib.util.spec_from_file_location("launch_module", LAUNCH_FILE)
            mod = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(mod)
            return mod.generate_launch_description()

    def test_returns_launch_description(self, launch_desc):
        from launch import LaunchDescription
        assert isinstance(launch_desc, LaunchDescription)

    def test_has_entities(self, launch_desc):
        """Should have a good number of entities (nodes, timers, includes)."""
        entities = launch_desc.entities
        assert len(entities) >= 10, f"Only {len(entities)} entities found"

    def test_config_files_exist(self):
        """All config files referenced in the launch should exist."""
        config_files = [
            os.path.join(PKG_DIR, 'config', 'nav2_params.yaml'),
            os.path.join(PKG_DIR, 'config', 'ekf.yaml'),
            os.path.join(PKG_DIR, 'config', 'slam_params.yaml'),
            os.path.join(PKG_DIR, 'urdf', 'mower.urdf.xacro'),
            os.path.join(PKG_DIR, 'worlds', 'hard_lawn.world'),
            os.path.join(PKG_DIR, 'rviz', 'mower.rviz'),
        ]
        for f in config_files:
            assert os.path.isfile(f), f"Missing file: {f}"

    def test_python_scripts_are_executable(self):
        """All installed Python scripts should have correct shebangs."""
        scripts = [
            'mow_lawn.py', 'simple_move.py', 'vision_processor.py',
            'gemini_planner.py', 'gemini_mow_planner.py',
            'gemini_mow_executor.py', 'coverage_planner.py',
        ]
        for script in scripts:
            path = os.path.join(PKG_DIR, script)
            assert os.path.isfile(path), f"Missing script: {script}"
            with open(path) as f:
                first_line = f.readline()
            assert '#!/usr/bin/env python3' in first_line, \
                f"{script} missing shebang"
