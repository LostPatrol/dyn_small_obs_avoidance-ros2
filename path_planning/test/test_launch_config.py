"""Check the Odin-specific velocity convention without starting ROS or flight hardware."""
import importlib.util
from pathlib import Path

import pytest
import yaml
from launch import LaunchContext


@pytest.mark.parametrize('entry,yaml_override,override,expected', [
    ('odin', None, '', False), ('odin', None, 'true', True), ('odin', None, 'false', False),
    ('odin', True, '', True), ('odin', True, 'false', False), ('odin', False, 'true', True),
    ('planner', None, '', True),
])
def test_velocity_convention_precedence(monkeypatch, tmp_path, entry, yaml_override, override, expected):
    """Odin-specific YAML overrides its world default, CLI wins, and generic body twist remains."""
    package = Path(__file__).resolve().parents[1]
    spec = importlib.util.spec_from_file_location(f'{entry}_launch', package / f'launch/{entry}.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'get_package_share_directory', lambda _: str(package))
    monkeypatch.setattr(module, 'Node', lambda **kwargs: kwargs)
    config = package / 'config/planner.yaml'
    if yaml_override is not None:
        data = yaml.safe_load(config.read_text())
        data['odin_launch']['ros__parameters']['odometry_twist_in_body'] = yaml_override
        config = tmp_path / 'planner.yaml'
        config.write_text(yaml.safe_dump(data))
    context = LaunchContext()
    context.launch_configurations.update({
        'config': str(config),
        'odometry_twist_in_body': override,
    })
    node = module._launch_node(context)[0]
    assert node['parameters'][0]['odometry_twist_in_body'] is expected
    assert all(name != 'odometry_twist_in_body' for name, _ in node['remappings'])
