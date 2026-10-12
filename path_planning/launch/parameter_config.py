"""Shared launch configuration: explicit arguments, YAML node profiles, then defaults."""
import sys

import yaml

# Planner wrapper parameters without the search/cloud/output namespace prefixes.
PLANNER_KEYS = {
    'planning_frame', 'planning_rate', 'replan_interval', 'replan_distance',
    'input_timeout', 'stamp_clock', 'odometry_twist_in_body', 'sample_step', 'service_only',
}
# Integer-valued ROS parameters must not be coerced to doubles.
INTEGER_KEYS = {
    'search.allocate_num', 'search.tree_period', 'search.max_cloud_points', 'output.max_samples',
}
# Preserve the existing short launch arguments while also accepting ROS parameter names.
ALIASES = {'blind_radius': 'cloud.blind_radius', 'map_mode': 'cloud.map_mode',
           'search_lower': 'search.lower'}


def load_config(path):
    """Read YAML once; unreadable or malformed files warn and use launch/node defaults."""
    try:
        with open(path, encoding='utf-8') as stream:
            data = yaml.safe_load(stream)
        if not isinstance(data, dict):
            raise ValueError('expected a mapping of node names')
        return data
    except (OSError, ValueError, yaml.YAMLError) as error:
        print(f'[planner config] Cannot read {path}: {error}; using defaults.', file=sys.stderr)
        return {}


def flatten(parameters, prefix=''):
    """Accept both ROS nested parameter namespaces and dotted parameter keys."""
    result = {}
    for key, value in parameters.items():
        name = f'{prefix}{key}'
        if isinstance(value, dict):
            result.update(flatten(value, name + '.'))
        else:
            result[name] = value
    return result


def node_parameters(data, names, defaults=None):
    """Merge common and node-specific YAML sections in increasing priority order."""
    result = dict(defaults or {})
    for name in names:
        if name not in data:
            continue
        section = data[name]
        parameters = section.get('ros__parameters') if isinstance(section, dict) else None
        if not isinstance(parameters, dict):
            print(f'[planner config] Invalid {name}.ros__parameters; using defaults for this section.',
                  file=sys.stderr)
            continue
        result.update(flatten(parameters))
    return result


def planner_overrides(context, parameters):
    """Apply explicit planner launch arguments after YAML, preserving ROS parameter types."""
    result = dict(parameters)
    arguments = context.launch_configurations
    for alias, name in ALIASES.items():
        if arguments.get(alias) and arguments.get(name):
            raise ValueError(f'Supply only one of {alias} and {name}')
    for key, raw in arguments.items():
        name = ALIASES.get(key, key)
        if not raw or not (name in PLANNER_KEYS or name.startswith(('search.', 'cloud.', 'output.'))):
            continue
        value = yaml.safe_load(raw)
        if name in ('planning_frame', 'stamp_clock', 'cloud.map_mode'):
            value = raw
        elif name in ('service_only', 'odometry_twist_in_body'):
            if not isinstance(value, bool):
                raise ValueError(f'{key} must be true or false')
        elif name in INTEGER_KEYS:
            if isinstance(value, bool) or not isinstance(value, int):
                raise ValueError(f'{key} must be an integer')
        elif isinstance(value, list):
            value = [float(item) for item in value]
        else:
            value = float(value)
        result[name] = value
    return result


def launch_value(context, name, configured):
    """Resolve a topic/mode launch argument; an omitted argument retains its YAML value."""
    return context.launch_configurations.get(name) or configured
