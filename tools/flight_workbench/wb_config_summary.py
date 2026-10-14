"""Read-only competition YAML switch summaries; never generate or launch ROS."""
import inspect
import json
import shlex
import yaml
import hashlib


def summarize(settings):
    if not isinstance(settings, dict):
        raise ValueError('场地 YAML 必须为对象')
    result = {}
    for name, block, key, default in (
            ('motion_optimization', 'motion_optimization', 'enabled', False),
            ('resume_survey', 'survey_policy', 'resume_survey_enabled', False),
            ('obstacle_columns', None, 'obstacle_columns_enabled', None)):
        values = settings if block is None else settings.get(block, {})
        if not isinstance(values, dict):
            raise ValueError('无效配置块：' + str(block))
        value = values.get(key, default)
        if value is not None and type(value) is not bool:
            raise ValueError('配置开关必须为布尔值：' + key)
        result[name] = {'value': value, 'configured': key in values,
                        'key': (block + '.' if block else '') + key}
    geometry_keys = ('flight_bounds', 'search_center_bounds', 'target_bounds', 'coverage_bounds',
                     'staging_xy', 'survey_xy', 'corridor_waypoints', 'landing_xy', 'high_agl',
                     'landing_transit_agl', 'landing_capture_agl')
    result['geometry'] = {key: settings.get(key) for key in geometry_keys}
    schedule = settings.get('corridor_speed_schedule') or {}
    result['geometry']['corridor_speed_schedule'] = {key: schedule.get(key)
        for key in ('axis', 'wall_coordinates', 'entry_waypoints')}
    result['geometry']['_source_digest'] = hashlib.sha256(json.dumps(settings, sort_keys=True,
        separators=(',', ':'), allow_nan=False).encode()).hexdigest()
    result['site_confirmed'] = settings.get('site_confirmed') is True
    return result


def parse(text):
    return summarize(yaml.safe_load(text))


def read_command(root, path):
    # Inline the same parser so reading board values requires no tool deployment.
    script = 'import json,pathlib,sys,yaml,hashlib\n' + inspect.getsource(summarize) + '''
root=pathlib.Path(sys.argv[1]).resolve()
path=pathlib.Path(sys.argv[2])
path=(root/path).resolve()
if root not in path.parents:raise ValueError("场地 YAML 必须位于当前工程内")
if path.stat().st_size>1048576:raise ValueError("场地 YAML 过大")
print(json.dumps(summarize(yaml.safe_load(path.read_text(encoding="utf-8"))),ensure_ascii=False))
'''
    return 'python3 -c ' + shlex.quote(script) + ' ' + shlex.quote(root) + ' ' + shlex.quote(path)
