"""Resolve a robot variant while retaining the task, object and scoring contract."""
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parents[2]
ROBOTS=('allegro','wuji','sharpa')
TASKS=('G01','G02','G03','G04','G05')


def load_profile(task_id, robot='allegro'):
    if robot not in ROBOTS or task_id not in TASKS:
        raise ValueError('Unknown robot or local grasp task')
    profile=json.loads((ROOT/'profiles/local_grasp'/f'{task_id}.json').read_text())
    if robot=='allegro':return profile
    robot_file=f'profiles/robots/dual_x5_{robot}.json'
    config=json.loads((ROOT/robot_file).read_text())
    profile.update(robot_config=robot_file,robot_template=config['template'],
        embodiment='Dual X5 arms with '+config['label']+'; virtual simulation assembly',
        scene=f'assets/generated/{robot}_grasp/{task_id}/scene.xml',
        output=f'runs/{robot}-grasp/{task_id}',limitations=config['limitations'])
    return profile
