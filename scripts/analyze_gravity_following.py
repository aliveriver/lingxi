"""Offline comparison of measured pose changes and model static gravity changes.

Does not use a later IMU sample as if it were contemporaneous with a trial.
"""
import argparse
import json
from pathlib import Path

from lingxi_x2.acceptance_report import evaluate_file
from lingxi_x2.gravity import StaticArmModel
from lingxi_x2.gravity_following import gravity_difference_bound


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('traces',type=Path,nargs='+')
    parser.add_argument('--urdf',type=Path,required=True)
    parser.add_argument('--assumed-kp',type=float,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    import math
    if not math.isfinite(args.assumed_kp) or args.assumed_kp<=0:parser.error('kp must be finite and positive')
    model=StaticArmModel(args.urdf)
    report={'kind':'offline_gravity_following_comparison','model_sha256':model.sha256,
            'assumed_kp_nm_rad':args.assumed_kp,'gravity_magnitude_m_s2':9.81,
            'payload_assumption':'original model inertials, no additional payload; distal composition unresolved',
            'command_output':False,'hardware_validated':False,'sessions':[],
            'limitations':['Assumes static equilibrium, unchanged torso gravity direction and linear position stiffness.',
                           'IMU was not captured in these motion traces; unchanged direction is NOT verified.',
                           'Does not model friction, damping/acceleration, contacts, drive internals or unmodeled loads.',
                           'Encoder medians from separate joints are not guaranteed simultaneous.',
                           'A discrepancy only challenges this restricted gravity-only explanation; it does not identify a fault.',
                           'Required torque change is an algebraic diagnostic, not calibrated torque or a command.']}
    for path in args.traces:
        acceptance=evaluate_file(path)
        if not acceptance['quality_checks'] or not all(c['passed'] for c in acceptance['quality_checks']):
            raise ValueError(f'{path}: trace quality does not support comparison')
        index=acceptance['joint_index'];side='left' if index<7 else 'right';local=index%7;start=0 if side=='left' else 7
        windows=acceptance['windows'];base=windows['baseline_hold'];target=windows['target_hold']
        q0=base['arm_state']['position_median_rad'];q1=target['arm_state']['position_median_rad']
        bound=gravity_difference_bound(model,side,q0[start:start+7],q1[start:start+7],
                                      gravity_magnitude_m_s2=9.81,payload_mass_kg=0.,payload_com_wrist_m=[0.,0.,0.])
        command_delta=target['hal_arm']['position_median_rad'][index]-base['hal_arm']['position_median_rad'][index]
        encoder_delta=q1[index]-q0[index]
        required=args.assumed_kp*(command_delta-encoder_delta)
        maximum=bound['max_abs_delta_gravity_torque_nm'][local]
        report['sessions'].append({'source_sha256':acceptance['source_sha256'],'source_path':str(path),
          'joint_name':acceptance['joint_name'],'command_delta_rad':command_delta,'encoder_delta_rad':encoder_delta,
          'proportional_term_change_nm':required,'fixed_direction_model_gravity_change_bound_nm':maximum,
          'proportional_change_exceeds_model_bound':abs(required)>maximum,
          'excess_magnitude_nm':max(0.,abs(required)-maximum),'full_arm_difference_bound':bound})
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open('x') as stream:json.dump(report,stream,indent=2,allow_nan=False)
    print(json.dumps(report,indent=2,allow_nan=False))


if __name__=='__main__':main()
