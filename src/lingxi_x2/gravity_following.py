"""Offline static-gravity difference bounds, never compensation commands."""
from __future__ import annotations

import math
import numpy as np
from .gravity import StaticArmModel


def gravity_difference_bound(model: StaticArmModel, side: str, baseline_q, target_q, *,
                             gravity_magnitude_m_s2: float, payload_mass_kg: float,
                             payload_com_wrist_m) -> dict:
    """Maximum |delta gravity torque| for any ONE fixed gravity direction.

    Static gravity torque is linear in g. Evaluate its three coefficients at
    both measured postures, then Cauchy-Schwarz gives an exact per-axis bound.
    This is not a bound on friction, contacts, acceleration, changing torso
    orientation, unmodeled mass, or uncertainty in joint/link calibration.
    Per-axis maxima generally cannot occur simultaneously.
    """
    magnitude = float(gravity_magnitude_m_s2)
    if not math.isfinite(magnitude) or not 9. <= magnitude <= 10.5:
        raise ValueError('Earth gravity magnitude must be within 9–10.5 m/s²')
    coefficients = []
    for direction in np.eye(3):
        before = model.evaluate(side, baseline_q, direction, payload_mass_kg=payload_mass_kg,
                                payload_com_wrist_m=payload_com_wrist_m)
        after = model.evaluate(side, target_q, direction, payload_mass_kg=payload_mass_kg,
                               payload_com_wrist_m=payload_com_wrist_m)
        coefficients.append(np.asarray(after['gravity_torque_nm'])-before['gravity_torque_nm'])
    coefficients = np.asarray(coefficients).T
    return {'joint_names': before['joint_names'], 'delta_torque_per_gravity_component_kg_m': coefficients.tolist(),
            'max_abs_delta_gravity_torque_nm': (magnitude*np.linalg.norm(coefficients,axis=1)).tolist(),
            'gravity_magnitude_m_s2': magnitude, 'fixed_gravity_direction_required': True,
            'payload_mass_kg': payload_mass_kg, 'payload_com_wrist_m': list(payload_com_wrist_m),
            'hardware_validated': False, 'command_output': False}
