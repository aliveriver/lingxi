import pytest
from lingxi_x2.safety import ARM_LIMITS_RAD, validate_arm_target
from lingxi_x2.errors import SafetyInterlockError


def test_mirrored_roll_limits_and_installed_model_intersection():
    assert ARM_LIMITS_RAD[1] == (-.061, 2.993)
    assert ARM_LIMITS_RAD[8] == (-2.993, .061)
    assert ARM_LIMITS_RAD[6] == (-1.5097, .724)
    assert ARM_LIMITS_RAD[13] == (-.724, 1.5097)
    target = [0.] * 14
    target[1], target[8], target[6], target[13] = .2, -.2, -1., 1.
    assert validate_arm_target(target) == tuple(target)
    for index, value in ((8,.1), (13,-1.), (1,3.), (8,-3.)):
        bad = target.copy();bad[index] = value
        with pytest.raises(SafetyInterlockError): validate_arm_target(bad)


@pytest.mark.parametrize('index', range(14))
def test_all_axis_bounds(index):
    for bound, direction in zip(ARM_LIMITS_RAD[index], (-1,1)):
        target = [0.] * 14;target[index] = bound
        validate_arm_target(target)
        target[index] += direction * 1e-6
        with pytest.raises(SafetyInterlockError): validate_arm_target(target)
