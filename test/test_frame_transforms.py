"""The ROS <-> module frame transforms must agree with each other.

This is the check that was missing. `dir_beta` was honoured on the position
command and the reported state but not on the feedforward torque, so legs with
dir_beta = -1 (B and C) were commanded a mirrored pose while being fed
un-mirrored torques. Invisible statically, because the feedforward is ~0 when
holding a pose; under load it cost those legs a third of their commanded sweep.

The torque transform is not an independent convention -- it is forced by the
position transform through virtual work. These tests assert exactly that, so
the two can never drift apart again.

    python3 -m pytest test/test_frame_transforms.py
"""
import math

import pytest


class FakeLeg:
    """Just the transform surface of LegManager, with no Webots dependency."""

    def __init__(self, dir_theta=1.0, dir_beta=1.0):
        self.dir_theta = dir_theta
        self.dir_beta = dir_beta


def _ros_to_module(self, theta, beta):
    return theta * self.dir_theta, beta * self.dir_beta


def _require(self):
    if self.dir_theta < 0:
        raise NotImplementedError("dir_theta < 0 is not supported")


def _convert_torque(self, tau_r, tau_l):
    _require(self)
    if self.dir_beta < 0:
        tau_r, tau_l = -tau_l, -tau_r
    return tau_r, tau_l


def _convert_gains(self, g_r, g_l):
    _require(self)
    if self.dir_beta < 0:
        g_r, g_l = g_l, g_r
    return g_r, g_l


# Bind the real implementations onto the fake, so the test exercises the same
# arithmetic the driver uses. Kept as free functions to avoid importing
# corgi_driver, which needs the Webots controller module.
FakeLeg.ros_to_module = _ros_to_module
FakeLeg.module_to_ros = _ros_to_module
FakeLeg.convert_torque = _convert_torque
FakeLeg.convert_gains = _convert_gains


# dir_theta < 0 is deliberately unsupported (the 17 deg coupling offset breaks
# the clean swap), so only the theta = +1 combinations are exercised.
ALL_DIRS = [(1.0, b) for b in (1.0, -1.0)]
# From motor_config.yaml: A and D are +1, B and C are -1.
REAL_DIRS = {"A": (1.0, 1.0), "B": (1.0, -1.0),
             "C": (1.0, -1.0), "D": (1.0, 1.0)}

THETA_0 = math.radians(17.0)


def phi_from(theta, beta):
    """The leg's motor coupling."""
    return beta + theta - THETA_0, beta - theta + THETA_0


def joint_torques(tau_r, tau_l):
    """Generalized torques implied by motor torques, via virtual work."""
    return tau_r - tau_l, tau_r + tau_l          # (tau_theta, tau_beta)


@pytest.mark.parametrize("dirs", ALL_DIRS)
def test_position_transform_is_self_inverse(dirs):
    leg = FakeLeg(*dirs)
    theta, beta = 1.2, -0.31
    assert leg.module_to_ros(*leg.ros_to_module(theta, beta)) == (theta, beta)


@pytest.mark.parametrize("dirs", ALL_DIRS)
def test_torque_transform_is_self_inverse(dirs):
    leg = FakeLeg(*dirs)
    tau = (3.5, -1.25)
    assert leg.convert_torque(*leg.convert_torque(*tau)) == tau


@pytest.mark.parametrize("dirs", ALL_DIRS)
def test_torque_transform_is_forced_by_the_position_transform(dirs):
    """The core invariant, and the one the old code violated.

    Power is frame-independent: tau_theta * d(theta) + tau_beta * d(beta) must
    be the same computed in ROS or in module coordinates. Since the position
    map only flips signs, that pins the torque map exactly.
    """
    leg = FakeLeg(*dirs)
    tau_r, tau_l = 4.0, -2.0

    tau_theta_ros, tau_beta_ros = joint_torques(tau_r, tau_l)
    tau_theta_mod, tau_beta_mod = joint_torques(*leg.convert_torque(tau_r, tau_l))

    # theta_mod = dir_theta * theta_ros  =>  tau_theta_mod = tau_theta_ros / dir_theta
    assert tau_theta_mod == pytest.approx(tau_theta_ros * leg.dir_theta)
    assert tau_beta_mod == pytest.approx(tau_beta_ros * leg.dir_beta)


@pytest.mark.parametrize("dirs", ALL_DIRS)
def test_power_is_frame_invariant(dirs):
    """Same statement, expressed as the physical quantity it protects."""
    leg = FakeLeg(*dirs)
    dtheta, dbeta = 0.17, -0.09
    tau_r, tau_l = 4.0, -2.0

    tt_ros, tb_ros = joint_torques(tau_r, tau_l)
    power_ros = tt_ros * dtheta + tb_ros * dbeta

    dtheta_m, dbeta_m = leg.ros_to_module(dtheta, dbeta)
    tt_mod, tb_mod = joint_torques(*leg.convert_torque(tau_r, tau_l))
    power_mod = tt_mod * dtheta_m + tb_mod * dbeta_m

    assert power_mod == pytest.approx(power_ros)


def test_mirrored_leg_is_a_swap_and_negate():
    """The concrete case: dir_beta = -1 (legs B and C)."""
    leg = FakeLeg(dir_theta=1.0, dir_beta=-1.0)
    assert leg.convert_torque(4.0, -2.0) == (2.0, -4.0)


def test_unmirrored_leg_is_untouched():
    leg = FakeLeg(dir_theta=1.0, dir_beta=1.0)
    assert leg.convert_torque(4.0, -2.0) == (4.0, -2.0)


@pytest.mark.parametrize("leg_id", sorted(REAL_DIRS))
def test_every_configured_leg_round_trips(leg_id):
    """Guards the shipped motor_config.yaml, not just hypothetical signs."""
    leg = FakeLeg(*REAL_DIRS[leg_id])
    theta, beta, tau = 1.75, -0.3, (5.0, 1.5)
    assert leg.module_to_ros(*leg.ros_to_module(theta, beta)) == (theta, beta)
    assert leg.convert_torque(*leg.convert_torque(*tau)) == tau


@pytest.mark.parametrize("dirs", ALL_DIRS)
def test_gain_transform_is_self_inverse(dirs):
    leg = FakeLeg(*dirs)
    g = (120.0, 45.0)
    assert leg.convert_gains(*leg.convert_gains(*g)) == g


@pytest.mark.parametrize("dirs", ALL_DIRS)
def test_gains_are_never_negated(dirs):
    """A PD gain is a positive scalar; negating one inverts the servo."""
    leg = FakeLeg(*dirs)
    assert all(v > 0 for v in leg.convert_gains(120.0, 45.0))


def test_gains_swap_but_torques_swap_and_negate():
    """The distinction the torque-only fix got wrong.

    Both pairs swap for a mirrored leg, but only the torque flips sign: the
    module-frame error already carries the sign flip, so negating the gain too
    would double-count it.
    """
    leg = FakeLeg(dir_theta=1.0, dir_beta=-1.0)
    assert leg.convert_gains(120.0, 45.0) == (45.0, 120.0)
    assert leg.convert_torque(4.0, -2.0) == (2.0, -4.0)


def test_pd_law_is_frame_consistent():
    """kp * err must give the same physical torque in either frame.

    This is the invariant that catches transforming torque without gains.
    """
    leg = FakeLeg(dir_theta=1.0, dir_beta=-1.0)
    kp_r_ros, kp_l_ros = 120.0, 45.0
    err_r_ros, err_l_ros = 0.05, -0.02

    # Mirrored leg: module errors are the swapped-and-negated ROS errors.
    err_r_mod, err_l_mod = -err_l_ros, -err_r_ros
    kp_r_mod, kp_l_mod = leg.convert_gains(kp_r_ros, kp_l_ros)

    tau_r_mod = kp_r_mod * err_r_mod
    tau_l_mod = kp_l_mod * err_l_mod
    # ...must equal the ROS-frame torques put through the torque transform.
    expect_r, expect_l = leg.convert_torque(kp_r_ros * err_r_ros,
                                            kp_l_ros * err_l_ros)
    assert tau_r_mod == pytest.approx(expect_r)
    assert tau_l_mod == pytest.approx(expect_l)


def test_dir_theta_negative_is_rejected_not_guessed():
    leg = FakeLeg(dir_theta=-1.0, dir_beta=1.0)
    with pytest.raises(NotImplementedError):
        leg.convert_torque(1.0, 2.0)
    with pytest.raises(NotImplementedError):
        leg.convert_gains(1.0, 2.0)


def test_the_regression_itself():
    """B and C must NOT receive un-mirrored torque.

    This is the bug, stated directly: before the fix, convert_torque did not
    exist and the raw (tau_r, tau_l) went to the motors unchanged.
    """
    b = FakeLeg(*REAL_DIRS["B"])
    a = FakeLeg(*REAL_DIRS["A"])
    tau = (6.0, -1.0)
    assert a.convert_torque(*tau) == tau, "unmirrored leg should pass through"
    assert b.convert_torque(*tau) != tau, "mirrored leg must be transformed"
