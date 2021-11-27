#!/usr/bin/env python3
from cereal import car
from selfdrive.config import Conversions as CV
from selfdrive.car.toyota.values import Ecu, CAR, TSS2_CAR, NO_DSU_CAR, MIN_ACC_SPEED, PEDAL_HYST_GAP, CarControllerParams
from selfdrive.car import STD_CARGO_KG, scale_rot_inertia, scale_tire_stiffness, gen_empty_fingerprint, get_safety_config
from selfdrive.car.interfaces import CarInterfaceBase

EventName = car.CarEvent.EventName


class CarInterface(CarInterfaceBase):
  @staticmethod
  def get_pid_accel_limits(CP, current_speed, cruise_speed):
    return CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX

  @staticmethod
  def get_params(candidate, fingerprint=gen_empty_fingerprint(), car_fw=[]):  # pylint: disable=dangerous-default-value
    ret = CarInterfaceBase.get_std_params(candidate, fingerprint)

    ret.carName = "toyota"
    ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.toyota)]

    ret.steerActuatorDelay = 0.12  # Default delay, Prius has larger delay
    ret.steerLimitTimer = 0.4

    ret.stoppingControl = False # Toyota starts braking more when it thinks you want to stop

    if candidate not in [CAR.PRIUS, CAR.RAV4, CAR.RAV4H]:  # These cars use LQR/INDI
      ret.lateralTuning.init('pid')
      ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[0.], [0.]]

    if candidate == CAR.NINE4x4:
      # steerRatioRear @21 :Float32;  # [] ratio of steering wheel angle to rear wheel angle (usually 0)
      stop_and_go = True
      ret.safetyConfigs[0].safetyParam = 66  # see conversion factor for STEER_TORQUE_EPS in dbc file
      ret.wheelbase = 2.70      # [m] distance from rear axle to front axle
      ret.steerRatio = 15.74   # [] ratio of steering wheel angle to front wheel angle
      tire_stiffness_factor = 0.6371   # hand-tune
      ret.mass = 3045. * CV.LB_TO_KG + STD_CARGO_KG       # [kg] curb weight: all fluids no cargo

      ret.lateralTuning.init('indi')
      ret.lateralTuning.indi.innerLoopGainBP = [0.]
      ret.lateralTuning.indi.innerLoopGainV = [4.0]
      ret.lateralTuning.indi.outerLoopGainBP = [0.]
      ret.lateralTuning.indi.outerLoopGainV = [3.0]
      ret.lateralTuning.indi.timeConstantBP = [0.]
      ret.lateralTuning.indi.timeConstantV = [1.0]
      ret.lateralTuning.indi.actuatorEffectivenessBP = [0.]
      ret.lateralTuning.indi.actuatorEffectivenessV = [1.0]
      ret.steerActuatorDelay = 0.3

    ret.steerRateCost = 1.
    ret.centerToFront = ret.wheelbase * 0.44  # [m] distance from center of mass to front axle

    # TODO: get actual value, for now starting with reasonable value for
    # civic and scaling by mass and wheelbase
    ret.rotationalInertia = scale_rot_inertia(ret.mass, ret.wheelbase)

    # TODO: start from empirically derived lateral slip stiffness for the civic and scale by
    # mass and CG position, so all cars will have approximately similar dyn behaviors
    ret.tireStiffnessFront, ret.tireStiffnessRear = scale_tire_stiffness(ret.mass, ret.wheelbase, ret.centerToFront,
                                                                         tire_stiffness_factor=tire_stiffness_factor)

    ret.enableBsm = 0x3F6 in fingerprint[0] and candidate in TSS2_CAR
    # Detect smartDSU, which intercepts ACC_CMD from the DSU allowing openpilot to send it
    smartDsu = 0x2FF in fingerprint[0]
    # In TSS2 cars the camera does long control
    found_ecus = [fw.ecu for fw in car_fw]
    ret.enableDsu = (len(found_ecus) > 0) and (Ecu.dsu not in found_ecus) and (candidate not in NO_DSU_CAR)
    ret.enableGasInterceptor = 0x201 in fingerprint[0]
    # if the smartDSU is detected, openpilot can send ACC_CMD (and the smartDSU will block it from the DSU) or not (the DSU is "connected")
    ret.openpilotLongitudinalControl = smartDsu or ret.enableDsu or candidate in TSS2_CAR

    # min speed to enable ACC. if car can do stop and go, then set enabling speed
    # to a negative value, so it won't matter.
    ret.minEnableSpeed = -1. if (stop_and_go or ret.enableGasInterceptor) else MIN_ACC_SPEED

    # removing the DSU disables AEB and it's considered a community maintained feature
    # intercepting the DSU is a community feature since it requires unofficial hardware
    ret.communityFeature = ret.enableGasInterceptor or ret.enableDsu or smartDsu

    if ret.enableGasInterceptor:
      ret.longitudinalTuning.kpBP = [0., 5., MIN_ACC_SPEED, MIN_ACC_SPEED + PEDAL_HYST_GAP, 35.]
      ret.longitudinalTuning.kpV = [1.2, 0.8, 0.765, 2.255, 1.5]
      ret.longitudinalTuning.kiBP = [0., MIN_ACC_SPEED, MIN_ACC_SPEED + PEDAL_HYST_GAP, 35.]
      ret.longitudinalTuning.kiV = [0.18, 0.165, 0.489, 0.36]
    elif candidate in [CAR.COROLLA_TSS2, CAR.COROLLAH_TSS2, CAR.RAV4_TSS2, CAR.RAV4H_TSS2, CAR.LEXUS_NX_TSS2]:
      # Improved longitudinal tune
      ret.longitudinalTuning.deadzoneBP = [0., 8.05]
      ret.longitudinalTuning.deadzoneV = [.0, .14]
      ret.longitudinalTuning.kpBP = [0., 5., 20.]
      ret.longitudinalTuning.kpV = [1.3, 1.0, 0.7]
      ret.longitudinalTuning.kiBP = [0., 5., 12., 20., 27.]
      ret.longitudinalTuning.kiV = [.35, .23, .20, .17, .1]
      ret.stoppingDecelRate = 0.3  # reach stopping target smoothly
      ret.startingAccelRate = 6.0  # release brakes fast
    else:
      # Default longitudinal tune
      ret.longitudinalTuning.deadzoneBP = [0., 9.]
      ret.longitudinalTuning.deadzoneV = [0., .15]
      ret.longitudinalTuning.kpBP = [0., 5., 35.]
      ret.longitudinalTuning.kiBP = [0., 35.]
      ret.longitudinalTuning.kpV = [3.6, 2.4, 1.5]
      ret.longitudinalTuning.kiV = [0.54, 0.36]

    return ret

  # returns a car.CarState
  def update(self, c, can_strings):
    # ******************* do can recv *******************
    self.cp.update_strings(can_strings)
    self.cp_cam.update_strings(can_strings)

    ret = self.CS.update(self.cp, self.cp_cam)

    # ret.canValid = self.cp.can_valid and self.cp_cam.can_valid
    ret.canValid = True
    ret.steeringRateLimited = self.CC.steer_rate_limited if self.CC is not None else False

    # events
    events = self.create_common_events(ret)

    if self.CS.low_speed_lockout and self.CP.openpilotLongitudinalControl:
      events.add(EventName.lowSpeedLockout)
    if ret.vEgo < self.CP.minEnableSpeed and self.CP.openpilotLongitudinalControl:
      events.add(EventName.belowEngageSpeed)
      if c.actuators.accel > 0.3:
        # some margin on the actuator to not false trigger cancellation while stopping
        events.add(EventName.speedTooLow)
      if ret.vEgo < 0.001:
        # while in standstill, send a user alert
        events.add(EventName.manualRestart)

    ret.events = events.to_msg()

    self.CS.out = ret.as_reader()
    return self.CS.out

  # pass in a car.CarControl
  # to be called @ 100hz
  def apply(self, c):

    can_sends = self.CC.update(c.enabled, self.CS, self.frame,
                               c.actuators, c.cruiseControl.cancel,
                               c.hudControl.visualAlert, c.hudControl.leftLaneVisible,
                               c.hudControl.rightLaneVisible, c.hudControl.leadVisible,
                               c.hudControl.leftLaneDepart, c.hudControl.rightLaneDepart)

    self.frame += 1
    return can_sends
