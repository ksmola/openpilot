# flake8: noqa

from cereal import car
from selfdrive.car import dbc_dict
from selfdrive.config import Conversions as CV

Ecu = car.CarParams.Ecu
MIN_ACC_SPEED = 19. * CV.MPH_TO_MS

PEDAL_HYST_GAP = 3. * CV.MPH_TO_MS
PEDAL_SCALE = 3.0

class CarControllerParams:
  ACCEL_HYST_GAP = 0.06  # don't change accel command for small oscilalitons within this value
  ACCEL_MAX = 1.5  # m/s2, lower than allowed 2.0 m/s2 for tuning reasons
  ACCEL_MIN = -3.5  # m/s2

  STEER_MAX = 1500
  STEER_DELTA_UP = 10       # 1.5s time to peak torque
  STEER_DELTA_DOWN = 25     # always lower than 45 otherwise the Rav4 faults (Prius seems ok with 50)
  STEER_ERROR_MAX = 350     # max delta between torque cmd and torque motor

class CAR:
  # Porsche
  NINE4x4 = "PORSCHE 94x4"

# (addr, cars, bus, 1/freq*100, vl)
STATIC_DSU_MSGS = [
  (0x128, (CAR.NINE4x4), 1,   3, b'\xf4\x01\x90\x83\x00\x37'),
  (0x141, (CAR.NINE4x4), 1,   2, b'\x00\x00\x00\x46'),
  (0x160, (CAR.NINE4x4), 1,   7, b'\x00\x00\x08\x12\x01\x31\x9c\x51'),
  (0x161, (CAR.NINE4x4), 1,   7, b'\x00\x1e\x00\x00\x00\x80\x07'),
  (0x283, (CAR.NINE4x4), 0,   3, b'\x00\x00\x00\x00\x00\x00\x8c'),
  (0x2E6, (CAR.NINE4x4), 0,   3, b'\xff\xf8\x00\x08\x7f\xe0\x00\x4e'),
  (0x2E7, (CAR.NINE4x4), 0,   3, b'\xa8\x9c\x31\x9c\x00\x00\x00\x02'),
  (0x33E, (CAR.NINE4x4), 0,  20, b'\x0f\xff\x26\x40\x00\x1f\x00'),
  (0x344, (CAR.NINE4x4), 0,   5, b'\x00\x00\x01\x00\x00\x00\x00\x50'),
  (0x365, (CAR.NINE4x4), 0,  20, b'\x00\x00\x00\x80\x03\x00\x08'),
  (0x366, (CAR.NINE4x4), 0,  20, b'\x00\x00\x4d\x82\x40\x02\x00'),
  (0x470, (CAR.NINE4x4), 1, 100, b'\x00\x00\x02\x7a'),
  (0x4CB, (CAR.NINE4x4), 0, 100, b'\x0c\x00\x00\x00\x00\x00\x00\x00'),
]


FINGERPRINTS = {
  CAR.NINE4x4: [{
    110:8, 120: 8, 240:8, 944: 8
  }]
}


FW_VERSIONS = {
  CAR.NINE4x4: {
    (Ecu.engine, 0x7e0, None): [b'\x0235883000\x00\x00\x00\x00\x00\x00\x00\x00A0202000\x00\x00\x00\x00\x00\x00\x00\x00',],
    (Ecu.eps, 0x7a1, None): [b'8965B58040\x00\x00\x00\x00\x00\x00',],
    (Ecu.fwdRadar, 0x750, 0xf): [b'\x018821F3301400\x00\x00\x00\x00',],
    (Ecu.fwdCamera, 0x750, 0x6d): [b'\x028646F5803200\x00\x00\x00\x008646G2601400\x00\x00\x00\x00',],
  }
}

STEER_THRESHOLD = 100

DBC = {
  CAR.NINE4x4: dbc_dict("porsche_94x4", "porsche_adas")
}


TSS2_CAR = set([CAR.NINE4x4])

# no resume button press required
NO_STOP_TIMER_CAR = TSS2_CAR