''' Module provides functions for robot object initialization '''

import numpy as np
from robCRSikt import robCRSikt
from robCRSgripper import robCRSgripper, robCRSgripperinit

#  based on BlueBot and Bosch Toolbox
#  by  O. Certik, V. Smutny, P. Krsek, M. Matousek

class Robot:
    """Robot specification: CRS97"""

    def __init__(self):
        self.description = 'CRS 1997 with gripper'

        # Base position in IRC - calibrated !!
        # (corresponds to position self.hhdeg in degrees)
        self.hhirc = [-182500, 252, -63625, 99200, 14300, -98150]

        # self bounds in IRC
        self.bound = [[-370000, -100000, -190000, -5000, -44000, -199500],
                      [10000, 100000, 65000, 202000, 72000, 4000]]

        # Speed reduced due to power suply problem
        self.defaultspeed = np.array([30, 8, 20, 30, 30, 55]) * 256
        self.defaultacceleration = np.rint(
            np.array([(30 / 400), (8 / 400), (20 / 400), (30 / 400), 1 / 2, 3 / 5]) * 256)

        # Gripper range
        self.gripper_bounds = [1001, 48]
        self.gripper_bounds_force = [1001 + 100, 48 - 1000]  # TODO must be verified

        # Axis(motor) of gripper
        self.gripper_ax = 'H'
        self.verbose = True
        # ------------------------------------------------------------------------------
        # Common parameters for CRS selfs
        self = robCRScommon(self)


def robCRScommon(robot):
    """Common specification for CRS robots."""

    robot.portname = 'COM4'
    robot.controller = 'MARS8'
    robot.demoname = 'robCRSdemo'

    robot.DOF = 6

    # Robot link dimensions (these constants are read from documentation)
    inch2mm = 25.4  # conversion constant
    robot.L1 = 13 * inch2mm - 25  # J2 above working desk
    robot.L2 = 12 * inch2mm  # J2-J3
    robot.L3 = 13 * inch2mm  # J3-J5
    robot.L4 = 3 * inch2mm  # J5-flange
    robot.L5 = inch2mm * (3.78 + 0.5)  # gripper length
    robot.L6 = inch2mm * 1  # gripper finger length

    # Posible change of coordinate direction
    robot.direction = np.array([1, 1, -1, -1, -1, -1])

    # IRC per rotation of motor.
    robot.irc = np.array([1000, 1000, 1000, 500, 500, 500])  # from manual, it should work
    # motors' gearing
    robot.gearing = np.array([100, 100, 100, 101, 100, 101])  # from manual
    # degtoirc=1 degree in IRC
    robot.degtoirc = robot.irc * robot.direction * robot.gearing * 4 / 360

    robot.activemotors = 'ABCDEF'
    robot.hh_axes_list = 'ABCDEF'
    robot.control_axes_list = 'ABCDEF'
    robot.coord_axes = 'ABCDEF'

    # DH notation
    robot.offset = np.array([0.0, 270.0, 90.0, 0.0, 0.0, 0.0]) / 180 * np.pi
    robot.sign = np.array([1, 1, 1, 1, 1, 1])
    robot.d = [robot.L1, 0, 0, robot.L3, 0, robot.L4 + robot.L5 + robot.L6]
    robot.a = [0, robot.L2, 0, 0, 0, 0]
    robot.alpha = np.array([90.0, 0.0, 270.0, 90.0, 270.0, 0]) / 180 * np.pi
    robot.base = np.eye(4)
    robot.tool = np.array([[0, 0, -1, 0],
                          [0, 1, 0, 0],
                          [1, 0, 0, 0],
                          [0, 0, 0, 1]])

    # Base position in degrees
    # (corresponds to position robot.hhirc in IRC)
    robot.hhdeg = [0, 0, 0, 0, 0, 0]

    # Softhome position in degrees
    #  - Robot is moved to this position during startup and closing.
    #  - The position is set by calling bbsofthome
    robot.shdeg = [0, 0, -45, 0, -45, 0]

    robot.REGME = [32000, 32000, 32000, 32000, 32000, 32000]

    # Constants of PID regulators
    robot.REGP = [10, 12, 70, 35, 45, 100]
    robot.REGI = [80, 63, 50, 80, 65, 300]
    robot.REGD = [300, 200, 200, 130, 230, 350]

    # min and max speeds (IRC/256/msec)
    robot.minspeed = np.rint(robot.defaultspeed / 5)
    robot.maxspeed = np.rint(robot.defaultspeed * 2)

    # min a max acceleration
    robot.minacceleration = np.rint(robot.defaultacceleration / 5)
    robot.maxacceleration = np.rint(robot.defaultacceleration * 2)

    # smutneho hodnoty robot.REGCFG = [1370 1369 1369 1361 1369 1369]
    # D ma HW problem, protoze indexova znacka se casto nachazi o
    # (mnoho)otacek jinde
    # E nema indexy nikdy,
    # problem D a E identifikovan: V rozporu s dokumentaci ma HEDS na D a E
    # signal znacky v podmnozine kombinace kanalu AB=00, ackoliv by mel jit az
    # do hrany A mezi 0 a 1 (ci naopak), pricemz B ma byt nula. Cip, ktery
    # zpracovava signaly to jinak neumi.
    # Potencialne to lze resit nejakym RC clankem mezi HEDS a budicem v robotu,
    # daji se ocekavat problemy s ruznymi rychlostmi. Od tohoto reseni je
    # upusteno.
    # V teto verzi tedy D a E najizdi jen na MARK, nikoliv na indexovou znacku.
    # robot.REGCFG = [1494 1494 1494 1484 1476 1492]# D melo 1498,
    # robot.REGCFG = [1492 1494 1494 1484 1476 1492]# D melo 1498,

    # Uprava rychlosti HOME
    # robot.REGCFG = [1490 1491 1491 1482 1474 1490]# D melo 1498,

    # Speed reduced due to power suply problem
    robot.REGCFG = [1489, 1490, 1490, 1481, 1474, 1490]  # D melo 1498,

    # CRS robot shall not release after some time
    robot.IDLEREL = 1200

    # homing sequence TODO
    # robot.hardhome = robCRShardhome


    robot.timeout = 200

    # Gripper parameters

    robot.gripper = robCRSgripper
    robot.gripper_init = robCRSgripperinit


    # Kind of gripper (CRSGripper, Magnetic)
    robot.gripper_type = 'CRSGripper'

    # Analog input of position sensor
    robot.gripper_ADC = 10
    # Maximal curent limit (0-255)
    # Value 16 corresponds to 250mA when feedback is 500
    robot.gripper_current = 16
    # Limitation constant (feeadback from overcurrent)
    robot.gripper_feedback = 500
    # Maximal energy limits voltage on motor
    # (0 - 32000 corresponds to 0-24V)
    # 2010-02-17 Martin Matousek note:
    # Manual says that gripper can survive up to 15 Volts, but
    # force more than 75# must not be used longer then 15 second. Thus
    # safe power seems to be 11.25 V. We are using more conservative value here
    # 10/32 * 24 = 7.5 V

    robot.gripper_REGME = 10000
    # Maximal speed
    robot.gripper_REGMS = 500
    # Axis configuration word
    robot.gripper_REGCFG = 256
    # PID parameter of controller
    robot.gripper_REGP = 200
    robot.gripper_REGI = 0
    robot.gripper_REGD = 100

    robot.gripper_poll_time = 0.2
    robot.gripper_poll_diff = 50

    robot.ikt = robCRSikt

    robot.degtoirc = np.multiply(robot.direction, np.multiply(robot.irc, robot.gearing)) * 4.0 / 360.0

    robot.anglestoirc = lambda angles: np.multiply((angles - robot.hhdeg), robot.degtoirc) + robot.hhirc
    robot.irctoangles = lambda irc: np.divide(irc - robot.hhirc, robot.degtoirc) + robot.hhdeg

    degtorad = lambda d: d * np.pi / 180.0
    radtodeg = lambda d: d * 180.0 / np.pi

# if __name__ == '__main__':
#     robot = Robot()
#     print robot.ikt(robot, [600, -200.0, 500.0, 0.0, 0.0, 0.0])