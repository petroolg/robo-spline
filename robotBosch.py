import numpy as np
from robotBoschikt import robotBoschikt

#  based on BlueBot and Bosch Toolbox
#  by  O. Certik, V. Smutny, P. Krsek, M. Matousek


class robotBosch:
    """Robot specification: Bosch"""

    def __init__(self):
        # Robot link dimensions (these constants are read from documentation)
        self.L1 = 250
        self.L2 = 200

        # Pavel Krsek measured 27th of September 2003
        # self.degtoirc = [947.4865, 611.9475, ? 395.9611]
        self.irc = [-1080, -1080, 1080, 1080]

        # Joint motors' gearing
        #  self.gearing = [80 50 1 (3*11)] # original numbers
        # self.gearing = [78.9572, 50.9956, 1 32.9967]
        self.gearing = [78.9572, 50.9956, 37, 32.9967]
        # TODO joint 3 must be precisely calibrated

        # 11200 (134428) = 300mm

        # ------------------------------------------------------------------------------
        # General parameters (required)
        self.description = 'Bosch SR450s'

        self.DOF = 4
        self.activemotors = 'ABCD'

        # Softhome position in IRC and degrees/mm.
        self.hhirc = [0, 0, 0, 0]
        self.hhdeg = [0, 0, 0, 0]

        # 1 degree/mm in IRC units
        self.degtoirc = np.multiply(self.irc, self.gearing) * 4.0 / 360.0

        # Robot bounds in deg/mm converted to IRC
        self.bound = bbdegtoirc(self, np.array([[85, 120, -330, -20],[-85, -120, 5, 1080]]))
        # 2010-01-29, Martin Matousek
        # this are the realy hard bounds. Its no possible to
        # to reach them with full speed.
        # robot.bound should be chosen with some reserve

        truebound = np.array([[-86200, -79000, -147000, -9000],[86900, 75400, 2500, 428000]])

        assert np.all(self.bound[0] > truebound[0]) \
               and np.all(self.bound[1] < truebound[1]), 'Internal err. in robot parameters.'


        # Speeds (IRC/256/msec)
        self.minspeed = [1000,  1000, 1000, 1000]
        self.maxspeed = [10000, 10000, 5000, 5000]
        self.defaultspeed = [10000, 10000, 5000, 5000]

        # Accelerations
        self.minacceleration = [0, 0, 1, 1]
        self.maxacceleration = [100, 100, 100, 100]
        self.defaultacceleration = [30, 30, 20, 30]

        self.controller = 'MARS8'

        # ------------------------------------------------------------------------------
        # General parameters (optional)

        self.ikt = robotBoschikt

        self.portname = 'COM2'  # corrected from COM4  26.3.2013 by Smutny
        self.BaudRate = 9600  # Comport baud rate (default 19200 baud)

        self.REGPWRON = 1  # user must press arm power button

        self.REGPWRFLG = 5
        self.REGCFG = [1498, 1498, 1370, 1498]

        # Set energies and PID
        self.REGME = [30000, 16000, 16000, 16000]
        self.REGP = [50, 50, 50, 50]
        self.REGI = [5, 5, 5, 5]
        self.REGD = [50, 50, 50, 50]

        # Bosch robot shall not release after some time
        self.IDLEREL = 0

        # This variable use bbmovex, for help see this fucntion.
        self.pose = 0  # TODO

def bbdegtoirc(robot, a):
    assert a.shape[1] == robot.DOF, 'Wrong number of joints (%i, should be %i).'%(a.shape[1], robot.DOF)

    n = a.shape[0]
    b = np.multiply((a - np.repeat(np.array([robot.hhdeg]), n, axis=0)),
                    np.repeat(np.array([robot.degtoirc]), n, axis=0)) \
    + np.repeat(np.array([robot.hhirc]), n, axis=0)
    return b


if __name__ == '__main__':
    r = robotBosch()
    print r.ikt(r, [200, 100, 10, 20])