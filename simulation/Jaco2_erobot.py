
import numpy as np
import os
from roboticstoolbox.robot.ERobot import ERobot

location = os.path.dirname(__file__)

class KinovaJaco2(ERobot):
    """
    Class that imports a KinovaGen3 URDF model
    ``KinovaGen3()`` is a class which imports a KinovaGen3 robot definition
    from a URDF file.  The model describes its kinematic and graphical
    characteristics.
    .. runblock:: pycon
        >>> import roboticstoolbox as rtb
        >>> robot = rtb.models.URDF.KinovaGen3()
        >>> print(robot)
    Defined joint configurations are:
    - qz, zero joint angle configuration, 'L' shaped configuration
    - qr, vertical 'READY' configuration
    - qs, arm is stretched out in the x-direction
    - qn, arm is at a nominal non-singular configuration
    .. codeauthor:: Jesse Haviland
    .. sectionauthor:: Peter Corke
    """


    def __init__(self):
    
        #The path need to be change according to where you have your xacro file
        links, name, urdf_string, urdf_filepath = self.URDF_read(
            location + '/urdf/j2n6s300_standalone.xacro'
        )

        super().__init__(
            links,
            name=name,
            manufacturer="Kinova",
            urdf_string=urdf_string,
            urdf_filepath=urdf_filepath,
            # gripper_links=elinks[9]
        )

        # self.qdlim = np.array([
        # 2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100, 3.0, 3.0])

        self.model = "Kinova Jaco2"

        #Theta værdier ved zero joint-values
        self.qr = np.array([-0, np.pi, np.pi, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        self.qz = np.zeros(12)

        self.addconfiguration("qr", self.qr)
        self.addconfiguration("qz", self.qz)


if __name__ == "__main__":  # pragma nocover

    robot = KinovaJaco2()
    print(robot)