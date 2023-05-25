
import numpy as np
from roboticstoolbox.robot.ERobot import ERobot

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

        links, name, urdf_string, urdf_filepath = self.URDF_read(
            "/home/daniel/Desktop/Solve IK/jaco2/j2n6s300_standalone_no_xacro.urdf"
        )

        super().__init__(
            links,
            name=name,
            manufacturer="Kinova",
            urdf_string=urdf_string,
            urdf_filepath=urdf_filepath,
            # gripper_links=elinks[9]
        )

        self.qr = np.array([-0, np.pi/2, -np.pi/2, 0, np.pi, -np.pi/2, 0, 0, 0, 0, 0, 0])
        self.qz = np.zeros(12)

        self.addconfiguration("qr", self.qr)
        self.addconfiguration("qz", self.qz)


if __name__ == "__main__":  # pragma nocover

    robot = KinovaJaco2()
    print(robot)