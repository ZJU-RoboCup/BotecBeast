from enum import Enum

class CtrlType(Enum):
    Torso   = "Torso"
    Torso_R = "Torso_R"
    Torso_P = "Torso_P"
    Torso_Y = "Torso_Y"
    Torso_x = "Torso_x"
    Torso_y = "Torso_y"
    Torso_z = "Torso_z"

    Lfoot   = "Lfoot"
    Lfoot_R = "Lfoot_R"
    Lfoot_P = "Lfoot_P"
    Lfoot_Y = "Lfoot_Y"
    Lfoot_x = "Lfoot_x"
    Lfoot_y = "Lfoot_y"
    Lfoot_z = "Lfoot_z"

    Rfoot   = "Rfoot"
    Rfoot_R = "Rfoot_R"
    Rfoot_P = "Rfoot_P"
    Rfoot_Y = "Rfoot_Y"
    Rfoot_x = "Rfoot_x"
    Rfoot_y = "Rfoot_y"
    Rfoot_z = "Rfoot_z"

    LArm_x = "LArm_x"
    LArm_y = "LArm_y"
    LArm_z = "LArm_z"
    
    RArm_x = "RArm_x"
    RArm_y = "RArm_y"
    RArm_z = "RArm_z"

    def key(self):
        if self == self.Lfoot or \
           self == self.Rfoot or \
           self == self.Torso:
            attr = ["_R", "_P", "_Y", "_x", "_y", "_z"]
            return [ self.value + att for att in attr ]

        return self.value