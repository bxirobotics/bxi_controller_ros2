"""Joint contracts shared by the ELF3 controllers."""

from bxi_example_py_elf3.framework.joints import JointCommandDefaults, JointDefault, JointLayout

ROBOT_NAME = "elf3"

ELF3_BODY_JOINTS = JointLayout(
    (
        "waist_y_joint",
        "waist_x_joint",
        "waist_z_joint",
        "l_hip_y_joint",
        "l_hip_x_joint",
        "l_hip_z_joint",
        "l_knee_y_joint",
        "l_ankle_y_joint",
        "l_ankle_x_joint",
        "r_hip_y_joint",
        "r_hip_x_joint",
        "r_hip_z_joint",
        "r_knee_y_joint",
        "r_ankle_y_joint",
        "r_ankle_x_joint",
        "l_shoulder_y_joint",
        "l_shoulder_x_joint",
        "l_shoulder_z_joint",
        "l_elbow_y_joint",
        "l_wrist_x_joint",
        "l_wrist_y_joint",
        "l_wrist_z_joint",
        "r_shoulder_y_joint",
        "r_shoulder_x_joint",
        "r_shoulder_z_joint",
        "r_elbow_y_joint",
        "r_wrist_x_joint",
        "r_wrist_y_joint",
        "r_wrist_z_joint",
    ),
    label="ELF3 29-joint policy",
)

ELF3_RESET_JOINTS = JointLayout(
    ELF3_BODY_JOINTS.names + ("head_z_joint", "head_y_joint"),
    label="ELF3 simulation reset",
)

# Apply head targets only when a policy omits those joints.
ELF3_COMMAND_DEFAULTS = JointCommandDefaults(
    {
        "head_y_joint": JointDefault(position=0.0, kp=16.747, kd=1.066),
        "head_z_joint": JointDefault(position=0.0, kp=16.747, kd=1.066),
    }
)
