controller_list:
@[if has_arm]@
  - name: fake_arm_torso_controller
    type: interpolate
    joints:
      - torso_lift_joint
      - arm_1_joint
      - arm_2_joint
      - arm_3_joint
      - arm_4_joint
      - arm_5_joint
      - arm_6_joint
      - arm_7_joint
  - name: fake_arm_controller
    type: interpolate
    joints:
      - arm_1_joint
      - arm_2_joint
      - arm_3_joint
      - arm_4_joint
      - arm_5_joint
      - arm_6_joint
      - arm_7_joint
@[end if]@
@[if end_effector == "pal-gripper"]@
  - name: fake_gripper_controller
    type: interpolate
    joints:
      - gripper_left_finger_joint
      - gripper_right_finger_joint
@[end if]@
@[if end_effector == "schunk-wsg" or end_effector == "robotiq-2f-85" or end_effector == "robotiq-2f-140"]@
  - name: fake_gripper_controller
    type: interpolate
    joints:
      - gripper_finger_joint
@[end if]@
  - name: fake_head_controller
    type: interpolate
    joints:
      - head_1_joint
      - head_2_joint
  - name: fake_torso_controller
    type: interpolate
    joints:
      - torso_lift_joint
@[if end_effector == "pal-hey5"]@
  - name: fake_hand_controller
    type: interpolate
    joints:
      - hand_index_joint
      - hand_thumb_joint
      - hand_mrl_joint
@[end if]@
