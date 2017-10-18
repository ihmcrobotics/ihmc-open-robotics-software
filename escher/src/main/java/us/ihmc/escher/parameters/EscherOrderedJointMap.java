package us.ihmc.escher.parameters;

import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class EscherOrderedJointMap
{
   public final static int LeftHipYaw= 0;
   public final static int LeftHipRoll= 1;
   public final static int LeftHipPitch= 2;
   public final static int LeftKneePitch= 3;
   public final static int LeftAnklePitch= 4;
   public final static int LeftAnkleRoll= 5;
   public final static int RightHipYaw= 6;
   public final static int RightHipRoll= 7;
   public final static int RightHipPitch= 8;
   public final static int RightKneePitch= 9;
   public final static int RightAnklePitch= 10;
   public final static int RightAnkleRoll= 11;
   public final static int TorsoYaw= 12;
   public final static int LeftShoulderPitch= 13;
   public final static int LeftShoulderRoll= 14;
   public final static int LeftShoulderYaw= 15;
   public final static int LeftElbowPitch= 16;
   public final static int LeftForearmYaw= 17;
   public final static int LeftWristRoll= 18;
   public final static int LeftWristPitch= 19;
   public final static int RightShoulderPitch= 20;
   public final static int RightShoulderRoll= 21;
   public final static int RightShoulderYaw= 22;
   public final static int RightElbowPitch= 23;
   public final static int RightForearmYaw= 24;
   public final static int RightWristRoll= 25;
   public final static int RightWristPitch= 26;
   public final static int NeckYaw= 27;
   public final static int NeckPitch= 28;
   public final static int LeftIndexFinger= 29;
   public final static int LeftRingFinger= 30;
   public final static int LeftThumbRoll= 31;
   public final static int LeftThumbPitch= 32;
   public final static int RightIndexFinger= 33;
   public final static int RightRingFinger= 34;
   public final static int RightThumbRoll= 35;
   public final static int RightThumbPitch= 36;
   public final static int MultisenseSLSpinnyJointFrame= 37;

   public final static int numberOfJoints = MultisenseSLSpinnyJointFrame + 1;

   public static String[]  jointNames = new String[numberOfJoints];
   static
   {
      jointNames[LeftHipYaw] = "l_hip_yaw";
      jointNames[LeftHipRoll] = "l_hip_roll";
      jointNames[LeftHipPitch] = "l_hip_pitch";
      jointNames[LeftKneePitch] = "l_knee_pitch";
      jointNames[LeftAnklePitch] = "l_ankle_pitch";
      jointNames[LeftAnkleRoll] = "l_ankle_roll";
      jointNames[RightHipYaw] = "r_hip_yaw";
      jointNames[RightHipRoll] = "r_hip_roll";
      jointNames[RightHipPitch] = "r_hip_pitch";
      jointNames[RightKneePitch] = "r_knee_pitch";
      jointNames[RightAnklePitch] = "r_ankle_pitch";
      jointNames[RightAnkleRoll] = "r_ankle_roll";
      jointNames[TorsoYaw] = "torso_yaw";
      jointNames[LeftShoulderPitch] = "l_shoulder_pitch";
      jointNames[LeftShoulderRoll] = "l_shoulder_roll";
      jointNames[LeftShoulderYaw] = "l_shoulder_yaw";
      jointNames[LeftElbowPitch] = "l_elbow_pitch";
      jointNames[LeftForearmYaw] = "l_elbow_roll";
      jointNames[LeftWristRoll] = "l_wrist_pitch";
      jointNames[LeftWristPitch] = "l_wrist_yaw";
      jointNames[RightShoulderPitch] = "r_shoulder_pitch";
      jointNames[RightShoulderRoll] = "r_shoulder_roll";
      jointNames[RightShoulderYaw] = "r_shoulder_yaw";
      jointNames[RightElbowPitch] = "r_elbow_pitch";
      jointNames[RightForearmYaw] = "r_elbow_roll";
      jointNames[RightWristRoll] = "r_wrist_pitch";
      jointNames[RightWristPitch] = "r_wrist_yaw";
      jointNames[NeckYaw] = "neck_yaw";
      jointNames[NeckPitch] = "neck_pitch";
      jointNames[LeftIndexFinger] = "l_index_yaw";
      jointNames[LeftRingFinger] = "l_ring_yaw";
      jointNames[LeftThumbRoll] = "l_thumb_roll";
      jointNames[LeftThumbPitch] = "l_thumb_pitch";
      jointNames[RightIndexFinger] = "r_index_finger";
      jointNames[RightRingFinger] = "r_ring_finger";
      jointNames[RightThumbRoll] = "r_thumb_roll";
      jointNames[RightThumbPitch] = "r_thumb_pitch";
      jointNames[MultisenseSLSpinnyJointFrame] = "head_hokuyo_joint";
   }

   public static final SideDependentList<String[]> forcedSideDependentJointNames = new SideDependentList<String[]>();
   static
   {
      String[] jointNamesRight = new String[numberOfJoints];
      jointNamesRight[LeftHipYaw] = "r_hip_yaw";
      jointNamesRight[LeftHipRoll] = "r_hip_roll";
      jointNamesRight[LeftHipPitch] = "r_hip_pitch";
      jointNamesRight[LeftKneePitch] = "r_knee_pitch";
      jointNamesRight[LeftAnklePitch] = "r_ankle_pitch";
      jointNamesRight[LeftAnkleRoll] = "r_ankle_roll";
      jointNamesRight[RightHipYaw] = "r_hip_yaw";
      jointNamesRight[RightHipRoll] = "r_hip_roll";
      jointNamesRight[RightHipPitch] = "r_hip_pitch";
      jointNamesRight[RightKneePitch] = "r_knee_pitch";
      jointNamesRight[RightAnklePitch] = "r_ankle_pitch";
      jointNamesRight[RightAnkleRoll] = "r_ankle_roll";
      jointNamesRight[TorsoYaw] = "torso_yaw";
      jointNamesRight[LeftShoulderPitch] = "r_shoulder_pitch";
      jointNamesRight[LeftShoulderRoll] = "r_shoulder_roll";
      jointNamesRight[LeftShoulderYaw] = "r_shoulder_yaw";
      jointNamesRight[LeftElbowPitch] = "r_elbow_pitch";
      jointNamesRight[LeftForearmYaw] = "r_elbow_roll";
      jointNamesRight[LeftWristRoll] = "r_wrist_pitch";
      jointNamesRight[LeftWristPitch] = "r_wrist_yaw";
      jointNamesRight[RightShoulderPitch] = "r_shoulder_pitch";
      jointNamesRight[RightShoulderRoll] = "r_shoulder_roll";
      jointNamesRight[RightShoulderYaw] = "r_shoulder_yaw";
      jointNamesRight[RightElbowPitch] = "r_elbow_pitch";
      jointNamesRight[RightForearmYaw] = "r_elbow_roll";
      jointNamesRight[RightWristRoll] = "r_wrist_pitch";
      jointNamesRight[RightWristPitch] = "r_wrist_yaw";
      jointNamesRight[NeckYaw] = "neck_yaw";
      jointNamesRight[NeckPitch] = "neck_pitch";
      jointNamesRight[LeftIndexFinger] = "r_index_yaw";
      jointNamesRight[LeftRingFinger] = "r_ring_yaw";
      jointNamesRight[LeftThumbRoll] = "r_thumb_roll";
      jointNamesRight[LeftThumbPitch] = "r_thumb_pitch";
      jointNamesRight[RightIndexFinger] = "r_index_finger";
      jointNamesRight[RightRingFinger] = "r_ring_finger";
      jointNamesRight[RightThumbRoll] = "r_thumb_roll";
      jointNamesRight[RightThumbPitch] = "r_thumb_pitch";
      jointNamesRight[MultisenseSLSpinnyJointFrame] = "head_hokuyo_joint";

      forcedSideDependentJointNames.put(RobotSide.RIGHT, jointNamesRight);

      String[] jointNamesLeft = new String[numberOfJoints];
      jointNamesLeft[LeftHipYaw] = "l_hip_yaw";
      jointNamesLeft[LeftHipRoll] = "l_hip_roll";
      jointNamesLeft[LeftHipPitch] = "l_hip_pitch";
      jointNamesLeft[LeftKneePitch] = "l_knee_pitch";
      jointNamesLeft[LeftAnklePitch] = "l_ankle_pitch";
      jointNamesLeft[LeftAnkleRoll] = "l_ankle_roll";
      jointNamesLeft[RightHipYaw] = "l_hip_yaw";
      jointNamesLeft[RightHipRoll] = "l_hip_roll";
      jointNamesLeft[RightHipPitch] = "l_hip_pitch";
      jointNamesLeft[RightKneePitch] = "l_knee_pitch";
      jointNamesLeft[RightAnklePitch] = "l_ankle_pitch";
      jointNamesLeft[RightAnkleRoll] = "l_ankle_roll";
      jointNamesLeft[TorsoYaw] = "torso_yaw";
      jointNamesLeft[LeftShoulderPitch] = "l_shoulder_pitch";
      jointNamesLeft[LeftShoulderRoll] = "l_shoulder_roll";
      jointNamesLeft[LeftShoulderYaw] = "l_shoulder_yaw";
      jointNamesLeft[LeftElbowPitch] = "l_elbow_pitch";
      jointNamesLeft[LeftForearmYaw] = "l_elbow_roll";
      jointNamesLeft[LeftWristRoll] = "l_wrist_pitch";
      jointNamesLeft[LeftWristPitch] = "l_wrist_yaw";
      jointNamesLeft[RightShoulderPitch] = "l_shoulder_pitch";
      jointNamesLeft[RightShoulderRoll] = "l_shoulder_roll";
      jointNamesLeft[RightShoulderYaw] = "l_shoulder_yaw";
      jointNamesLeft[RightElbowPitch] = "l_elbow_pitch";
      jointNamesLeft[RightForearmYaw] = "l_elbow_roll";
      jointNamesLeft[RightWristRoll] = "l_wrist_pitch";
      jointNamesLeft[RightWristPitch] = "l_wrist_yaw";
      jointNamesLeft[NeckYaw] = "neck_yaw";
      jointNamesLeft[NeckPitch] = "neck_pitch";
      jointNamesLeft[LeftIndexFinger] = "l_index_yaw";
      jointNamesLeft[LeftRingFinger] = "l_ring_yaw";
      jointNamesLeft[LeftThumbRoll] = "l_thumb_roll";
      jointNamesLeft[LeftThumbPitch] = "l_thumb_pitch";
      jointNamesLeft[RightIndexFinger] = "l_index_finger";
      jointNamesLeft[RightRingFinger] = "l_ring_finger";
      jointNamesLeft[RightThumbRoll] = "l_thumb_roll";
      jointNamesLeft[RightThumbPitch] = "l_thumb_pitch";
      jointNamesLeft[MultisenseSLSpinnyJointFrame] = "head_hokuyo_joint";

      forcedSideDependentJointNames.put(RobotSide.LEFT, jointNamesLeft);
   }
}
