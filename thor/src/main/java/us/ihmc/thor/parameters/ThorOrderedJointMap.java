package us.ihmc.thor.parameters;

import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class ThorOrderedJointMap
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
   public final static int TorsoPitch= 13;
   public final static int LeftShoulderPitch= 14;
   public final static int LeftShoulderRoll= 15;
   public final static int LeftShoulderYaw= 16;
   public final static int LeftElbowPitch= 17;
   public final static int LeftForearmYaw= 18;
   public final static int LeftWristRoll= 19;
   public final static int LeftWristPitch= 20;
   public final static int RightShoulderPitch= 21;
   public final static int RightShoulderRoll= 22;
   public final static int RightShoulderYaw= 23;
   public final static int RightElbowPitch= 24;
   public final static int RightForearmYaw= 25;
   public final static int RightWristRoll= 26;
   public final static int RightWristPitch= 27;
   public final static int NeckYaw= 28;
   public final static int NeckPitch= 29;
   public final static int LeftIndexFinger= 30;
   public final static int LeftPinkyFinger= 31;
   public final static int RightIndexFinger= 32;
   public final static int RightPinkyFinger= 33;
   public final static int ChestSpinnyJointFrame= 34;
   public final static int MultisenseSLSpinnyJointFrame= 35;

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
      jointNames[TorsoPitch] = "torso_pitch";
      jointNames[LeftShoulderPitch] = "l_shoulder_pitch";
      jointNames[LeftShoulderRoll] = "l_shoulder_roll";
      jointNames[LeftShoulderYaw] = "l_shoulder_yaw";
      jointNames[LeftElbowPitch] = "l_elbow_pitch";
      jointNames[LeftForearmYaw] = "l_wrist_yaw";
      jointNames[LeftWristRoll] = "l_wrist_roll";
      jointNames[LeftWristPitch] = "l_wrist_pitch";
      jointNames[RightShoulderPitch] = "r_shoulder_pitch";
      jointNames[RightShoulderRoll] = "r_shoulder_roll";
      jointNames[RightShoulderYaw] = "r_shoulder_yaw";
      jointNames[RightElbowPitch] = "r_elbow_pitch";
      jointNames[RightForearmYaw] = "r_wrist_yaw";
      jointNames[RightWristRoll] = "r_wrist_roll";
      jointNames[RightWristPitch] = "r_wrist_pitch";
      jointNames[NeckYaw] = "neck_yaw";
      jointNames[NeckPitch] = "neck_pitch";
      jointNames[LeftIndexFinger] = "l_index_finger";
      jointNames[LeftPinkyFinger] = "l_pinky_finger";
      jointNames[RightIndexFinger] = "r_index_finger";
      jointNames[RightPinkyFinger] = "r_pinky_finger";
      jointNames[ChestSpinnyJointFrame] = "chest_lidar_pitch";
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
      jointNamesRight[TorsoPitch] = "torso_pitch";
      jointNamesRight[LeftShoulderPitch] = "r_shoulder_pitch";
      jointNamesRight[LeftShoulderRoll] = "r_shoulder_roll";
      jointNamesRight[LeftShoulderYaw] = "r_shoulder_yaw";
      jointNamesRight[LeftElbowPitch] = "r_elbow_pitch";
      jointNamesRight[LeftForearmYaw] = "r_wrist_yaw";
      jointNamesRight[LeftWristRoll] = "r_wrist_roll";
      jointNamesRight[LeftWristPitch] = "r_wrist_pitch";
      jointNamesRight[RightShoulderPitch] = "r_shoulder_pitch";
      jointNamesRight[RightShoulderRoll] = "r_shoulder_roll";
      jointNamesRight[RightShoulderYaw] = "r_shoulder_yaw";
      jointNamesRight[RightElbowPitch] = "r_elbow_pitch";
      jointNamesRight[RightForearmYaw] = "r_wrist_yaw";
      jointNamesRight[RightWristRoll] = "r_wrist_roll";
      jointNamesRight[RightWristPitch] = "r_wrist_pitch";
      jointNamesRight[NeckYaw] = "neck_yaw";
      jointNamesRight[NeckPitch] = "neck_pitch";
      jointNamesRight[LeftIndexFinger] = "r_index_finger";
      jointNamesRight[LeftPinkyFinger] = "r_pinky_finger";
      jointNamesRight[RightIndexFinger] = "r_index_finger";
      jointNamesRight[RightPinkyFinger] = "r_pinky_finger";
      jointNamesRight[ChestSpinnyJointFrame] = "chest_lidar_pitch";
      jointNamesRight[MultisenseSLSpinnyJointFrame] = "head_lidar_roll";

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
      jointNamesLeft[TorsoPitch] = "torso_pitch";
      jointNamesLeft[LeftShoulderPitch] = "l_shoulder_pitch";
      jointNamesLeft[LeftShoulderRoll] = "l_shoulder_roll";
      jointNamesLeft[LeftShoulderYaw] = "l_shoulder_yaw";
      jointNamesLeft[LeftElbowPitch] = "l_elbow_pitch";
      jointNamesLeft[LeftForearmYaw] = "l_wrist_yaw";
      jointNamesLeft[LeftWristRoll] = "l_wrist_roll";
      jointNamesLeft[LeftWristPitch] = "l_wrist_pitch";
      jointNamesLeft[RightShoulderPitch] = "l_shoulder_pitch";
      jointNamesLeft[RightShoulderRoll] = "l_shoulder_roll";
      jointNamesLeft[RightShoulderYaw] = "l_shoulder_yaw";
      jointNamesLeft[RightElbowPitch] = "l_elbow_pitch";
      jointNamesLeft[RightForearmYaw] = "l_wrist_yaw";
      jointNamesLeft[RightWristRoll] = "l_wrist_roll";
      jointNamesLeft[RightWristPitch] = "l_wrist_pitch";
      jointNamesLeft[NeckYaw] = "neck_yaw";
      jointNamesLeft[NeckPitch] = "neck_pitch";
      jointNamesLeft[LeftIndexFinger] = "l_index_finger";
      jointNamesLeft[LeftPinkyFinger] = "l_pinky_finger";
      jointNamesLeft[RightIndexFinger] = "l_index_finger";
      jointNamesLeft[RightPinkyFinger] = "l_pinky_finger";
      jointNamesLeft[ChestSpinnyJointFrame] = "chest_lidar_pitch";
      jointNamesLeft[MultisenseSLSpinnyJointFrame] = "head_lidar_roll";

      forcedSideDependentJointNames.put(RobotSide.LEFT, jointNamesLeft);
   }
}
