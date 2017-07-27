package us.ihmc.exampleSimulations.stickRobot;

import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class StickRobotOrderedJointMap
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
   public final static int TorsoRoll= 14;
   public final static int LeftShoulderPitch= 15;
   public final static int LeftShoulderRoll= 16;
   public final static int LeftShoulderYaw= 17;
   public final static int LeftElbowPitch= 18;
   public final static int LeftForearmYaw= 19;
   public final static int LeftWristRoll= 20;
   public final static int LeftWristPitch= 21;
   public final static int NeckYaw= 22;
   public final static int NeckRoll= 23;
   public final static int NeckPitch= 24;
   public final static int RightShoulderPitch= 25;
   public final static int RightShoulderRoll= 26;
   public final static int RightShoulderYaw= 27;
   public final static int RightElbowPitch= 28;
   public final static int RightForearmYaw= 29;
   public final static int RightWristRoll= 30;
   public final static int RightWristPitch= 31;
   
   // since the count begins from 0 and RightWristPitch being the last joint in the chain
   public final static int numberOfJoints = RightWristPitch + 1;

   public static String[]  jointNames = new String[numberOfJoints];
   static
   {
      jointNames[LeftHipYaw] = "leftHipYaw";
      jointNames[LeftHipRoll] = "leftHipRoll";
      jointNames[LeftHipPitch] = "leftHipPitch";
      jointNames[LeftKneePitch] = "leftKneePitch";
      jointNames[LeftAnklePitch] = "leftAnklePitch";
      jointNames[LeftAnkleRoll] = "leftAnkleRoll";
      jointNames[RightHipYaw] = "rightHipYaw";
      jointNames[RightHipRoll] = "rightHipRoll";
      jointNames[RightHipPitch] = "rightHipPitch";
      jointNames[RightKneePitch] = "rightKneePitch";
      jointNames[RightAnklePitch] = "rightAnklePitch";
      jointNames[RightAnkleRoll] = "rightAnkleRoll";
      jointNames[TorsoYaw] = "torsoYaw";
      jointNames[TorsoPitch] = "torsoPitch";
      jointNames[TorsoRoll] = "torsoRoll";
      jointNames[LeftShoulderPitch] = "leftShoulderPitch";
      jointNames[LeftShoulderRoll] = "leftShoulderRoll";
      jointNames[LeftShoulderYaw] = "leftShoulderYaw";
      jointNames[LeftElbowPitch] = "leftElbowPitch";
      jointNames[LeftForearmYaw] = "leftForearmYaw";
      jointNames[LeftWristRoll] = "leftWristRoll";
      jointNames[LeftWristPitch] = "leftWristPitch";
      jointNames[NeckYaw] = "neckYaw";
      jointNames[NeckRoll] = "neckRoll";
      jointNames[NeckPitch] = "neckPitch";
      jointNames[RightShoulderPitch] = "rightShoulderPitch";
      jointNames[RightShoulderRoll] = "rightShoulderRoll";
      jointNames[RightShoulderYaw] = "rightShoulderYaw";
      jointNames[RightElbowPitch] = "rightElbowPitch";
      jointNames[RightForearmYaw] = "rightForearmYaw";
      jointNames[RightWristRoll] = "rightWristRoll";
      jointNames[RightWristPitch] = "rightWristPitch";
   }

   public static final SideDependentList<String[]> forcedSideDependentJointNames = new SideDependentList<String[]>();
   static
   {
      String[] jointNamesRight = new String[numberOfJoints];
      jointNamesRight[LeftHipYaw] = "rightHipYaw";
      jointNamesRight[LeftHipRoll] = "rightHipRoll";
      jointNamesRight[LeftHipPitch] = "rightHipPitch";
      jointNamesRight[LeftKneePitch] = "rightKneePitch";
      jointNamesRight[LeftAnklePitch] = "rightAnklePitch";
      jointNamesRight[LeftAnkleRoll] = "rightAnkleRoll";
      jointNamesRight[RightHipYaw] = "rightHipYaw";
      jointNamesRight[RightHipRoll] = "rightHipRoll";
      jointNamesRight[RightHipPitch] = "rightHipPitch";
      jointNamesRight[RightKneePitch] = "rightKneePitch";
      jointNamesRight[RightAnklePitch] = "rightAnklePitch";
      jointNamesRight[RightAnkleRoll] = "rightAnkleRoll";
      jointNamesRight[TorsoYaw] = "torsoYaw";
      jointNamesRight[TorsoPitch] = "torsoPitch";
      jointNamesRight[TorsoRoll] = "torsoRoll";
      jointNamesRight[LeftShoulderPitch] = "rightShoulderPitch";
      jointNamesRight[LeftShoulderRoll] = "rightShoulderRoll";
      jointNamesRight[LeftShoulderYaw] = "rightShoulderYaw";
      jointNamesRight[LeftElbowPitch] = "rightElbowPitch";
      jointNamesRight[LeftForearmYaw] = "rightForearmYaw";
      jointNamesRight[LeftWristRoll] = "rightWristRoll";
      jointNamesRight[LeftWristPitch] = "rightWristPitch";
      jointNamesRight[NeckYaw] = "neckYaw";
      jointNamesRight[NeckRoll] = "neckRoll";
      jointNamesRight[NeckPitch] = "neckPitch";
      jointNamesRight[RightShoulderPitch] = "rightShoulderPitch";
      jointNamesRight[RightShoulderRoll] = "rightShoulderRoll";
      jointNamesRight[RightShoulderYaw] = "rightShoulderYaw";
      jointNamesRight[RightElbowPitch] = "rightElbowPitch";
      jointNamesRight[RightForearmYaw] = "rightForearmYaw";
      jointNamesRight[RightWristRoll] = "rightWristRoll";
      jointNamesRight[RightWristPitch] = "rightWristPitch";
      
      forcedSideDependentJointNames.put(RobotSide.RIGHT, jointNamesRight);

      String[] jointNamesLeft = new String[numberOfJoints];
      jointNamesLeft[LeftHipYaw] = "leftHipYaw";
      jointNamesLeft[LeftHipRoll] = "leftHipRoll";
      jointNamesLeft[LeftHipPitch] = "leftHipPitch";
      jointNamesLeft[LeftKneePitch] = "leftKneePitch";
      jointNamesLeft[LeftAnklePitch] = "leftAnklePitch";
      jointNamesLeft[LeftAnkleRoll] = "leftAnkleRoll";
      jointNamesLeft[RightHipYaw] = "leftHipYaw";
      jointNamesLeft[RightHipRoll] = "leftHipRoll";
      jointNamesLeft[RightHipPitch] = "leftHipPitch";
      jointNamesLeft[RightKneePitch] = "leftKneePitch";
      jointNamesLeft[RightAnklePitch] = "leftAnklePitch";
      jointNamesLeft[RightAnkleRoll] = "leftAnkleRoll";
      jointNamesLeft[TorsoYaw] = "torsoYaw";
      jointNamesLeft[TorsoPitch] = "torsoPitch";
      jointNamesLeft[TorsoRoll] = "torsoRoll";
      jointNamesLeft[LeftShoulderPitch] = "leftShoulderPitch";
      jointNamesLeft[LeftShoulderRoll] = "leftShoulderRoll";
      jointNamesLeft[LeftShoulderYaw] = "leftShoulderYaw";
      jointNamesLeft[LeftElbowPitch] = "leftElbowPitch";
      jointNamesLeft[LeftForearmYaw] = "leftForearmYaw";
      jointNamesLeft[LeftWristRoll] = "leftWristRoll";
      jointNamesLeft[LeftWristPitch] = "leftWristPitch";
      jointNamesLeft[NeckYaw] = "neckYaw";
      jointNamesLeft[NeckRoll] = "neckRoll";
      jointNamesLeft[NeckPitch] = "neckPitch";
      jointNamesLeft[RightShoulderPitch] = "leftShoulderPitch";
      jointNamesLeft[RightShoulderRoll] = "leftShoulderRoll";
      jointNamesLeft[RightShoulderYaw] = "leftShoulderYaw";
      jointNamesLeft[RightElbowPitch] = "leftElbowPitch";
      jointNamesLeft[RightForearmYaw] = "leftForearmYaw";
      jointNamesLeft[RightWristRoll] = "leftWristRoll";
      jointNamesLeft[RightWristPitch] = "leftWristPitch";
      
      forcedSideDependentJointNames.put(RobotSide.LEFT, jointNamesLeft);
   }

}
