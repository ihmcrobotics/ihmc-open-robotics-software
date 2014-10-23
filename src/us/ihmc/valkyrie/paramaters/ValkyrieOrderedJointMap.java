package us.ihmc.valkyrie.paramaters;

import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.robotSide.SideDependentList;

public class ValkyrieOrderedJointMap
{
   public final static int LeftHipRotator         = 0;
   public final static int LeftHipAdductor        = 1;
   public final static int LeftHipExtensor        = 2;
   public final static int LeftKneeExtensor       = 3;
   public final static int LeftAnkleExtensor      = 4;
   public final static int LeftAnkle              = 5;
   public final static int RightHipRotator        = 6;
   public final static int RightHipAdductor       = 7;
   public final static int RightHipExtensor       = 8;
   public final static int RightKneeExtensor      = 9;
   public final static int RightAnkleExtensor     = 10;
   public final static int RightAnkle             = 11;
   public final static int WaistRotator           = 12;
   public final static int WaistExtensor          = 13;
   public final static int WaistLateralExtensor   = 14;
   public final static int LeftShoulderExtensor   = 15;
   public final static int LeftShoulderAdductor   = 16;
   public final static int LeftShoulderSupinator  = 17;
   public final static int LeftElbowExtensor      = 18;
   public final static int LeftForearmSupinator   = 19;
   public final static int LeftWristExtensor      = 20;
   public final static int LeftWrist              = 21;
   public final static int LowerNeckExtensor      = 22;
   public final static int NeckRotator            = 23;
   public final static int UpperNeckExtensor      = 24;
   public final static int RightShoulderExtensor  = 25;
   public final static int RightShoulderAdductor  = 26;
   public final static int RightShoulderSupinator = 27;
   public final static int RightElbowExtensor     = 28;
   public final static int RightForearmSupinator  = 29;
   public final static int RightWristExtensor     = 30;
   public final static int RightWrist             = 31;

   public final static int numberOfJoints = RightWrist + 1;

   public static String[]  jointNames = new String[numberOfJoints];
   static
   {

      jointNames[LeftHipRotator] = "LeftHipRotator";
      jointNames[LeftHipAdductor] = "LeftHipAdductor";
      jointNames[LeftHipExtensor] = "LeftHipExtensor";
      jointNames[LeftKneeExtensor] = "LeftKneeExtensor";
      jointNames[LeftAnkleExtensor] = "LeftAnkleExtensor";
      jointNames[LeftAnkle] = "LeftAnkle";
      jointNames[RightHipRotator] = "RightHipRotator";
      jointNames[RightHipAdductor] = "RightHipAdductor";
      jointNames[RightHipExtensor] = "RightHipExtensor";
      jointNames[RightKneeExtensor] = "RightKneeExtensor";
      jointNames[RightAnkleExtensor] = "RightAnkleExtensor";
      jointNames[RightAnkle] = "RightAnkle";
      jointNames[WaistRotator] = "WaistRotator";
      jointNames[WaistExtensor] = "WaistExtensor";
      jointNames[WaistLateralExtensor] = "WaistLateralExtensor";
      jointNames[LeftShoulderExtensor] = "LeftShoulderExtensor";
      jointNames[LeftShoulderAdductor] = "LeftShoulderAdductor";
      jointNames[LeftShoulderSupinator] = "LeftShoulderSupinator";
      jointNames[LeftElbowExtensor] = "LeftElbowExtensor";
      jointNames[LeftForearmSupinator] = "LeftForearmSupinator";
      jointNames[LeftWristExtensor] = "LeftWristExtensor";
      jointNames[LeftWrist] = "LeftWrist";
      jointNames[LowerNeckExtensor] = "LowerNeckExtensor";
      jointNames[NeckRotator] = "NeckRotator";
      jointNames[UpperNeckExtensor] = "UpperNeckExtensor";
      jointNames[RightShoulderExtensor] = "RightShoulderExtensor";
      jointNames[RightShoulderAdductor] = "RightShoulderAdductor";
      jointNames[RightShoulderSupinator] = "RightShoulderSupinator";
      jointNames[RightElbowExtensor] = "RightElbowExtensor";
      jointNames[RightForearmSupinator] = "RightForearmSupinator";
      jointNames[RightWristExtensor] = "RightWristExtensor";
      jointNames[RightWrist] = "RightWrist";
   }

   public static final SideDependentList<String[]> forcedSideDependentJointNames = new SideDependentList<String[]>();
   static
   {
      String[] jointNamesRight = new String[numberOfJoints];
      jointNamesRight[LeftHipRotator] = jointNames[RightHipRotator];
      jointNamesRight[LeftHipAdductor] = jointNames[RightHipAdductor];
      jointNamesRight[LeftHipExtensor] = jointNames[RightHipExtensor];
      jointNamesRight[LeftKneeExtensor] = jointNames[RightKneeExtensor];
      jointNamesRight[LeftAnkleExtensor] = jointNames[RightAnkleExtensor];
      jointNamesRight[LeftAnkle] = jointNames[RightAnkle];
      jointNamesRight[RightHipRotator] = jointNames[RightHipRotator];
      jointNamesRight[RightHipAdductor] = jointNames[RightHipAdductor];
      jointNamesRight[RightHipExtensor] = jointNames[RightHipExtensor];
      jointNamesRight[RightKneeExtensor] = jointNames[RightKneeExtensor];
      jointNamesRight[RightAnkleExtensor] = jointNames[RightAnkleExtensor];
      jointNamesRight[RightAnkle] = jointNames[RightAnkle];
      jointNamesRight[WaistRotator] = jointNames[WaistRotator];
      jointNamesRight[WaistExtensor] = jointNames[WaistExtensor];
      jointNamesRight[WaistLateralExtensor] = jointNames[WaistLateralExtensor];
      jointNamesRight[LeftShoulderExtensor] = jointNames[RightShoulderExtensor];
      jointNamesRight[LeftShoulderAdductor] = jointNames[RightShoulderAdductor];
      jointNamesRight[LeftShoulderSupinator] = jointNames[RightShoulderSupinator];
      jointNamesRight[LeftElbowExtensor] = jointNames[RightElbowExtensor];
      jointNamesRight[LeftForearmSupinator] = jointNames[RightForearmSupinator];
      jointNamesRight[LeftWristExtensor] = jointNames[RightWristExtensor];
      jointNamesRight[LeftWrist] = jointNames[RightWrist];
      jointNamesRight[LowerNeckExtensor] = jointNames[LowerNeckExtensor];
      jointNamesRight[NeckRotator] = jointNames[NeckRotator];
      jointNamesRight[UpperNeckExtensor] = jointNames[UpperNeckExtensor];
      jointNamesRight[RightShoulderExtensor] = jointNames[RightShoulderExtensor];
      jointNamesRight[RightShoulderAdductor] = jointNames[RightShoulderAdductor];
      jointNamesRight[RightShoulderSupinator] = jointNames[RightShoulderSupinator];
      jointNamesRight[RightElbowExtensor] = jointNames[RightElbowExtensor];
      jointNamesRight[RightForearmSupinator] = jointNames[RightForearmSupinator];
      jointNamesRight[RightWristExtensor] = jointNames[RightWristExtensor];
      jointNamesRight[RightWrist] = jointNames[RightWrist];

      forcedSideDependentJointNames.put(RobotSide.RIGHT, jointNamesRight);

      String[] jointNamesLeft = new String[numberOfJoints];
      jointNamesLeft[LeftHipRotator] = jointNames[LeftHipRotator];
      jointNamesLeft[LeftHipAdductor] = jointNames[LeftHipAdductor];
      jointNamesLeft[LeftHipExtensor] = jointNames[LeftHipExtensor];
      jointNamesLeft[LeftKneeExtensor] = jointNames[LeftKneeExtensor];
      jointNamesLeft[LeftAnkleExtensor] = jointNames[LeftAnkleExtensor];
      jointNamesLeft[LeftAnkle] = jointNames[LeftAnkle];
      jointNamesLeft[RightHipRotator] = jointNames[LeftHipRotator];
      jointNamesLeft[RightHipAdductor] = jointNames[LeftHipAdductor];
      jointNamesLeft[RightHipExtensor] = jointNames[LeftHipExtensor];
      jointNamesLeft[RightKneeExtensor] = jointNames[LeftKneeExtensor];
      jointNamesLeft[RightAnkleExtensor] = jointNames[LeftAnkleExtensor];
      jointNamesLeft[RightAnkle] = jointNames[LeftAnkle];
      jointNamesLeft[WaistRotator] = jointNames[WaistRotator];
      jointNamesLeft[WaistExtensor] = jointNames[WaistExtensor];
      jointNamesLeft[WaistLateralExtensor] = jointNames[WaistLateralExtensor];
      jointNamesLeft[LeftShoulderExtensor] = jointNames[LeftShoulderExtensor];
      jointNamesLeft[LeftShoulderAdductor] = jointNames[LeftShoulderAdductor];
      jointNamesLeft[LeftShoulderSupinator] = jointNames[LeftShoulderSupinator];
      jointNamesLeft[LeftElbowExtensor] = jointNames[LeftElbowExtensor];
      jointNamesLeft[LeftForearmSupinator] = jointNames[LeftForearmSupinator];
      jointNamesLeft[LeftWristExtensor] = jointNames[LeftWristExtensor];
      jointNamesLeft[LeftWrist] = jointNames[LeftWrist];
      jointNamesLeft[LowerNeckExtensor] = jointNames[LowerNeckExtensor];
      jointNamesLeft[NeckRotator] = jointNames[NeckRotator];
      jointNamesLeft[UpperNeckExtensor] = jointNames[UpperNeckExtensor];
      jointNamesLeft[RightShoulderExtensor] = jointNames[LeftShoulderExtensor];
      jointNamesLeft[RightShoulderAdductor] = jointNames[LeftShoulderAdductor];
      jointNamesLeft[RightShoulderSupinator] = jointNames[LeftShoulderSupinator];
      jointNamesLeft[RightElbowExtensor] = jointNames[LeftElbowExtensor];
      jointNamesLeft[RightForearmSupinator] = jointNames[LeftForearmSupinator];
      jointNamesLeft[RightWristExtensor] = jointNames[LeftWristExtensor];
      jointNamesLeft[RightWrist] = jointNames[LeftWrist];

      forcedSideDependentJointNames.put(RobotSide.LEFT, jointNamesLeft);
   }
}
