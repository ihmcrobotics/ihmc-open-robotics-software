package us.ihmc.darpaRoboticsChallenge.initialSetup;

import java.util.EnumMap;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.ArmJointName;
import us.ihmc.projectM.R2Sim02.initialSetup.RobotInitialSetup;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import static us.ihmc.darpaRoboticsChallenge.ros.ROSAtlasJointMap.*;

public class MultiContactDRCRobotInitialSetup implements RobotInitialSetup<SDFRobot>
{
   public void initializeRobot(SDFRobot robot)
   {
      SideDependentList<EnumMap<ArmJointName, Double>> defaultArmPosition = getDefaultArmPositionForMultiContactSimulation();

      // arms
      for (RobotSide robotSide : RobotSide.values)
      {
         String[] forcedSideJointNames = forcedSideDependentJointNames.get(robotSide);
         robot.getOneDoFJoint(forcedSideJointNames[l_arm_shy]).setQ(defaultArmPosition.get(robotSide).get(ArmJointName.SHOULDER_PITCH));
         robot.getOneDoFJoint(forcedSideJointNames[l_arm_shx]).setQ(defaultArmPosition.get(robotSide).get(ArmJointName.SHOULDER_ROLL));
         robot.getOneDoFJoint(forcedSideJointNames[l_arm_ely]).setQ(defaultArmPosition.get(robotSide).get(ArmJointName.ELBOW_PITCH));
         robot.getOneDoFJoint(forcedSideJointNames[l_arm_elx]).setQ(defaultArmPosition.get(robotSide).get(ArmJointName.ELBOW_ROLL));
         robot.getOneDoFJoint(forcedSideJointNames[l_arm_wry]).setQ(defaultArmPosition.get(robotSide).get(ArmJointName.WRIST_PITCH));
         robot.getOneDoFJoint(forcedSideJointNames[l_arm_wrx]).setQ(defaultArmPosition.get(robotSide).get(ArmJointName.WRIST_ROLL));
      }

      // left leg
      robot.getOneDoFJoint(jointNames[l_leg_hpz]).setQ(0.4);
      robot.getOneDoFJoint(jointNames[l_leg_hpx]).setQ(0.4);
      robot.getOneDoFJoint(jointNames[l_leg_hpy]).setQ(0.3);
      robot.getOneDoFJoint(jointNames[l_leg_kny]).setQ(0.8);

      // right leg
      robot.getOneDoFJoint(jointNames[r_leg_hpx]).setQ(-0.4);
      robot.getOneDoFJoint(jointNames[r_leg_hpy]).setQ(0.3);
      robot.getOneDoFJoint(jointNames[r_leg_kny]).setQ(0.4);
   }

   public static SideDependentList<EnumMap<ArmJointName, Double>> getDefaultArmPositionForMultiContactSimulation()
   {
      SideDependentList<EnumMap<ArmJointName, Double>> defaultArmPosition = SideDependentList.createListOfEnumMaps(ArmJointName.class);

      defaultArmPosition.get(RobotSide.LEFT).put(ArmJointName.SHOULDER_PITCH, -0.4);
      defaultArmPosition.get(RobotSide.LEFT).put(ArmJointName.SHOULDER_ROLL, -0.7);
      defaultArmPosition.get(RobotSide.LEFT).put(ArmJointName.ELBOW_PITCH, 1.8);
      defaultArmPosition.get(RobotSide.LEFT).put(ArmJointName.ELBOW_ROLL, 1.4);
      defaultArmPosition.get(RobotSide.LEFT).put(ArmJointName.WRIST_PITCH, 0.0);
      defaultArmPosition.get(RobotSide.LEFT).put(ArmJointName.WRIST_ROLL, 0.5);

      defaultArmPosition.get(RobotSide.RIGHT).put(ArmJointName.SHOULDER_PITCH, 0.3);
      defaultArmPosition.get(RobotSide.RIGHT).put(ArmJointName.SHOULDER_ROLL, 1.0);
      defaultArmPosition.get(RobotSide.RIGHT).put(ArmJointName.ELBOW_PITCH, 1.0);
      defaultArmPosition.get(RobotSide.RIGHT).put(ArmJointName.ELBOW_ROLL, -1.6);
      defaultArmPosition.get(RobotSide.RIGHT).put(ArmJointName.WRIST_PITCH, 0.0);
      defaultArmPosition.get(RobotSide.RIGHT).put(ArmJointName.WRIST_ROLL, 0.0);

      return defaultArmPosition;
   }
}
