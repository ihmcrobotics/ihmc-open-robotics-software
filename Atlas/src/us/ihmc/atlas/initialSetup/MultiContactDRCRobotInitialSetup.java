package us.ihmc.atlas.initialSetup;

import static us.ihmc.atlas.ros.AtlasOrderedJointMap.forcedSideDependentJointNames;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.jointNames;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.l_arm_elx;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.l_arm_ely;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.l_arm_shx;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.l_arm_shz;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.l_arm_wrx;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.l_arm_wry;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.l_leg_hpx;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.l_leg_hpy;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.l_leg_hpz;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.l_leg_kny;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.r_leg_hpx;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.r_leg_hpy;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.r_leg_kny;

import java.util.EnumMap;

import javax.vecmath.Vector3d;

import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.wholeBodyController.DRCRobotJointMap;

public class MultiContactDRCRobotInitialSetup implements DRCRobotInitialSetup<HumanoidFloatingRootJointRobot>
{
   @Override
   public void initializeRobot(HumanoidFloatingRootJointRobot robot, DRCRobotJointMap jointMap)
   {
      SideDependentList<EnumMap<ArmJointName, Double>> defaultArmPosition = getDefaultArmPositionForMultiContactSimulation();

      // arms
      for (RobotSide robotSide : RobotSide.values)
      {
         String[] forcedSideJointNames = forcedSideDependentJointNames.get(robotSide);
         robot.getOneDegreeOfFreedomJoint(forcedSideJointNames[l_arm_shz]).setQ(defaultArmPosition.get(robotSide).get(ArmJointName.SHOULDER_YAW));
         robot.getOneDegreeOfFreedomJoint(forcedSideJointNames[l_arm_shx]).setQ(defaultArmPosition.get(robotSide).get(ArmJointName.SHOULDER_ROLL));
         robot.getOneDegreeOfFreedomJoint(forcedSideJointNames[l_arm_ely]).setQ(defaultArmPosition.get(robotSide).get(ArmJointName.ELBOW_PITCH));
         robot.getOneDegreeOfFreedomJoint(forcedSideJointNames[l_arm_elx]).setQ(defaultArmPosition.get(robotSide).get(ArmJointName.ELBOW_ROLL));
         robot.getOneDegreeOfFreedomJoint(forcedSideJointNames[l_arm_wry]).setQ(defaultArmPosition.get(robotSide).get(ArmJointName.FIRST_WRIST_PITCH));
         robot.getOneDegreeOfFreedomJoint(forcedSideJointNames[l_arm_wrx]).setQ(defaultArmPosition.get(robotSide).get(ArmJointName.WRIST_ROLL));
      }

      // left leg
      robot.getOneDegreeOfFreedomJoint(jointNames[l_leg_hpz]).setQ(0.4);
      robot.getOneDegreeOfFreedomJoint(jointNames[l_leg_hpx]).setQ(0.4);
      robot.getOneDegreeOfFreedomJoint(jointNames[l_leg_hpy]).setQ(0.3);
      robot.getOneDegreeOfFreedomJoint(jointNames[l_leg_kny]).setQ(0.8);

      // right leg
      robot.getOneDegreeOfFreedomJoint(jointNames[r_leg_hpx]).setQ(-0.4);
      robot.getOneDegreeOfFreedomJoint(jointNames[r_leg_hpy]).setQ(0.3);
      robot.getOneDegreeOfFreedomJoint(jointNames[r_leg_kny]).setQ(0.4);
   }

   public static SideDependentList<EnumMap<ArmJointName, Double>> getDefaultArmPositionForMultiContactSimulation()
   {
      SideDependentList<EnumMap<ArmJointName, Double>> defaultArmPosition = SideDependentList.createListOfEnumMaps(ArmJointName.class);

      defaultArmPosition.get(RobotSide.LEFT).put(ArmJointName.SHOULDER_YAW, -0.4);
      defaultArmPosition.get(RobotSide.LEFT).put(ArmJointName.SHOULDER_ROLL, -0.7);
      defaultArmPosition.get(RobotSide.LEFT).put(ArmJointName.ELBOW_PITCH, 1.8);
      defaultArmPosition.get(RobotSide.LEFT).put(ArmJointName.ELBOW_ROLL, 1.4);
      defaultArmPosition.get(RobotSide.LEFT).put(ArmJointName.FIRST_WRIST_PITCH, 0.0);
      defaultArmPosition.get(RobotSide.LEFT).put(ArmJointName.WRIST_ROLL, 0.5);
      defaultArmPosition.get(RobotSide.LEFT).put(ArmJointName.SECOND_WRIST_PITCH, 0.0);

      defaultArmPosition.get(RobotSide.RIGHT).put(ArmJointName.SHOULDER_YAW, 0.3);
      defaultArmPosition.get(RobotSide.RIGHT).put(ArmJointName.SHOULDER_ROLL, 1.0);
      defaultArmPosition.get(RobotSide.RIGHT).put(ArmJointName.ELBOW_PITCH, 1.0);
      defaultArmPosition.get(RobotSide.RIGHT).put(ArmJointName.ELBOW_ROLL, -1.6);
      defaultArmPosition.get(RobotSide.RIGHT).put(ArmJointName.FIRST_WRIST_PITCH, 0.0);
      defaultArmPosition.get(RobotSide.RIGHT).put(ArmJointName.WRIST_ROLL, 0.0);
      defaultArmPosition.get(RobotSide.RIGHT).put(ArmJointName.SECOND_WRIST_PITCH, 0.0);

      return defaultArmPosition;
   }
   
   @Override
   public void getOffset(Vector3d offsetToPack)
   {
   }

   @Override
   public void setOffset(Vector3d offset)
   {
   }

   @Override
   public void setInitialYaw(double yaw)
   {
   }

   @Override
   public void setInitialGroundHeight(double groundHeight)
   {
   }

   @Override
   public double getInitialYaw()
   {
      return 0;
   }

   @Override
   public double getInitialGroundHeight()
   {
      return 0;
   }
}
