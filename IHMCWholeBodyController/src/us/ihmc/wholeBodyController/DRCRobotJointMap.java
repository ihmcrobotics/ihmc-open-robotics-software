package us.ihmc.wholeBodyController;

import us.ihmc.SdfLoader.SDFJointNameMap;
import org.apache.commons.lang3.tuple.ImmutablePair;
import us.ihmc.robotics.humanoidRobot.partNames.ArmJointName;
import us.ihmc.robotics.humanoidRobot.partNames.LegJointName;
import us.ihmc.robotics.humanoidRobot.partNames.NeckJointName;
import us.ihmc.robotics.humanoidRobot.partNames.SpineJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoUtilities.controllers.YoPDGains;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;

import java.util.List;

public interface DRCRobotJointMap extends SDFJointNameMap
{
   public abstract String getNameOfJointBeforeChest();

   public abstract SideDependentList<String> getNameOfJointBeforeThighs();

   public abstract SideDependentList<String> getNameOfJointBeforeHands();

   public abstract String[] getOrderedJointNames();

   public abstract String getLegJointName(RobotSide robotSide, LegJointName legJointName);

   public abstract String getArmJointName(RobotSide robotSide, ArmJointName armJointName);

   public abstract String getNeckJointName(NeckJointName neckJointName);

   public abstract String getSpineJointName(SpineJointName spineJointName);

   public abstract String[] getPositionControlledJointsForSimulation();

   public abstract DRCRobotContactPointParameters getContactPointParameters();

   public List<ImmutablePair<String, YoPDGains>> getPassiveJointNameWithGains(YoVariableRegistry registry);
}