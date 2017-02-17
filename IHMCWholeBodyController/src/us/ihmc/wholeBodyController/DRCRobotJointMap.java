package us.ihmc.wholeBodyController;

import java.util.List;

import org.apache.commons.lang3.tuple.ImmutablePair;

import us.ihmc.robotics.controllers.YoPDGains;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public interface DRCRobotJointMap extends HumanoidJointNameMap
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

   public abstract RobotContactPointParameters getContactPointParameters();

   public List<ImmutablePair<String, YoPDGains>> getPassiveJointNameWithGains(YoVariableRegistry registry);
}