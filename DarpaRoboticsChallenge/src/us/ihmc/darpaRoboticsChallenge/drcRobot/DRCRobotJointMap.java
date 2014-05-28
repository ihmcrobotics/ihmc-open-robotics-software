package us.ihmc.darpaRoboticsChallenge.drcRobot;

import us.ihmc.SdfLoader.SDFJointNameMap;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.humanoidRobot.partNames.ArmJointName;
import us.ihmc.utilities.humanoidRobot.partNames.LegJointName;
import us.ihmc.utilities.humanoidRobot.partNames.NeckJointName;
import us.ihmc.utilities.humanoidRobot.partNames.SpineJointName;

public abstract class DRCRobotJointMap implements SDFJointNameMap
{
   public abstract String getNameOfJointBeforeChest();

   public abstract String getNameOfJointBeforeThigh(RobotSide robotSide);

   public abstract String getNameOfJointBeforeHand(RobotSide robotSide);

   public abstract String[] getOrderedJointNames();

   public abstract String getLegJointName(RobotSide robotSide, LegJointName legJointName);

   public abstract String getArmJointName(RobotSide robotSide, ArmJointName armJointName);

   public abstract String getNeckJointName(NeckJointName neckJointName);

   public abstract String getSpineJointName(SpineJointName spineJointName);
}