package us.ihmc.wholeBodyController;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.lang3.tuple.ImmutablePair;

import us.ihmc.robotics.controllers.pidGains.implementations.YoPDGains;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

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

   public List<ImmutablePair<String, YoPDGains>> getPassiveJointNameWithGains(YoVariableRegistry registry);

   public default List<String> getNeckJointNamesAsStrings()
   {
      List<String> neckJointNames = new ArrayList<>();
      for (NeckJointName jointName : getNeckJointNames())
      {
         neckJointNames.add(getNeckJointName(jointName));
      }
      return neckJointNames;
   }

   public default List<String> getSpineJointNamesAsStrings()
   {
      List<String> spineJointNames = new ArrayList<>();
      for (SpineJointName jointName : getSpineJointNames())
      {
         spineJointNames.add(getSpineJointName(jointName));
      }
      return spineJointNames;
   }

   public default List<String> getArmJointNamesAsStrings()
   {
      List<String> armJointNames = new ArrayList<>();
      for (RobotSide robotSide : RobotSide.values)
      {
         armJointNames.addAll(getArmJointNamesAsStrings(robotSide));
      }
      return armJointNames;
   }

   public default List<String> getArmJointNamesAsStrings(RobotSide robotSide)
   {
      List<String> armJointNames = new ArrayList<>();
      for (ArmJointName jointName : getArmJointNames())
      {
         armJointNames.add(getArmJointName(robotSide, jointName));
      }
      return armJointNames;
   }

   public default List<String> getLegJointNamesAsStrings()
   {
      List<String> legJointNames = new ArrayList<>();
      for (RobotSide robotSide : RobotSide.values)
      {
         legJointNames.addAll(getLegJointNamesAsStrings(robotSide));
      }
      return legJointNames;
   }

   public default List<String> getLegJointNamesAsStrings(RobotSide robotSide)
   {
      List<String> legJointNames = new ArrayList<>();
      for (LegJointName jointName : getLegJointNames())
      {
         legJointNames.add(getLegJointName(robotSide, jointName));
      }
      return legJointNames;
   }

   public default List<String> getLeftAndRightJointNames(LegJointName legJointName)
   {
      List<String> jointNames = new ArrayList<>();
      for (RobotSide side : RobotSide.values)
      {
         jointNames.add(getLegJointName(side, legJointName));
      }
      return jointNames;
   }

   public default List<String> getLeftAndRightJointNames(ArmJointName armJointName)
   {
      List<String> jointNames = new ArrayList<>();
      for (RobotSide side : RobotSide.values)
      {
         jointNames.add(getArmJointName(side, armJointName));
      }
      return jointNames;
   }
}