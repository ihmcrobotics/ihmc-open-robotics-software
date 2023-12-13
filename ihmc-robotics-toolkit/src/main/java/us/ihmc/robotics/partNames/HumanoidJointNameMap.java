package us.ihmc.robotics.partNames;

import org.apache.commons.lang3.tuple.ImmutablePair;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.controllers.pidGains.implementations.YoPDGains;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.List;

public interface HumanoidJointNameMap extends LeggedJointNameMap<RobotSide>
{
   ImmutablePair<RobotSide, ArmJointName> getArmJointName(String jointName);

   default String getJointBeforeHandName(RobotSide robotSide)
   {
      if (getNameOfJointBeforeHands() == null)
         return null;
      else
         return getNameOfJointBeforeHands().get(robotSide);
   }

   SideDependentList<String> getNameOfJointBeforeHands();

   RigidBodyTransform getHandControlFrameToWristTransform(RobotSide robotSide);

   String getPelvisName();

   String getChestName();

   @Override
   default RobotSide[] getRobotSegments()
   {
      return RobotSide.values;
   }

   @Override
   default String getRootBodyName()
   {
      return getPelvisName();
   }

   String getNameOfJointBeforeChest();

   String[] getOrderedJointNames();

   String getLegJointName(RobotSide robotSide, LegJointName legJointName);

   String getArmJointName(RobotSide robotSide, ArmJointName armJointName);

   String getNeckJointName(NeckJointName neckJointName);

   String getSpineJointName(SpineJointName spineJointName);

   String[] getPositionControlledJointsForSimulation();

   default List<ImmutablePair<String, YoPDGains>> getPassiveJointNameWithGains(YoRegistry registry)
   {
      return null;
   }

   String getHandName(RobotSide robotSide);

   String getForearmName(RobotSide robotSide);

   String getFootName(RobotSide robotSide);

   default List<String> getNeckJointNamesAsStrings()
   {
      List<String> neckJointNames = new ArrayList<>();
      for (NeckJointName jointName : getNeckJointNames())
      {
         neckJointNames.add(getNeckJointName(jointName));
      }
      return neckJointNames;
   }

   default List<String> getSpineJointNamesAsStrings()
   {
      List<String> spineJointNames = new ArrayList<>();
      for (SpineJointName jointName : getSpineJointNames())
      {
         spineJointNames.add(getSpineJointName(jointName));
      }
      return spineJointNames;
   }

   default List<String> getArmJointNamesAsStrings()
   {
      List<String> armJointNames = new ArrayList<>();
      for (RobotSide robotSide : RobotSide.values)
      {
         armJointNames.addAll(getArmJointNamesAsStrings(robotSide));
      }
      return armJointNames;
   }

   default List<String> getArmJointNamesAsStrings(RobotSide robotSide)
   {
      List<String> armJointNames = new ArrayList<>();

      // Must iterate in kinematic joint order
      // Iterating over Set<Enum>#values will reorder the entries
      for (ArmJointName jointName : getArmJointNames())
      {
         String armJointName = getArmJointName(robotSide, jointName);
         if (armJointName != null) // Account for asymetrical arms
         {
            armJointNames.add(armJointName);
         }
      }
      return armJointNames;
   }

   default ArmJointName[] getArmJointNames(RobotSide robotSide)
   {
      ArrayList<ArmJointName> armJointNames = new ArrayList<>();
      for (String armJointNamesAsString : getArmJointNamesAsStrings(robotSide))
      {
         armJointNames.add(getArmJointName(armJointNamesAsString).getValue());
      }
      return armJointNames.toArray(new ArmJointName[armJointNames.size()]);
   }

   default List<String> getLegJointNamesAsStrings()
   {
      List<String> legJointNames = new ArrayList<>();
      for (RobotSide robotSide : RobotSide.values)
      {
         legJointNames.addAll(getLegJointNamesAsStrings(robotSide));
      }
      return legJointNames;
   }

   default List<String> getLegJointNamesAsStrings(RobotSide robotSide)
   {
      List<String> legJointNames = new ArrayList<>();
      for (LegJointName jointName : getLegJointNames())
      {
         legJointNames.add(getLegJointName(robotSide, jointName));
      }
      return legJointNames;
   }

   default List<String> getLeftAndRightJointNames(LegJointName legJointName)
   {
      List<String> jointNames = new ArrayList<>();
      for (RobotSide side : RobotSide.values)
      {
         jointNames.add(getLegJointName(side, legJointName));
      }
      return jointNames;
   }

   default List<String> getLeftAndRightJointNames(ArmJointName armJointName)
   {
      List<String> jointNames = new ArrayList<>();
      for (RobotSide side : RobotSide.values)
      {
         jointNames.add(getArmJointName(side, armJointName));
      }
      return jointNames;
   }

   default List<String> getHandNames()
   {
      List<String> names = new ArrayList<>();
      for (RobotSide robotSide : RobotSide.values)
      {
         names.add(getHandName(robotSide));
      }
      return names;
   }

   default List<String> getFootNames()
   {
      List<String> names = new ArrayList<>();
      for (RobotSide robotSide : RobotSide.values)
      {
         names.add(getFootName(robotSide));
      }
      return names;
   }
}
