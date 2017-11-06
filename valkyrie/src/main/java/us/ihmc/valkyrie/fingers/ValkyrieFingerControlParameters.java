package us.ihmc.valkyrie.fingers;

import java.util.EnumMap;

import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class ValkyrieFingerControlParameters
{
   private static final SideDependentList<EnumMap<ValkyrieHandJointName, Double>> openDesiredDefinitions = SideDependentList.createListOfEnumMaps(ValkyrieHandJointName.class);
   private static final SideDependentList<EnumMap<ValkyrieHandJointName, Double>> closedDesiredDefinitions = SideDependentList.createListOfEnumMaps(ValkyrieHandJointName.class);

   static
   {
      createOpenDefinition();
      createClosedDefinition();
   }

   private static void createOpenDefinition()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         EnumMap<ValkyrieHandJointName, Double> openDesiredDefinition = openDesiredDefinitions.get(robotSide);

         openDesiredDefinition.put(ValkyrieHandJointName.ThumbRoll, 0.0);
         openDesiredDefinition.put(ValkyrieHandJointName.ThumbPitch1, 0.0);
         openDesiredDefinition.put(ValkyrieHandJointName.ThumbPitch2, 0.0);
         openDesiredDefinition.put(ValkyrieHandJointName.IndexFingerPitch1, 0.0);
         openDesiredDefinition.put(ValkyrieHandJointName.MiddleFingerPitch1, 0.0);
         openDesiredDefinition.put(ValkyrieHandJointName.PinkyPitch1, 0.0);
      }
   }

   private static void createClosedDefinition()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         EnumMap<ValkyrieHandJointName, Double> closedDesiredDefinition = closedDesiredDefinitions.get(robotSide);

         closedDesiredDefinition.put(ValkyrieHandJointName.ThumbRoll, robotSide.negateIfLeftSide(0.0));
         closedDesiredDefinition.put(ValkyrieHandJointName.ThumbPitch1, robotSide.negateIfLeftSide(0.9));
         closedDesiredDefinition.put(ValkyrieHandJointName.ThumbPitch2, robotSide.negateIfLeftSide(0.9));
         closedDesiredDefinition.put(ValkyrieHandJointName.IndexFingerPitch1, robotSide.negateIfLeftSide(1.9));
         closedDesiredDefinition.put(ValkyrieHandJointName.MiddleFingerPitch1, robotSide.negateIfLeftSide(1.9));
         closedDesiredDefinition.put(ValkyrieHandJointName.PinkyPitch1, robotSide.negateIfLeftSide(1.9));
      }
   }

   public static EnumMap<ValkyrieHandJointName, Double> getOpenDesiredDefinition(RobotSide robotSide)
   {
      return openDesiredDefinitions.get(robotSide);
   }

   public static EnumMap<ValkyrieHandJointName, Double> getClosedDesiredDefinition(RobotSide robotSide)
   {
      return closedDesiredDefinitions.get(robotSide);
   }
}
