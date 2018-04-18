package us.ihmc.valkyrie.fingers;

import java.util.EnumMap;

import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class ValkyrieFingerControlParameters
{
   private static final SideDependentList<EnumMap<ValkyrieFingerMotorName, Double>> openDesiredDefinitions = SideDependentList.createListOfEnumMaps(ValkyrieFingerMotorName.class);
   private static final SideDependentList<EnumMap<ValkyrieFingerMotorName, Double>> closedDesiredDefinitions = SideDependentList.createListOfEnumMaps(ValkyrieFingerMotorName.class);

   static
   {
      createOpenDefinition();
      createClosedDefinition();
   }

   private static void createOpenDefinition()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         EnumMap<ValkyrieFingerMotorName, Double> openDesiredDefinition = openDesiredDefinitions.get(robotSide);

         openDesiredDefinition.put(ValkyrieFingerMotorName.ThumbMotorRoll, 0.0);
         openDesiredDefinition.put(ValkyrieFingerMotorName.ThumbMotorPitch1, 0.0);
         openDesiredDefinition.put(ValkyrieFingerMotorName.ThumbMotorPitch2, 0.0);
         openDesiredDefinition.put(ValkyrieFingerMotorName.IndexFingerMotorPitch1, 0.0);
         openDesiredDefinition.put(ValkyrieFingerMotorName.MiddleFingerMotorPitch1, 0.0);
         openDesiredDefinition.put(ValkyrieFingerMotorName.PinkyMotorPitch1, 0.0);
      }
   }

   private static void createClosedDefinition()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         EnumMap<ValkyrieFingerMotorName, Double> closedDesiredDefinition = closedDesiredDefinitions.get(robotSide);

         closedDesiredDefinition.put(ValkyrieFingerMotorName.ThumbMotorRoll, robotSide.negateIfLeftSide(0.0));
         closedDesiredDefinition.put(ValkyrieFingerMotorName.ThumbMotorPitch1, robotSide.negateIfLeftSide(0.9));
         closedDesiredDefinition.put(ValkyrieFingerMotorName.ThumbMotorPitch2, robotSide.negateIfLeftSide(0.9));
         closedDesiredDefinition.put(ValkyrieFingerMotorName.IndexFingerMotorPitch1, robotSide.negateIfLeftSide(1.9));
         closedDesiredDefinition.put(ValkyrieFingerMotorName.MiddleFingerMotorPitch1, robotSide.negateIfLeftSide(1.9));
         closedDesiredDefinition.put(ValkyrieFingerMotorName.PinkyMotorPitch1, robotSide.negateIfLeftSide(1.9));
      }
   }

   public static EnumMap<ValkyrieFingerMotorName, Double> getOpenDesiredDefinition(RobotSide robotSide)
   {
      return openDesiredDefinitions.get(robotSide);
   }

   public static EnumMap<ValkyrieFingerMotorName, Double> getClosedDesiredDefinition(RobotSide robotSide)
   {
      return closedDesiredDefinitions.get(robotSide);
   }
}
