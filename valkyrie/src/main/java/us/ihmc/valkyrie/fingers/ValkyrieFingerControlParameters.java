package us.ihmc.valkyrie.fingers;

import java.util.EnumMap;

import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class ValkyrieFingerControlParameters
{
   private static final SideDependentList<EnumMap<ValkyrieFingerMotorName, Double>> openDesiredDefinitions = SideDependentList.createListOfEnumMaps(ValkyrieFingerMotorName.class);
   private static final SideDependentList<EnumMap<ValkyrieFingerMotorName, Double>> closedDesiredDefinitions = SideDependentList.createListOfEnumMaps(ValkyrieFingerMotorName.class);

   private static final SideDependentList<EnumMap<ValkyrieHandJointName, Double>> openDesiredHandJointDefinitions = SideDependentList.createListOfEnumMaps(ValkyrieHandJointName.class);
   private static final SideDependentList<EnumMap<ValkyrieHandJointName, Double>> closedDesiredHandJointDefinitions = SideDependentList.createListOfEnumMaps(ValkyrieHandJointName.class);

   static
   {
      createOpenDefinition();
      createClosedDefinition();
      createOpenHandJointDefinition();
      createClosedHandJointDefinition();
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

   private static void createOpenHandJointDefinition()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         EnumMap<ValkyrieHandJointName, Double> openDesiredDefinition = openDesiredHandJointDefinitions.get(robotSide);

         openDesiredDefinition.put(ValkyrieHandJointName.ThumbRoll, 0.0);
         openDesiredDefinition.put(ValkyrieHandJointName.ThumbPitch1, 0.0);
         openDesiredDefinition.put(ValkyrieHandJointName.ThumbPitch2, 0.0);
         openDesiredDefinition.put(ValkyrieHandJointName.ThumbPitch3, 0.0);
         openDesiredDefinition.put(ValkyrieHandJointName.IndexFingerPitch1, 0.0);
         openDesiredDefinition.put(ValkyrieHandJointName.IndexFingerPitch2, 0.0);
         openDesiredDefinition.put(ValkyrieHandJointName.IndexFingerPitch3, 0.0);
         openDesiredDefinition.put(ValkyrieHandJointName.MiddleFingerPitch1, 0.0);
         openDesiredDefinition.put(ValkyrieHandJointName.MiddleFingerPitch2, 0.0);
         openDesiredDefinition.put(ValkyrieHandJointName.MiddleFingerPitch3, 0.0);
         openDesiredDefinition.put(ValkyrieHandJointName.PinkyPitch1, 0.0);
         openDesiredDefinition.put(ValkyrieHandJointName.PinkyPitch2, 0.0);
         openDesiredDefinition.put(ValkyrieHandJointName.PinkyPitch3, 0.0);
      }
   }

   private static void createClosedHandJointDefinition()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         EnumMap<ValkyrieHandJointName, Double> closedDesiredDefinition = closedDesiredHandJointDefinitions.get(robotSide);

         closedDesiredDefinition.put(ValkyrieHandJointName.ThumbRoll, 1.5);
         closedDesiredDefinition.put(ValkyrieHandJointName.ThumbPitch1, robotSide.negateIfLeftSide(1.5));
         closedDesiredDefinition.put(ValkyrieHandJointName.ThumbPitch2, robotSide.negateIfLeftSide(1.5));
         closedDesiredDefinition.put(ValkyrieHandJointName.ThumbPitch3, robotSide.negateIfLeftSide(1.5));
         closedDesiredDefinition.put(ValkyrieHandJointName.IndexFingerPitch1, robotSide.negateIfLeftSide(1.5));
         closedDesiredDefinition.put(ValkyrieHandJointName.IndexFingerPitch2, robotSide.negateIfLeftSide(1.5));
         closedDesiredDefinition.put(ValkyrieHandJointName.IndexFingerPitch3, robotSide.negateIfLeftSide(1.5));
         closedDesiredDefinition.put(ValkyrieHandJointName.MiddleFingerPitch1, robotSide.negateIfLeftSide(1.5));
         closedDesiredDefinition.put(ValkyrieHandJointName.MiddleFingerPitch2, robotSide.negateIfLeftSide(1.5));
         closedDesiredDefinition.put(ValkyrieHandJointName.MiddleFingerPitch3, robotSide.negateIfLeftSide(1.5));
         closedDesiredDefinition.put(ValkyrieHandJointName.PinkyPitch1, robotSide.negateIfLeftSide(1.5));
         closedDesiredDefinition.put(ValkyrieHandJointName.PinkyPitch2, robotSide.negateIfLeftSide(1.5));
         closedDesiredDefinition.put(ValkyrieHandJointName.PinkyPitch3, robotSide.negateIfLeftSide(1.5));
      }
   }

   public static EnumMap<ValkyrieHandJointName, Double> getOpenedDesiredHandJointDefinition(RobotSide robotSide)
   {
      return openDesiredHandJointDefinitions.get(robotSide);
   }

   public static EnumMap<ValkyrieHandJointName, Double> getClosedDesiredHandJointDefinition(RobotSide robotSide)
   {
      return closedDesiredHandJointDefinitions.get(robotSide);
   }
}
