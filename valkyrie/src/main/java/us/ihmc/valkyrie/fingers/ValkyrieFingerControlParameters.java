package us.ihmc.valkyrie.fingers;

import java.util.EnumMap;

import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class ValkyrieFingerControlParameters
{
   private static final SideDependentList<EnumMap<ValkyrieFingerMotorName, Double>> openDesiredFingerMotorPositions = SideDependentList.createListOfEnumMaps(ValkyrieFingerMotorName.class);
   private static final SideDependentList<EnumMap<ValkyrieFingerMotorName, Double>> closedDesiredFingerMotorPositions = SideDependentList.createListOfEnumMaps(ValkyrieFingerMotorName.class);

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
         EnumMap<ValkyrieFingerMotorName, Double> openDesiredMotorPosition = openDesiredFingerMotorPositions.get(robotSide);

         openDesiredMotorPosition.put(ValkyrieFingerMotorName.ThumbMotorRoll, 0.0);
         openDesiredMotorPosition.put(ValkyrieFingerMotorName.ThumbMotorPitch1, 0.0);
         openDesiredMotorPosition.put(ValkyrieFingerMotorName.ThumbMotorPitch2, 0.0);
         openDesiredMotorPosition.put(ValkyrieFingerMotorName.IndexFingerMotorPitch1, 0.0);
         openDesiredMotorPosition.put(ValkyrieFingerMotorName.MiddleFingerMotorPitch1, 0.0);
         openDesiredMotorPosition.put(ValkyrieFingerMotorName.PinkyMotorPitch1, 0.0);
      }
   }

   private static void createClosedDefinition()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         EnumMap<ValkyrieFingerMotorName, Double> closedDesiredFingerMotorPosition = closedDesiredFingerMotorPositions.get(robotSide);

         closedDesiredFingerMotorPosition.put(ValkyrieFingerMotorName.ThumbMotorRoll, 1.7);
         closedDesiredFingerMotorPosition.put(ValkyrieFingerMotorName.ThumbMotorPitch1, 2.0);
         closedDesiredFingerMotorPosition.put(ValkyrieFingerMotorName.ThumbMotorPitch2, 2.0);
         closedDesiredFingerMotorPosition.put(ValkyrieFingerMotorName.IndexFingerMotorPitch1, 3.6);
         closedDesiredFingerMotorPosition.put(ValkyrieFingerMotorName.MiddleFingerMotorPitch1, 3.6);
         closedDesiredFingerMotorPosition.put(ValkyrieFingerMotorName.PinkyMotorPitch1, 3.6);
      }
   }

   public static EnumMap<ValkyrieFingerMotorName, Double> getOpenDesiredFingerMotorPosition(RobotSide robotSide)
   {
      return openDesiredFingerMotorPositions.get(robotSide);
   }

   public static EnumMap<ValkyrieFingerMotorName, Double> getClosedDesiredFingerMotorPosition(RobotSide robotSide)
   {
      return closedDesiredFingerMotorPositions.get(robotSide);
   }

   private static void createOpenHandJointDefinition()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         EnumMap<ValkyrieHandJointName, Double> openDesiredHandJointDefinition = openDesiredHandJointDefinitions.get(robotSide);

         openDesiredHandJointDefinition.put(ValkyrieHandJointName.ThumbRoll, 0.0);
         openDesiredHandJointDefinition.put(ValkyrieHandJointName.ThumbPitch1, 0.0);
         openDesiredHandJointDefinition.put(ValkyrieHandJointName.ThumbPitch2, 0.0);
         openDesiredHandJointDefinition.put(ValkyrieHandJointName.ThumbPitch3, 0.0);
         openDesiredHandJointDefinition.put(ValkyrieHandJointName.IndexFingerPitch1, 0.0);
         openDesiredHandJointDefinition.put(ValkyrieHandJointName.IndexFingerPitch2, 0.0);
         openDesiredHandJointDefinition.put(ValkyrieHandJointName.IndexFingerPitch3, 0.0);
         openDesiredHandJointDefinition.put(ValkyrieHandJointName.MiddleFingerPitch1, 0.0);
         openDesiredHandJointDefinition.put(ValkyrieHandJointName.MiddleFingerPitch2, 0.0);
         openDesiredHandJointDefinition.put(ValkyrieHandJointName.MiddleFingerPitch3, 0.0);
         openDesiredHandJointDefinition.put(ValkyrieHandJointName.PinkyPitch1, 0.0);
         openDesiredHandJointDefinition.put(ValkyrieHandJointName.PinkyPitch2, 0.0);
         openDesiredHandJointDefinition.put(ValkyrieHandJointName.PinkyPitch3, 0.0);
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

   /**
    * @param desiredGraspingState : if this is close to 0, return value will close to 'opened angle'.
    * @return
    */
   public static double getDesiredHandJoint(RobotSide robotSide, ValkyrieHandJointName valkyrieHandJointName, double desiredGraspingState)
   {
      double opened = getOpenedDesiredHandJointDefinition(robotSide).get(valkyrieHandJointName);
      double closed = getClosedDesiredHandJointDefinition(robotSide).get(valkyrieHandJointName);

      return opened + (closed - opened) * desiredGraspingState;
   }

   /**
    * @param desiredGraspingState : if this is close to 0, return value will close to 'opened angle'.
    * @return
    */
   public static double getDesiredFingerMotorPosition(RobotSide robotSide, ValkyrieFingerMotorName valkyrieFingerMotorName, double desiredGraspingState)
   {
      double opened = getOpenDesiredFingerMotorPosition(robotSide).get(valkyrieFingerMotorName);
      double closed = getClosedDesiredFingerMotorPosition(robotSide).get(valkyrieFingerMotorName);

      return opened + (closed - opened) * desiredGraspingState;
   }
}
