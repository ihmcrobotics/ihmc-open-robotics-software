package us.ihmc.valkyrie.hands.athena;

import java.util.EnumMap;

import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.valkyrie.hands.athena.AthenaHandModel.AthenaFingerMotorName;
import us.ihmc.valkyrie.hands.athena.AthenaHandModel.AthenaJointName;

public class AthenaFingerControlParameters
{
   private static final SideDependentList<EnumMap<AthenaFingerMotorName, Double>> openDesiredFingerMotorPositions = SideDependentList.createListOfEnumMaps(AthenaFingerMotorName.class);
   private static final SideDependentList<EnumMap<AthenaFingerMotorName, Double>> closedDesiredFingerMotorPositions = SideDependentList.createListOfEnumMaps(AthenaFingerMotorName.class);

   private static final SideDependentList<EnumMap<AthenaJointName, Double>> openDesiredHandJointDefinitions = SideDependentList.createListOfEnumMaps(AthenaJointName.class);
   private static final SideDependentList<EnumMap<AthenaJointName, Double>> closedDesiredHandJointDefinitions = SideDependentList.createListOfEnumMaps(AthenaJointName.class);

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
         EnumMap<AthenaFingerMotorName, Double> openDesiredMotorPosition = openDesiredFingerMotorPositions.get(robotSide);

         openDesiredMotorPosition.put(AthenaFingerMotorName.ThumbMotorRoll, 0.0);
         openDesiredMotorPosition.put(AthenaFingerMotorName.ThumbMotorPitch1, 0.0);
         openDesiredMotorPosition.put(AthenaFingerMotorName.ThumbMotorPitch2, 0.0);
         openDesiredMotorPosition.put(AthenaFingerMotorName.IndexFingerMotorPitch1, 0.0);
         openDesiredMotorPosition.put(AthenaFingerMotorName.MiddleFingerMotorPitch1, 0.0);
         openDesiredMotorPosition.put(AthenaFingerMotorName.PinkyMotorPitch1, 0.0);
      }
   }

   private static void createClosedDefinition()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         EnumMap<AthenaFingerMotorName, Double> closedDesiredFingerMotorPosition = closedDesiredFingerMotorPositions.get(robotSide);

         closedDesiredFingerMotorPosition.put(AthenaFingerMotorName.ThumbMotorRoll, 1.64);
         closedDesiredFingerMotorPosition.put(AthenaFingerMotorName.ThumbMotorPitch1, 2.0);
         closedDesiredFingerMotorPosition.put(AthenaFingerMotorName.ThumbMotorPitch2, 2.0);
         closedDesiredFingerMotorPosition.put(AthenaFingerMotorName.IndexFingerMotorPitch1, 3.6);
         closedDesiredFingerMotorPosition.put(AthenaFingerMotorName.MiddleFingerMotorPitch1, 3.6);
         closedDesiredFingerMotorPosition.put(AthenaFingerMotorName.PinkyMotorPitch1, 3.6);
      }
   }

   public static EnumMap<AthenaFingerMotorName, Double> getOpenDesiredFingerMotorPosition(RobotSide robotSide)
   {
      return openDesiredFingerMotorPositions.get(robotSide);
   }

   public static EnumMap<AthenaFingerMotorName, Double> getClosedDesiredFingerMotorPosition(RobotSide robotSide)
   {
      return closedDesiredFingerMotorPositions.get(robotSide);
   }

   private static void createOpenHandJointDefinition()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         EnumMap<AthenaJointName, Double> openDesiredHandJointDefinition = openDesiredHandJointDefinitions.get(robotSide);

         openDesiredHandJointDefinition.put(AthenaJointName.ThumbRoll, 0.0);
         openDesiredHandJointDefinition.put(AthenaJointName.ThumbPitch1, 0.0);
         openDesiredHandJointDefinition.put(AthenaJointName.ThumbPitch2, 0.0);
         openDesiredHandJointDefinition.put(AthenaJointName.ThumbPitch3, 0.0);
         openDesiredHandJointDefinition.put(AthenaJointName.IndexFingerPitch1, 0.0);
         openDesiredHandJointDefinition.put(AthenaJointName.IndexFingerPitch2, 0.0);
         openDesiredHandJointDefinition.put(AthenaJointName.IndexFingerPitch3, 0.0);
         openDesiredHandJointDefinition.put(AthenaJointName.MiddleFingerPitch1, 0.0);
         openDesiredHandJointDefinition.put(AthenaJointName.MiddleFingerPitch2, 0.0);
         openDesiredHandJointDefinition.put(AthenaJointName.MiddleFingerPitch3, 0.0);
         openDesiredHandJointDefinition.put(AthenaJointName.PinkyPitch1, 0.0);
         openDesiredHandJointDefinition.put(AthenaJointName.PinkyPitch2, 0.0);
         openDesiredHandJointDefinition.put(AthenaJointName.PinkyPitch3, 0.0);
      }
   }

   private static void createClosedHandJointDefinition()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         EnumMap<AthenaJointName, Double> closedDesiredDefinition = closedDesiredHandJointDefinitions.get(robotSide);

         closedDesiredDefinition.put(AthenaJointName.ThumbRoll, 1.5);
         closedDesiredDefinition.put(AthenaJointName.ThumbPitch1, robotSide.negateIfLeftSide(1.5));
         closedDesiredDefinition.put(AthenaJointName.ThumbPitch2, robotSide.negateIfLeftSide(1.5));
         closedDesiredDefinition.put(AthenaJointName.ThumbPitch3, robotSide.negateIfLeftSide(1.5));
         closedDesiredDefinition.put(AthenaJointName.IndexFingerPitch1, robotSide.negateIfLeftSide(1.5));
         closedDesiredDefinition.put(AthenaJointName.IndexFingerPitch2, robotSide.negateIfLeftSide(1.5));
         closedDesiredDefinition.put(AthenaJointName.IndexFingerPitch3, robotSide.negateIfLeftSide(1.5));
         closedDesiredDefinition.put(AthenaJointName.MiddleFingerPitch1, robotSide.negateIfLeftSide(1.5));
         closedDesiredDefinition.put(AthenaJointName.MiddleFingerPitch2, robotSide.negateIfLeftSide(1.5));
         closedDesiredDefinition.put(AthenaJointName.MiddleFingerPitch3, robotSide.negateIfLeftSide(1.5));
         closedDesiredDefinition.put(AthenaJointName.PinkyPitch1, robotSide.negateIfLeftSide(1.5));
         closedDesiredDefinition.put(AthenaJointName.PinkyPitch2, robotSide.negateIfLeftSide(1.5));
         closedDesiredDefinition.put(AthenaJointName.PinkyPitch3, robotSide.negateIfLeftSide(1.5));
      }
   }

   public static EnumMap<AthenaJointName, Double> getOpenedDesiredHandJointDefinition(RobotSide robotSide)
   {
      return openDesiredHandJointDefinitions.get(robotSide);
   }

   public static EnumMap<AthenaJointName, Double> getClosedDesiredHandJointDefinition(RobotSide robotSide)
   {
      return closedDesiredHandJointDefinitions.get(robotSide);
   }

   /**
    * @param desiredGraspingState : if this is close to 0, return value will close to 'opened angle'.
    * @return
    */
   public static double getDesiredHandJoint(RobotSide robotSide, AthenaJointName valkyrieHandJointName, double desiredGraspingState)
   {
      double opened = getOpenedDesiredHandJointDefinition(robotSide).get(valkyrieHandJointName);
      double closed = getClosedDesiredHandJointDefinition(robotSide).get(valkyrieHandJointName);

      return opened + (closed - opened) * desiredGraspingState;
   }

   /**
    * @param desiredGraspingState : if this is close to 0, return value will close to 'opened angle'.
    * @return
    */
   public static double getDesiredFingerMotorPosition(RobotSide robotSide, AthenaFingerMotorName valkyrieFingerMotorName, double desiredGraspingState)
   {
      double opened = getOpenDesiredFingerMotorPosition(robotSide).get(valkyrieFingerMotorName);
      double closed = getClosedDesiredFingerMotorPosition(robotSide).get(valkyrieFingerMotorName);

      return opened + (closed - opened) * desiredGraspingState;
   }
}
