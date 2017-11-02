package us.ihmc.valkyrie.fingers;

import java.util.EnumMap;

import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class ValkyrieFingerPoseDefinitions
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
         
         closedDesiredDefinition.put(ValkyrieHandJointName.ThumbRoll, 0.0);
//         closedDesiredDefinition.put(ValkyrieFingerJoint.ThumbRoll, ValkyrieFingerJointLimits.getFullyFlexedPositionLimit(robotSide, ValkyrieFingerJoint.ThumbRoll));
//         closedDesiredDefinition.put(ValkyrieRealRobotFingerJoint.Thumb, ValkyrieFingerJointLimits.getFullyFlexedPositionLimit(robotSide, ValkyrieFingerJoint.Thumb) / 2.0);
//         closedDesiredDefinition.put(ValkyrieRealRobotFingerJoint.Index, ValkyrieFingerJointLimits.getFullyFlexedPositionLimit(robotSide, ValkyrieFingerJoint.Index));
//         closedDesiredDefinition.put(ValkyrieRealRobotFingerJoint.Middle, ValkyrieFingerJointLimits.getFullyFlexedPositionLimit(robotSide, ValkyrieRealRobotFingerJoint.Middle));
//         closedDesiredDefinition.put(ValkyrieRealRobotFingerJoint.Pinky, ValkyrieFingerJointLimits.getFullyFlexedPositionLimit(robotSide, ValkyrieRealRobotFingerJoint.Pinky));
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
