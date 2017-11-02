package us.ihmc.valkyrie.fingers;

import java.util.EnumMap;

import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class ValkyrieFingerPoseDefinitions
{
   private static final SideDependentList<EnumMap<ValkyrieFingerJoint, Double>> openDesiredDefinitions = SideDependentList.createListOfEnumMaps(ValkyrieFingerJoint.class);
   private static final SideDependentList<EnumMap<ValkyrieFingerJoint, Double>> closedDesiredDefinitions = SideDependentList.createListOfEnumMaps(ValkyrieFingerJoint.class);
   
   static
   {
      createOpenDefinition();
      createClosedDefinition();
   }
   
   private static void createOpenDefinition()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         EnumMap<ValkyrieFingerJoint, Double> openDesiredDefinition = openDesiredDefinitions.get(robotSide);
         
         openDesiredDefinition.put(ValkyrieFingerJoint.ThumbRoll, 0.0);
         openDesiredDefinition.put(ValkyrieFingerJoint.ThumbPitch1, 0.0);
         openDesiredDefinition.put(ValkyrieFingerJoint.IndexFingerPitch1, 0.0);
         openDesiredDefinition.put(ValkyrieFingerJoint.MiddleFingerPitch1, 0.0);
         openDesiredDefinition.put(ValkyrieFingerJoint.PinkyPitch1, 0.0);
      }
   }
   
   private static void createClosedDefinition()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         EnumMap<ValkyrieFingerJoint, Double> closedDesiredDefinition = closedDesiredDefinitions.get(robotSide);
         
         closedDesiredDefinition.put(ValkyrieFingerJoint.ThumbRoll, 0.0);
//         closedDesiredDefinition.put(ValkyrieFingerJoint.ThumbRoll, ValkyrieFingerJointLimits.getFullyFlexedPositionLimit(robotSide, ValkyrieFingerJoint.ThumbRoll));
//         closedDesiredDefinition.put(ValkyrieRealRobotFingerJoint.Thumb, ValkyrieFingerJointLimits.getFullyFlexedPositionLimit(robotSide, ValkyrieFingerJoint.Thumb) / 2.0);
//         closedDesiredDefinition.put(ValkyrieRealRobotFingerJoint.Index, ValkyrieFingerJointLimits.getFullyFlexedPositionLimit(robotSide, ValkyrieFingerJoint.Index));
//         closedDesiredDefinition.put(ValkyrieRealRobotFingerJoint.Middle, ValkyrieFingerJointLimits.getFullyFlexedPositionLimit(robotSide, ValkyrieRealRobotFingerJoint.Middle));
//         closedDesiredDefinition.put(ValkyrieRealRobotFingerJoint.Pinky, ValkyrieFingerJointLimits.getFullyFlexedPositionLimit(robotSide, ValkyrieRealRobotFingerJoint.Pinky));
      }
   }
   
   public static EnumMap<ValkyrieFingerJoint, Double> getOpenDesiredDefinition(RobotSide robotSide)
   {
      return openDesiredDefinitions.get(robotSide);
   }
   
   public static EnumMap<ValkyrieFingerJoint, Double> getClosedDesiredDefinition(RobotSide robotSide)
   {
      return closedDesiredDefinitions.get(robotSide);
   }
}
