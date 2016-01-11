package us.ihmc.valkyrie.fingers;

import java.util.EnumMap;

import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class ValkyrieFingerPoseDefinitions
{
   private static final SideDependentList<EnumMap<ValkyrieRealRobotFingerJoint, Double>> openDesiredDefinitions = SideDependentList.createListOfEnumMaps(ValkyrieRealRobotFingerJoint.class);
   private static final SideDependentList<EnumMap<ValkyrieRealRobotFingerJoint, Double>> closedDesiredDefinitions = SideDependentList.createListOfEnumMaps(ValkyrieRealRobotFingerJoint.class);
   
   static
   {
      createOpenDefinition();
      createClosedDefinition();
   }
   
   private static void createOpenDefinition()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         EnumMap<ValkyrieRealRobotFingerJoint, Double> openDesiredDefinition = openDesiredDefinitions.get(robotSide);
         
         openDesiredDefinition.put(ValkyrieRealRobotFingerJoint.ThumbRoll, 0.0);
         openDesiredDefinition.put(ValkyrieRealRobotFingerJoint.Thumb, 0.0);
         openDesiredDefinition.put(ValkyrieRealRobotFingerJoint.Index, 0.0);
         openDesiredDefinition.put(ValkyrieRealRobotFingerJoint.Middle, 0.0);
         openDesiredDefinition.put(ValkyrieRealRobotFingerJoint.Pinky, 0.0);
      }
   }
   
   private static void createClosedDefinition()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         EnumMap<ValkyrieRealRobotFingerJoint, Double> closedDesiredDefinition = closedDesiredDefinitions.get(robotSide);
         
         closedDesiredDefinition.put(ValkyrieRealRobotFingerJoint.ThumbRoll, 0.0);
//         closedDesiredDefinition.put(ValkyrieRealRobotFingerJoint.ThumbRoll, ValkyrieFingerJointLimits.getFullyFlexedPositionLimit(robotSide, ValkyrieRealRobotFingerJoint.ThumbRoll));
         closedDesiredDefinition.put(ValkyrieRealRobotFingerJoint.Thumb, ValkyrieFingerJointLimits.getFullyFlexedPositionLimit(robotSide, ValkyrieRealRobotFingerJoint.Thumb) / 2.0);
         closedDesiredDefinition.put(ValkyrieRealRobotFingerJoint.Index, ValkyrieFingerJointLimits.getFullyFlexedPositionLimit(robotSide, ValkyrieRealRobotFingerJoint.Index));
         closedDesiredDefinition.put(ValkyrieRealRobotFingerJoint.Middle, ValkyrieFingerJointLimits.getFullyFlexedPositionLimit(robotSide, ValkyrieRealRobotFingerJoint.Middle));
         closedDesiredDefinition.put(ValkyrieRealRobotFingerJoint.Pinky, ValkyrieFingerJointLimits.getFullyFlexedPositionLimit(robotSide, ValkyrieRealRobotFingerJoint.Pinky));
      }
   }
   
   public static EnumMap<ValkyrieRealRobotFingerJoint, Double> getOpenDesiredDefinition(RobotSide robotSide)
   {
      return openDesiredDefinitions.get(robotSide);
   }
   
   public static EnumMap<ValkyrieRealRobotFingerJoint, Double> getClosedDesiredDefinition(RobotSide robotSide)
   {
      return closedDesiredDefinitions.get(robotSide);
   }
}
