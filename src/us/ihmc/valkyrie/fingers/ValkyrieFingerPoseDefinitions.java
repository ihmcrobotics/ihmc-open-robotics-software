package us.ihmc.valkyrie.fingers;

import java.util.EnumMap;

import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.robotSide.SideDependentList;

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
         
         closedDesiredDefinition.put(ValkyrieRealRobotFingerJoint.ThumbRoll, 1.75);
         closedDesiredDefinition.put(ValkyrieRealRobotFingerJoint.Thumb, -4.0);
         closedDesiredDefinition.put(ValkyrieRealRobotFingerJoint.Index, -4.0);
         closedDesiredDefinition.put(ValkyrieRealRobotFingerJoint.Middle, -4.0);
         closedDesiredDefinition.put(ValkyrieRealRobotFingerJoint.Pinky, -4.0);
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
