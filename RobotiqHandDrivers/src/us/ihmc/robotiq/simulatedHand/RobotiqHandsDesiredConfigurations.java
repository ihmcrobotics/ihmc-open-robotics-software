package us.ihmc.robotiq.simulatedHand;

import java.util.EnumMap;

import us.ihmc.robotiq.model.RobotiqHandModel.RobotiqHandJointNameMinimal;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.robotSide.SideDependentList;

public class RobotiqHandsDesiredConfigurations
{
   private static final SideDependentList<EnumMap<RobotiqHandJointNameMinimal, Double>> closedHandsDesiredConfigurations = SideDependentList.createListOfEnumMaps(RobotiqHandJointNameMinimal.class);
   private static final SideDependentList<EnumMap<RobotiqHandJointNameMinimal, Double>> openHandsDesiredConfigurations = SideDependentList.createListOfEnumMaps(RobotiqHandJointNameMinimal.class);
   
   static
   {
      createCloseHandConfiguration();
      createOpenHandConfiguration();
   }

   private static void createCloseHandConfiguration()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         EnumMap<RobotiqHandJointNameMinimal, Double> handDesiredConfigurations = closedHandsDesiredConfigurations.get(robotSide);
         
         handDesiredConfigurations.put(RobotiqHandJointNameMinimal.PALM_FINGER_1_JOINT, 0.0);
         handDesiredConfigurations.put(RobotiqHandJointNameMinimal.FINGER_1_JOINT_1, 1.12);
         handDesiredConfigurations.put(RobotiqHandJointNameMinimal.FINGER_1_JOINT_2, 1.57);
         handDesiredConfigurations.put(RobotiqHandJointNameMinimal.FINGER_1_JOINT_3, 0.75);
         handDesiredConfigurations.put(RobotiqHandJointNameMinimal.PALM_FINGER_2_JOINT, 0.0);
         handDesiredConfigurations.put(RobotiqHandJointNameMinimal.FINGER_2_JOINT_1, 1.12);
         handDesiredConfigurations.put(RobotiqHandJointNameMinimal.FINGER_2_JOINT_2, 1.57);
         handDesiredConfigurations.put(RobotiqHandJointNameMinimal.FINGER_2_JOINT_3, 0.75);
         handDesiredConfigurations.put(RobotiqHandJointNameMinimal.FINGER_MIDDLE_JOINT_1, 1.22);
         handDesiredConfigurations.put(RobotiqHandJointNameMinimal.FINGER_MIDDLE_JOINT_2, 1.57);
         handDesiredConfigurations.put(RobotiqHandJointNameMinimal.FINGER_MIDDLE_JOINT_3, 1.04);
      }
   }

   private static void createOpenHandConfiguration()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         EnumMap<RobotiqHandJointNameMinimal, Double> handDesiredConfigurations = openHandsDesiredConfigurations.get(robotSide);
         
         handDesiredConfigurations.put(RobotiqHandJointNameMinimal.PALM_FINGER_1_JOINT, 0.0);
         handDesiredConfigurations.put(RobotiqHandJointNameMinimal.FINGER_1_JOINT_1, -0.09);
         handDesiredConfigurations.put(RobotiqHandJointNameMinimal.FINGER_1_JOINT_2, 0.0);
         handDesiredConfigurations.put(RobotiqHandJointNameMinimal.FINGER_1_JOINT_3, -0.95);
         handDesiredConfigurations.put(RobotiqHandJointNameMinimal.PALM_FINGER_2_JOINT, 0.0);
         handDesiredConfigurations.put(RobotiqHandJointNameMinimal.FINGER_2_JOINT_1, -0.09);
         handDesiredConfigurations.put(RobotiqHandJointNameMinimal.FINGER_2_JOINT_2, 0.0);
         handDesiredConfigurations.put(RobotiqHandJointNameMinimal.FINGER_2_JOINT_3, -0.95);
         handDesiredConfigurations.put(RobotiqHandJointNameMinimal.FINGER_MIDDLE_JOINT_1, 0.0);
         handDesiredConfigurations.put(RobotiqHandJointNameMinimal.FINGER_MIDDLE_JOINT_2, 0.0);
         handDesiredConfigurations.put(RobotiqHandJointNameMinimal.FINGER_MIDDLE_JOINT_3, -0.66);
      }
   }

   public static EnumMap<RobotiqHandJointNameMinimal, Double> getClosedHandDesiredConfiguration(RobotSide robotSide)
   {
      return closedHandsDesiredConfigurations.get(robotSide);
   }

   public static EnumMap<RobotiqHandJointNameMinimal, Double> getOpenHandDesiredConfiguration(RobotSide robotSide)
   {
      return openHandsDesiredConfigurations.get(robotSide);
   }
}
