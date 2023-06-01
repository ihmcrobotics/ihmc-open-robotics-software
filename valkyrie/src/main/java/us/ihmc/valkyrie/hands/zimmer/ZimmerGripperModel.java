package us.ihmc.valkyrie.hands.zimmer;

import com.google.common.base.CaseFormat;

import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandJointName;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListBasics;
import us.ihmc.valkyrie.hands.ValkyrieHandModel;
import us.ihmc.valkyrie.hands.ValkyrieHandVersion;
import us.ihmc.yoVariables.providers.DoubleProvider;

public class ZimmerGripperModel implements ValkyrieHandModel
{
   @Override
   public ZimmerJointName[] getHandJointNames()
   {
      return ZimmerJointName.values;
   }

   @Override
   public ValkyrieHandVersion getHandVersion()
   {
      return ValkyrieHandVersion.Zimmer;
   }

   @Override
   public SimulatedZimmerController newSimulatedHandController(RobotSide robotSide,
                                                               FullHumanoidRobotModel fullRobotModel,
                                                               JointDesiredOutputListBasics jointDesiredOutputList,
                                                               DoubleProvider yoTime,
                                                               RealtimeROS2Node realtimeROS2Node,
                                                               ROS2Topic<?> inputTopic)
   {
      return new SimulatedZimmerController(robotSide, fullRobotModel, jointDesiredOutputList, yoTime, realtimeROS2Node, inputTopic);
   }

   public static boolean hasZimmerGripper(RobotSide robotSide, RobotDefinition robotDefinition)
   {
      for (ZimmerJointName jointName : ZimmerJointName.values)
      {
         if (robotDefinition.getOneDoFJointDefinition(jointName.getJointName(robotSide)) == null)
            return false;
      }
      return true;
   }

   public static enum ZimmerJointName implements HandJointName
   {
      gripper_finger1_joint, gripper_finger2_joint;

      public static final ZimmerJointName[] values = values();

      @Override
      public String getJointName(RobotSide robotSide)
      {
         return robotSide.getCamelCaseName() + "_" + name();
      }

      public String getCamelCaseJointName(RobotSide side)
      {
         return side.getCamelCaseName() + CaseFormat.LOWER_UNDERSCORE.to(CaseFormat.UPPER_CAMEL, name());
      }

      public String getPascalCaseJointName(RobotSide side)
      {
         return side.getPascalCaseName() + CaseFormat.LOWER_UNDERSCORE.to(CaseFormat.UPPER_CAMEL, name());
      }

      public ZimmerFingerName getFingerName()
      {
         switch (this)
         {
            case gripper_finger1_joint:
               return ZimmerFingerName.finger1;
            case gripper_finger2_joint:
               return ZimmerFingerName.finger2;
            default:
               throw new IllegalStateException("Unexpected value: " + this);
         }
      }

      @Override
      public int getIndex(RobotSide robotSide)
      {
         return ordinal();
      }
   }

   public static enum ZimmerFingerName
   {
      finger1, finger2;
   }
}
