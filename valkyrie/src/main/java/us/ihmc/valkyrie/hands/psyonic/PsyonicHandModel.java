package us.ihmc.valkyrie.hands.psyonic;

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

public class PsyonicHandModel implements ValkyrieHandModel
{
   @Override
   public PsyonicJointName[] getHandJointNames()
   {
      return PsyonicJointName.values;
   }

   @Override
   public ValkyrieHandVersion getHandVersion()
   {
      return ValkyrieHandVersion.Psyonic;
   }

   @Override
   public SimulatedPsyonicController newSimulatedHandController(RobotSide robotSide,
                                                                FullHumanoidRobotModel fullRobotModel,
                                                                JointDesiredOutputListBasics jointDesiredOutputList,
                                                                DoubleProvider yoTime,
                                                                RealtimeROS2Node realtimeROS2Node,
                                                                ROS2Topic<?> inputTopic)
   {
      return new SimulatedPsyonicController(robotSide, fullRobotModel, jointDesiredOutputList, yoTime, realtimeROS2Node, inputTopic);
   }

   public static boolean hasPsyonicHand(RobotSide robotSide, RobotDefinition robotDefinition)
   {
      for (PsyonicJointName jointName : PsyonicJointName.values)
      {
         if (robotDefinition.getOneDoFJointDefinition(jointName.getJointName(robotSide)) == null)
            return false;
      }
      return true;
   }

   public static enum PsyonicJointName implements HandJointName
   {
      thumb_q1, thumb_q2, index_q1, index_q2, middle_q1, middle_q2, ring_q1, ring_q2, pinky_q1, pinky_q2;

      public static final PsyonicJointName[] values = values();
      public static final PsyonicJointName[] fingerJoints = {index_q1, index_q2, middle_q1, middle_q2, ring_q1, ring_q2, pinky_q1, pinky_q2};
      public static final PsyonicJointName[] thumbJoints = {thumb_q1, thumb_q2};

      @Override
      public String getJointName(RobotSide robotSide)
      {
         return name() + "_" + robotSide.getCamelCaseName();
      }

      public String getCamelCaseJointName(RobotSide side)
      {
         return side.getCamelCaseName() + CaseFormat.LOWER_UNDERSCORE.to(CaseFormat.UPPER_CAMEL, name());
      }

      public String getPascalCaseJointName(RobotSide side)
      {
         return side.getPascalCaseName() + CaseFormat.LOWER_UNDERSCORE.to(CaseFormat.UPPER_CAMEL, name());
      }

      public static PsyonicJointName fromByte(byte jointNameByteValue)
      {
         if (jointNameByteValue < 0)
            return null;
         else
            return values[jointNameByteValue];
      }

      public byte toByte()
      {
         return (byte) ordinal();
      }

      @Override
      public int getIndex(RobotSide robotSide)
      {
         return ordinal();
      }
   }
}
