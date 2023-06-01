package us.ihmc.valkyrie.hands;

import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandJointName;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.scs2.definition.robot.OneDoFJointDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListBasics;
import us.ihmc.yoVariables.providers.DoubleProvider;

public class ArmMassSimModel implements ValkyrieHandModel
{
   @Override
   public HandJointName[] getHandJointNames()
   {
      return new HandJointName[0];
   }

   @Override
   public ValkyrieHandVersion getHandVersion()
   {
      return ValkyrieHandVersion.MassSim;
   }

   public static boolean hasArmMassSim(RobotSide robotSide, RobotDefinition robotDefinition)
   {
      String elbowPitchName = robotSide.getCamelCaseName() + "ElbowPitch";
      OneDoFJointDefinition elbowPitchJoint = robotDefinition.getOneDoFJointDefinition(elbowPitchName);
      if (elbowPitchJoint == null)
         return false;
      RigidBodyDefinition massSimLink = elbowPitchJoint.getSuccessor();
      if (massSimLink == null)
         return false;
      return massSimLink.getChildrenJoints() == null || massSimLink.getChildrenJoints().isEmpty();
   }

   @Override
   public ValkyrieHandController newSimulatedHandController(RobotSide robotSide,
                                                            FullHumanoidRobotModel fullRobotModel,
                                                            JointDesiredOutputListBasics jointDesiredOutputList,
                                                            DoubleProvider yoTime,
                                                            RealtimeROS2Node realtimeROS2Node,
                                                            ROS2Topic<?> inputTopic)
   {
      return null;
   }
}
