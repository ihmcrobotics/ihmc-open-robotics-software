package us.ihmc.valkyrie.hands.athena;

import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandJointName;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.scs2.definition.robot.OneDoFJointDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListBasics;
import us.ihmc.valkyrie.hands.ValkyrieHandController;
import us.ihmc.valkyrie.hands.ValkyrieHandModel;
import us.ihmc.valkyrie.hands.ValkyrieHandVersion;
import us.ihmc.yoVariables.providers.DoubleProvider;

public class AthenaFingerlessHandModel implements ValkyrieHandModel
{
   @Override
   public HandJointName[] getHandJointNames()
   {
      return new HandJointName[0];
   }

   @Override
   public ValkyrieHandVersion getHandVersion()
   {
      return ValkyrieHandVersion.AthenaFingerless;
   }

   public static boolean hasAthenaFingerlessHand(RobotSide robotSide, RobotDefinition robotDefinition)
   {
      String wristPitchName = robotSide.getCamelCaseName() + "WristPitch";
      OneDoFJointDefinition wristPitchJoint = robotDefinition.getOneDoFJointDefinition(wristPitchName);
      if (wristPitchJoint == null)
         return false;
      RigidBodyDefinition palmLink = wristPitchJoint.getSuccessor();
      if (palmLink == null)
         return false;
      return palmLink.getChildrenJoints() == null || palmLink.getChildrenJoints().isEmpty();
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
