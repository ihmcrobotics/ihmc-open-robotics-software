package us.ihmc.valkyrie.joystick;

import controller_msgs.msg.dds.ArmTrajectoryMessage;
import controller_msgs.msg.dds.HighLevelStateMessage;
import us.ihmc.avatar.joystickBasedJavaFXController.HumanoidRobotPunchMessenger;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.communication.controllerAPI.RobotLowLevelMessenger;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.Ros2Node;

public class ValkyriePunchMessenger implements HumanoidRobotPunchMessenger, RobotLowLevelMessenger
{
   private final IHMCROS2Publisher<ArmTrajectoryMessage> armTrajectoryPublisher;
   private final IHMCROS2Publisher<HighLevelStateMessage> highLevelStatePublisher;

   public ValkyriePunchMessenger(String robotName, Ros2Node ros2Node)
   {
      MessageTopicNameGenerator subscriberTopicNameGenerator = ControllerAPIDefinition.getSubscriberTopicNameGenerator(robotName);
      armTrajectoryPublisher = ROS2Tools.createPublisher(ros2Node, ArmTrajectoryMessage.class, subscriberTopicNameGenerator);
      highLevelStatePublisher = ROS2Tools.createPublisher(ros2Node, HighLevelStateMessage.class, subscriberTopicNameGenerator);
   }

   @Override
   public void sendArmHomeConfiguration(double trajectoryDuration, RobotSide... robotSides)
   {
      for (RobotSide robotSide : robotSides)
      {
         double[] jointAngles = new double[7];
         int index = 0;
         jointAngles[index++] = 0.0; // shoulderPitch
         jointAngles[index++] = 0.0; // shoulderRoll
         jointAngles[index++] = 0.0; // shoulderYaw
         jointAngles[index++] = robotSide.negateIfRightSide(-2.0); // elbowPitch
         jointAngles[index++] = robotSide.negateIfRightSide(0.0); // forearmYaw
         jointAngles[index++] = robotSide.negateIfRightSide(0.0); // wristRoll
         jointAngles[index++] = 0.0; // wristPitch
         ArmTrajectoryMessage message = HumanoidMessageTools.createArmTrajectoryMessage(robotSide, trajectoryDuration, jointAngles);
         armTrajectoryPublisher.publish(message);
      }
   }

   @Override
   public void sendArmStraightConfiguration(double trajectoryDuration, RobotSide robotSide)
   {
   }

   @Override
   public void sendFreezeRequest()
   {
      HighLevelStateMessage message = new HighLevelStateMessage();
      message.setHighLevelControllerName(HighLevelControllerName.EXIT_WALKING.toByte());
      highLevelStatePublisher.publish(message);
   }

   @Override
   public void sendStandRequest()
   {
      HighLevelStateMessage message = new HighLevelStateMessage();
      message.setHighLevelControllerName(HighLevelControllerName.STAND_TRANSITION_STATE.toByte());
      highLevelStatePublisher.publish(message);
   }
}
