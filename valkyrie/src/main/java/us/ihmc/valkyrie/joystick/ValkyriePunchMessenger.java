package us.ihmc.valkyrie.joystick;

import controller_msgs.msg.dds.ArmTrajectoryMessage;
import controller_msgs.msg.dds.HighLevelStateMessage;
import us.ihmc.avatar.joystickBasedJavaFXController.HumanoidRobotLowLevelMessenger;
import us.ihmc.avatar.joystickBasedJavaFXController.HumanoidRobotPunchMessenger;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.robotics.robotSide.RobotSide;

public class ValkyriePunchMessenger implements HumanoidRobotPunchMessenger, HumanoidRobotLowLevelMessenger
{
   @Override
   public void sendArmHomeConfiguration(PacketCommunicator packetCommunicator, double trajectoryDuration, RobotSide... robotSides)
   {
      for (RobotSide robotSide : robotSides)
      {
         double[] jointAngles = new double[7];
         int index = 0;
         jointAngles[index++] = -1.1; // shoulderPitch
         jointAngles[index++] = robotSide.negateIfRightSide(-1.4); // shoulderRoll
         jointAngles[index++] = 0.3; // shoulderYaw
         jointAngles[index++] = robotSide.negateIfRightSide(-2.0); // elbowPitch
         jointAngles[index++] = robotSide.negateIfRightSide(0.0); // forearmYaw
         jointAngles[index++] = robotSide.negateIfRightSide(0.0); // wristRoll
         jointAngles[index++] = 0.0; // wristPitch
         ArmTrajectoryMessage message = HumanoidMessageTools.createArmTrajectoryMessage(robotSide, trajectoryDuration, jointAngles);
         packetCommunicator.send(message);
      }
   }

   @Override
   public void sendArmStraightConfiguration(PacketCommunicator packetCommunicator, double trajectoryDuration, RobotSide robotSide)
   {
      double[] jointAngles = new double[7];
      int index = 0;
      jointAngles[index++] = -1.5; // shoulderPitch
      jointAngles[index++] = robotSide.negateIfRightSide(-1.4); // shoulderRoll
      jointAngles[index++] = 1.5; // shoulderYaw
      jointAngles[index++] = robotSide.negateIfRightSide(-0.5); // elbowPitch
      jointAngles[index++] = robotSide.negateIfRightSide(0.0); // forearmYaw
      jointAngles[index++] = robotSide.negateIfRightSide(0.0); // wristRoll
      jointAngles[index++] = 0.0; // wristPitch
      ArmTrajectoryMessage message = HumanoidMessageTools.createArmTrajectoryMessage(robotSide, 0.4, jointAngles);
      packetCommunicator.send(message);
   }

   @Override
   public void sendFreezeRequest(PacketCommunicator packetCommunicator)
   {
      HighLevelStateMessage message = new HighLevelStateMessage();
      message.setHighLevelControllerName(HighLevelControllerName.EXIT_WALKING.toByte());
      packetCommunicator.send(message);
   }

   @Override
   public void sendStandRequest(PacketCommunicator packetCommunicator)
   {
      HighLevelStateMessage message = new HighLevelStateMessage();
      message.setHighLevelControllerName(HighLevelControllerName.STAND_TRANSITION_STATE.toByte());
      packetCommunicator.send(message);      
   }
}
