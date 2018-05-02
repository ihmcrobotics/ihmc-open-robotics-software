package us.ihmc.atlas.joystickBasedStepping;

import static us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools.createTrajectoryPoint1DMessage;

import controller_msgs.msg.dds.ArmTrajectoryMessage;
import controller_msgs.msg.dds.AtlasLowLevelControlModeMessage;
import controller_msgs.msg.dds.FootTrajectoryMessage;
import controller_msgs.msg.dds.OneDoFJointTrajectoryMessage;
import controller_msgs.msg.dds.TrajectoryPoint1DMessage;
import us.ihmc.avatar.joystickBasedJavaFXController.HumanoidRobotKickMessenger;
import us.ihmc.avatar.joystickBasedJavaFXController.HumanoidRobotLowLevelMessenger;
import us.ihmc.avatar.joystickBasedJavaFXController.HumanoidRobotPunchMessenger;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.atlas.AtlasLowLevelControlMode;
import us.ihmc.humanoidRobotics.communication.packets.walking.LoadBearingRequest;
import us.ihmc.idl.IDLSequence.Object;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SegmentDependentList;

public class AtlasKickAndPunchMessenger implements HumanoidRobotPunchMessenger, HumanoidRobotKickMessenger, HumanoidRobotLowLevelMessenger
{
   @Override
   public void sendArmHomeConfiguration(PacketCommunicator packetCommunicator, double trajectoryDuration, RobotSide... robotSides)
   {
      for (RobotSide robotSide : robotSides)
      {
         double[] jointAngles = new double[7];
         int index = 0;
         jointAngles[index++] = robotSide.negateIfRightSide(0.785398); // ArmJointName.SHOULDER_YAW        
         jointAngles[index++] = robotSide.negateIfRightSide(-0.52379); // ArmJointName.SHOULDER_ROLL       
         jointAngles[index++] = 2.33708; // ArmJointName.ELBOW_PITCH         
         jointAngles[index++] = robotSide.negateIfRightSide(2.35619); // ArmJointName.ELBOW_ROLL          
         jointAngles[index++] = -0.337807; // ArmJointName.FIRST_WRIST_PITCH   
         jointAngles[index++] = robotSide.negateIfRightSide(0.207730); // ArmJointName.WRIST_ROLL          
         jointAngles[index++] = -0.026599; // ArmJointName.SECOND_WRIST_PITCH
         ArmTrajectoryMessage message = HumanoidMessageTools.createArmTrajectoryMessage(robotSide, trajectoryDuration, jointAngles);
         packetCommunicator.send(message);
      }
   }

   @Override
   public void sendArmStraightConfiguration(PacketCommunicator packetCommunicator, double trajectoryDuration, RobotSide robotSide)
   {
      double[] jointAngles0 = new double[7];
      int index = 0;
      jointAngles0[index++] = robotSide.negateIfRightSide(-0.2); // ArmJointName.SHOULDER_YAW        
      jointAngles0[index++] = robotSide.negateIfRightSide(-0.17); // ArmJointName.SHOULDER_ROLL       
      jointAngles0[index++] = 1.4; // ArmJointName.ELBOW_PITCH         
      jointAngles0[index++] = robotSide.negateIfRightSide(1.8); // ArmJointName.ELBOW_ROLL          
      jointAngles0[index++] = -0.337807; // ArmJointName.FIRST_WRIST_PITCH   
      jointAngles0[index++] = robotSide.negateIfRightSide(0.207730); // ArmJointName.WRIST_ROLL          
      jointAngles0[index++] = -0.026599; // ArmJointName.SECOND_WRIST_PITCH

      double[] jointAngles1 = new double[7];
      index = 0;
      jointAngles1[index++] = robotSide.negateIfRightSide(-1.2); // ArmJointName.SHOULDER_YAW        
      jointAngles1[index++] = robotSide.negateIfRightSide(-0.0); // ArmJointName.SHOULDER_ROLL       
      jointAngles1[index++] = 1.8; // ArmJointName.ELBOW_PITCH         
      jointAngles1[index++] = robotSide.negateIfRightSide(0.6); // ArmJointName.ELBOW_ROLL          
      jointAngles1[index++] = -0.337807; // ArmJointName.FIRST_WRIST_PITCH   
      jointAngles1[index++] = robotSide.negateIfRightSide(0.207730); // ArmJointName.WRIST_ROLL          
      jointAngles1[index++] = -0.026599; // ArmJointName.SECOND_WRIST_PITCH
      ArmTrajectoryMessage message = HumanoidMessageTools.createArmTrajectoryMessage(robotSide, trajectoryDuration / 2.0, jointAngles0);
      Object<OneDoFJointTrajectoryMessage> jointTrajectoryMessages = message.getJointspaceTrajectory().getJointTrajectoryMessages();
      for (int i = 0; i < jointTrajectoryMessages.size(); i++)
      {
         TrajectoryPoint1DMessage trajectoryPoint1DMessage = createTrajectoryPoint1DMessage(trajectoryDuration, jointAngles1[i], 0.0);
         jointTrajectoryMessages.get(i).getTrajectoryPoints().add().set(trajectoryPoint1DMessage);
      }
      packetCommunicator.send(message);
   }

   @Override
   public void sendFlamingoHomeStance(PacketCommunicator packetCommunicator, RobotSide robotSide, double trajectoryDuration, double stanceWidth,
                                      SegmentDependentList<RobotSide, ? extends ReferenceFrame> soleFrames)
   {
      FramePose3D footPose = new FramePose3D(soleFrames.get(robotSide.getOppositeSide()));
      footPose.appendTranslation(0.0, robotSide.negateIfRightSide(stanceWidth), 0.15);
      footPose.changeFrame(ReferenceFrame.getWorldFrame());

      FootTrajectoryMessage message = HumanoidMessageTools.createFootTrajectoryMessage(robotSide, trajectoryDuration, footPose);
      packetCommunicator.send(message);
   }

   @Override
   public void sendKick(PacketCommunicator packetCommunicator, RobotSide robotSide, double trajectoryDuration, double stanceWidth,
                        SegmentDependentList<RobotSide, ? extends ReferenceFrame> soleFrames)
   {
      FramePose3D footPose = new FramePose3D(soleFrames.get(robotSide.getOppositeSide()));
      footPose.appendTranslation(0.60, robotSide.negateIfRightSide(stanceWidth), 0.35);
      footPose.appendPitchRotation(-0.8);
      footPose.changeFrame(ReferenceFrame.getWorldFrame());

      FootTrajectoryMessage message = HumanoidMessageTools.createFootTrajectoryMessage(robotSide, 0.33 * trajectoryDuration, footPose);
      packetCommunicator.send(message);
   }

   @Override
   public void sendPutFootDown(PacketCommunicator packetCommunicator, RobotSide robotSide, double trajectoryDuration, double stanceWidth,
                               SegmentDependentList<RobotSide, ? extends ReferenceFrame> soleFrames)
   {
      FramePose3D footPose = new FramePose3D(soleFrames.get(robotSide.getOppositeSide()));
      footPose.appendTranslation(0.0, robotSide.negateIfRightSide(stanceWidth), 0.0);
      footPose.changeFrame(ReferenceFrame.getWorldFrame());

      FootTrajectoryMessage message = HumanoidMessageTools.createFootTrajectoryMessage(robotSide, trajectoryDuration, footPose);
      packetCommunicator.send(message);

      packetCommunicator.send(HumanoidMessageTools.createFootLoadBearingMessage(robotSide, LoadBearingRequest.LOAD));
   }

   @Override
   public void sendFreezeRequest(PacketCommunicator packetCommunicator)
   {
      AtlasLowLevelControlModeMessage message = new AtlasLowLevelControlModeMessage();
      message.setRequestedAtlasLowLevelControlMode(AtlasLowLevelControlMode.FREEZE.toByte());
      packetCommunicator.send(message);
   }

   @Override
   public void sendStandRequest(PacketCommunicator packetCommunicator)
   {
      AtlasLowLevelControlModeMessage message = new AtlasLowLevelControlModeMessage();
      message.setRequestedAtlasLowLevelControlMode(AtlasLowLevelControlMode.STAND_PREP.toByte());
      packetCommunicator.send(message);      
   }
}
