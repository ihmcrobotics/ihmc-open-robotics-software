package us.ihmc.atlas.joystickBasedStepping;

import static us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools.createTrajectoryPoint1DMessage;

import atlas_msgs.msg.dds.AtlasLowLevelControlModeMessage;
import controller_msgs.msg.dds.*;
import ihmc_common_msgs.msg.dds.TrajectoryPoint1DMessage;
import us.ihmc.avatar.joystickBasedJavaFXController.HumanoidRobotKickMessenger;
import us.ihmc.avatar.joystickBasedJavaFXController.HumanoidRobotPunchMessenger;
import us.ihmc.ros2.ROS2PublisherBasics;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.controllerAPI.RobotLowLevelMessenger;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.atlas.AtlasLowLevelControlMode;
import us.ihmc.humanoidRobotics.communication.packets.walking.LoadBearingRequest;
import us.ihmc.idl.IDLSequence.Object;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SegmentDependentList;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.ROS2Node;

public class AtlasKickAndPunchMessenger implements HumanoidRobotPunchMessenger, HumanoidRobotKickMessenger, RobotLowLevelMessenger
{
   private final ROS2PublisherBasics<ArmTrajectoryMessage> armTrajectoryPublisher;
   private final ROS2PublisherBasics<FootTrajectoryMessage> footTrajectoryPublisher;
   private final ROS2PublisherBasics<FootLoadBearingMessage> footLoadBearingPublisher;
   private final ROS2PublisherBasics<AtlasLowLevelControlModeMessage> atlasLowLevelControlModePublisher;
   private final ROS2PublisherBasics<AbortWalkingMessage> abortWalkingPublisher;
   private final ROS2PublisherBasics<PauseWalkingMessage> pauseWalkingPublisher;

   public AtlasKickAndPunchMessenger(ROS2Node ros2Node, String robotName)
   {
      ROS2Topic inputTopic = ROS2Tools.getControllerInputTopic(robotName);
      armTrajectoryPublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, ArmTrajectoryMessage.class, inputTopic);
      footTrajectoryPublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, FootTrajectoryMessage.class, inputTopic);
      footLoadBearingPublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, FootLoadBearingMessage.class, inputTopic);
      atlasLowLevelControlModePublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, AtlasLowLevelControlModeMessage.class, inputTopic);
      abortWalkingPublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, AbortWalkingMessage.class, inputTopic);
      pauseWalkingPublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, PauseWalkingMessage.class, inputTopic);
   }

   @Override
   public void sendArmHomeConfiguration(double trajectoryDuration, RobotSide... robotSides)
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
         armTrajectoryPublisher.publish(message);
      }
   }

   @Override
   public void sendArmStraightConfiguration(double trajectoryDuration, RobotSide robotSide)
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
      armTrajectoryPublisher.publish(message);
   }

   @Override
   public void sendFlamingoHomeStance(RobotSide robotSide, double trajectoryDuration, double stanceWidth,
                                      SegmentDependentList<RobotSide, ? extends ReferenceFrame> soleFrames)
   {
      FramePose3D footPose = new FramePose3D(soleFrames.get(robotSide.getOppositeSide()));
      footPose.appendTranslation(0.0, robotSide.negateIfRightSide(stanceWidth), 0.15);
      footPose.changeFrame(ReferenceFrame.getWorldFrame());

      FootTrajectoryMessage message = HumanoidMessageTools.createFootTrajectoryMessage(robotSide, trajectoryDuration, footPose);
      footTrajectoryPublisher.publish(message);
   }

   @Override
   public void sendKick(RobotSide robotSide, double trajectoryDuration, double stanceWidth,
                        SegmentDependentList<RobotSide, ? extends ReferenceFrame> soleFrames)
   {
      FramePose3D footPose = new FramePose3D(soleFrames.get(robotSide.getOppositeSide()));
      footPose.appendTranslation(0.60, robotSide.negateIfRightSide(stanceWidth), 0.35);
      footPose.appendPitchRotation(-0.8);
      footPose.changeFrame(ReferenceFrame.getWorldFrame());

      FootTrajectoryMessage message = HumanoidMessageTools.createFootTrajectoryMessage(robotSide, 0.33 * trajectoryDuration, footPose);
      footTrajectoryPublisher.publish(message);
   }

   @Override
   public void sendPutFootDown(RobotSide robotSide, double trajectoryDuration, double stanceWidth,
                               SegmentDependentList<RobotSide, ? extends ReferenceFrame> soleFrames)
   {
      FramePose3D footPose = new FramePose3D(soleFrames.get(robotSide.getOppositeSide()));
      footPose.appendTranslation(0.0, robotSide.negateIfRightSide(stanceWidth), 0.0);
      footPose.changeFrame(ReferenceFrame.getWorldFrame());

      FootTrajectoryMessage message = HumanoidMessageTools.createFootTrajectoryMessage(robotSide, trajectoryDuration, footPose);
      footTrajectoryPublisher.publish(message);

      footLoadBearingPublisher.publish(HumanoidMessageTools.createFootLoadBearingMessage(robotSide, LoadBearingRequest.LOAD));
   }

   @Override
   public void sendFreezeRequest()
   {
      AtlasLowLevelControlModeMessage message = new AtlasLowLevelControlModeMessage();
      message.setRequestedAtlasLowLevelControlMode(AtlasLowLevelControlMode.FREEZE.toByte());
      atlasLowLevelControlModePublisher.publish(message);
   }

   @Override
   public void sendStandRequest()
   {
      AtlasLowLevelControlModeMessage message = new AtlasLowLevelControlModeMessage();
      message.setRequestedAtlasLowLevelControlMode(AtlasLowLevelControlMode.STAND_PREP.toByte());
      atlasLowLevelControlModePublisher.publish(message);
   }

   @Override
   public void sendAbortWalkingRequest()
   {
      abortWalkingPublisher.publish(new AbortWalkingMessage());
   }

   @Override
   public void sendPauseWalkingRequest()
   {
      PauseWalkingMessage message = new PauseWalkingMessage();
      message.setPause(true);
      pauseWalkingPublisher.publish(message);
   }

   @Override
   public void sendContinueWalkingRequest()
   {
      PauseWalkingMessage message = new PauseWalkingMessage();
      message.setPause(false);
      pauseWalkingPublisher.publish(message);
   }
}
