package us.ihmc.darpaRoboticsChallenge.footstepGenerator;

import java.util.ArrayList;

import geometry_msgs.PoseStamped;
import us.ihmc.SdfLoader.models.FullHumanoidRobotModel;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.subscribers.HumanoidRobotDataReceiver;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.footstepGenerator.AbstractSimpleParametersFootstepGenerator;
import us.ihmc.humanoidRobotics.footstep.footstepGenerator.SimplePathParameters;
import us.ihmc.humanoidRobotics.footstep.footstepGenerator.TurnStraightTurnFootstepGenerator;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotics.geometry.FrameOrientation2d;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FramePose2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.utilities.ros.msgToPacket.converter.GenericRosMessageConverter;
import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;

public class TimestampedPoseFootStepGenerator extends AbstractRosTopicSubscriber<geometry_msgs.PoseStamped>
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private SimplePathParameters pathType = new SimplePathParameters(0.4, 0.30, 0.0, Math.toRadians(10.0), Math.toRadians(5.0), 0.4);
   private final HumanoidRobotDataReceiver robotDataReceiver;
   private final PacketCommunicator controllerCommunicator;
   private final SideDependentList<RigidBody> feet;
   private final SideDependentList<ReferenceFrame> soleFrames;

   public TimestampedPoseFootStepGenerator(HumanoidRobotDataReceiver robotDataReceiver, FullHumanoidRobotModel fullRobotModel, PacketCommunicator controllerCommunicationBridge)
   {
      super(geometry_msgs.PoseStamped._TYPE);
      this.robotDataReceiver = robotDataReceiver;
      this.controllerCommunicator = controllerCommunicationBridge;
      this.feet = new SideDependentList<RigidBody>();
      this.soleFrames = new SideDependentList<ReferenceFrame>();
      for (RobotSide side : RobotSide.values())
      {
         feet.set(side, fullRobotModel.getFoot(side));
         soleFrames.set(side, fullRobotModel.getSoleFrame(side));
      }
   }

   @Override
   public void onNewMessage(PoseStamped message)
   {
      HumanoidReferenceFrames referenceFramesCopy = robotDataReceiver.getUpdatedReferenceFramesCopy();
      FramePose pose = GenericRosMessageConverter.convertPoseStampedToFramePose(referenceFramesCopy.getPelvisFrame(), message.getPose());//referenceFrames.getPelvisFrame()
      pose.changeFrame(worldFrame);
      FramePoint2d position = new FramePoint2d(worldFrame, pose.getX(), pose.getY());
      FrameOrientation2d orientation = new FrameOrientation2d(worldFrame, pose.getYaw());

      FramePose2d framePose2d = new FramePose2d(position, orientation);
      generateFootsteps(framePose2d);
   }

   private void generateFootsteps(FramePose2d endPose)
   {
      AbstractSimpleParametersFootstepGenerator footstepGenerator = new TurnStraightTurnFootstepGenerator(feet, soleFrames, endPose, pathType);
      footstepGenerator.initialize();

      ArrayList<Footstep> footsteps = new ArrayList<Footstep>();
      footsteps.addAll(footstepGenerator.generateDesiredFootstepList());

      sendFootStepsToController(footsteps);
   }

   public void sendFootStepsToController(ArrayList<Footstep> footsteps)
   {
      FootstepDataListMessage footsepDataList = new FootstepDataListMessage();

      for (int i = 0; i < footsteps.size(); i++)
      {
         footsepDataList.add(new FootstepDataMessage(footsteps.get(i)));
      }
      controllerCommunicator.send(footsepDataList);
   }
}
