package us.ihmc.humanoidBehaviors.behaviors.fiducialLocation;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.TextToSpeechPacket;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.behaviors.behaviorServices.FiducialDetectorBehaviorService;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidRobotics.communication.packets.walking.HeadTrajectoryMessage;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.math.frames.YoFramePoseUsingQuaternions;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.time.YoTimer;

public class FindFiducialBehavior extends AbstractBehavior
{
   private final String prefix = "findFiducial";

   private final FiducialDetectorBehaviorService fiducialDetectorBehaviorService;
   private final long fiducialToTrack;
   private final FullHumanoidRobotModel fullRobotModel;
   private final HumanoidReferenceFrames referenceFrames;

   private final YoTimer headTrajectorySentTimer;

   private final BooleanYoVariable foundFiducial = new BooleanYoVariable(prefix + "FoundFiducial", registry);
   private final IntegerYoVariable behaviorEnteredCount = new IntegerYoVariable(prefix + "BehaviorEnteredCount", registry);
   private final DoubleYoVariable headPitchToFindFucdicial = new DoubleYoVariable(prefix + "HeadPitchToFindFucdicial", registry);
   private final DoubleYoVariable headPitchToCenterFucdicial = new DoubleYoVariable(prefix + "HeadPitchToCenterFucdicial", registry);

   private final YoFramePoseUsingQuaternions foundFiducialYoFramePose;
   private final FramePose foundFiducialPose = new FramePose();

   public FindFiducialBehavior(DoubleYoVariable yoTime, CommunicationBridge behaviorCommunicationBridge, FullHumanoidRobotModel fullRobotModel, HumanoidReferenceFrames referenceFrames,
         FiducialDetectorBehaviorService fiducialDetectorBehaviorService, long fiducialToTrack)
   {
      super(FollowFiducialBehavior.class.getSimpleName(), behaviorCommunicationBridge);

      this.fiducialToTrack = fiducialToTrack;
      this.fullRobotModel = fullRobotModel;
      this.referenceFrames = referenceFrames;

      foundFiducialYoFramePose = new YoFramePoseUsingQuaternions(prefix + "FoundFiducialPose", ReferenceFrame.getWorldFrame(), registry);
      this.fiducialDetectorBehaviorService = fiducialDetectorBehaviorService;
      addBehaviorService(fiducialDetectorBehaviorService);

      fiducialDetectorBehaviorService.setTargetIDToLocate(this.fiducialToTrack);

      headPitchToFindFucdicial.set(0.6);

      headTrajectorySentTimer = new YoTimer(yoTime);
      headTrajectorySentTimer.start();
      //      behaviorCommunicationBridge.attachNetworkListeningQueue(robotConfigurationDataQueue, RobotConfigurationData.class);
      //      behaviorCommunicationBridge.attachNetworkListeningQueue(footstepStatusQueue, FootstepStatus.class);
      //      behaviorCommunicationBridge.attachNetworkListeningQueue(walkingStatusQueue, WalkingStatusMessage.class);
      //      behaviorCommunicationBridge.attachNetworkListeningQueue(planarRegionsListQueue, PlanarRegionsListMessage.class);
   }

   @Override
   public void doControl()
   {
      if (fiducialDetectorBehaviorService.getTargetIDHasBeenLocated())
      {
         fiducialDetectorBehaviorService.getReportedFiducialPoseWorldFrame(foundFiducialPose);
         foundFiducialYoFramePose.set(foundFiducialPose);
         foundFiducial.set(true);
      }

      if (headTrajectorySentTimer.totalElapsed() < 2.0)
      {
         return;
      }

      pitchHeadToFindFiducial();
      //      pitchHeadToCenterFiducial();

   }

   public void getFiducialPose(FramePose framePoseToPack)
   {
      framePoseToPack.setPoseIncludingFrame(foundFiducialPose);
   }

   private void sendTextToSpeechPacket(String message)
   {
      TextToSpeechPacket textToSpeechPacket = new TextToSpeechPacket(message);
      textToSpeechPacket.setbeep(false);
      sendPacketToUI(textToSpeechPacket);
   }

   private void pitchHeadToFindFiducial()
   {
      headPitchToFindFucdicial.mul(-1.0);
      AxisAngle4d axisAngleOrientation = new AxisAngle4d(new Vector3d(0.0, 1.0, 0.0), headPitchToFindFucdicial.getDoubleValue());

      Quat4d headOrientation = new Quat4d();
      headOrientation.set(axisAngleOrientation);
      sendHeadTrajectoryMessage(1.0, headOrientation);
   }

   private void pitchHeadToCenterFiducial()
   {
      //      headPitchToCenterFucdicial.set(0.0);
      //      AxisAngle4d axisAngleOrientation = new AxisAngle4d(new Vector3d(0.0, 1.0, 0.0), headPitchToCenterFucdicial.getDoubleValue());
      //
      //      Quat4d headOrientation = new Quat4d();
      //      headOrientation.set(axisAngleOrientation);
      //      sendHeadTrajectoryMessage(1.0, headOrientation);
   }

   private void sendHeadTrajectoryMessage(double trajectoryTime, Quat4d desiredOrientation)
   {
      HeadTrajectoryMessage headTrajectoryMessage = new HeadTrajectoryMessage(trajectoryTime, desiredOrientation);

      headTrajectoryMessage.setDestination(PacketDestination.UI);
      sendPacket(headTrajectoryMessage);

      headTrajectoryMessage.setDestination(PacketDestination.CONTROLLER);
      sendPacketToController(headTrajectoryMessage);

      headTrajectorySentTimer.reset();
   }

   @Override
   public void onBehaviorEntered()
   {
      behaviorEnteredCount.increment();
      foundFiducial.set(false);
      fiducialDetectorBehaviorService.setTargetIDToLocate(this.fiducialToTrack);
   }

   @Override
   public boolean isDone()
   {
      return foundFiducial.getBooleanValue();
   }

   @Override
   public void onBehaviorAborted()
   {
   }

   @Override
   public void onBehaviorPaused()
   {
   }

   @Override
   public void onBehaviorResumed()
   {
   }

   @Override
   public void onBehaviorExited()
   {
   }
}
