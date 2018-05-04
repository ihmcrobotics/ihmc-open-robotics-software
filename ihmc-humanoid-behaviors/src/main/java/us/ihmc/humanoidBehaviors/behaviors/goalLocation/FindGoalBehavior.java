package us.ihmc.humanoidBehaviors.behaviors.goalLocation;

import controller_msgs.msg.dds.HeadTrajectoryMessage;
import controller_msgs.msg.dds.TextToSpeechPacket;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.behaviors.fiducialLocation.FollowFiducialBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.time.YoStopwatch;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePose3D;
import us.ihmc.yoVariables.variable.YoInteger;

public class FindGoalBehavior extends AbstractBehavior
{
   private final String prefix = "findGoal";

   private final GoalDetectorBehaviorService fiducialDetectorBehaviorService;
   private final FullHumanoidRobotModel fullRobotModel;
   private final HumanoidReferenceFrames referenceFrames;

   private final YoStopwatch headTrajectorySentTimer;

   private final YoBoolean foundFiducial = new YoBoolean(prefix + "FoundGoal", registry);
   private final YoInteger behaviorEnteredCount = new YoInteger(prefix + "BehaviorEnteredCount", registry);
   private final YoDouble headPitchToFindFucdicial = new YoDouble(prefix + "HeadPitchToFindFucdicial", registry);
   private final YoDouble headPitchToCenterFucdicial = new YoDouble(prefix + "HeadPitchToCenterFucdicial", registry);

   private final YoFramePose3D foundFiducialYoFramePose;
   private final FramePose3D foundFiducialPose = new FramePose3D();

   public FindGoalBehavior(YoDouble yoTime, CommunicationBridge behaviorCommunicationBridge, FullHumanoidRobotModel fullRobotModel, HumanoidReferenceFrames referenceFrames,
                           GoalDetectorBehaviorService fiducialDetectorBehaviorService)
   {
      super(FollowFiducialBehavior.class.getSimpleName(), behaviorCommunicationBridge);

      this.fullRobotModel = fullRobotModel;
      this.referenceFrames = referenceFrames;

      foundFiducialYoFramePose = new YoFramePose3D(prefix + "FoundGoalPose", ReferenceFrame.getWorldFrame(), registry);
      this.fiducialDetectorBehaviorService = fiducialDetectorBehaviorService;
      addBehaviorService(fiducialDetectorBehaviorService);

      headPitchToFindFucdicial.set(0.6);

      headTrajectorySentTimer = new YoStopwatch(yoTime);
      headTrajectorySentTimer.start();
      //      behaviorCommunicationBridge.attachNetworkListeningQueue(robotConfigurationDataQueue, RobotConfigurationData.class);
      //      behaviorCommunicationBridge.attachNetworkListeningQueue(footstepStatusQueue, FootstepStatus.class);
      //      behaviorCommunicationBridge.attachNetworkListeningQueue(walkingStatusQueue, WalkingStatusMessage.class);
      //      behaviorCommunicationBridge.attachNetworkListeningQueue(planarRegionsListQueue, PlanarRegionsListMessage.class);
   }

   @Override
   public void doControl()
   {
      if (fiducialDetectorBehaviorService.getGoalHasBeenLocated())
      {
         fiducialDetectorBehaviorService.getReportedGoalPoseWorldFrame(foundFiducialPose);
         foundFiducialYoFramePose.set(foundFiducialPose);
         foundFiducial.set(true);
         Point3D position = new Point3D(foundFiducialPose.getPosition());
         sendTextToSpeechPacket("Target object located at " + position);
      } else {
         sendTextToSpeechPacket("Target object not located");
      }

      if (headTrajectorySentTimer.totalElapsed() < 2.0)
      {
         return;
      }

      pitchHeadToFindFiducial();
      //      pitchHeadToCenterFiducial();

   }

   public void getFiducialPose(FramePose3D framePoseToPack)
   {
      framePoseToPack.setIncludingFrame(foundFiducialPose);
   }

   public void getGoalPose(FramePose3D framePoseToPack)
   {
      getFiducialPose(framePoseToPack);
   }

   private void sendTextToSpeechPacket(String message)
   {
      TextToSpeechPacket textToSpeechPacket = MessageTools.createTextToSpeechPacket(message);
      textToSpeechPacket.setBeep(false);
      sendPacketToUI(textToSpeechPacket);
   }

   private void pitchHeadToFindFiducial()
   {
      headPitchToFindFucdicial.mul(-1.0);
      AxisAngle axisAngleOrientation = new AxisAngle(new Vector3D(0.0, 1.0, 0.0), headPitchToFindFucdicial.getDoubleValue());

      Quaternion headOrientation = new Quaternion();
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

   private void sendHeadTrajectoryMessage(double trajectoryTime, Quaternion desiredOrientation)
   {
      ReferenceFrame chestCoMFrame = fullRobotModel.getChest().getBodyFixedFrame();
      HeadTrajectoryMessage headTrajectoryMessage = HumanoidMessageTools.createHeadTrajectoryMessage(trajectoryTime, desiredOrientation, ReferenceFrame.getWorldFrame(), chestCoMFrame);

      headTrajectoryMessage.setDestination(PacketDestination.UI.ordinal());
      sendPacket(headTrajectoryMessage);

      headTrajectoryMessage.setDestination(PacketDestination.CONTROLLER.ordinal());
      sendPacketToController(headTrajectoryMessage);

      headTrajectorySentTimer.reset();
   }

   @Override
   public void onBehaviorEntered()
   {
      behaviorEnteredCount.increment();
      foundFiducial.set(false);
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
