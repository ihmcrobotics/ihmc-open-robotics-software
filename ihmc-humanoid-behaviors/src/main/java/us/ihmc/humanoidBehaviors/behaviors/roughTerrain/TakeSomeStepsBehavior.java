package us.ihmc.humanoidBehaviors.behaviors.roughTerrain;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.FootstepStatusMessage;
import controller_msgs.msg.dds.TextToSpeechPacket;
import controller_msgs.msg.dds.WalkingStatusMessage;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.SimpleFootstep;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingStatus;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.time.YoStopwatch;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoFramePoseUsingYawPitchRoll;

public class TakeSomeStepsBehavior extends AbstractBehavior
{
   private final String prefix = "takeSomeSteps";

   private final FullHumanoidRobotModel fullRobotModel;
   private final HumanoidReferenceFrames referenceFrames;

   private final ConcurrentListeningQueue<WalkingStatusMessage> walkingStatusQueue = new ConcurrentListeningQueue<WalkingStatusMessage>(40);
   private final ConcurrentListeningQueue<FootstepStatusMessage> footstepStatusQueue = new ConcurrentListeningQueue<FootstepStatusMessage>(40);

   private final SideDependentList<FootstepStatusMessage> latestFootstepStatus;
   private final SideDependentList<YoEnum<FootstepStatus>> latestFootstepStatusEnum;
   private final SideDependentList<YoFramePoseUsingYawPitchRoll> desiredFootStatusPoses;
   private final SideDependentList<YoFramePoseUsingYawPitchRoll> actualFootStatusPoses;

   private final YoEnum<RobotSide> nextSideToSwing;
   private final YoEnum<RobotSide> currentlySwingingFoot;

   private final YoBoolean doneTakingSteps;

   private final YoFramePoseUsingYawPitchRoll footstepPlannerInitialStepPose;
   private final YoFramePoseUsingYawPitchRoll footstepPlannerGoalPose;

   private final FramePose3D tempFootstepPlannerGoalPose = new FramePose3D();
   private final FramePose3D leftFootPose = new FramePose3D();
   private final FramePose3D rightFootPose = new FramePose3D();
   private final FramePose3D tempStanceFootPose = new FramePose3D();
   private final FramePose3D tempFirstFootstepPose = new FramePose3D();
   private final Point3D tempFootstepPosePosition = new Point3D();
   private final Quaternion tempFirstFootstepPoseOrientation = new Quaternion();
   private final YoStopwatch footstepSentTimer;

   public TakeSomeStepsBehavior(YoDouble yoTime, CommunicationBridge behaviorCommunicationBridge, FullHumanoidRobotModel fullRobotModel, HumanoidReferenceFrames referenceFrames)
   {
      super(TakeSomeStepsBehavior.class.getSimpleName(), behaviorCommunicationBridge);

      this.fullRobotModel = fullRobotModel;
      this.referenceFrames = referenceFrames;

      nextSideToSwing = new YoEnum<>("nextSideToSwing", registry, RobotSide.class);
      nextSideToSwing.set(RobotSide.LEFT);

      doneTakingSteps = new YoBoolean(prefix + "DoneTakingSteps", registry);

      currentlySwingingFoot = new YoEnum<>("currentlySwingingFoot", registry, RobotSide.class, true);

      footstepSentTimer = new YoStopwatch(yoTime);
      footstepSentTimer.start();

      footstepPlannerGoalPose = new YoFramePoseUsingYawPitchRoll(prefix + "FootstepGoalPose", ReferenceFrame.getWorldFrame(), registry);
      footstepPlannerInitialStepPose = new YoFramePoseUsingYawPitchRoll(prefix + "InitialStepPose", ReferenceFrame.getWorldFrame(), registry);

      YoFramePoseUsingYawPitchRoll desiredLeftFootstepStatusPose = new YoFramePoseUsingYawPitchRoll(prefix + "DesiredLeftFootstepStatusPose", ReferenceFrame.getWorldFrame(), registry);
      YoFramePoseUsingYawPitchRoll desiredRightFootstepStatusPose = new YoFramePoseUsingYawPitchRoll(prefix + "DesiredRightFootstepStatusPose", ReferenceFrame.getWorldFrame(), registry);
      desiredFootStatusPoses = new SideDependentList<>(desiredLeftFootstepStatusPose, desiredRightFootstepStatusPose);

      YoFramePoseUsingYawPitchRoll leftFootstepStatusPose = new YoFramePoseUsingYawPitchRoll(prefix + "LeftFootstepStatusPose", ReferenceFrame.getWorldFrame(), registry);
      YoFramePoseUsingYawPitchRoll rightFootstepStatusPose = new YoFramePoseUsingYawPitchRoll(prefix + "RightFootstepStatusPose", ReferenceFrame.getWorldFrame(), registry);
      actualFootStatusPoses = new SideDependentList<>(leftFootstepStatusPose, rightFootstepStatusPose);

      latestFootstepStatus = new SideDependentList<>();

      YoEnum<FootstepStatus> leftFootstepStatus = new YoEnum<FootstepStatus>("leftFootstepStatus", registry,
            FootstepStatus.class);
      YoEnum<FootstepStatus> rightFootstepStatus = new YoEnum<FootstepStatus>("rightFootstepStatus", registry,
            FootstepStatus.class);
      latestFootstepStatusEnum = new SideDependentList<>(leftFootstepStatus, rightFootstepStatus);

     attachNetworkListeningQueue(footstepStatusQueue, FootstepStatusMessage.class);
     attachNetworkListeningQueue(walkingStatusQueue, WalkingStatusMessage.class);
   }

   @Override
   public void doControl()
   {
      checkFootstepStatusAndDetermineSwingingFoot();

      if (!walkingStatusQueue.isNewPacketAvailable())
      {
         return;
      }

      WalkingStatusMessage walkingStatusLatestPacket = walkingStatusQueue.getLatestPacket();
      if (walkingStatusLatestPacket != null && walkingStatusLatestPacket.getWalkingStatus() == WalkingStatus.COMPLETED.toByte())
      {
         doneTakingSteps.set(true);
      }

      //      if (walkingStatusLatestPacket != null && walkingStatusLatestPacket.getWalkingStatus() != Status.COMPLETED)
      //      {
      ////         determineWhichFootIsSwinging();
      //         return;
      //      }
      //      else if (walkingStatusLatestPacket != null)
      //      {
      //         currentlySwingingFoot.set(null);
      //      }

      //      boolean okToSendMoreFootsteps = getLatestFootstepStatusAndCheckIfOkToSendMoreFootsteps();
      //      if (!okToSendMoreFootsteps)
      //      {
      //         return;
      //      }

   }

//   private FootstepDataListMessage footstepDataListMessage;

   public void setFootstepsToTake(FootstepDataListMessage footstepDataListMessage)
   {
//      System.out.println("Setting footstepDataListMessage in TakeSomeStepsBehavior");

//      this.footstepDataListMessage = footstepDataListMessage;

      doneTakingSteps.set(false);
      walkingStatusQueue.clear();

      if (footstepDataListMessage == null)
      {
         doneTakingSteps.set(true);
         return;
      }

      sendFootstepDataListMessage(footstepDataListMessage);
   }

   private void sendTextToSpeechPacket(String message)
   {
      TextToSpeechPacket textToSpeechPacket = MessageTools.createTextToSpeechPacket(message);
      textToSpeechPacket.setBeep(false);
      sendPacketToUI(textToSpeechPacket);
   }

   private void checkFootstepStatusAndDetermineSwingingFoot()
   {
      if (footstepStatusQueue.isNewPacketAvailable())
      {
         FootstepStatusMessage footstepStatus = footstepStatusQueue.getLatestPacket();
         RobotSide robotSide = RobotSide.fromByte(footstepStatus.getRobotSide());
         latestFootstepStatus.set(robotSide, footstepStatus);
         nextSideToSwing.set(robotSide.getOppositeSide());
      }

      currentlySwingingFoot.set(null);

      for (RobotSide side : RobotSide.values)
      {
         FootstepStatusMessage status = latestFootstepStatus.get(side);
         if (status != null)
         {
            latestFootstepStatusEnum.get(side).set(status.getFootstepStatus());

            if (status.getFootstepStatus() == FootstepStatus.STARTED.toByte())
            {
               currentlySwingingFoot.set(side);
            }
         }
      }

      for (RobotSide side : RobotSide.values)
      {
         FootstepStatusMessage status = latestFootstepStatus.get(side);
         if (status != null)
         {
            Point3D desiredFootPositionInWorld = status.getDesiredFootPositionInWorld();
            Quaternion desiredFootOrientationInWorld = status.getDesiredFootOrientationInWorld();

            desiredFootStatusPoses.get(side).setPosition(desiredFootPositionInWorld);
            desiredFootStatusPoses.get(side).setOrientation(desiredFootOrientationInWorld);

            Point3D actualFootPositionInWorld = status.getActualFootPositionInWorld();
            Quaternion actualFootOrientationInWorld = status.getActualFootOrientationInWorld();

            actualFootStatusPoses.get(side).setPosition(actualFootPositionInWorld);
            actualFootStatusPoses.get(side).setOrientation(actualFootOrientationInWorld);
         }
      }
   }

   private void sendFootstepDataListMessage(FootstepDataListMessage footstepDataListMessage)
   {
      footstepDataListMessage.setDestination(PacketDestination.UI.ordinal());
      sendPacket(footstepDataListMessage);

      footstepDataListMessage.setDestination(PacketDestination.CONTROLLER.ordinal());
      sendPacketToController(footstepDataListMessage);
      footstepSentTimer.reset();
   }

   private FootstepDataListMessage createFootstepDataListFromPlan(FootstepPlan plan, int maxNumberOfStepsToTake)
   {
      FootstepDataListMessage footstepDataListMessage = new FootstepDataListMessage();
      footstepDataListMessage.setDefaultSwingDuration(0.5);
      footstepDataListMessage.setDefaultTransferDuration(0.1);
      int lastStepIndex = Math.min(maxNumberOfStepsToTake + 1, plan.getNumberOfSteps());
      for (int i = 1; i < lastStepIndex; i++)
      {
         SimpleFootstep footstep = plan.getFootstep(i);
         footstep.getSoleFramePose(tempFirstFootstepPose);
         tempFootstepPosePosition.set(tempFirstFootstepPose.getPosition());
         tempFirstFootstepPoseOrientation.set(tempFirstFootstepPose.getOrientation());

         //         sendTextToSpeechPacket("Sending footstep " + footstep.getRobotSide() + " " + tempFootstepPosePosition + " " + tempFirstFootstepPoseOrientation);

         FootstepDataMessage firstFootstepMessage = HumanoidMessageTools.createFootstepDataMessage(footstep.getRobotSide(), new Point3D(tempFootstepPosePosition), new Quaternion(tempFirstFootstepPoseOrientation));

         footstepDataListMessage.getFootstepDataList().add().set(firstFootstepMessage);
      }

      return footstepDataListMessage;
   }

   @Override
   public boolean isDone()
   {
      return doneTakingSteps.getBooleanValue();
   }

   @Override
   public void onBehaviorEntered()
   {
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
