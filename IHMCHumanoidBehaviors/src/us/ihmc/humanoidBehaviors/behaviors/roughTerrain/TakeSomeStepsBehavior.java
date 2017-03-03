package us.ihmc.humanoidBehaviors.behaviors.roughTerrain;

import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.TextToSpeechPacket;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.SimpleFootstep;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidRobotics.communication.packets.ExecutionMode;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage.FootstepOrigin;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingStatusMessage;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.math.frames.YoFramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.time.YoStopwatch;

public class TakeSomeStepsBehavior extends AbstractBehavior
{
   private final String prefix = "takeSomeSteps";

   private final FullHumanoidRobotModel fullRobotModel;
   private final HumanoidReferenceFrames referenceFrames;

   private final ConcurrentListeningQueue<WalkingStatusMessage> walkingStatusQueue = new ConcurrentListeningQueue<WalkingStatusMessage>(40);
   private final ConcurrentListeningQueue<FootstepStatus> footstepStatusQueue = new ConcurrentListeningQueue<FootstepStatus>(40);

   private final SideDependentList<FootstepStatus> latestFootstepStatus;
   private final SideDependentList<EnumYoVariable<FootstepStatus.Status>> latestFootstepStatusEnum;
   private final SideDependentList<YoFramePose> desiredFootStatusPoses;
   private final SideDependentList<YoFramePose> actualFootStatusPoses;

   private final EnumYoVariable<RobotSide> nextSideToSwing;
   private final EnumYoVariable<RobotSide> currentlySwingingFoot;

   private final BooleanYoVariable doneTakingSteps;

   private final YoFramePose footstepPlannerInitialStepPose;
   private final YoFramePose footstepPlannerGoalPose;

   private final FramePose tempFootstepPlannerGoalPose = new FramePose();
   private final FramePose leftFootPose = new FramePose();
   private final FramePose rightFootPose = new FramePose();
   private final FramePose tempStanceFootPose = new FramePose();
   private final FramePose tempFirstFootstepPose = new FramePose();
   private final Point3D tempFootstepPosePosition = new Point3D();
   private final Quaternion tempFirstFootstepPoseOrientation = new Quaternion();
   private final YoStopwatch footstepSentTimer;

   public TakeSomeStepsBehavior(DoubleYoVariable yoTime, CommunicationBridge behaviorCommunicationBridge, FullHumanoidRobotModel fullRobotModel, HumanoidReferenceFrames referenceFrames)
   {
      super(TakeSomeStepsBehavior.class.getSimpleName(), behaviorCommunicationBridge);

      this.fullRobotModel = fullRobotModel;
      this.referenceFrames = referenceFrames;

      nextSideToSwing = new EnumYoVariable<>("nextSideToSwing", registry, RobotSide.class);
      nextSideToSwing.set(RobotSide.LEFT);

      doneTakingSteps = new BooleanYoVariable(prefix + "DoneTakingSteps", registry);

      currentlySwingingFoot = new EnumYoVariable<>("currentlySwingingFoot", registry, RobotSide.class, true);

      footstepSentTimer = new YoStopwatch(yoTime);
      footstepSentTimer.start();

      footstepPlannerGoalPose = new YoFramePose(prefix + "FootstepGoalPose", ReferenceFrame.getWorldFrame(), registry);
      footstepPlannerInitialStepPose = new YoFramePose(prefix + "InitialStepPose", ReferenceFrame.getWorldFrame(), registry);

      YoFramePose desiredLeftFootstepStatusPose = new YoFramePose(prefix + "DesiredLeftFootstepStatusPose", ReferenceFrame.getWorldFrame(), registry);
      YoFramePose desiredRightFootstepStatusPose = new YoFramePose(prefix + "DesiredRightFootstepStatusPose", ReferenceFrame.getWorldFrame(), registry);
      desiredFootStatusPoses = new SideDependentList<>(desiredLeftFootstepStatusPose, desiredRightFootstepStatusPose);

      YoFramePose leftFootstepStatusPose = new YoFramePose(prefix + "LeftFootstepStatusPose", ReferenceFrame.getWorldFrame(), registry);
      YoFramePose rightFootstepStatusPose = new YoFramePose(prefix + "RightFootstepStatusPose", ReferenceFrame.getWorldFrame(), registry);
      actualFootStatusPoses = new SideDependentList<>(leftFootstepStatusPose, rightFootstepStatusPose);

      latestFootstepStatus = new SideDependentList<>();

      EnumYoVariable<FootstepStatus.Status> leftFootstepStatus = new EnumYoVariable<FootstepStatus.Status>("leftFootstepStatus", registry,
            FootstepStatus.Status.class);
      EnumYoVariable<FootstepStatus.Status> rightFootstepStatus = new EnumYoVariable<FootstepStatus.Status>("rightFootstepStatus", registry,
            FootstepStatus.Status.class);
      latestFootstepStatusEnum = new SideDependentList<>(leftFootstepStatus, rightFootstepStatus);

     attachNetworkListeningQueue(footstepStatusQueue, FootstepStatus.class);
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
      if (walkingStatusLatestPacket != null && walkingStatusLatestPacket.getWalkingStatus() == WalkingStatusMessage.Status.COMPLETED)
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
      TextToSpeechPacket textToSpeechPacket = new TextToSpeechPacket(message);
      textToSpeechPacket.setbeep(false);
      sendPacketToUI(textToSpeechPacket);
   }

   private void checkFootstepStatusAndDetermineSwingingFoot()
   {
      if (footstepStatusQueue.isNewPacketAvailable())
      {
         FootstepStatus footstepStatus = footstepStatusQueue.getLatestPacket();
         latestFootstepStatus.set(footstepStatus.getRobotSide(), footstepStatus);
         nextSideToSwing.set(footstepStatus.getRobotSide().getOppositeSide());
      }

      currentlySwingingFoot.set(null);

      for (RobotSide side : RobotSide.values)
      {
         FootstepStatus status = latestFootstepStatus.get(side);
         if (status != null)
         {
            latestFootstepStatusEnum.get(side).set(status.getStatus());

            if (status.getStatus() == FootstepStatus.Status.STARTED)
            {
               currentlySwingingFoot.set(side);
            }
         }
      }

      for (RobotSide side : RobotSide.values)
      {
         FootstepStatus status = latestFootstepStatus.get(side);
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
      footstepDataListMessage.setDestination(PacketDestination.UI);
      sendPacket(footstepDataListMessage);

      footstepDataListMessage.setDestination(PacketDestination.CONTROLLER);
      sendPacketToController(footstepDataListMessage);
      footstepSentTimer.reset();
   }

   private FootstepDataListMessage createFootstepDataListFromPlan(FootstepPlan plan, int maxNumberOfStepsToTake)
   {
      FootstepDataListMessage footstepDataListMessage = new FootstepDataListMessage();
      footstepDataListMessage.setDefaultSwingTime(0.5);
      footstepDataListMessage.setDefaultTransferTime(0.1);
      int lastStepIndex = Math.min(maxNumberOfStepsToTake + 1, plan.getNumberOfSteps());
      for (int i = 1; i < lastStepIndex; i++)
      {
         SimpleFootstep footstep = plan.getFootstep(i);
         footstep.getSoleFramePose(tempFirstFootstepPose);
         tempFirstFootstepPose.getPosition(tempFootstepPosePosition);
         tempFirstFootstepPose.getOrientation(tempFirstFootstepPoseOrientation);

         //         sendTextToSpeechPacket("Sending footstep " + footstep.getRobotSide() + " " + tempFootstepPosePosition + " " + tempFirstFootstepPoseOrientation);

         FootstepDataMessage firstFootstepMessage = new FootstepDataMessage(footstep.getRobotSide(), new Point3D(tempFootstepPosePosition),
               new Quaternion(tempFirstFootstepPoseOrientation));
         firstFootstepMessage.setOrigin(FootstepOrigin.AT_SOLE_FRAME);

         footstepDataListMessage.add(firstFootstepMessage);
      }

      footstepDataListMessage.setExecutionMode(ExecutionMode.OVERRIDE);
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
