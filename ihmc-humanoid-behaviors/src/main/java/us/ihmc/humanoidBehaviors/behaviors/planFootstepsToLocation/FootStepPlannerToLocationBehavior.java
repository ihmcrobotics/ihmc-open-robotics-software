package us.ihmc.humanoidBehaviors.behaviors.planFootstepsToLocation;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.FootstepStatusMessage;
import controller_msgs.msg.dds.PlanarRegionsListMessage;
import controller_msgs.msg.dds.RequestPlanarRegionsListMessage;
import controller_msgs.msg.dds.RobotConfigurationData;
import controller_msgs.msg.dds.TextToSpeechPacket;
import controller_msgs.msg.dds.WalkingStatusMessage;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.packets.PlanarRegionsRequestType;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlanner;
import us.ihmc.footstepPlanning.FootstepPlannerGoal;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.footstepPlanning.SimpleFootstep;
import us.ihmc.footstepPlanning.simplePlanners.TurnWalkTurnPlanner;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.behaviors.behaviorServices.FiducialDetectorBehaviorService;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridgeInterface;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoFramePoseUsingYawPitchRoll;
import us.ihmc.yoVariables.variable.YoInteger;

public class FootStepPlannerToLocationBehavior extends AbstractBehavior
{

   private final long fiducialToTrack;
   private final String prefix = "toLocation";

   private final HumanoidReferenceFrames referenceFrames;

   private final FramePose3D tempStanceFootPose = new FramePose3D();
   private final FramePose3D tempLeftFootPose = new FramePose3D();
   private final FramePose3D tempRightFootPose = new FramePose3D();

   private final ConcurrentListeningQueue<RobotConfigurationData> robotConfigurationDataQueue;
   private final ConcurrentListeningQueue<FootstepStatusMessage> footstepStatusQueue;
   private final ConcurrentListeningQueue<WalkingStatusMessage> walkingStatusQueue;
   private final ConcurrentListeningQueue<PlanarRegionsListMessage> planarRegionsListQueue = new ConcurrentListeningQueue<>(10);

   private final SideDependentList<FootstepStatusMessage> latestFootstepStatus;
   private final SideDependentList<YoEnum<FootstepStatus>> latestFootstepStatusEnum;
   private final SideDependentList<YoFramePoseUsingYawPitchRoll> actualFootStatusPoses;
   private final SideDependentList<YoFramePoseUsingYawPitchRoll> desiredFootStatusPoses;

   private final YoEnum<RobotSide> nextSideToSwing;
   private final YoEnum<RobotSide> currentlySwingingFoot;

   private final YoInteger planarRegionsListCount = new YoInteger(prefix + "PlanarRegionsListCount", registry);
   private final YoDouble headPitchToFindFucdicial = new YoDouble(prefix + "HeadPitchToFindFucdicial", registry);

   private final YoFramePoseUsingYawPitchRoll footstepPlannerInitialStancePose;

   private final Stopwatch footstepSentTimer;

   private final FramePose3D tempFirstFootstepPose = new FramePose3D();
   private final us.ihmc.euclid.tuple3D.Point3D tempFootstepPosePosition = new us.ihmc.euclid.tuple3D.Point3D();
   private final Quaternion tempFirstFootstepPoseOrientation = new Quaternion();

   private FootstepPlanner planner;
   private final FramePose3D tempFootstepPlannerGoalPose = new FramePose3D();

   private final FootstepPlannerGoal footstepPlannerGoal;
   private FootstepPlanningResult footstepPlanningResult;

   private final FiducialDetectorBehaviorService fiducialDetectorBehaviorService;

   private final YoDouble swingTime = new YoDouble(prefix + "SwingTime", registry);
   private final YoDouble transferTime = new YoDouble(prefix + "TransferTime", registry);

   public FootStepPlannerToLocationBehavior(CommunicationBridgeInterface communicationBridge, HumanoidReferenceFrames referenceFrames, FiducialDetectorBehaviorService fiducialDetectorBehaviorService, long fiducialToTrack)
   {
      super(communicationBridge);

      this.referenceFrames = referenceFrames;
      this.planner = createFootstepPlanner();
      this.fiducialDetectorBehaviorService = fiducialDetectorBehaviorService;
      this.fiducialToTrack = fiducialToTrack;
      fiducialDetectorBehaviorService.setTargetIDToLocate(this.fiducialToTrack);

      footstepPlannerGoal = new FootstepPlannerGoal();

      headPitchToFindFucdicial.set(1.0);

      nextSideToSwing = new YoEnum<RobotSide>(prefix + "nextSideToSwing", registry, RobotSide.class, true);
      currentlySwingingFoot = new YoEnum<RobotSide>(prefix + "currentlySwingingFoot", registry, RobotSide.class, true);
      footstepPlannerInitialStancePose = new YoFramePoseUsingYawPitchRoll(prefix + "footstepPlannerInitialStancePose", ReferenceFrame.getWorldFrame(), registry);

      footstepSentTimer = new Stopwatch();
      footstepSentTimer.start();

      latestFootstepStatus = new SideDependentList<>();
      YoEnum<FootstepStatus> leftFootstepStatus = new YoEnum<FootstepStatus>(prefix + "leftFootstepStatus", registry, FootstepStatus.class);
      YoEnum<FootstepStatus> rightFootstepStatus = new YoEnum<FootstepStatus>(prefix + "rightFootstepStatus", registry, FootstepStatus.class);
      latestFootstepStatusEnum = new SideDependentList<>(leftFootstepStatus, rightFootstepStatus);

      YoFramePoseUsingYawPitchRoll desiredLeftFootstepStatusPose = new YoFramePoseUsingYawPitchRoll(prefix + "DesiredLeftFootstepStatusPose", ReferenceFrame.getWorldFrame(), registry);
      YoFramePoseUsingYawPitchRoll desiredRightFootstepStatusPose = new YoFramePoseUsingYawPitchRoll(prefix + "DesiredRightFootstepStatusPose", ReferenceFrame.getWorldFrame(), registry);
      desiredFootStatusPoses = new SideDependentList<>(desiredLeftFootstepStatusPose, desiredRightFootstepStatusPose);

      YoFramePoseUsingYawPitchRoll leftFootstepStatusPose = new YoFramePoseUsingYawPitchRoll(prefix + "LeftFootstepStatusPose", ReferenceFrame.getWorldFrame(), registry);
      YoFramePoseUsingYawPitchRoll rightFootstepStatusPose = new YoFramePoseUsingYawPitchRoll(prefix + "RightFootstepStatusPose", ReferenceFrame.getWorldFrame(), registry);
      actualFootStatusPoses = new SideDependentList<>(leftFootstepStatusPose, rightFootstepStatusPose);

      footstepStatusQueue = new ConcurrentListeningQueue<FootstepStatusMessage>(40);
      robotConfigurationDataQueue = new ConcurrentListeningQueue<RobotConfigurationData>(40);
      walkingStatusQueue = new ConcurrentListeningQueue<WalkingStatusMessage>(10);
      attachNetworkListeningQueue(robotConfigurationDataQueue, RobotConfigurationData.class);
      attachNetworkListeningQueue(footstepStatusQueue, FootstepStatusMessage.class);
      attachNetworkListeningQueue(walkingStatusQueue, WalkingStatusMessage.class);
      attachNetworkListeningQueue(planarRegionsListQueue, PlanarRegionsListMessage.class);
      
      swingTime.set(1.0);
      transferTime.set(0.3);
   }

   private void sendFootstepDataListMessage(FootstepDataListMessage footstepDataListMessage)
   {
      footstepDataListMessage.setDestination(PacketDestination.UI.ordinal());
      sendPacket(footstepDataListMessage);

      footstepDataListMessage.setDestination(PacketDestination.CONTROLLER.ordinal());
      sendPacketToController(footstepDataListMessage);
      footstepSentTimer.reset();
   }

   private void setGoalAndInitialFootClosestToGoal(FramePose3D goalPose)
   {
      tempLeftFootPose.setToZero(referenceFrames.getFootFrame(RobotSide.LEFT));
      tempRightFootPose.setToZero(referenceFrames.getFootFrame(RobotSide.RIGHT));
      tempLeftFootPose.changeFrame(ReferenceFrame.getWorldFrame());
      tempRightFootPose.changeFrame(ReferenceFrame.getWorldFrame());

      us.ihmc.euclid.tuple3D.Point3D temp = new us.ihmc.euclid.tuple3D.Point3D();
      us.ihmc.euclid.tuple3D.Point3D pointBetweenFeet = new us.ihmc.euclid.tuple3D.Point3D();
      us.ihmc.euclid.tuple3D.Vector3D vectorFromFeetToGoal = new us.ihmc.euclid.tuple3D.Vector3D();

      temp.set(tempLeftFootPose.getPosition());
      pointBetweenFeet.set(temp);
      temp.set(tempLeftFootPose.getPosition());
      pointBetweenFeet.add(temp);
      pointBetweenFeet.scale(0.5);

      vectorFromFeetToGoal.set(goalPose.getPosition());
      vectorFromFeetToGoal.sub(pointBetweenFeet);

      double headingFromFeetToGoal = Math.atan2(vectorFromFeetToGoal.getY(), vectorFromFeetToGoal.getX());
      AxisAngle goalOrientation = new AxisAngle(0.0, 0.0, 1.0, headingFromFeetToGoal);
      goalPose.setOrientation(goalOrientation);

      RobotSide stanceSide = nextSideToSwing.getEnumValue().getOppositeSide();

      if (stanceSide == RobotSide.LEFT)
      {
         tempStanceFootPose.set(tempLeftFootPose);
         goalPose.setZ(tempLeftFootPose.getZ());
      }
      else
      {
         tempStanceFootPose.set(tempRightFootPose);
         goalPose.setZ(tempRightFootPose.getZ());
      }

      sendTextToSpeechPacket("Planning footsteps from " + tempStanceFootPose + " to " + goalPose);
      footstepPlannerGoal.setGoalPoseBetweenFeet(goalPose);
      planner.setGoal(footstepPlannerGoal);

      planner.setInitialStanceFoot(tempStanceFootPose, stanceSide);
      footstepPlannerInitialStancePose.set(tempStanceFootPose);
   }

   private void checkFootstepStatusAndDetermineSwingingFoot()
   {
      while(footstepStatusQueue.isNewPacketAvailable())
      {
         FootstepStatusMessage poll = footstepStatusQueue.poll();
         RobotSide robotSide = RobotSide.fromByte(poll.getRobotSide());
         latestFootstepStatus.set(robotSide, poll);
         nextSideToSwing.set(robotSide.getOppositeSide());
      }

      currentlySwingingFoot.set(null);

      for (RobotSide side: RobotSide.values())
      {
         FootstepStatusMessage status = latestFootstepStatus.get(side);
         if(status != null)
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
            us.ihmc.euclid.tuple3D.Point3D desiredFootPositionInWorld = status.getDesiredFootPositionInWorld();
            Quaternion desiredFootOrientationInWorld = status.getDesiredFootOrientationInWorld();

            desiredFootStatusPoses.get(side).setPosition(desiredFootPositionInWorld);
            desiredFootStatusPoses.get(side).setOrientation(desiredFootOrientationInWorld);

            us.ihmc.euclid.tuple3D.Point3D actualFootPositionInWorld = status.getActualFootPositionInWorld();
            Quaternion actualFootOrientationInWorld = status.getActualFootOrientationInWorld();

            actualFootStatusPoses.get(side).setPosition(actualFootPositionInWorld);
            actualFootStatusPoses.get(side).setOrientation(actualFootOrientationInWorld);
         }
      }

   }


   private void requestPlanarRegionsList()
   {
      RequestPlanarRegionsListMessage requestPlanarRegionsListMessage = MessageTools.createRequestPlanarRegionsListMessage(PlanarRegionsRequestType.SINGLE_UPDATE);
      requestPlanarRegionsListMessage.setDestination(PacketDestination.REA_MODULE.ordinal());
      sendPacket(requestPlanarRegionsListMessage);
   }

   private void updatePlannerIfPlanarRegionsListIsAvailable()
   {
      if (planarRegionsListQueue.isNewPacketAvailable())
      {
         planarRegionsListCount.increment();

         PlanarRegionsListMessage planarRegionsListMessage = planarRegionsListQueue.getLatestPacket();
         PlanarRegionsList planarRegionsList = PlanarRegionMessageConverter.convertToPlanarRegionsList(planarRegionsListMessage);
         planner.setPlanarRegions(planarRegionsList);
      }
   }

   private FootstepPlanner createFootstepPlanner()
   {
      FootstepPlanner planner = new TurnWalkTurnPlanner();

      return planner;
   }

   private void sendTextToSpeechPacket(String message)
   {
      TextToSpeechPacket textToSpeechPacket = MessageTools.createTextToSpeechPacket(message);
      sendPacketToUI(textToSpeechPacket);
   }

   private void pitchHeadToFindFiducial()
   {

   }

   private void pitchHeadToCenterFiducial()
   {

   }

   private FootstepDataListMessage createFootstepDataListFromPlan(FootstepPlan plan, int maxNumberOfStepsToTake)
   {
      FootstepDataListMessage footstepDataListMessage = new FootstepDataListMessage();
      footstepDataListMessage.setDefaultSwingDuration(swingTime.getDoubleValue());
      footstepDataListMessage.setDefaultTransferDuration(transferTime.getDoubleValue());
      int lastStepIndex = Math.min(maxNumberOfStepsToTake + 1, plan.getNumberOfSteps());
      for (int i = 1; i < lastStepIndex; i++)
      {
         SimpleFootstep footstep = plan.getFootstep(i);
         footstep.getSoleFramePose(tempFirstFootstepPose);
         tempFootstepPosePosition.set(tempFirstFootstepPose.getPosition());
         tempFirstFootstepPoseOrientation.set(tempFirstFootstepPose.getOrientation());

         //         sendTextToSpeechPacket("Sending footstep " + footstep.getRobotSide() + " " + tempFootstepPosePosition + " " + tempFirstFootstepPoseOrientation);

         FootstepDataMessage firstFootstepMessage = HumanoidMessageTools.createFootstepDataMessage(footstep.getRobotSide(), new us.ihmc.euclid.tuple3D.Point3D(tempFootstepPosePosition), new Quaternion(tempFirstFootstepPoseOrientation));
         footstepDataListMessage.getFootstepDataList().add().set(firstFootstepMessage);
      }

      return footstepDataListMessage;
   }

   @Override
   public boolean isDone()
   {
      return false;
   }

   @Override
   public void doControl()
   {
      checkFootstepStatusAndDetermineSwingingFoot();

      if (footstepSentTimer.totalElapsed() < 0.5)
      {
         return;
      }

      updatePlannerIfPlanarRegionsListIsAvailable();
      requestPlanarRegionsList();

      if (!fiducialDetectorBehaviorService.getGoalHasBeenLocated())
      {
         sendTextToSpeechPacket("Fiducial not located.");
         footstepSentTimer.reset();
         pitchHeadToFindFiducial();
         return;
      }

      pitchHeadToCenterFiducial();

      fiducialDetectorBehaviorService.getReportedGoalPoseWorldFrame(tempFootstepPlannerGoalPose);
      setGoalAndInitialFootClosestToGoal(tempFootstepPlannerGoalPose);


      planner.plan();
      FootstepPlan plan = planner.getPlan();

      if (plan == null)
      {
         sendTextToSpeechPacket("No Plan was found!");
         footstepSentTimer.reset();
         return;
      }

      int maxNumberOfStepsToTake = 5;
      FootstepDataListMessage footstepDataListMessage = createFootstepDataListFromPlan(plan, maxNumberOfStepsToTake);
      sendFootstepDataListMessage(footstepDataListMessage);
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