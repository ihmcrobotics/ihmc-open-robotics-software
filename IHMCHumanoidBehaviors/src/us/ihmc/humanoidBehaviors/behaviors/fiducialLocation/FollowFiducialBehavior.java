package us.ihmc.humanoidBehaviors.behaviors.fiducialLocation;

import us.ihmc.commons.time.Stopwatch;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.packets.PlanarRegionsListMessage;
import us.ihmc.communication.packets.RequestPlanarRegionsListMessage;
import us.ihmc.communication.packets.RequestPlanarRegionsListMessage.RequestType;
import us.ihmc.communication.packets.TextToSpeechPacket;
import us.ihmc.communication.packets.UIPositionCheckerPacket;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlanner;
import us.ihmc.footstepPlanning.FootstepPlannerGoal;
import us.ihmc.footstepPlanning.FootstepPlannerGoalType;
import us.ihmc.footstepPlanning.SimpleFootstep;
import us.ihmc.footstepPlanning.graphSearch.BipedalFootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.PlanarRegionBipedalFootstepPlanner;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.behaviors.goalLocation.GoalDetectorBehaviorService;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidRobotics.communication.packets.ExecutionMode;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage.FootstepOrigin;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.humanoidRobotics.communication.packets.walking.HeadTrajectoryMessage;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.math.frames.YoFramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class FollowFiducialBehavior extends AbstractBehavior
{
   private final String prefix = "followFiducial";

   private final GoalDetectorBehaviorService fiducialDetectorBehaviorService;
   private final FullHumanoidRobotModel fullRobotModel;
   private final HumanoidReferenceFrames referenceFrames;

   //   private final ConcurrentListeningQueue<RobotConfigurationData> robotConfigurationDataQueue = new ConcurrentListeningQueue<RobotConfigurationData>(10);
   private final ConcurrentListeningQueue<FootstepStatus> footstepStatusQueue = new ConcurrentListeningQueue<FootstepStatus>(40);
   //   private final ConcurrentListeningQueue<WalkingStatusMessage> walkingStatusQueue = new ConcurrentListeningQueue<WalkingStatusMessage>(10);
   private final ConcurrentListeningQueue<PlanarRegionsListMessage> planarRegionsListQueue = new ConcurrentListeningQueue<>(10);

   private final SideDependentList<FootstepStatus> latestFootstepStatus;
   private final SideDependentList<EnumYoVariable<FootstepStatus.Status>> latestFootstepStatusEnum;
   private final SideDependentList<YoFramePose> desiredFootStatusPoses;
   private final SideDependentList<YoFramePose> actualFootStatusPoses;

   private final DoubleYoVariable headPitchToFindFucdicial = new DoubleYoVariable(prefix + "HeadPitchToFindFucdicial", registry);
   private final DoubleYoVariable headPitchToCenterFucdicial = new DoubleYoVariable(prefix + "HeadPitchToCenterFucdicial", registry);

   private final IntegerYoVariable planarRegionsListCount = new IntegerYoVariable(prefix + "PlanarRegionsListCount", registry);

   private final EnumYoVariable<RobotSide> nextSideToSwing;
   private final EnumYoVariable<RobotSide> currentlySwingingFoot;

   private final FootstepPlanner footstepPlanner;

   private final YoFramePose footstepPlannerInitialStepPose;
   private final YoFramePose footstepPlannerGoalPose;

   private final FootstepPlannerGoal footstepPlannerGoal = new FootstepPlannerGoal();
   private final FramePose tempFootstepPlannerGoalPose = new FramePose();
   private final FramePose leftFootPose = new FramePose();
   private final FramePose rightFootPose = new FramePose();
   private final FramePose tempStanceFootPose = new FramePose();
   private final FramePose tempFirstFootstepPose = new FramePose();
   private final Point3D tempFootstepPosePosition = new Point3D();
   private final Quaternion tempFirstFootstepPoseOrientation = new Quaternion();
   private final Stopwatch footstepSentTimer = new Stopwatch();

   public FollowFiducialBehavior(CommunicationBridge behaviorCommunicationBridge, FullHumanoidRobotModel fullRobotModel,
                                 HumanoidReferenceFrames referenceFrames, GoalDetectorBehaviorService goalDetectorBehaviorService)
   {
      super(FollowFiducialBehavior.class.getSimpleName() + "_" + goalDetectorBehaviorService.getClass().getSimpleName(), behaviorCommunicationBridge);

      this.fullRobotModel = fullRobotModel;
      this.referenceFrames = referenceFrames;

      this.fiducialDetectorBehaviorService = goalDetectorBehaviorService;
      addBehaviorService(fiducialDetectorBehaviorService);

      headPitchToFindFucdicial.set(1.0);

      footstepPlanner = createFootstepPlanner();

      nextSideToSwing = new EnumYoVariable<>("nextSideToSwing", registry, RobotSide.class);
      nextSideToSwing.set(RobotSide.LEFT);

      currentlySwingingFoot = new EnumYoVariable<>("currentlySwingingFoot", registry, RobotSide.class, true);

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

      //      behaviorCommunicationBridge.attachNetworkListeningQueue(robotConfigurationDataQueue, RobotConfigurationData.class);
      attachNetworkListeningQueue(footstepStatusQueue, FootstepStatus.class);
      //      behaviorCommunicationBridge.attachNetworkListeningQueue(walkingStatusQueue, WalkingStatusMessage.class);
      attachNetworkListeningQueue(planarRegionsListQueue, PlanarRegionsListMessage.class);
   }

   private FootstepPlanner createFootstepPlanner()
   {
      BipedalFootstepPlannerParameters parameters = new BipedalFootstepPlannerParameters(registry);
      parameters.setMaximumStepReach(0.4);
      parameters.setMaximumStepZ(0.25);
      parameters.setMaximumStepXWhenForwardAndDown(0.25);
      parameters.setMaximumStepZWhenForwardAndDown(0.25);
      parameters.setMaximumStepYaw(0.25);
      parameters.setMaximumStepWidth(0.4);
      parameters.setMinimumStepWidth(0.15);
      parameters.setMinimumFootholdPercent(0.8);

      double idealFootstepLength = 0.4;
      double idealFootstepWidth = 0.25;
      parameters.setIdealFootstep(idealFootstepLength, idealFootstepWidth);

      PlanarRegionBipedalFootstepPlanner planner = new PlanarRegionBipedalFootstepPlanner(parameters, registry);

      SideDependentList<ConvexPolygon2d> footPolygonsInSoleFrame = createDefaultFootPolygons();
      planner.setFeetPolygons(footPolygonsInSoleFrame);

      planner.setMaximumNumberOfNodesToExpand(500);

      return planner;
   }

   public static ConvexPolygon2d createDefaultFootPolygon()
   {
      double footLength = 0.2;
      double footWidth = 0.1;

      ConvexPolygon2d footPolygon = new ConvexPolygon2d();
      footPolygon.addVertex(footLength / 2.0, footWidth / 2.0);
      footPolygon.addVertex(footLength / 2.0, -footWidth / 2.0);
      footPolygon.addVertex(-footLength / 2.0, footWidth / 2.0);
      footPolygon.addVertex(-footLength / 2.0, -footWidth / 2.0);
      footPolygon.update();

      return footPolygon;
   }

   public static SideDependentList<ConvexPolygon2d> createDefaultFootPolygons()
   {
      SideDependentList<ConvexPolygon2d> footPolygons = new SideDependentList<>();
      for (RobotSide side : RobotSide.values)
         footPolygons.put(side, createDefaultFootPolygon());
      return footPolygons;
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

      //      WalkingStatusMessage walkingStatusLatestPacket = walkingStatusQueue.getLatestPacket();
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

      if (!fiducialDetectorBehaviorService.getGoalHasBeenLocated())
      {
         sendTextToSpeechPacket("Fiducial not located.");
         footstepSentTimer.reset();
         pitchHeadToFindFiducial();
         return;
      }

      pitchHeadToCenterFiducial();

      fiducialDetectorBehaviorService.getReportedGoalPoseWorldFrame(tempFootstepPlannerGoalPose);
      setGoalAndInitialStanceFootToBeClosestToGoal(tempFootstepPlannerGoalPose);

      footstepPlanner.plan();
      FootstepPlan plan = footstepPlanner.getPlan();

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

   private void sendTextToSpeechPacket(String message)
   {
      TextToSpeechPacket textToSpeechPacket = new TextToSpeechPacket(message);
      textToSpeechPacket.setbeep(false);
      sendPacketToUI(textToSpeechPacket);
   }

   private void pitchHeadToFindFiducial()
   {
      //      headPitchToFindFucdicial.mul(-1.0);
      //      AxisAngle4d axisAngleOrientation = new AxisAngle4d(new Vector3d(0.0, 1.0, 0.0), headPitchToFindFucdicial.getDoubleValue());
      //
      //      Quat4d headOrientation = new Quat4d();
      //      headOrientation.set(axisAngleOrientation);
      //      sendHeadTrajectoryMessage(1.0, headOrientation);
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
      HeadTrajectoryMessage headTrajectoryMessage = new HeadTrajectoryMessage(trajectoryTime, desiredOrientation);

      headTrajectoryMessage.setDestination(PacketDestination.UI);
      sendPacket(headTrajectoryMessage);

      headTrajectoryMessage.setDestination(PacketDestination.CONTROLLER);
      sendPacketToController(headTrajectoryMessage);
      //      footstepSentTimer.reset();
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

   private void requestPlanarRegionsList()
   {
      RequestPlanarRegionsListMessage requestPlanarRegionsListMessage = new RequestPlanarRegionsListMessage(RequestType.SINGLE_UPDATE);
      requestPlanarRegionsListMessage.setDestination(PacketDestination.REA_MODULE);
      sendPacket(requestPlanarRegionsListMessage);
   }

   private void updatePlannerIfPlanarRegionsListIsAvailable()
   {
      if (planarRegionsListQueue.isNewPacketAvailable())
      {
         planarRegionsListCount.increment();

         PlanarRegionsListMessage planarRegionsListMessage = planarRegionsListQueue.getLatestPacket();
         PlanarRegionsList planarRegionsList = PlanarRegionMessageConverter.convertToPlanarRegionsList(planarRegionsListMessage);
         footstepPlanner.setPlanarRegions(planarRegionsList);
      }
   }

   private void setGoalAndInitialStanceFootToBeClosestToGoal(FramePose goalPose)
   {
//      sendPacketToUI(new UIPositionCheckerPacket(goalPose.getFramePointCopy().getPoint(), goalPose.getFrameOrientationCopy().getQuaternion()));

      leftFootPose.setToZero(referenceFrames.getFootFrame(RobotSide.LEFT));
      rightFootPose.setToZero(referenceFrames.getFootFrame(RobotSide.RIGHT));
      leftFootPose.changeFrame(ReferenceFrame.getWorldFrame());
      rightFootPose.changeFrame(ReferenceFrame.getWorldFrame());

      Point3D temp = new Point3D();
      Point3D pointBetweenFeet = new Point3D();
      Point3D goalPosition = new Point3D();
      Point3D shorterGoalPosition = new Point3D();
      Vector3D vectorFromFeetToGoal = new Vector3D();

      leftFootPose.getPosition(temp);
      pointBetweenFeet.set(temp);
      rightFootPose.getPosition(temp);
      pointBetweenFeet.add(temp);
      pointBetweenFeet.scale(0.5);

      goalPose.getPosition(goalPosition);
      vectorFromFeetToGoal.sub(goalPosition, pointBetweenFeet);

      double shorterGoalLength = 2.0;

      if (vectorFromFeetToGoal.length() > shorterGoalLength)
      {
         vectorFromFeetToGoal.scale(shorterGoalLength / vectorFromFeetToGoal.length());
      }
      shorterGoalPosition.set(pointBetweenFeet);
      shorterGoalPosition.add(vectorFromFeetToGoal);
      goalPose.setPosition(shorterGoalPosition);

      double headingFromFeetToGoal = Math.atan2(vectorFromFeetToGoal.getY(), vectorFromFeetToGoal.getX());
      AxisAngle goalOrientation = new AxisAngle(0.0, 0.0, 1.0, headingFromFeetToGoal);
      goalPose.setOrientation(goalOrientation);

      RobotSide stanceSide;
      if (currentlySwingingFoot.getEnumValue() != null)
      {
         stanceSide = currentlySwingingFoot.getEnumValue();

         this.desiredFootStatusPoses.get(stanceSide).getFramePose(tempStanceFootPose);
         goalPose.setZ(tempStanceFootPose.getZ());
      }

      else
      {
         stanceSide = nextSideToSwing.getEnumValue().getOppositeSide();

         if (stanceSide == RobotSide.LEFT)
         {
            tempStanceFootPose.set(leftFootPose);
            goalPose.setZ(leftFootPose.getZ());
         }
         else
         {
            tempStanceFootPose.set(rightFootPose);
            goalPose.setZ(rightFootPose.getZ());
         }
      }

      //      sendTextToSpeechPacket("Planning footsteps from " + tempStanceFootPose + " to " + goalPose);
      sendTextToSpeechPacket("Planning footsteps to the fiducial");
      footstepPlannerGoal.setGoalPoseBetweenFeet(goalPose);

      // For now, just get close to the Fiducial, don't need to get exactly on it.
      Point2D xyGoal = new Point2D();
      xyGoal.setX(goalPose.getX());
      xyGoal.setY(goalPose.getY());
      double distanceFromXYGoal = 1.0;
      footstepPlannerGoal.setXYGoal(xyGoal, distanceFromXYGoal);
//      footstepPlannerGoal.setFootstepPlannerGoalType(FootstepPlannerGoalType.POSE_BETWEEN_FEET);
      footstepPlannerGoal.setFootstepPlannerGoalType(FootstepPlannerGoalType.CLOSE_TO_XY_POSITION);

    sendPacketToUI(new UIPositionCheckerPacket(new Point3D(xyGoal.getX(), xyGoal.getY(), leftFootPose.getZ()), new Quaternion()));

      footstepPlanner.setGoal(footstepPlannerGoal);

      footstepPlanner.setInitialStanceFoot(tempStanceFootPose, stanceSide);

      footstepPlannerGoalPose.set(goalPose);
      footstepPlannerInitialStepPose.set(tempStanceFootPose);
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
      return false;
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