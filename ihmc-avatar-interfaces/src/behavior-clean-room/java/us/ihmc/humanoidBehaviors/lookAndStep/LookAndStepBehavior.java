package us.ihmc.humanoidBehaviors.lookAndStep;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.WalkingStatusMessage;
import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.communication.util.SimpleTimer;
import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.graphSearch.VisibilityGraphPathPlanner;
import us.ihmc.humanoidBehaviors.tools.RemoteEnvironmentMapInterface;
import us.ihmc.communication.RemoteREAInterface;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.footstepPlanning.*;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.log.FootstepPlannerLogger;
import us.ihmc.humanoidBehaviors.BehaviorDefinition;
import us.ihmc.humanoidBehaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.tools.BehaviorHelper;
import us.ihmc.humanoidBehaviors.tools.HumanoidRobotState;
import us.ihmc.humanoidBehaviors.tools.RemoteHumanoidRobotInterface;
import us.ihmc.log.LogTools;
import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.messager.MessagerAPIFactory.Category;
import us.ihmc.messager.MessagerAPIFactory.CategoryTheme;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersBasics;
import us.ihmc.pathPlanning.visibilityGraphs.postProcessing.BodyPathPostProcessor;
import us.ihmc.pathPlanning.visibilityGraphs.postProcessing.ObstacleAvoidanceProcessor;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.time.LocalDateTime;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehavior.LookAndStepBehaviorAPI.*;

public class LookAndStepBehavior implements BehaviorInterface
{
   public static final BehaviorDefinition DEFINITION = new BehaviorDefinition("Look and Step", LookAndStepBehavior::new, create());

   private final LookAndStepBehaviorParameters lookAndStepParameters = new LookAndStepBehaviorParameters();

   private final BehaviorHelper helper;
   private final RemoteREAInterface rea;
   private final RemoteEnvironmentMapInterface environmentMap;
   private final FootstepPlannerParametersBasics footstepPlannerParameters;
   private final RemoteHumanoidRobotInterface robot;
   private final FootstepPlanningModule footstepPlanningModule;
   private final VisibilityGraphsParametersBasics visibilityGraphParameters;

   private final AtomicReference<Boolean> operatorReviewEnabledInput;
   private final TypedNotification<Boolean> approvalNotification;
   private final FramePose3D goalPoseBetweenFeet = new FramePose3D();
   private RobotSide lastStanceSide = null;
   private FramePose3D leftFootPoseTemp = new FramePose3D();
   private FramePose3D rightFootPoseTemp = new FramePose3D();

   private PlanarRegionsList bodyPathModulePlanarRegionsList;
   private SimpleTimer bodyPathModulePlanarRegionExpirationTimer = new SimpleTimer();
   private SimpleTimer bodyPathModuleFailedTimer = new SimpleTimer();
   private volatile Pose3D bodyPathModuleGoalInput;

   // TODO: Add String for what is being reviewed
   private volatile boolean bodyPathBeingReviewed = false;

   private PlanarRegionsList footstepPlanningNearRegions;
   private SimpleTimer footstepPlanningNearRegionsExpirationTimer = new SimpleTimer();
   private volatile ArrayList<Pose3D> footstepPlanningBodyPathPlan;
   private SimpleTimer footstepPlanningModuleFailedTimer = new SimpleTimer();

   private volatile boolean footstepPlanBeingReviewed = false;

   private FootstepPlan robotWalkingModuleFootstepPlan;

   public LookAndStepBehavior(BehaviorHelper helper)
   {
      this.helper = helper;
      rea = helper.getOrCreateREAInterface();
      environmentMap = helper.getOrCreateEnvironmentMapInterface();
      robot = helper.getOrCreateRobotInterface();
      footstepPlanningModule = helper.getOrCreateFootstepPlanner();

      visibilityGraphParameters = helper.getRobotModel().getVisibilityGraphsParameters();

      helper.createUICallback(LookAndStepParameters, lookAndStepParameters::setAllFromStrings);
      footstepPlannerParameters = helper.getRobotModel().getFootstepPlannerParameters();
      helper.createUICallback(FootstepPlannerParameters, footstepPlannerParameters::setAllFromStrings);

      operatorReviewEnabledInput = helper.createUIInput(OperatorReviewEnabled, true);
      approvalNotification = helper.createUITypedNotification(Approval);

      // TODO: Want to be able to wire up behavior here and see all present modules

      // build event flow
      rea.addPlanarRegionsListCallback(this::bodyPathModuleAcceptREARegions);
      helper.createUICallback(GoalInput, this::bodyPathModuleAcceptGoalPlacement);

      environmentMap.addPlanarRegionsListCallback(this::footstepPlanningAcceptNearPlanarRegions);

//      helper.createUICallback(RePlan, this::);

      helper.setCommunicationCallbacksEnabled(true);
      helper.setCommunicationCallbacksEnabled(false);

   }

   @Override
   public void setEnabled(boolean enabled)
   {
      LogTools.info("Look and step behavior selected = {}", enabled);

      helper.setCommunicationCallbacksEnabled(enabled);

      robot.pitchHeadWithRespectToChest(0.38);
//      Commanding neck trajectory: slider: 43.58974358974359 angle: 0.3824055641025641
   }

   // TODO: Use data input-callback classes ?

   private void bodyPathModuleAcceptREARegions(PlanarRegionsList planarRegionsList)
   {
//      LogTools.debug("bodyPathModuleAcceptREARegions, {}", planarRegionsList);
      bodyPathModulePlanarRegionsList = planarRegionsList;
      bodyPathModulePlanarRegionExpirationTimer.reset();

//      helper.publishToUI(MapRegionsForUI, bodyPathModulePlanarRegionsList); // TODO This takes forever?

      bodyPathModuleEvaluteAndRun();
   }

   private void bodyPathModuleAcceptGoalPlacement(Pose3D goalInput)
   {
      LogTools.debug("bodyPathModuleAcceptGoalPlacement, {}", goalInput);
      bodyPathModuleGoalInput = goalInput;

      bodyPathModuleEvaluteAndRun();
   }

   private void bodyPathModuleEvaluteAndRun()
   {
      // TODO Goal input comes from user click / Kryo thread
      Pose3D goal = bodyPathModuleGoalInput;
      boolean hasGoal = goal != null && !goal.containsNaN();
      if (!hasGoal)
      {
         LogTools.warn("Body path: does not have goal");
         LogTools.debug("Sending planar regions to UI: {}: {}", LocalDateTime.now(), bodyPathModulePlanarRegionsList.hashCode());
         helper.publishToUI(MapRegionsForUI, bodyPathModulePlanarRegionsList);
         return;
      }

      boolean regionsOK = bodyPathModulePlanarRegionsList != null
                      && !bodyPathModulePlanarRegionExpirationTimer.isPastOrNaN(lookAndStepParameters.getPlanarRegionsExpiration())
                      && !bodyPathModulePlanarRegionsList.isEmpty();
      if (!regionsOK)
      {
         LogTools.warn("Body path: Regions not OK: {}, timePassed: {}, isEmpty: {}",
                       bodyPathModulePlanarRegionsList,
                       bodyPathModulePlanarRegionExpirationTimer.timePassedSinceReset(),
                       bodyPathModulePlanarRegionsList == null ? null : bodyPathModulePlanarRegionsList.isEmpty());
         return;
      }

      ArrayList<Pose3D> bodyPathPlan = footstepPlanningBodyPathPlan;
      if (hasGoal && bodyPathPlan != null && !bodyPathPlan.isEmpty())
      {
         LogTools.warn("Body path: Robot has goal and a body path {}, isEmpty: {}",
                       bodyPathPlan,
                       bodyPathPlan == null ? null : true);
         return;
      }

      // TODO: This could be "run recently" instead of failed recently
      boolean failedRecently = !bodyPathModuleFailedTimer.isPastOrNaN(lookAndStepParameters.get(LookAndStepBehaviorParameters.waitTimeAfterPlanFailed));
      if (failedRecently)
      {
         LogTools.warn("Body path: failedRecently = true");
         return;
      }

      if (bodyPathBeingReviewed)
      {
         LogTools.debug("Body path: bodyPathBeingReviewed = true");
         return;
      }

      // TODO: Add robot standing still for 20s for real robot

      helper.publishToUI(MapRegionsForUI, bodyPathModulePlanarRegionsList);

      LogTools.info("Planning body path...");

      // calculate and send body path plan
      visibilityGraphParameters.setIncludePreferredExtrusions(false);
      BodyPathPostProcessor pathPostProcessor = new ObstacleAvoidanceProcessor(visibilityGraphParameters);
      VisibilityGraphPathPlanner bodyPathPlanner = new VisibilityGraphPathPlanner(visibilityGraphParameters,
                                                                                  pathPostProcessor,
                                                                                  new YoVariableRegistry(getClass().getSimpleName()));

      bodyPathPlanner.setGoal(goal);
      bodyPathPlanner.setPlanarRegionsList(bodyPathModulePlanarRegionsList);
      HumanoidRobotState humanoidRobotState = robot.pollHumanoidRobotState();
      leftFootPoseTemp.setToZero(humanoidRobotState.getSoleFrame(RobotSide.LEFT));
      rightFootPoseTemp.setToZero(humanoidRobotState.getSoleFrame(RobotSide.RIGHT));
      leftFootPoseTemp.changeFrame(ReferenceFrame.getWorldFrame());
      rightFootPoseTemp.changeFrame(ReferenceFrame.getWorldFrame());
      bodyPathPlanner.setStanceFootPoses(leftFootPoseTemp, rightFootPoseTemp);
      Stopwatch stopwatch = new Stopwatch().start();
      final ArrayList<Pose3D> bodyPathPlanForReview = new ArrayList<>(); // TODO Review making this final
      bodyPathPlanner.planWaypoints();
      LogTools.info("Body path planning took {}", stopwatch.totalElapsed()); // 0.1 s
      //      bodyPathPlan = bodyPathPlanner.getWaypoints();
      if (bodyPathPlanner.getWaypoints() != null)
      {
         for (Pose3DReadOnly poseWaypoint : bodyPathPlanner.getWaypoints())
         {
            bodyPathPlanForReview.add(new Pose3D(poseWaypoint));
         }
         helper.publishToUI(BodyPathPlanForUI, bodyPathPlanForReview);
      }

      if (bodyPathPlanForReview.size() >= 2)
      {
         if (operatorReviewEnabledInput.get())
         {
            ThreadTools.startAsDaemon(() -> reviewBodyPathPlan(bodyPathPlanForReview), "BodyPathReview");
         }
         else
         {
            footstepPlanningAcceptBodyPath(bodyPathPlanForReview);
         }
      }
      else
      {
         bodyPathModuleFailedTimer.reset();
      }
   }

   private void reviewBodyPathPlan(ArrayList<Pose3D> bodyPathPlan) // TODO: Extract review logic?
   {
      bodyPathBeingReviewed = true;
      LogTools.info("Waiting for body path operator review...");
      boolean approved = approvalNotification.blockingPoll();
      LogTools.info("Operator reviewed: {}", approved);
      bodyPathBeingReviewed = false;

      if (approved)
      {
         footstepPlanningAcceptBodyPath(bodyPathPlan);
      }
   }

   private void footstepPlanningAcceptNearPlanarRegions(PlanarRegionsList planarRegionsNear)
   {
      footstepPlanningNearRegions = planarRegionsNear;
      footstepPlanningNearRegionsExpirationTimer.reset();
   }

   private void footstepPlanningAcceptBodyPath(ArrayList<Pose3D> bodyPathPlan)
   {
      LogTools.debug("footstepPlanningAcceptBodyPath, {}", bodyPathPlan);
      footstepPlanningBodyPathPlan = bodyPathPlan;

      footstepPlanningEvaluateAndRun();
   }

   private void footstepPlanningEvaluateAndRun()
   {
      boolean regionsOK = footstepPlanningNearRegions != null && !footstepPlanningNearRegions.isEmpty();
      regionsOK &= !footstepPlanningNearRegionsExpirationTimer.isPastOrNaN(lookAndStepParameters.getPlanarRegionsExpiration());
      if (!regionsOK)
      {
         LogTools.warn("Find next footstep planning goal: Regions not OK: {}, timePassed: {}, isEmpty: {}",
                       footstepPlanningNearRegions,
                       footstepPlanningNearRegionsExpirationTimer.timePassedSinceReset(),
                       footstepPlanningNearRegions == null ? null : footstepPlanningNearRegions.isEmpty());
         return;
      }

      boolean failedRecently = !footstepPlanningModuleFailedTimer.isPastOrNaN(lookAndStepParameters.get(LookAndStepBehaviorParameters.waitTimeAfterPlanFailed));
      if (failedRecently)
      {
         LogTools.warn("Find next footstep planning goal: failedRecently = true");
         return;
      }


      ArrayList<Pose3D> bodyPathPlan = footstepPlanningBodyPathPlan;
      boolean bodyPathOkay = bodyPathPlan != null && !bodyPathPlan.isEmpty();
      if (!bodyPathOkay)
      {
         LogTools.warn("Find next footstep planning goal: Body path not OK {}, isEmpty: {}", bodyPathPlan, bodyPathPlan == null ? null : true);
         return;
      }

      if (footstepPlanBeingReviewed)
      {
         LogTools.warn("Find next footstep planning goal: footstepPlanBeingReviewed = true");
         return;
      }

      LogTools.info("Finding next sub goal for footstep planning...");
      helper.publishToUI(MapRegionsForUI, footstepPlanningNearRegions);

      HumanoidRobotState latestHumanoidRobotState = robot.pollHumanoidRobotState();
      FramePose3D initialPoseBetweenFeet = new FramePose3D();
      initialPoseBetweenFeet.setToZero(latestHumanoidRobotState.getMidFeetZUpFrame());
      initialPoseBetweenFeet.changeFrame(ReferenceFrame.getWorldFrame());
      double midFeetZ = initialPoseBetweenFeet.getZ();

      FramePose3D pelvisPose = new FramePose3D();
      pelvisPose.setToZero(latestHumanoidRobotState.getPelvisFrame());
      pelvisPose.changeFrame(ReferenceFrame.getWorldFrame());

      goalPoseBetweenFeet.setIncludingFrame(pelvisPose);
      goalPoseBetweenFeet.setZ(midFeetZ);

      // find closest point along body path plan
      Point3D closestPointAlongPath = bodyPathPlan.get(0).getPosition();
      double closestDistance = closestPointAlongPath.distance(goalPoseBetweenFeet.getPosition());
      int closestSegmentIndex = 0;
      for (int i = 0; i < bodyPathPlan.size() - 1; i++)
      {
         LogTools.info("Finding closest point along body path. Segment: {}, closestDistance: {}", i, closestDistance);
         LineSegment3D lineSegment = new LineSegment3D();
         lineSegment.set(bodyPathPlan.get(i).getPosition(), bodyPathPlan.get(i + 1).getPosition());

         Point3D closestPointOnBodyPathSegment = new Point3D();
         EuclidGeometryTools.closestPoint3DsBetweenTwoLineSegment3Ds(lineSegment.getFirstEndpoint(),
                                                                     lineSegment.getSecondEndpoint(),
                                                                     goalPoseBetweenFeet.getPosition(),
                                                                     goalPoseBetweenFeet.getPosition(),
                                                                     closestPointOnBodyPathSegment,
                                                                     new Point3D()); // TODO find a better way to do this

         double distance = closestPointOnBodyPathSegment.distance(goalPoseBetweenFeet.getPosition());
         if (distance < closestDistance)
         {
            closestPointAlongPath = closestPointOnBodyPathSegment;
            closestDistance = distance;
            closestSegmentIndex = i;
         }
      }
      LogTools.info("closestPointAlongPath: {}, closestDistance: {}, closestLineSegmentIndex: {}", closestPointAlongPath, closestDistance, closestSegmentIndex);

      helper.publishToUI(ClosestPointForUI, new Pose3D(closestPointAlongPath, new Quaternion()));

      // move point along body path plan by plan horizon
      Point3D goalPoint = new Point3D(closestPointAlongPath);

      double moveAmountToGo = lookAndStepParameters.get(LookAndStepBehaviorParameters.planHorizon);

      Point3D previousComparisonPoint = closestPointAlongPath;
      int segmentIndexOfGoal = closestSegmentIndex;
      for (int i = closestSegmentIndex; i < bodyPathPlan.size() - 1 && moveAmountToGo > 0; i++)
      {
         Point3D endOfSegment = bodyPathPlan.get(i + 1).getPosition();

         double distanceToEndOfSegment = endOfSegment.distance(previousComparisonPoint);
         LogTools.info("Evaluating segment {}, moveAmountToGo: {}, distanceToEndOfSegment: {}", i, moveAmountToGo, distanceToEndOfSegment);

         if (distanceToEndOfSegment < moveAmountToGo)
         {
            previousComparisonPoint = bodyPathPlan.get(i + 1).getPosition();
            moveAmountToGo -= distanceToEndOfSegment;
         }
         else
         {
            goalPoint.interpolate(previousComparisonPoint, endOfSegment, moveAmountToGo / distanceToEndOfSegment);
            moveAmountToGo = 0;
         }

         goalPoint.set(previousComparisonPoint);
         segmentIndexOfGoal = i;
      }
      LogTools.info("previousComparisonPoint: {}, goalPoint: {}", previousComparisonPoint, goalPoint);

      if (closestPointAlongPath.distanceXY(goalPoint) < lookAndStepParameters.get(LookAndStepBehaviorParameters.goalSatisfactionRadius))
      {
         LogTools.warn("Footstep planning: Robot reached goal. Not planning");
         bodyPathModuleGoalInput = null;
         footstepPlanningBodyPathPlan = null;
         return;
      }

      //      double trailingBy = goalPoseBetweenFeet.getPositionDistance(initialPoseBetweenFeet);
      //      goalPoseBetweenFeet.getOrientation().setYawPitchRoll(lookAndStepParameters.get(LookAndStepBehaviorParameters.direction), 0.0, 0.0);
      //      goalPoseBetweenFeet.appendTranslation(lookAndStepParameters.get(LookAndStepBehaviorParameters.planHorizon) - trailingBy, 0.0, 0.0);

      //      Vector2D headingVector = new Vector2D();
      //      headingVector.set(goalPoint.getX(), goalPoint.getY());
      //      headingVector.sub(goalPoseBetweenFeet.getPosition().getX(), goalPoseBetweenFeet.getPosition().getY());

      LogTools.info("Setting goalPoint: {}", goalPoint);
      goalPoseBetweenFeet.getPosition().set(goalPoint);

      //      double yaw = Math.atan2(headingVector.getX(), headingVector.getY());
      //      LogTools.info("Setting yaw: {}", yaw);
      //      goalPoseBetweenFeet.getOrientation().setYawPitchRoll(yaw, 0.0, 0.0);

      goalPoseBetweenFeet.getOrientation().set(bodyPathPlan.get(segmentIndexOfGoal + 1).getOrientation());

      FramePose3D leftSolePose = new FramePose3D();
      leftSolePose.setToZero(latestHumanoidRobotState.getSoleZUpFrame(RobotSide.LEFT));
      leftSolePose.changeFrame(ReferenceFrame.getWorldFrame());
      FramePose3D rightSolePose = new FramePose3D();
      rightSolePose.setToZero(latestHumanoidRobotState.getSoleZUpFrame(RobotSide.RIGHT));
      rightSolePose.changeFrame(ReferenceFrame.getWorldFrame());

      RobotSide stanceSide;
      if (lastStanceSide != null)
      {
         stanceSide = lastStanceSide.getOppositeSide();
      }
      else
      {
         if (leftSolePose.getPosition().distance(goalPoseBetweenFeet.getPosition()) <= rightSolePose.getPosition().distance(goalPoseBetweenFeet.getPosition()))
         {
            stanceSide = RobotSide.LEFT;
         }
         else
         {
            stanceSide = RobotSide.RIGHT;
         }
      }

      lastStanceSide = stanceSide;

      helper.publishToUI(SubGoalForUI, new Pose3D(goalPoseBetweenFeet));
      ArrayList<Pair<RobotSide, Pose3D>> startFootPoses = new ArrayList<>();
      startFootPoses.add(Pair.of(RobotSide.LEFT, new Pose3D(leftSolePose)));
      startFootPoses.add(Pair.of(RobotSide.RIGHT, new Pose3D(rightSolePose)));
      helper.publishToUI(StartAndGoalFootPosesForUI, startFootPoses);

      footstepPlannerParameters.setIdealFootstepLength(lookAndStepParameters.get(LookAndStepBehaviorParameters.idealFootstepLengthOverride));
      footstepPlannerParameters.setWiggleInsideDelta(lookAndStepParameters.get(LookAndStepBehaviorParameters.wiggleInsideDeltaOverride));
      footstepPlannerParameters.setCliffBaseHeightToAvoid(lookAndStepParameters.get(LookAndStepBehaviorParameters.cliffBaseHeightToAvoidOverride));
      footstepPlannerParameters.setEnableConcaveHullWiggler(lookAndStepParameters.get(LookAndStepBehaviorParameters.enableConcaveHullWigglerOverride));

      FootstepPlannerRequest footstepPlannerRequest = new FootstepPlannerRequest();
      footstepPlannerRequest.setPlanBodyPath(false);
      footstepPlannerRequest.setRequestedInitialStanceSide(stanceSide);
      footstepPlannerRequest.setStartFootPoses(leftSolePose, rightSolePose);
      footstepPlannerRequest.setGoalFootPoses(footstepPlannerParameters.getIdealFootstepWidth(), goalPoseBetweenFeet);
      footstepPlannerRequest.setPlanarRegionsList(footstepPlanningNearRegions);
      footstepPlannerRequest.setTimeout(lookAndStepParameters.get(LookAndStepBehaviorParameters.footstepPlannerTimeout));

      footstepPlanningModule.getFootstepPlannerParameters().set(footstepPlannerParameters);
      footstepPlanningModule.addStatusCallback(this::footstepPlanningStatusUpdate);
      footstepPlanningModule.addCustomTerminationCondition((plannerTime, iterations, bestPathFinalStep, bestPathSize) -> bestPathSize >= 1);

      ThreadTools.startAsDaemon(() -> footstepPlanningThread(footstepPlannerRequest), "FootstepPlanner");
   }

   private void footstepPlanningStatusUpdate(FootstepPlannerOutput status)
   {
      helper.publishToUI(FootstepPlanForUI, FootstepDataMessageConverter.reduceFootstepPlanForUIMessager(status.getFootstepPlan()));
   }

   private void footstepPlanningThread(FootstepPlannerRequest footstepPlannerRequest)
   {
      LogTools.info("Footstep planner started...");
      Stopwatch stopwatch = new Stopwatch().start();
      FootstepPlannerOutput footstepPlannerOutput = footstepPlanningModule.handleRequest(footstepPlannerRequest);
      
      LogTools.info("Footstep planner completed in {} s!", stopwatch.totalElapsed());
      
      FootstepPlannerLogger footstepPlannerLogger = new FootstepPlannerLogger(footstepPlanningModule);
      footstepPlannerLogger.logSession();
      FootstepPlannerLogger.deleteOldLogs(30);

      helper.publishToUI(FootstepPlanForUI, FootstepDataMessageConverter.reduceFootstepPlanForUIMessager(footstepPlannerOutput.getFootstepPlan()));

      if (operatorReviewEnabledInput.get())
      {
         ThreadTools.startAsDaemon(() -> reviewFootstepPlan(footstepPlannerOutput.getFootstepPlan()), "FootstepPlanReview");
      }
      else
      {
         robotWalkingModuleAcceptFootstepPlan(footstepPlannerOutput.getFootstepPlan());
      }
   }

   private void reviewFootstepPlan(FootstepPlan footstepPlan)
   {
      footstepPlanBeingReviewed = true;
      LogTools.info("Waiting for footstep plan operator review...");
      boolean approved = approvalNotification.blockingPoll();
      LogTools.info("Operator reviewed footstep plan: {}", approved);
      footstepPlanBeingReviewed = false;

      if (approved)
      {
         robotWalkingModuleAcceptFootstepPlan(footstepPlan);
      }
   }

   private void robotWalkingModuleAcceptFootstepPlan(FootstepPlan footstepPlan)
   {
      LogTools.debug("robotWalkingModuleAcceptFootstepPlan, {}", footstepPlan);
      robotWalkingModuleFootstepPlan = footstepPlan;

      robotWalkingEvaluateAndRun();
   }

   private void robotWalkingEvaluateAndRun()
   {
      boolean footstepPlanOK = robotWalkingModuleFootstepPlan != null && robotWalkingModuleFootstepPlan.getNumberOfSteps() > 0;
      if (!footstepPlanOK)
      {
         LogTools.warn("Robot walking: Footstep plan not OK {}, numberOfSteps = {}. Planning again...",
                       robotWalkingModuleFootstepPlan,
                       robotWalkingModuleFootstepPlan == null ? null : robotWalkingModuleFootstepPlan.getNumberOfSteps());
         footstepPlanningEvaluateAndRun();
         return;
      }

      FootstepPlan shortenedFootstepPlan = new FootstepPlan();
      if (robotWalkingModuleFootstepPlan.getNumberOfSteps() > 0)
      {
         shortenedFootstepPlan.addFootstep(robotWalkingModuleFootstepPlan.getFootstep(0));
      }

      LogTools.info("Requesting walk");
      double swingTime = lookAndStepParameters.get(LookAndStepBehaviorParameters.swingTime);
      double transferTime = lookAndStepParameters.get(LookAndStepBehaviorParameters.transferTime);
      FootstepDataListMessage footstepDataListMessage = FootstepDataMessageConverter.createFootstepDataListFromPlan(shortenedFootstepPlan,
                                                                                                                    swingTime,
                                                                                                                    transferTime,
                                                                                                                    ExecutionMode.QUEUE);
      TypedNotification<WalkingStatusMessage> walkingStatusNotification = robot.requestWalk(footstepDataListMessage,
                                                                                            robot.pollHumanoidRobotState(),
                                                                                            environmentMap.getLatestCombinedRegionsList());

      helper.publishToUI(FootstepPlanForUI, FootstepDataMessageConverter.reduceFootstepPlanForUIMessager(footstepDataListMessage));

      ThreadTools.startAsDaemon(() -> sleepForPartOfSwingThread(swingTime), "RobotWalking");
      ThreadTools.startAsDaemon(() -> robotWalkingThread(walkingStatusNotification), "RobotWalking");
   }

   private void sleepForPartOfSwingThread(double swingTime)
   {
      double percentSwingToWait = lookAndStepParameters.get(LookAndStepBehaviorParameters.percentSwingToWait);
      double waitTime = swingTime * percentSwingToWait;
      LogTools.info("Waiting {} for {} % of swing...", waitTime, percentSwingToWait);
      ThreadTools.sleepSeconds(waitTime);
      LogTools.info("{} % of swing complete!", percentSwingToWait);

//      if (isRobotAtGoal()) // TODO this is a test
//      {
//         LogTools.warn("Step {}% complete: Robot reached goal: Body path planning...", percentSwingToWait);
//         bodyPathModuleEvaluteAndRun();
//      }
//      else
//      {
         LogTools.warn("Step {}% complete: Robot not reached goal: Find next footstep planning goal...", percentSwingToWait);
         footstepPlanningEvaluateAndRun();
//      }
   }

   private void robotWalkingThread(TypedNotification<WalkingStatusMessage> walkingStatusNotification)
   {
      LogTools.info("Waiting for robot walking...");
      walkingStatusNotification.blockingPoll();
      LogTools.info("Robot walk complete.");
   }

   private boolean isRobotAtGoal()
   {
      List<Pose3D> bodyPathPlan = footstepPlanningBodyPathPlan;
      if (bodyPathPlan == null || bodyPathPlan.isEmpty())
      {
         LogTools.warn("No body path plan yet: {}, isEmpty: {}",
                       bodyPathPlan,
                       bodyPathPlan == null ? null : true);
         return true;
      }

      // if close to goal, far, else near
      HumanoidRobotState latestHumanoidRobotState = robot.pollHumanoidRobotState();
      FramePose3D initialPoseBetweenFeet = new FramePose3D();
      initialPoseBetweenFeet.setToZero(latestHumanoidRobotState.getMidFeetZUpFrame());
      initialPoseBetweenFeet.changeFrame(ReferenceFrame.getWorldFrame());
      double midFeetZ = initialPoseBetweenFeet.getZ();

      FramePose3D pelvisMidFeetPose = new FramePose3D();
      pelvisMidFeetPose.setToZero(latestHumanoidRobotState.getPelvisFrame());
      pelvisMidFeetPose.changeFrame(ReferenceFrame.getWorldFrame());
      pelvisMidFeetPose.setZ(midFeetZ);

      double distanceToEnd = bodyPathPlan.get(bodyPathPlan.size() - 1).getPosition().distanceXY(pelvisMidFeetPose.getPosition());

      boolean robotReachedGoal = distanceToEnd < lookAndStepParameters.getGoalSatisfactionRadius();

      return robotReachedGoal;
   }

   public static class LookAndStepBehaviorAPI
   {
      private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();
      private static final Category RootCategory = apiFactory.createRootCategory("LookAndStepBehavior");
      private static final CategoryTheme LookAndStepTheme = apiFactory.createCategoryTheme("LookAndStep");

      public static final Topic<String> CurrentState = topic("CurrentState");
      public static final Topic<Object> TakeStep = topic("TakeStep"); // TODO remove?
      public static final Topic<Object> RePlan = topic("RePlan"); // TODO remove?
      public static final Topic<Boolean> Approval = topic("Approval");
      public static final Topic<Boolean> OperatorReviewEnabled = topic("OperatorReview");
      public static final Topic<ArrayList<Pair<RobotSide, Pose3D>>> StartAndGoalFootPosesForUI = topic("StartAndGoalFootPosesForUI");
      public static final Topic<ArrayList<Pair<RobotSide, Pose3D>>> FootstepPlanForUI = topic("FootstepPlanForUI");
      public static final Topic<Pose3D> ClosestPointForUI = topic("ClosestPointForUI");
      public static final Topic<Pose3D> SubGoalForUI = topic("SubGoalForUI");
      public static final Topic<PlanarRegionsList> MapRegionsForUI = topic("MapRegionsForUI");
      public static final Topic<List<String>> LookAndStepParameters = topic("LookAndStepParameters");
      public static final Topic<List<String>> FootstepPlannerParameters = topic("FootstepPlannerParameters");
      public static final Topic<Pose3D> GoalInput = topic("GoalInput");
      public static final Topic<List<Pose3D>> BodyPathPlanForUI = topic("BodyPathPlanForUI");

      private static <T> Topic<T> topic(String name)
      {
         return RootCategory.child(LookAndStepTheme).topic(apiFactory.createTypedTopicTheme(name));
      }

      public static MessagerAPIFactory.MessagerAPI create()
      {
         return apiFactory.getAPIAndCloseFactory();
      }
   }
}
