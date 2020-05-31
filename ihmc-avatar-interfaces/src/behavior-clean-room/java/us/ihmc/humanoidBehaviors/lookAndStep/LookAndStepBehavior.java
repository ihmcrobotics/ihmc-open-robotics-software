package us.ihmc.humanoidBehaviors.lookAndStep;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.WalkingStatusMessage;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.util.SimpleTimer;
import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.graphSearch.VisibilityGraphPathPlanner;
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
import us.ihmc.humanoidBehaviors.tools.footstepPlanner.FootstepForUI;
import us.ihmc.humanoidRobotics.footstep.SimpleFootstep;
import us.ihmc.log.LogTools;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersBasics;
import us.ihmc.pathPlanning.visibilityGraphs.postProcessing.BodyPathPostProcessor;
import us.ihmc.pathPlanning.visibilityGraphs.postProcessing.ObstacleAvoidanceProcessor;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.time.LocalDateTime;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;

import static us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehaviorAPI.*;

public class LookAndStepBehavior implements BehaviorInterface
{
   public static final BehaviorDefinition DEFINITION = new BehaviorDefinition("Look and Step", LookAndStepBehavior::new, create());

   private final LookAndStepBehaviorParameters lookAndStepParameters = new LookAndStepBehaviorParameters();

   private final BehaviorHelper helper;
   private final FootstepPlannerParametersBasics footstepPlannerParameters;
   private final RemoteHumanoidRobotInterface robot;
   private final FootstepPlanningModule footstepPlanningModule;
   private final VisibilityGraphsParametersBasics visibilityGraphParameters;
   private final Supplier<PlanarRegionsList> realsenseSLAMRegions;

   private final LookAndStepReviewPart<ArrayList<Pose3D>> bodyPathReview;
   private final LookAndStepReviewPart<FootstepPlan> footstepPlanReview;

   private final AtomicReference<Boolean> operatorReviewEnabledInput;
   private final TypedNotification<Boolean> approvalNotification;
   private final FramePose3D goalPoseBetweenFeet = new FramePose3D();
   private volatile RobotSide lastStanceSide = null;
   private volatile SideDependentList<FramePose3D> lastSteppedSolePoses = new SideDependentList<>();

   private PlanarRegionsList bodyPathModulePlanarRegionsList;
   private SimpleTimer bodyPathModulePlanarRegionExpirationTimer = new SimpleTimer();
   private SimpleTimer bodyPathModuleFailedTimer = new SimpleTimer();
   private volatile Pose3D bodyPathModuleGoalInput;

   private PlanarRegionsList footstepPlanningNearRegions;
   private SimpleTimer footstepPlanningNearRegionsExpirationTimer = new SimpleTimer();
   private volatile ArrayList<Pose3D> footstepPlanningBodyPathPlan;
   private SimpleTimer footstepPlanningModuleFailedTimer = new SimpleTimer();

   private FootstepPlan robotWalkingModuleFootstepPlan;

   public LookAndStepBehavior(BehaviorHelper helper)
   {
      this.helper = helper;
      robot = helper.getOrCreateRobotInterface();
      footstepPlanningModule = helper.getOrCreateFootstepPlanner();
      realsenseSLAMRegions = helper.createROS2PlanarRegionsListInput(ROS2Tools.REALSENSE_SLAM_REGIONS);

      visibilityGraphParameters = helper.getRobotModel().getVisibilityGraphsParameters();

      helper.createUICallback(LookAndStepParameters, lookAndStepParameters::setAllFromStrings);
      footstepPlannerParameters = helper.getRobotModel().getFootstepPlannerParameters();
      helper.createUICallback(FootstepPlannerParameters, footstepPlannerParameters::setAllFromStrings);

      operatorReviewEnabledInput = helper.createUIInput(OperatorReviewEnabled, true);
      approvalNotification = helper.createUITypedNotification(Approval);

      bodyPathReview = new LookAndStepReviewPart<>("body path", approvalNotification, this::footstepPlanningAcceptBodyPath);
      footstepPlanReview = new LookAndStepReviewPart<>("footstep plan", approvalNotification, this::robotWalkingModuleAcceptFootstepPlan);

      // TODO: Want to be able to wire up behavior here and see all present modules

      // build event flow
      helper.createROS2PlanarRegionsListCallback(ROS2Tools.MAP_REGIONS, this::bodyPathModuleAcceptREARegions);
      helper.createUICallback(GoalInput, this::bodyPathModuleAcceptGoalPlacement);

      helper.createROS2PlanarRegionsListCallback(ROS2Tools.REALSENSE_SLAM_REGIONS, this::footstepPlanningAcceptNearPlanarRegions);

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

      if (bodyPathReview.isBeingReviewed())
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
      FramePose3D leftFootPoseTemp = new FramePose3D();
      leftFootPoseTemp.setToZero(humanoidRobotState.getSoleFrame(RobotSide.LEFT));
      FramePose3D rightFootPoseTemp = new FramePose3D();
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
            ThreadTools.startAsDaemon(() -> bodyPathReview.review(bodyPathPlanForReview), "BodyPathReview");
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

      if (footstepPlanReview.isBeingReviewed())
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

      // update last stepped poses to plan from; initialize to current poses
      for (RobotSide side : RobotSide.values)
      {
         if (!lastSteppedSolePoses.containsKey(side))
         {
            lastSteppedSolePoses.put(side, new FramePose3D(latestHumanoidRobotState.getSoleZUpFrame(side)));
            sendLastSteppedSolePoses();
         }

         // make sure world frame
         lastSteppedSolePoses.get(side).changeFrame(ReferenceFrame.getWorldFrame());
      }

      RobotSide stanceSide;
      if (lastStanceSide != null)
      {
         stanceSide = lastStanceSide.getOppositeSide();
      }
      else // if first step, step with furthest foot from the goal
      {
         if (lastSteppedSolePoses.get(RobotSide.LEFT) .getPosition().distance(goalPoseBetweenFeet.getPosition())
          <= lastSteppedSolePoses.get(RobotSide.RIGHT).getPosition().distance(goalPoseBetweenFeet.getPosition()))
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

      footstepPlannerParameters.setIdealFootstepLength(lookAndStepParameters.get(LookAndStepBehaviorParameters.idealFootstepLengthOverride));
      footstepPlannerParameters.setWiggleInsideDelta(lookAndStepParameters.get(LookAndStepBehaviorParameters.wiggleInsideDeltaOverride));
      footstepPlannerParameters.setCliffBaseHeightToAvoid(lookAndStepParameters.get(LookAndStepBehaviorParameters.cliffBaseHeightToAvoidOverride));
      footstepPlannerParameters.setEnableConcaveHullWiggler(lookAndStepParameters.get(LookAndStepBehaviorParameters.enableConcaveHullWigglerOverride));

      FootstepPlannerRequest footstepPlannerRequest = new FootstepPlannerRequest();
      footstepPlannerRequest.setPlanBodyPath(false);
      footstepPlannerRequest.setRequestedInitialStanceSide(stanceSide);
      footstepPlannerRequest.setStartFootPoses(lastSteppedSolePoses.get(RobotSide.LEFT), lastSteppedSolePoses.get(RobotSide.RIGHT));
      footstepPlannerRequest.setGoalFootPoses(footstepPlannerParameters.getIdealFootstepWidth(), goalPoseBetweenFeet);
      footstepPlannerRequest.setPlanarRegionsList(footstepPlanningNearRegions);
      footstepPlannerRequest.setTimeout(lookAndStepParameters.get(LookAndStepBehaviorParameters.footstepPlannerTimeout));

      footstepPlanningModule.getFootstepPlannerParameters().set(footstepPlannerParameters);
      footstepPlanningModule.addStatusCallback(this::footstepPlanningStatusUpdate);
      footstepPlanningModule.addCustomTerminationCondition((plannerTime, iterations, bestPathFinalStep, bestPathSize) -> bestPathSize >= 1);

      ThreadTools.startAsDaemon(() -> footstepPlanningThread(footstepPlannerRequest), "FootstepPlanner");
   }

   private void sendLastSteppedSolePoses()
   {
      ArrayList<FootstepForUI> startFootPoses = new ArrayList<>();
      if (lastSteppedSolePoses.containsKey(RobotSide.LEFT))
         startFootPoses.add(new FootstepForUI(RobotSide.LEFT, new Pose3D(lastSteppedSolePoses.get(RobotSide.LEFT)), "Left Start"));
      if (lastSteppedSolePoses.containsKey(RobotSide.RIGHT))
         startFootPoses.add(new FootstepForUI(RobotSide.RIGHT, new Pose3D(lastSteppedSolePoses.get(RobotSide.RIGHT)), "Right Start"));
      helper.publishToUI(StartAndGoalFootPosesForUI, startFootPoses);
   }

   private void footstepPlanningStatusUpdate(FootstepPlannerOutput status)
   {
//      helper.publishToUI(FootstepPlanForUI, reduceFootstepPlanForUIMessager2(status.getFootstepPlan(), "Update"));
   }

   public static ArrayList<FootstepForUI> reduceFootstepPlanForUIMessager2(FootstepPlan footstepPlan, String description)
   {
      ArrayList<FootstepForUI> footstepLocations = new ArrayList<>();
      for (int i = 0; i < footstepPlan.getNumberOfSteps(); i++)  // this code makes the message smaller to send over the network, TODO investigate
      {
         FramePose3D soleFramePoseToPack = new FramePose3D();
         footstepPlan.getFootstep(i).getSoleFramePose(soleFramePoseToPack);
         soleFramePoseToPack.changeFrame(ReferenceFrame.getWorldFrame());
         footstepLocations.add(new FootstepForUI(footstepPlan.getFootstep(i).getRobotSide(), new Pose3D(soleFramePoseToPack), description));
      }
      return footstepLocations;
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

      helper.publishToUI(FootstepPlanForUI, reduceFootstepPlanForUIMessager2(footstepPlannerOutput.getFootstepPlan(), "Planned"));

      if (operatorReviewEnabledInput.get())
      {
         ThreadTools.startAsDaemon(() -> footstepPlanReview.review(footstepPlannerOutput.getFootstepPlan()), "FootstepPlanReview");
      }
      else
      {
         robotWalkingModuleAcceptFootstepPlan(footstepPlannerOutput.getFootstepPlan());
      }
   }

   private void robotWalkingModuleAcceptFootstepPlan(FootstepPlan footstepPlan)
   {
      LogTools.debug("robotWalkingModuleAcceptFootstepPlan, {} steps", footstepPlan.getNumberOfSteps());
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
         SimpleFootstep footstepToTake = robotWalkingModuleFootstepPlan.getFootstep(0);
         shortenedFootstepPlan.addFootstep(footstepToTake);
         lastSteppedSolePoses.put(footstepToTake.getRobotSide(), new FramePose3D(footstepToTake.getSoleFramePose()));
         sendLastSteppedSolePoses();
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
                                                                                            realsenseSLAMRegions.get());

      helper.publishToUI(FootstepPlanForUI,
                         reduceFootstepPlanForUIMessager2(FootstepDataMessageConverter.convertToFootstepPlan(footstepDataListMessage), "Stepping"));

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
}
