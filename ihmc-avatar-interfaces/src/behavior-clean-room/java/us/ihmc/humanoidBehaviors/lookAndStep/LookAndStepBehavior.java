package us.ihmc.humanoidBehaviors.lookAndStep;

import com.google.common.collect.Lists;
import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.WalkingStatusMessage;
import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.thread.Notification;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.communication.ROS2PlanarRegionsInput;
import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
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
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.extra.EnumBasedStateMachineFactory;
import us.ihmc.tools.thread.PausablePeriodicThread;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehavior.LookAndStepBehaviorAPI.*;
import static us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehavior.LookAndStepBehaviorState.*;

public class LookAndStepBehavior implements BehaviorInterface
{
   public static final BehaviorDefinition DEFINITION = new BehaviorDefinition("Look and Step", LookAndStepBehavior::new, create());

   public enum LookAndStepBehaviorState
   {
      PERCEPT_FAR,
      BODY_PATH_PLAN,
      BODY_PATH_PLAN_FAILED,
      PERCEPT_NEAR,
      FOOTSTEP_PLAN,
      FOOTSTEP_PLAN_FAILED,
      REVIEW,
      STEP,
   }

   private final LookAndStepBehaviorParameters lookAndStepParameters = new LookAndStepBehaviorParameters();

   private final BehaviorHelper helper;
   private final StateMachine<LookAndStepBehaviorState, State> stateMachine;
   private final PausablePeriodicThread mainThread;
   private final RemoteREAInterface rea;
   private final ROS2PlanarRegionsInput combinedRegionsInput;
   private final RemoteEnvironmentMapInterface environmentMap;
   private final FootstepPlannerParametersBasics footstepPlannerParameters;
   private final RemoteHumanoidRobotInterface robot;
   private final FootstepPlanningModule footstepPlanningModule;
   private final VisibilityGraphsParametersBasics visibilityGraphParameters;

   private final AtomicReference<Boolean> operatorReviewEnabledInput;
   private AtomicReference<FootstepPlannerOutput> latestFootstepPlannerOutput = new AtomicReference<>();
   private final TypedNotification<FootstepPlannerOutput> footstepPlannerOutputNotification = new TypedNotification<>();
   private TypedNotification<WalkingStatusMessage> walkingStatusNotification;
   private TypedNotification<Pose3D> goalInput;
   private final Notification takeStepNotification;
   private final Notification rePlanNotification;
   private final FramePose3D goalPoseBetweenFeet = new FramePose3D();
   private List<Pose3D> bodyPathPlan;
   private RobotSide lastStanceSide = null;
   private FramePose3D leftFootPoseTemp = new FramePose3D();
   private FramePose3D rightFootPoseTemp = new FramePose3D();

   public LookAndStepBehavior(BehaviorHelper helper)
   {
      this.helper = helper;
      rea = helper.getOrCreateREAInterface();
      combinedRegionsInput = helper.createPlanarRegionsInput("");
      environmentMap = helper.getOrCreateEnvironmentMapInterface();
      robot = helper.getOrCreateRobotInterface();
      operatorReviewEnabledInput = helper.createUIInput(OperatorReviewEnabled, true);
      rePlanNotification = helper.createUINotification(RePlan);
      takeStepNotification = helper.createUINotification(TakeStep);
      goalInput = helper.createUITypedNotification(GoalInput);
      helper.createUICallback(LookAndStepParameters, lookAndStepParameters::setAllFromStrings);
      footstepPlannerParameters = helper.getRobotModel().getFootstepPlannerParameters();
      helper.createUICallback(FootstepPlannerParameters, footstepPlannerParameters::setAllFromStrings);

      footstepPlanningModule = helper.getOrCreateFootstepPlanner();
      visibilityGraphParameters = helper.getRobotModel().getVisibilityGraphsParameters();

      EnumBasedStateMachineFactory<LookAndStepBehaviorState> stateMachineFactory = new EnumBasedStateMachineFactory<>(LookAndStepBehaviorState.class);
      stateMachineFactory.addTransition(PERCEPT_FAR, BODY_PATH_PLAN, this::transitionFromPerceptFar);
      stateMachineFactory.setOnEntry(BODY_PATH_PLAN, this::onBodyPathPlanEntry);
      stateMachineFactory.addTransition(BODY_PATH_PLAN, Lists.newArrayList(PERCEPT_NEAR, BODY_PATH_PLAN_FAILED), this::transitionFromBodyPathPlan);
      stateMachineFactory.addTransition(BODY_PATH_PLAN_FAILED, PERCEPT_FAR, this::transitionFromBodyPathPlanFailed);
      stateMachineFactory.addTransition(PERCEPT_NEAR, FOOTSTEP_PLAN, this::transitionFromPerceptNear);
      stateMachineFactory.setOnEntry(FOOTSTEP_PLAN, this::onFootstepPlanEntry);
      stateMachineFactory.addTransition(FOOTSTEP_PLAN, Lists.newArrayList(REVIEW, STEP, FOOTSTEP_PLAN_FAILED), this::transitionFromPlan);
      stateMachineFactory.addTransition(FOOTSTEP_PLAN_FAILED, PERCEPT_NEAR, this::transitionFromFootstepPlanFailed);
      stateMachineFactory.addTransition(REVIEW, Lists.newArrayList(STEP, PERCEPT_NEAR), this::transitionFromReview);
      stateMachineFactory.setOnEntry(STEP, this::onStepStateEntry);
      stateMachineFactory.addTransition(STEP, Lists.newArrayList(PERCEPT_NEAR, PERCEPT_FAR), this::transitionFromStep);
      Arrays.stream(values()).forEach(state -> stateMachineFactory.setDoAction(state, this::pollInterrupts));
      stateMachineFactory.getFactory().addStateChangedListener(this::stateChanged);
      stateMachineFactory.getFactory().buildClock(() -> Conversions.nanosecondsToSeconds(System.nanoTime()));
      stateMachine = stateMachineFactory.getFactory().build(PERCEPT_FAR);

      double period = 0.1;
      int crashesBeforeGivingUp = 1;
      mainThread = helper.createPausablePeriodicThread(getClass(), period, crashesBeforeGivingUp, stateMachine::doActionAndTransition);
   }

   @Override
   public void setEnabled(boolean enabled)
   {
      LogTools.info("Look and step behavior selected = {}", enabled);

      mainThread.setRunning(enabled);
      helper.setCommunicationCallbacksEnabled(enabled);
   }

   private void stateChanged(LookAndStepBehaviorState from, LookAndStepBehaviorState to)
   {
      helper.publishToUI(CurrentState, to.name());
      LogTools.debug("{} -> {}", from == null ? null : from.name(), to.name());
   }

   private boolean transitionFromPerceptFar()
   {
      return !rea.getPlanarRegionsListExpired(lookAndStepParameters.getPlanarRegionsExpiration())
          && !rea.getLatestPlanarRegionsList().isEmpty()
          && goalInput.read() != null;
   }

   private void onBodyPathPlanEntry()
   {
      // calculate and send body path plan
      visibilityGraphParameters.setIncludePreferredExtrusions(false);
      BodyPathPostProcessor pathPostProcessor = new ObstacleAvoidanceProcessor(visibilityGraphParameters);
      VisibilityGraphPathPlanner bodyPathPlanner = new VisibilityGraphPathPlanner(visibilityGraphParameters,
                                                                                  pathPostProcessor,
                                                                                  new YoVariableRegistry(getClass().getSimpleName()));

      bodyPathPlanner.setGoal(goalInput.read());
      bodyPathPlanner.setPlanarRegionsList(rea.getLatestPlanarRegionsList());
      HumanoidRobotState humanoidRobotState = robot.pollHumanoidRobotState();
      leftFootPoseTemp.setToZero(humanoidRobotState.getSoleFrame(RobotSide.LEFT));
      rightFootPoseTemp.setToZero(humanoidRobotState.getSoleFrame(RobotSide.RIGHT));
      leftFootPoseTemp.changeFrame(ReferenceFrame.getWorldFrame());
      rightFootPoseTemp.changeFrame(ReferenceFrame.getWorldFrame());
      bodyPathPlanner.setStanceFootPoses(leftFootPoseTemp, rightFootPoseTemp);
      Stopwatch stopwatch = new Stopwatch().start();
      LogTools.info("Planning body path...");
      bodyPathPlan = null; // TODO: Is this necessary?
      bodyPathPlanner.planWaypoints();
      LogTools.info("Body path planning took {}", stopwatch.totalElapsed());
//      bodyPathPlan = bodyPathPlanner.getWaypoints();
      if (bodyPathPlanner.getWaypoints() != null)
      {
         bodyPathPlan = new ArrayList<>();
         for (Pose3DReadOnly poseWaypoint : bodyPathPlanner.getWaypoints())
         {
            bodyPathPlan.add(new Pose3D(poseWaypoint));
         }
         helper.publishToUI(BodyPathPlanForUI, bodyPathPlan);
      }
   }

   private LookAndStepBehaviorState transitionFromBodyPathPlan()
   {
      if (bodyPathPlan != null && bodyPathPlan.size() >= 2)
      {
         return PERCEPT_NEAR;
      }
      else
      {
         return BODY_PATH_PLAN_FAILED;
      }
   }

   private boolean transitionFromBodyPathPlanFailed(double timeInState)
   {
      return timeInState > lookAndStepParameters.get(LookAndStepBehaviorParameters.waitTimeAfterPlanFailed);
   }

   private boolean transitionFromPerceptNear()
   {
      return !environmentMap.getPlanarRegionsListExpired(lookAndStepParameters.getPlanarRegionsExpiration())
          && !environmentMap.getLatestCombinedRegionsList().isEmpty();
   }

   private void onFootstepPlanEntry()
   {
      LogTools.info("Entering plan state");
      PlanarRegionsList latestPlanarRegionList = environmentMap.getLatestCombinedRegionsList();
      helper.publishToUI(MapRegionsForUI, latestPlanarRegionList);

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

      footstepPlannerParameters.setIdealFootstepLength(lookAndStepParameters.get(LookAndStepBehaviorParameters.idealFootstepLengthOverride));
      footstepPlannerParameters.setWiggleInsideDelta(lookAndStepParameters.get(LookAndStepBehaviorParameters.wiggleInsideDeltaOverride));
      footstepPlannerParameters.setCliffHeightToAvoid(lookAndStepParameters.get(LookAndStepBehaviorParameters.cliffHeightToAvoidOverride));
      footstepPlannerParameters.setEnableConcaveHullWiggler(lookAndStepParameters.get(LookAndStepBehaviorParameters.enableConcaveHullWigglerOverride));

      FootstepPlannerRequest footstepPlannerRequest = new FootstepPlannerRequest();
      footstepPlannerRequest.setPlanBodyPath(false);
      footstepPlannerRequest.setRequestedInitialStanceSide(stanceSide);
      footstepPlannerRequest.setStartFootPoses(leftSolePose, rightSolePose);
      footstepPlannerRequest.setGoalFootPoses(footstepPlannerParameters.getIdealFootstepWidth(), goalPoseBetweenFeet);
      footstepPlannerRequest.setPlanarRegionsList(latestPlanarRegionList);
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
      LogTools.info("Footstep planner started");
      FootstepPlannerOutput footstepPlannerOutput = footstepPlanningModule.handleRequest(footstepPlannerRequest);
      
      LogTools.info("Footstep planner completed!");
      
      footstepPlannerOutputNotification.set(footstepPlannerOutput);

      latestFootstepPlannerOutput.set(footstepPlannerOutput);

      FootstepPlannerLogger footstepPlannerLogger = new FootstepPlannerLogger(footstepPlanningModule);
      footstepPlannerLogger.logSession();
      FootstepPlannerLogger.deleteOldLogs(10);

      helper.publishToUI(FootstepPlanForUI, FootstepDataMessageConverter.reduceFootstepPlanForUIMessager(footstepPlannerOutput.getFootstepPlan()));
   }

   private LookAndStepBehaviorState transitionFromPlan()
   {
      if (footstepPlannerOutputNotification.hasValue())
      {
         if (footstepPlannerOutputNotification.read().getFootstepPlan().getNumberOfSteps() > 0) // at least 1 footstep
         {
            if (operatorReviewEnabledInput.get())
            {
               return REVIEW;
            }
            else
            {
               return STEP;
            }
         }
         else
         {
            return FOOTSTEP_PLAN_FAILED;
         }
      }

      return null;
   }

   private boolean transitionFromFootstepPlanFailed(double timeInState)
   {
      return timeInState > lookAndStepParameters.get(LookAndStepBehaviorParameters.waitTimeAfterPlanFailed);
   }

   private LookAndStepBehaviorState transitionFromReview()
   {
      if (rePlanNotification.read())
      {
         return PERCEPT_NEAR;
      }
      else if (takeStepNotification.read())
      {
         return STEP;
      }

      return null;
   }

   private void onStepStateEntry()
   {
      FootstepPlan footstepPlan = latestFootstepPlannerOutput.get().getFootstepPlan();

      FootstepPlan shortenedFootstepPlan = new FootstepPlan();
      if (footstepPlan.getNumberOfSteps() > 0)
      {
         shortenedFootstepPlan.addFootstep(footstepPlan.getFootstep(0));
      }

      LogTools.info("Requesting walk");
      double swingTime = lookAndStepParameters.get(LookAndStepBehaviorParameters.swingTime);
      double transferTime = lookAndStepParameters.get(LookAndStepBehaviorParameters.transferTime);
      FootstepDataListMessage footstepDataListMessage = FootstepDataMessageConverter.createFootstepDataListFromPlan(shortenedFootstepPlan,
                                                                                                                    swingTime,
                                                                                                                    transferTime,
                                                                                                                    ExecutionMode.OVERRIDE);
      walkingStatusNotification = robot.requestWalk(footstepDataListMessage, robot.pollHumanoidRobotState(), environmentMap.getLatestCombinedRegionsList());

      helper.publishToUI(FootstepPlanForUI, FootstepDataMessageConverter.reduceFootstepPlanForUIMessager(footstepDataListMessage));
   }

   private LookAndStepBehaviorState transitionFromStep()
   {
      if (walkingStatusNotification.hasValue()) // use rea.isRobotWalking?
      {
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

         double distanceToEnd = bodyPathPlan.get(bodyPathPlan.size() - 1).getPosition().distance(pelvisMidFeetPose.getPosition());

         if (distanceToEnd < lookAndStepParameters.getGoalSatisfactionRadius())
         {
            return PERCEPT_FAR;
         }
         else
         {
            return PERCEPT_NEAR;
         }
      }

      return null;
   }

   private void pollInterrupts()
   {
      if (walkingStatusNotification != null)
      {
         walkingStatusNotification.poll();
      }

      footstepPlannerOutputNotification.poll();
      takeStepNotification.poll();
      rePlanNotification.poll();
      goalInput.poll();
   }

   public static class LookAndStepBehaviorAPI
   {
      private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();
      private static final Category RootCategory = apiFactory.createRootCategory("LookAndStepBehavior");
      private static final CategoryTheme LookAndStepTheme = apiFactory.createCategoryTheme("LookAndStep");

      public static final Topic<String> CurrentState = topic("CurrentState");
      public static final Topic<Object> TakeStep = topic("TakeStep");
      public static final Topic<Object> RePlan = topic("RePlan");
      public static final Topic<Boolean> OperatorReviewEnabled = topic("OperatorReview");
      public static final Topic<ArrayList<Pair<RobotSide, Pose3D>>> FootstepPlanForUI = topic("FootstepPlan");
      public static final Topic<Pose3D> SubGoalForUI = topic("GoalForUI");
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
