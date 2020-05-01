package us.ihmc.humanoidBehaviors.lookAndStep;

import com.google.common.collect.Lists;
import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.WalkingStatusMessage;
import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.avatar.networkProcessor.footstepPlanningModule.FootstepPlanningModuleLauncher;
import us.ihmc.commons.thread.Notification;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
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
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.messager.MessagerAPIFactory.Category;
import us.ihmc.messager.MessagerAPIFactory.CategoryTheme;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.extra.EnumBasedStateMachineFactory;
import us.ihmc.tools.thread.PausablePeriodicThread;
import us.ihmc.tools.thread.TypedNotification;

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
      AQUIRE_PATH, PERCEPT, PLAN, REVIEW, STEP, PLAN_FAILED
   }

   private final LookAndStepBehaviorParameters lookAndStepParameters = new LookAndStepBehaviorParameters();

   private final BehaviorHelper helper;
   private final StateMachine<LookAndStepBehaviorState, State> stateMachine;
   private final PausablePeriodicThread mainThread;
   private final RemoteREAInterface rea;
   private final RemoteEnvironmentMapInterface environmentMap;
   private final FootstepPlannerParametersBasics footstepPlannerParameters;
   private final RemoteHumanoidRobotInterface robot;
   private final FootstepPlanningModule footstepPlanningModule;

   private final AtomicReference<Boolean> operatorReviewEnabledInput;
   private AtomicReference<FootstepPlannerOutput> latestFootstepPlannerOutput = new AtomicReference<>();
   private final TypedNotification<FootstepPlannerOutput> footstepPlannerOutputNotification = new TypedNotification<>();
   private TypedNotification<WalkingStatusMessage> walkingStatusNotification;
   private final Notification takeStepNotification;
   private final Notification rePlanNotification;
   private final Stopwatch planFailedWait = new Stopwatch();
   private final FramePose3D goalPoseBetweenFeet = new FramePose3D();
   private final TypedNotification<List<Point3D>> bodyPathPlanNotificationInput;
   private List<Point3D> bodyPathPlan;

   public LookAndStepBehavior(BehaviorHelper helper)
   {
      this.helper = helper;
      rea = helper.getOrCreateREAInterface();
      environmentMap = helper.getOrCreateEnvironmentMapInterface();
      robot = helper.getOrCreateRobotInterface();
      operatorReviewEnabledInput = helper.createUIInput(OperatorReviewEnabled, true);
      rePlanNotification = helper.createUINotification(RePlan);
      takeStepNotification = helper.createUINotification(TakeStep);
      bodyPathPlanNotificationInput = helper.createUITypedNotification(BodyPathPlanInput);
      helper.createUICallback(LookAndStepParameters, lookAndStepParameters::setAllFromStrings);
      footstepPlannerParameters = helper.getRobotModel().getFootstepPlannerParameters();
      helper.createUICallback(FootstepPlannerParameters, footstepPlannerParameters::setAllFromStrings);

      footstepPlanningModule = FootstepPlanningModuleLauncher.createModule(helper.getRobotModel());

      EnumBasedStateMachineFactory<LookAndStepBehaviorState> stateMachineFactory = new EnumBasedStateMachineFactory<>(LookAndStepBehaviorState.class);
      stateMachineFactory.addTransition(AQUIRE_PATH, PERCEPT, this::transitionFromAquirePath);
      stateMachineFactory.addTransition(PERCEPT, PLAN, this::transitionFromPercept);
      stateMachineFactory.setOnEntry(PLAN, this::onPlanStateEntry);
      stateMachineFactory.addTransition(PLAN, Lists.newArrayList(REVIEW, STEP, PLAN_FAILED), this::transitionFromPlan);
      stateMachineFactory.addTransition(REVIEW, Lists.newArrayList(STEP, AQUIRE_PATH), this::transitionFromReview);
      stateMachineFactory.setOnEntry(STEP, this::onStepStateEntry);
      stateMachineFactory.addTransition(STEP, AQUIRE_PATH, this::transitionFromStep);
      stateMachineFactory.setOnEntry(PLAN_FAILED, this::onPlanFailedStateEntry);
      stateMachineFactory.addTransition(PLAN_FAILED, AQUIRE_PATH, this::transitionFromPlanFailed);
      Arrays.stream(values()).forEach(state -> stateMachineFactory.setDoAction(state, this::pollInterrupts));
      stateMachineFactory.getFactory().addStateChangedListener(this::stateChanged);
      stateMachine = stateMachineFactory.getFactory().build(AQUIRE_PATH);

      mainThread = helper.createPausablePeriodicThread(getClass(), 0.1, stateMachine::doActionAndTransition);
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
      LogTools.debug(2, "{} -> {}", from == null ? null : from.name(), to.name());
   }

   private boolean transitionFromAquirePath()
   {
      // check for new body path plans
      bodyPathPlanNotificationInput.poll();
      
      if (bodyPathPlanNotificationInput.hasNext())
      {
         bodyPathPlan = bodyPathPlanNotificationInput.peek();
      }

      boolean transition = true;
      // check there is a path
      boolean hasAtLeastTwoPoints = bodyPathPlan != null && !bodyPathPlan.isEmpty() && bodyPathPlan.size() >= 2;
      transition &= hasAtLeastTwoPoints;

      if (hasAtLeastTwoPoints)
      {
         // visualize the body path in UI
         helper.publishToUI(BodyPathPlanForUI, bodyPathPlan);

         FramePose3DReadOnly midFeetUnderPelvis = robot.quickPollPoseReadOnly(HumanoidReferenceFrames::getMidFeetUnderPelvisFrame);

         // check you are within radius of path
         for (int i = 0; i < bodyPathPlan.size() - 1; i++)
         {
            LineSegment3D lineSegment = new LineSegment3D();
            lineSegment.set(bodyPathPlan.get(i), bodyPathPlan.get(i + 1));

            transition &= lineSegment.distance(midFeetUnderPelvis.getPosition()) < lookAndStepParameters.getMaxPlanStrayDistance();
         }

         // check you aren't close to end of path
         transition &= bodyPathPlan.get(bodyPathPlan.size() - 1).distance(midFeetUnderPelvis.getPosition()) > lookAndStepParameters.getGoalSatisfactionRadius();
      }

      return transition;
   }

   private boolean transitionFromPercept()
   {
      return !arePlanarRegionsExpired() && !environmentMap.getLatestCombinedRegionsList().isEmpty();
   }

   private void onPlanStateEntry()
   {
      HumanoidRobotState latestHumanoidRobotState = robot.pollHumanoidRobotState();
      PlanarRegionsList latestPlanarRegionList = environmentMap.getLatestCombinedRegionsList();
      helper.publishToUI(MapRegionsForUI, latestPlanarRegionList);

      List<Point3D> bodyPathWaypoints = bodyPathPlanNotificationInput.peek();
      
      FramePose3D initialPoseBetweenFeet = new FramePose3D();
      initialPoseBetweenFeet.setToZero(latestHumanoidRobotState.getMidFeetZUpFrame());
      initialPoseBetweenFeet.changeFrame(ReferenceFrame.getWorldFrame());
      double midFeetZ = initialPoseBetweenFeet.getZ();

      goalPoseBetweenFeet.setIncludingFrame(robot.quickPollPoseReadOnly(HumanoidReferenceFrames::getPelvisFrame));
      goalPoseBetweenFeet.setZ(midFeetZ);
      
      // find closest point along body path plan
      Point3D closestPointAlongPath = bodyPathWaypoints.get(0);
      double closestDistance = closestPointAlongPath.distance(goalPoseBetweenFeet.getPosition());
      int closestSegmentIndex = 0;
      for (int i = 0; i < bodyPathWaypoints.size() - 1; i++)
      {
         LogTools.info("Finding closest point along body path. Segment: {}, closestDistance: {}", i, closestDistance);
         LineSegment3D lineSegment = new LineSegment3D();
         lineSegment.set(bodyPathWaypoints.get(i), bodyPathWaypoints.get(i + 1));
         
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
      for (int i = closestSegmentIndex; i < bodyPathWaypoints.size() - 1 && moveAmountToGo > 0; i++)
      {
         Point3D endOfSegment = bodyPathWaypoints.get(i + 1);
         
         double distanceToEndOfSegment = endOfSegment.distance(previousComparisonPoint);
         LogTools.info("Evaluating segment {}, moveAmountToGo: {}, distanceToEndOfSegment: {}", i, moveAmountToGo, distanceToEndOfSegment);
         
         if (distanceToEndOfSegment < moveAmountToGo)
         {
            previousComparisonPoint = bodyPathWaypoints.get(i + 1);
            moveAmountToGo -= distanceToEndOfSegment;
         }
         else
         {
            goalPoint.interpolate(previousComparisonPoint, endOfSegment, moveAmountToGo / distanceToEndOfSegment);
            moveAmountToGo = 0;
         }
         
         goalPoint.set(previousComparisonPoint);
      }
      LogTools.info("previousComparisonPoint: {}, goalPoint: {}", previousComparisonPoint, goalPoint);
      
//      double trailingBy = goalPoseBetweenFeet.getPositionDistance(initialPoseBetweenFeet);
//      goalPoseBetweenFeet.getOrientation().setYawPitchRoll(lookAndStepParameters.get(LookAndStepBehaviorParameters.direction), 0.0, 0.0);
//      goalPoseBetweenFeet.appendTranslation(lookAndStepParameters.get(LookAndStepBehaviorParameters.planHorizon) - trailingBy, 0.0, 0.0);

      Vector2D headingVector = new Vector2D();
      headingVector.set(goalPoint.getX(), goalPoint.getY());
      headingVector.sub(goalPoseBetweenFeet.getPosition().getX(), goalPoseBetweenFeet.getPosition().getY());
      
      LogTools.info("Setting goalPoint: {}", goalPoint);
      goalPoseBetweenFeet.getPosition().set(goalPoint);
      
      double yaw = Math.atan2(headingVector.getX(), headingVector.getY());
      LogTools.info("Setting yaw: {}", yaw);
      goalPoseBetweenFeet.getOrientation().setYawPitchRoll(yaw, 0.0, 0.0);

      RobotSide initialStanceFootSide = null;
      FramePose3D initialStanceFootPose = null;
      FramePose3D leftSolePose = new FramePose3D();
      leftSolePose.setToZero(latestHumanoidRobotState.getSoleZUpFrame(RobotSide.LEFT));
      leftSolePose.changeFrame(ReferenceFrame.getWorldFrame());
      FramePose3D rightSolePose = new FramePose3D();
      rightSolePose.setToZero(latestHumanoidRobotState.getSoleZUpFrame(RobotSide.RIGHT));
      rightSolePose.changeFrame(ReferenceFrame.getWorldFrame());

      if (leftSolePose.getPosition().distance(goalPoseBetweenFeet.getPosition()) <= rightSolePose.getPosition().distance(goalPoseBetweenFeet.getPosition()))
      {
         initialStanceFootSide = RobotSide.LEFT;
         initialStanceFootPose = leftSolePose;
      }
      else
      {
         initialStanceFootSide = RobotSide.RIGHT;
         initialStanceFootPose = rightSolePose;
      }

      helper.publishToUI(SubGoalForUI, new Point3D(goalPoseBetweenFeet.getPosition()));

      FootstepPlannerRequest footstepPlannerRequest = new FootstepPlannerRequest();
      footstepPlannerRequest.setPlanBodyPath(false);
      footstepPlannerRequest.setRequestedInitialStanceSide(initialStanceFootSide);
      footstepPlannerRequest.setStartFootPoses(leftSolePose, rightSolePose);
      footstepPlannerRequest.setGoalFootPoses(footstepPlannerParameters.getIdealFootstepWidth(), goalPoseBetweenFeet);
      footstepPlannerRequest.setPlanarRegionsList(latestPlanarRegionList);
      footstepPlannerRequest.setTimeout(lookAndStepParameters.get(LookAndStepBehaviorParameters.footstepPlannerTimeout));

      footstepPlannerParameters.setIdealFootstepLength(lookAndStepParameters.get(LookAndStepBehaviorParameters.idealFootstepLengthOverride));
      footstepPlannerParameters.setWiggleInsideDelta(lookAndStepParameters.get(LookAndStepBehaviorParameters.wiggleInsideDeltaOverride));
      footstepPlannerParameters.setCliffHeightToAvoid(lookAndStepParameters.get(LookAndStepBehaviorParameters.cliffHeightToAvoidOverride));

      footstepPlanningModule.getFootstepPlannerParameters().set(footstepPlannerParameters);

      footstepPlanningModule.addStatusCallback(this::footstepPlanningStatusUpdate);

      ThreadTools.startAsDaemon(() -> footstepPlanningThread(footstepPlannerRequest), "FootstepPlanner");
   }

   private void footstepPlanningStatusUpdate(FootstepPlannerOutput status)
   {
      helper.publishToUI(FootstepPlanForUI, FootstepDataMessageConverter.reduceFootstepPlanForUIMessager(status.getFootstepPlan()));
   }

   private void footstepPlanningThread(FootstepPlannerRequest footstepPlannerRequest)
   {
      footstepPlanningModule.addCustomTerminationCondition((plannerTime, iterations, bestPathFinalStep, bestPathSize) -> bestPathSize >= 1);
      FootstepPlannerOutput footstepPlannerOutput = footstepPlanningModule.handleRequest(footstepPlannerRequest);
      footstepPlannerOutputNotification.add(footstepPlannerOutput);

      latestFootstepPlannerOutput.set(footstepPlannerOutput);

      FootstepPlannerLogger footstepPlannerLogger = new FootstepPlannerLogger(footstepPlanningModule);
      footstepPlannerLogger.logSession();
      FootstepPlannerLogger.deleteOldLogs(10);

      helper.publishToUI(FootstepPlanForUI, FootstepDataMessageConverter.reduceFootstepPlanForUIMessager(footstepPlannerOutput.getFootstepPlan()));
   }

   private LookAndStepBehaviorState transitionFromPlan()
   {
      if (footstepPlannerOutputNotification.hasNext())
      {
         if (footstepPlannerOutputNotification.peek().getFootstepPlan().getNumberOfSteps() > 0) // at least 1 footstep
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
            return PLAN_FAILED;
         }
      }

      return null;
   }

   private void onPlanFailedStateEntry()
   {
      planFailedWait.start();
   }

   private boolean transitionFromPlanFailed()
   {
      return planFailedWait.lapElapsed() > lookAndStepParameters.get(LookAndStepBehaviorParameters.waitTimeAfterPlanFailed);
   }

   private LookAndStepBehaviorState transitionFromReview()
   {
      if (rePlanNotification.read())
      {
         return AQUIRE_PATH;
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

   private boolean transitionFromStep()
   {
      return walkingStatusNotification.hasNext(); // use rea.isRobotWalking?
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
   }

   private boolean arePlanarRegionsExpired()
   {
      return environmentMap.getPlanarRegionsListExpired(lookAndStepParameters.getPlanarRegionsExpiration());
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
      public static final Topic<Point3D> SubGoalForUI = topic("GoalForUI");
      public static final Topic<PlanarRegionsList> MapRegionsForUI = topic("MapRegionsForUI");
      public static final Topic<List<String>> LookAndStepParameters = topic("LookAndStepParameters");
      public static final Topic<List<String>> FootstepPlannerParameters = topic("FootstepPlannerParameters");
      public static final Topic<List<Point3D>> BodyPathPlanInput = topic("BodyPathPlanInput");
      public static final Topic<List<Point3D>> BodyPathPlanForUI = topic("BodyPathPlanForUI");

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
