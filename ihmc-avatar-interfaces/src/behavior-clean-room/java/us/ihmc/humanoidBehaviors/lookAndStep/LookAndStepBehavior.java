package us.ihmc.humanoidBehaviors.lookAndStep;

import com.google.common.collect.Lists;
import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.WalkingStatusMessage;
import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.avatar.networkProcessor.footstepPlanningModule.FootstepPlanningModuleLauncher;
import us.ihmc.commons.thread.Notification;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidBehaviors.tools.RemoteEnvironmentMapInterface;
import us.ihmc.communication.RemoteREAInterface;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.euclid.geometry.Pose3D;
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
import us.ihmc.commons.thread.TypedNotification;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehavior.LookAndStepBehaviorAPI.*;
import static us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehavior.LookAndStepBehaviorState.*;

public class LookAndStepBehavior implements BehaviorInterface
{
   public static final BehaviorDefinition DEFINITION = new BehaviorDefinition("Look and Step", LookAndStepBehavior::new, create());
   private TypedNotification<WalkingStatusMessage> walkingStatusNotification;

   public enum LookAndStepBehaviorState
   {
      PERCEPT, PLAN, USER, STEP, PLAN_FAILED
   }

   private final LookAndStepBehaviorParameters lookAndStepParameters = new LookAndStepBehaviorParameters();

   private final BehaviorHelper helper;
   private final AtomicReference<Boolean> operatorReviewEnabledInput;
   private final StateMachine<LookAndStepBehaviorState, State> stateMachine;
   private final PausablePeriodicThread mainThread;
   private final RemoteREAInterface rea;
   private final RemoteEnvironmentMapInterface environmentMap;

   private final FootstepPlannerParametersBasics footstepPlannerParameters;
   private final RemoteHumanoidRobotInterface robot;
   private final FootstepPlanningModule footstepPlanningModule;

   private AtomicReference<FootstepPlannerOutput> latestFootstepPlannerOutput = new AtomicReference<>();
   private final TypedNotification<FootstepPlannerOutput> footstepPlannerOutputNotification = new TypedNotification<>();
   private final Notification takeStepNotification;
   private final Notification rePlanNotification;
   private final Stopwatch planFailedWait = new Stopwatch();
   private final FramePose3D goalPoseBetweenFeet = new FramePose3D();

   public LookAndStepBehavior(BehaviorHelper helper)
   {
      this.helper = helper;
      rea = helper.getOrCreateREAInterface();
      environmentMap = helper.getOrCreateEnvironmentMapInterface();
      robot = helper.getOrCreateRobotInterface();
      operatorReviewEnabledInput = helper.createUIInput(OperatorReviewEnabled, true);
      rePlanNotification = helper.createUINotification(RePlan);
      takeStepNotification = helper.createUINotification(TakeStep);
      helper.createUICallback(LookAndStepParameters, lookAndStepParameters::setAllFromStrings);
      footstepPlannerParameters = helper.getRobotModel().getFootstepPlannerParameters();
      helper.createUICallback(FootstepPlannerParameters, footstepPlannerParameters::setAllFromStrings);

      footstepPlanningModule = FootstepPlanningModuleLauncher.createModule(helper.getRobotModel());

      EnumBasedStateMachineFactory<LookAndStepBehaviorState> stateMachineFactory = new EnumBasedStateMachineFactory<>(LookAndStepBehaviorState.class);
      stateMachineFactory.setDoAction(PERCEPT, this::pollInterrupts);
      stateMachineFactory.addTransition(PERCEPT, PLAN, this::transitionFromPercept);
      stateMachineFactory.setOnEntry(PLAN, this::onPlanStateEntry);
      stateMachineFactory.setDoAction(PLAN, this::pollInterrupts);
      stateMachineFactory.addTransition(PLAN, Lists.newArrayList(USER, STEP, PLAN_FAILED), this::transitionFromPlan);
      stateMachineFactory.setDoAction(USER, this::pollInterrupts);
      stateMachineFactory.addTransition(USER, Lists.newArrayList(STEP, PERCEPT), this::transitionFromUser);
      stateMachineFactory.setOnEntry(STEP, this::onStepStateEntry);
      stateMachineFactory.setDoAction(STEP, this::pollInterrupts);
      stateMachineFactory.addTransition(STEP, PERCEPT, this::transitionFromStep);
      stateMachineFactory.setOnEntry(PLAN_FAILED, this::onPlanFailedStateEntry);
      stateMachineFactory.setDoAction(PLAN_FAILED, this::pollInterrupts);
      stateMachineFactory.addTransition(PLAN_FAILED, PERCEPT, this::transitionFromPlanFailed);
      stateMachineFactory.getFactory().addStateChangedListener(this::stateChanged);
      stateMachine = stateMachineFactory.getFactory().build(PERCEPT);

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
      LogTools.debug("{} -> {}", from == null ? null : from.name(), to == null ? null : to.name());
   }

   private boolean transitionFromPercept(double timeInState)
   {
      return !arePlanarRegionsExpired() && !environmentMap.getLatestCombinedRegionsList().isEmpty();
   }

   private void onPlanStateEntry()
   {
      HumanoidRobotState latestHumanoidRobotState = robot.pollHumanoidRobotState();
      PlanarRegionsList latestPlanarRegionList = environmentMap.getLatestCombinedRegionsList();
      helper.publishToUI(MapRegionsForUI, latestPlanarRegionList);

      FramePose3D initialPoseBetweenFeet = new FramePose3D();
      initialPoseBetweenFeet.setToZero(latestHumanoidRobotState.getMidFeetZUpFrame());
      initialPoseBetweenFeet.changeFrame(ReferenceFrame.getWorldFrame());
      double midFeetZ = initialPoseBetweenFeet.getZ();

      goalPoseBetweenFeet.setIncludingFrame(robot.quickPollPoseReadOnly(HumanoidReferenceFrames::getPelvisFrame));
      goalPoseBetweenFeet.setZ(midFeetZ);
      double trailingBy = goalPoseBetweenFeet.getPositionDistance(initialPoseBetweenFeet);
      goalPoseBetweenFeet.setOrientationYawPitchRoll(lookAndStepParameters.get(LookAndStepBehaviorParameters.direction), 0.0, 0.0);
      goalPoseBetweenFeet.appendTranslation(lookAndStepParameters.get(LookAndStepBehaviorParameters.stepLength) - trailingBy, 0.0, 0.0);

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

      helper.publishToUI(GoalForUI, new Point3D(goalPoseBetweenFeet.getPosition()));

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
      footstepPlannerOutputNotification.set(footstepPlannerOutput);

      latestFootstepPlannerOutput.set(footstepPlannerOutput);

      FootstepPlannerLogger footstepPlannerLogger = new FootstepPlannerLogger(footstepPlanningModule);
      footstepPlannerLogger.logSession();
      footstepPlannerLogger.deleteOldLogs(10);

      helper.publishToUI(FootstepPlanForUI, FootstepDataMessageConverter.reduceFootstepPlanForUIMessager(footstepPlannerOutput.getFootstepPlan()));
   }

   private LookAndStepBehaviorState transitionFromPlan(double timeInState)
   {
      if (footstepPlannerOutputNotification.hasValue())
      {
         if (footstepPlannerOutputNotification.read().getFootstepPlan().getNumberOfSteps() > 0) // at least 1 footstep
         {
            if (operatorReviewEnabledInput.get())
            {
               return USER;
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

   private boolean transitionFromPlanFailed(double timeInState)
   {
      return planFailedWait.lapElapsed() > lookAndStepParameters.get(LookAndStepBehaviorParameters.waitTimeAfterPlanFailed);
   }

   private LookAndStepBehaviorState transitionFromUser(double timeInState)
   {
      if (rePlanNotification.read())
      {
         return PERCEPT;
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

   private boolean transitionFromStep(double timeInState)
   {
      return walkingStatusNotification.hasValue(); // use rea.isRobotWalking?
   }

   private void pollInterrupts(double timeInState)
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
      public static final Topic<Point3D> GoalForUI = topic("GoalForUI");
      public static final Topic<PlanarRegionsList> MapRegionsForUI = topic("MapRegionsForUI");
      public static final Topic<List<String>> LookAndStepParameters = topic("LookAndStepParameters");
      public static final Topic<List<String>> FootstepPlannerParameters = topic("FootstepPlannerParameters");

      private static final <T> Topic<T> topic(String name)
      {
         return RootCategory.child(LookAndStepTheme).topic(apiFactory.createTypedTopicTheme(name));
      }

      public static final MessagerAPIFactory.MessagerAPI create()
      {
         return apiFactory.getAPIAndCloseFactory();
      }
   }
}
