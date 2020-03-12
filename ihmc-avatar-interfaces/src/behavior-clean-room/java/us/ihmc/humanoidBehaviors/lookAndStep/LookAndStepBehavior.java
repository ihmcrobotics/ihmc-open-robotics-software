package us.ihmc.humanoidBehaviors.lookAndStep;

import com.google.common.collect.Lists;
import controller_msgs.msg.dds.WalkingStatusMessage;
import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.avatar.networkProcessor.footstepPlanningModule.FootstepPlanningModuleLauncher;
import us.ihmc.commons.thread.Notification;
import us.ihmc.commons.thread.ThreadTools;
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

import java.nio.file.Paths;
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
      PERCEPT, PLAN, USER, STEP
   }

   private final LookAndStepBehaviorParameters lookAndStepParameters = new LookAndStepBehaviorParameters();

   private final BehaviorHelper helper;
   private final AtomicReference<Boolean> operatorReviewEnabledInput;
   private final StateMachine<LookAndStepBehaviorState, State> stateMachine;
   private final PausablePeriodicThread mainThread;
   private final RemoteREAInterface rea;

   private final FootstepPlannerParametersBasics footstepPlannerParameters;
   private final RemoteHumanoidRobotInterface robot;
   private final FootstepPlanningModule footstepPlanningModule;

   private AtomicReference<FootstepPlannerOutput> latestFootstepPlannerOutput = new AtomicReference<>();
   private final TypedNotification<FootstepPlannerOutput> footstepPlannerOutputNotification = new TypedNotification<>();
   private final Notification takeStepNotification;
   private final Notification rePlanNotification;

   public LookAndStepBehavior(BehaviorHelper helper)
   {
      this.helper = helper;
      rea = helper.getOrCreateREAInterface();
      robot = helper.getOrCreateRobotInterface();
      operatorReviewEnabledInput = helper.createUIInput(OperatorReviewEnabled, true);
      rePlanNotification = helper.createUINotification(RePlan);
      takeStepNotification = helper.createUINotification(TakeStep);
      helper.createUICallback(LookAndStepParameters, lookAndStepParameters::setAllFromStrings);
      footstepPlannerParameters = helper.getRobotModel().getFootstepPlannerParameters();
      helper.createUICallback(FootstepPlannerParameters, footstepPlannerParameters::setAllFromStrings);

      footstepPlanningModule = FootstepPlanningModuleLauncher.createModule(helper.getRobotModel());

      EnumBasedStateMachineFactory<LookAndStepBehaviorState> stateMachineFactory = new EnumBasedStateMachineFactory<>(LookAndStepBehaviorState.class);
      stateMachineFactory.setDoAction(PERCEPT, this::doPerceptStateAction);
      stateMachineFactory.addTransition(PERCEPT, PLAN, this::transitionFromPercept);
      stateMachineFactory.setOnEntry(PLAN, this::onPlanStateEntry);
      stateMachineFactory.setDoAction(PLAN, this::doPlanStateAction);
      stateMachineFactory.addTransition(PLAN, Lists.newArrayList(USER, PERCEPT, STEP), this::transitionFromPlan);
      stateMachineFactory.setDoAction(USER, this::doUserStateAction);
      stateMachineFactory.addTransition(USER, Lists.newArrayList(STEP, PERCEPT), this::transitionFromUser);
      stateMachineFactory.setOnEntry(STEP, this::onStepStateEntry);
      stateMachineFactory.setDoAction(STEP, this::doStepStateAction);
      stateMachineFactory.addTransition(STEP, PERCEPT, this::transitionFromStep);
      stateMachineFactory.getFactory().addStateChangedListener(this::stateChanged);
      stateMachine = stateMachineFactory.getFactory().build(PERCEPT);

      mainThread = helper.createPausablePeriodicThread(getClass(), 0.1, this::lookAndStepThread);
   }

   @Override
   public void setEnabled(boolean enabled)
   {
      LogTools.info("Look and step behavior selected = {}", enabled);

      mainThread.setRunning(enabled);
      helper.setCommunicationCallbacksEnabled(enabled);
   }

   private void lookAndStepThread()   // update the active state
   {
      stateMachine.doActionAndTransition();
   }

   private void stateChanged(LookAndStepBehaviorState from, LookAndStepBehaviorState to)
   {
      helper.publishToUI(CurrentState, to.name());
      LogTools.debug("{} -> {}", from == null ? null : from.name(), to == null ? null : to.name());
   }

   private void doPerceptStateAction(double timeInState)
   {
      pollInterrupts();
   }

   private boolean transitionFromPercept(double timeInState)
   {
      return !arePlanarRegionsExpired();
   }

   private void onPlanStateEntry()
   {
      HumanoidRobotState latestHumanoidRobotState = robot.pollHumanoidRobotState();
      PlanarRegionsList latestPlanarRegionList = rea.getLatestPlanarRegionsList();
      helper.publishToUI(MapRegionsForUI, latestPlanarRegionList);

      FramePose3D initialPoseBetweenFeet = new FramePose3D();
      initialPoseBetweenFeet.setToZero(latestHumanoidRobotState.getMidFeetZUpFrame());
      initialPoseBetweenFeet.changeFrame(ReferenceFrame.getWorldFrame());
      double midFeetZ = initialPoseBetweenFeet.getZ();

      FramePose3D goalPoseBetweenFeet = new FramePose3D(initialPoseBetweenFeet);
      goalPoseBetweenFeet.setToZero(latestHumanoidRobotState.getPelvisFrame());
      goalPoseBetweenFeet.appendTranslation(lookAndStepParameters.get(LookAndStepBehaviorParameters.stepLength), 0.0, 0.0);
      goalPoseBetweenFeet.changeFrame(ReferenceFrame.getWorldFrame());
      goalPoseBetweenFeet.setZ(midFeetZ);

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

      FootstepPlannerRequest footstepPlannerRequest = new FootstepPlannerRequest();

      footstepPlannerRequest.setPlanBodyPath(false);
      footstepPlannerRequest.setInitialStanceSide(initialStanceFootSide);
      footstepPlannerRequest.setInitialStancePose(initialStanceFootPose);
      footstepPlannerRequest.setGoalPose(goalPoseBetweenFeet);
      footstepPlannerRequest.setPlanarRegionsList(latestPlanarRegionList);

      footstepPlannerParameters.setIdealFootstepLength(lookAndStepParameters.get(LookAndStepBehaviorParameters.idealFootstepLengthOverride));

      footstepPlanningModule.getFootstepPlannerParameters().set(footstepPlannerParameters);

      footstepPlanningModule.addStatusCallback(status -> LogTools.trace("Planning steps: {}", status.getFootstepPlan().getNumberOfSteps()));

      ThreadTools.startAsDaemon(() -> footstepPlanningThread(footstepPlannerRequest), "FootstepPlanner");
   }

   private void footstepPlanningThread(FootstepPlannerRequest footstepPlannerRequest)
   {
      FootstepPlannerOutput footstepPlannerOutput = footstepPlanningModule.handleRequest(footstepPlannerRequest);
      footstepPlannerOutputNotification.add(footstepPlannerOutput);

      latestFootstepPlannerOutput.set(footstepPlannerOutput);

      FootstepPlannerLogger footstepPlannerLogger = new FootstepPlannerLogger(footstepPlanningModule);
      footstepPlannerLogger.logSession();
      footstepPlannerLogger.deleteOldLogs(10);

      helper.publishToUI(FootstepPlanForUI, FootstepDataMessageConverter.reduceFootstepPlanForUIMessager(footstepPlannerOutput.getFootstepPlan()));
   }

   private void doPlanStateAction(double timeInState)
   {
      pollInterrupts();
   }

   private LookAndStepBehaviorState transitionFromPlan(double timeInState)
   {
      if (footstepPlannerOutputNotification.hasNext())
      {
         if (footstepPlannerOutputNotification.peek().getResult().validForExecution())
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
            return PERCEPT;
         }
      }

      return null;
   }

   private void doUserStateAction(double timeInState)
   {
      pollInterrupts();
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
      boolean swingOverPlanarRegions = true;
      double swingTime = lookAndStepParameters.get(LookAndStepBehaviorParameters.swingTime);
      double transferTime = lookAndStepParameters.get(LookAndStepBehaviorParameters.transferTime);
      walkingStatusNotification = robot.requestWalk(FootstepDataMessageConverter.createFootstepDataListFromPlan(shortenedFootstepPlan,
                                                                                                                swingTime,
                                                                                                                transferTime,
                                                                                                                ExecutionMode.OVERRIDE),
                                                    robot.pollHumanoidRobotState(),
                                                    swingOverPlanarRegions,
                                                    rea.getLatestPlanarRegionsList());
   }

   private void doStepStateAction(double timeInState)
   {
      pollInterrupts();
   }

   private boolean transitionFromStep(double timeInState)
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
      return rea.getPlanarRegionsListExpired(lookAndStepParameters.getPlanarRegionsExpiration());
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
