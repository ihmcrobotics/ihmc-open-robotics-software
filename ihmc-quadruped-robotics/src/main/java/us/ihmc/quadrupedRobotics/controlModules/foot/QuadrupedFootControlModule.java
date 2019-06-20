package us.ihmc.quadrupedRobotics.controlModules.foot;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualModelControlCommand;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.quadrupedBasics.QuadrupedSteppingStateEnum;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControllerToolbox;
import us.ihmc.quadrupedRobotics.controller.toolbox.QuadrupedStepTransitionCallback;
import us.ihmc.quadrupedRobotics.controller.toolbox.QuadrupedWaypointCallback;
import us.ihmc.quadrupedRobotics.planning.ContactState;
import us.ihmc.quadrupedRobotics.util.YoQuadrupedTimedStep;
import us.ihmc.robotics.math.trajectories.trajectorypoints.lists.FrameEuclideanTrajectoryPointList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.stateMachine.core.StateChangedListener;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.extra.EventTrigger;
import us.ihmc.robotics.stateMachine.factories.EventBasedStateMachineFactory;
import us.ihmc.robotics.time.TimeIntervalReadOnly;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class QuadrupedFootControlModule
{
   // control variables
   private final YoVariableRegistry registry;
   private final YoQuadrupedTimedStep currentStepCommand;
   private final YoBoolean stepCommandIsValid;

   // foot state machine
   public enum FootEvent
   {
      TIMEOUT, LOADED
   }

   public enum QuadrupedFootRequest
   {
      REQUEST_SUPPORT, REQUEST_SWING, REQUEST_MOVE_VIA_WAYPOINTS
   }

   private final QuadrupedMoveViaWaypointsState moveViaWaypointsState;
   private final EventTrigger eventTrigger;
   private final StateMachine<QuadrupedFootStates, QuadrupedFootState> footStateMachine;
   private final DoubleProvider currentTime;

   // window after support is triggered by touchdown but before step time is up to make sure swing isn't triggered again
   // TODO find a better solution. could be done by indexing steps
   private final QuadrupedFootControlModuleParameters parameters;
   private final YoBoolean isFirstStep;

   private final QuadrupedSwingState swingState;
   private final QuadrupedSupportState supportState;

   public QuadrupedFootControlModule(RobotQuadrant robotQuadrant, QuadrupedControllerToolbox controllerToolbox, YoGraphicsListRegistry graphicsListRegistry,
                                     YoVariableRegistry parentRegistry)
   {
      // control variables
      String prefix = robotQuadrant.getCamelCaseName();
      this.registry = new YoVariableRegistry(robotQuadrant.getPascalCaseName() + getClass().getSimpleName());
      this.currentStepCommand = new YoQuadrupedTimedStep(prefix + "CurrentStepCommand", registry);
      this.stepCommandIsValid = new YoBoolean(prefix + "StepCommandIsValid", registry);
      parameters = controllerToolbox.getFootControlModuleParameters();
      currentTime = controllerToolbox.getRuntimeEnvironment().getRobotTimestamp();

      supportState = new QuadrupedSupportState(robotQuadrant, controllerToolbox, registry);
      swingState = new QuadrupedSwingState(robotQuadrant, controllerToolbox, stepCommandIsValid, currentStepCommand, graphicsListRegistry, registry);
      moveViaWaypointsState = new QuadrupedMoveViaWaypointsState(robotQuadrant, controllerToolbox, registry);

      EventBasedStateMachineFactory<QuadrupedFootStates, QuadrupedFootState> factory = new EventBasedStateMachineFactory<>(QuadrupedFootStates.class);
      factory.setNamePrefix(prefix + "QuadrupedFootStates").setRegistry(registry).buildYoClock(controllerToolbox.getRuntimeEnvironment().getRobotTimestamp());

      factory.addState(QuadrupedFootStates.SUPPORT, supportState);
      factory.addState(QuadrupedFootStates.SWING, swingState);
      factory.addState(QuadrupedFootStates.MOVE_VIA_WAYPOINTS, moveViaWaypointsState);

      factory.addTransition(FootEvent.TIMEOUT, QuadrupedFootStates.SUPPORT, QuadrupedFootStates.SWING);
      factory.addTransition(FootEvent.TIMEOUT, QuadrupedFootStates.SWING, QuadrupedFootStates.SUPPORT);

      factory.addTransition(FootEvent.LOADED, QuadrupedFootStates.SWING, QuadrupedFootStates.SUPPORT);
      factory.addTransition(FootEvent.LOADED, QuadrupedFootStates.MOVE_VIA_WAYPOINTS, QuadrupedFootStates.SUPPORT);

      factory.addTransition(QuadrupedFootRequest.REQUEST_SUPPORT, QuadrupedFootStates.SWING, QuadrupedFootStates.SUPPORT);
      factory.addTransition(QuadrupedFootRequest.REQUEST_SUPPORT, QuadrupedFootStates.MOVE_VIA_WAYPOINTS, QuadrupedFootStates.SUPPORT);
      factory.addTransition(QuadrupedFootRequest.REQUEST_MOVE_VIA_WAYPOINTS, QuadrupedFootStates.SUPPORT, QuadrupedFootStates.MOVE_VIA_WAYPOINTS);
      factory.addTransition(QuadrupedFootRequest.REQUEST_MOVE_VIA_WAYPOINTS, QuadrupedFootStates.SWING, QuadrupedFootStates.MOVE_VIA_WAYPOINTS);
      factory.addTransition(QuadrupedFootRequest.REQUEST_SWING, QuadrupedFootStates.SUPPORT, QuadrupedFootStates.SWING);
      factory.addTransition(QuadrupedFootRequest.REQUEST_SWING, QuadrupedFootStates.MOVE_VIA_WAYPOINTS, QuadrupedFootStates.SWING);

      eventTrigger = factory.buildEventTrigger();
      footStateMachine = factory.build(QuadrupedFootStates.SUPPORT);
      isFirstStep = new YoBoolean(robotQuadrant.getShortName() + "_FirstStep", registry);
      isFirstStep.set(true);

      parentRegistry.addChild(registry);
   }

   public void setControllerCoreMode(WholeBodyControllerCoreMode controllerCoreMode)
   {
      supportState.setControllerCoreMode(controllerCoreMode);
      swingState.setControllerCoreMode(controllerCoreMode);
      moveViaWaypointsState.setControllerCoreMode(controllerCoreMode);
   }

   public void registerStepTransitionCallback(QuadrupedStepTransitionCallback stepTransitionCallback)
   {
      for (QuadrupedFootStates footState : QuadrupedFootStates.values)
      {
         if (footStateMachine.getState(footState) != null)
            footStateMachine.getState(footState).registerStepTransitionCallback(stepTransitionCallback);
      }
   }

   public void registerWaypointCallback(QuadrupedWaypointCallback waypointCallback)
   {
      for (QuadrupedFootStates footState : QuadrupedFootStates.values)
      {
         if (footStateMachine.getState(footState) != null)
            footStateMachine.getState(footState).registerWaypointCallback(waypointCallback);
      }
   }

   public void attachStateChangedListener(StateChangedListener<QuadrupedFootStates> stateChangedListener)
   {
      footStateMachine.addStateChangedListener(stateChangedListener);
   }

   public void initializeWaypointTrajectory(FrameEuclideanTrajectoryPointList trajectoryPointList)
   {
      moveViaWaypointsState.handleWaypointList(trajectoryPointList);
   }

   public void requestSupport()
   {
      eventTrigger.fireEvent(QuadrupedFootRequest.REQUEST_SUPPORT);
   }

   public void requestSwing()
   {
      eventTrigger.fireEvent(QuadrupedFootRequest.REQUEST_SWING);
   }

   public void requestMoveViaWaypoints()
   {
      eventTrigger.fireEvent(QuadrupedFootRequest.REQUEST_MOVE_VIA_WAYPOINTS);
   }

   public QuadrupedFootStates getCurrentState()
   {
      return footStateMachine.getCurrentStateKey();
   }

   public void reset()
   {
      stepCommandIsValid.set(false);
      footStateMachine.resetToInitialState();
      isFirstStep.set(true);
   }

   public void triggerStep(QuadrupedTimedStep stepCommand)
   {
      if (footStateMachine.getCurrentStateKey() == QuadrupedFootStates.SUPPORT && isValidTrigger(stepCommand))
      {
         this.currentStepCommand.set(stepCommand);
         this.stepCommandIsValid.set(true);
         requestSwing();
      }
   }

   private boolean isValidTrigger(QuadrupedTimedStep desiredStep)
   {
      if (currentStepCommand.epsilonEquals(desiredStep, 1e-3))
         return false;

      TimeIntervalReadOnly timeInterval = desiredStep.getTimeInterval();
      double timeInStep = currentTime.getValue() - timeInterval.getStartTime();
      double phaseThroughStep = timeInStep / timeInterval.getDuration();
      if (phaseThroughStep > parameters.getMaximumPhaseThroughStepToAllowStart())
         return false;

      if (isFirstStep.getBooleanValue())
      {
         isFirstStep.set(false);
         return true;
      }
      else
      {
         return footStateMachine.getTimeInCurrentState() > parameters.getMinimumTimeInSupportState();
      }
   }

   public void adjustStep(FramePoint3DReadOnly newGoalPosition)
   {
      this.currentStepCommand.setGoalPosition(newGoalPosition);
   }

   /**
    * Request the swing trajectory to speed up using the given speed up factor.
    * It is clamped w.r.t. to {@link QuadrupedSwingState#minSwingTimeForDisturbanceRecovery}.
    * @param speedUpTime
    * @return the current swing time remaining for the swing foot trajectory
    */
   public double requestSwingSpeedUp(double speedUpTime)
   {
      return swingState.requestSwingSpeedUp(speedUpTime);
   }

   public double computeClampedSwingSpeedUpTime(double requestedSpeedUpTime)
   {
      return swingState.computeClampedSpeedUpTime(requestedSpeedUpTime);
   }

   public ContactState getContactState()
   {
      if (footStateMachine.getCurrentStateKey() == QuadrupedFootStates.SUPPORT)
         return ContactState.IN_CONTACT;
      else
         return ContactState.NO_CONTACT;
   }

   public void compute()
   {
      // Update foot state machine.
      footStateMachine.doTransitions();
      footStateMachine.doAction();
   }

   public FeedbackControlCommandList createFeedbackControlTemplate()
   {
      FeedbackControlCommandList ret = new FeedbackControlCommandList();
      for (QuadrupedFootStates state : QuadrupedFootStates.values)
      {
         QuadrupedFootState footState = footStateMachine.getState(state);
         if (footState != null && footState.createFeedbackControlTemplate() != null)
            ret.addCommand(footState.createFeedbackControlTemplate());
      }

      return ret;
   }

   public VirtualModelControlCommand<?> getVirtualModelControlCommand()
   {
      return footStateMachine.getCurrentState().getVirtualModelControlCommand();
   }

   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      return footStateMachine.getCurrentState().getInverseDynamicsCommand();
   }

   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      return footStateMachine.getCurrentState().getFeedbackControlCommand();
   }
}
