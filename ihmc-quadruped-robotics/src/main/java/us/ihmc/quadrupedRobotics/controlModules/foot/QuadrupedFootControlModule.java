package us.ihmc.quadrupedRobotics.controlModules.foot;

import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualModelControlCommand;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControllerToolbox;
import us.ihmc.quadrupedRobotics.controller.toolbox.QuadrupedStepTransitionCallback;
import us.ihmc.quadrupedRobotics.controller.toolbox.QuadrupedWaypointCallback;
import us.ihmc.quadrupedRobotics.planning.ContactState;
import us.ihmc.quadrupedRobotics.planning.QuadrupedTimedStep;
import us.ihmc.quadrupedRobotics.planning.YoQuadrupedTimedStep;
import us.ihmc.robotics.math.trajectories.waypoints.FrameEuclideanTrajectoryPointList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.stateMachine.core.StateChangedListener;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.extra.EventTrigger;
import us.ihmc.robotics.stateMachine.factories.EventBasedStateMachineFactory;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class QuadrupedFootControlModule
{
   // control variables
   private final YoVariableRegistry registry;
   private final YoQuadrupedTimedStep currentStepCommand;
   private final YoBoolean stepCommandIsValid;

   // foot state machine
   public enum FootEvent
   {
      TIMEOUT
   }

   public enum QuadrupedFootRequest
   {
      REQUEST_SUPPORT, REQUEST_SWING, REQUEST_MOVE_VIA_WAYPOINTS, REQUEST_HOLD
   }

   private final QuadrupedMoveViaWaypointsState moveViaWaypointsState;
   private final EventTrigger eventTrigger;
   private final StateMachine<QuadrupedFootStates, QuadrupedFootState> footStateMachine;

   public QuadrupedFootControlModule(RobotQuadrant robotQuadrant, QuadrupedControllerToolbox controllerToolbox,
                                     YoGraphicsListRegistry graphicsListRegistry, YoVariableRegistry parentRegistry)
   {
      // control variables
      String prefix = robotQuadrant.getCamelCaseName();
      this.registry = new YoVariableRegistry(robotQuadrant.getPascalCaseName() + getClass().getSimpleName());
      this.currentStepCommand = new YoQuadrupedTimedStep(prefix + "CurrentStepCommand", registry);
      this.stepCommandIsValid = new YoBoolean(prefix + "StepCommandIsValid", registry);

      // state machine
      QuadrupedSupportState supportState = new QuadrupedSupportState(robotQuadrant, controllerToolbox.getFootContactState(robotQuadrant), stepCommandIsValid,
                                                                     controllerToolbox.getRuntimeEnvironment().getRobotTimestamp(), currentStepCommand);
      QuadrupedSwingState swingState = new QuadrupedSwingState(robotQuadrant, controllerToolbox, stepCommandIsValid, currentStepCommand, graphicsListRegistry,
                                                               registry);
      moveViaWaypointsState = new QuadrupedMoveViaWaypointsState(robotQuadrant, controllerToolbox, registry);
      QuadrupedHoldPositionState holdState = new QuadrupedHoldPositionState(robotQuadrant, controllerToolbox, registry);

      EventBasedStateMachineFactory<QuadrupedFootStates, QuadrupedFootState> factory = new EventBasedStateMachineFactory<>(QuadrupedFootStates.class);
      factory.setNamePrefix(prefix + "QuadrupedFootStates").setRegistry(registry).buildYoClock(controllerToolbox.getRuntimeEnvironment().getRobotTimestamp());

      factory.addState(QuadrupedFootStates.SUPPORT, supportState);
      factory.addState(QuadrupedFootStates.SWING, swingState);
      factory.addState(QuadrupedFootStates.MOVE_VIA_WAYPOINTS, moveViaWaypointsState);
      factory.addState(QuadrupedFootStates.HOLD, holdState);

      factory.addTransition(FootEvent.TIMEOUT, QuadrupedFootStates.SUPPORT, QuadrupedFootStates.SWING);
      factory.addTransition(FootEvent.TIMEOUT, QuadrupedFootStates.SWING, QuadrupedFootStates.SUPPORT);
      factory.addTransition(FootEvent.TIMEOUT, QuadrupedFootStates.MOVE_VIA_WAYPOINTS, QuadrupedFootStates.HOLD);

      factory.addTransition(QuadrupedFootRequest.REQUEST_SUPPORT, QuadrupedFootStates.SWING, QuadrupedFootStates.SUPPORT);
      factory.addTransition(QuadrupedFootRequest.REQUEST_SUPPORT, QuadrupedFootStates.MOVE_VIA_WAYPOINTS, QuadrupedFootStates.SUPPORT);
      factory.addTransition(QuadrupedFootRequest.REQUEST_SUPPORT, QuadrupedFootStates.HOLD, QuadrupedFootStates.SUPPORT);
      factory.addTransition(QuadrupedFootRequest.REQUEST_MOVE_VIA_WAYPOINTS, QuadrupedFootStates.SUPPORT, QuadrupedFootStates.MOVE_VIA_WAYPOINTS);
      factory.addTransition(QuadrupedFootRequest.REQUEST_MOVE_VIA_WAYPOINTS, QuadrupedFootStates.SWING, QuadrupedFootStates.MOVE_VIA_WAYPOINTS);
      factory.addTransition(QuadrupedFootRequest.REQUEST_MOVE_VIA_WAYPOINTS, QuadrupedFootStates.HOLD, QuadrupedFootStates.MOVE_VIA_WAYPOINTS);
      factory.addTransition(QuadrupedFootRequest.REQUEST_SWING, QuadrupedFootStates.SUPPORT, QuadrupedFootStates.SWING);
      factory.addTransition(QuadrupedFootRequest.REQUEST_SWING, QuadrupedFootStates.MOVE_VIA_WAYPOINTS, QuadrupedFootStates.SWING);
      factory.addTransition(QuadrupedFootRequest.REQUEST_SWING, QuadrupedFootStates.HOLD, QuadrupedFootStates.SWING);
      factory.addTransition(QuadrupedFootRequest.REQUEST_HOLD, QuadrupedFootStates.SUPPORT, QuadrupedFootStates.HOLD);
      factory.addTransition(QuadrupedFootRequest.REQUEST_HOLD, QuadrupedFootStates.MOVE_VIA_WAYPOINTS, QuadrupedFootStates.HOLD);
      factory.addTransition(QuadrupedFootRequest.REQUEST_HOLD, QuadrupedFootStates.SWING, QuadrupedFootStates.HOLD);

      eventTrigger = factory.buildEventTrigger();
      footStateMachine = factory.build(QuadrupedFootStates.HOLD);

      parentRegistry.addChild(registry);
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

   public void initializeWaypointTrajectory(FrameEuclideanTrajectoryPointList trajectoryPointList, boolean useInitialSoleForceAsFeedforwardTerm)
   {
      moveViaWaypointsState.handleWaypointList(trajectoryPointList);
      moveViaWaypointsState.initialize(useInitialSoleForceAsFeedforwardTerm);
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

   public void requestHold()
   {
      eventTrigger.fireEvent(QuadrupedFootRequest.REQUEST_HOLD);
   }

   public void reset()
   {
      stepCommandIsValid.set(false);
      footStateMachine.resetToInitialState();
   }

   public void triggerStep(QuadrupedTimedStep stepCommand)
   {
      if (footStateMachine.getCurrentStateKey() == QuadrupedFootStates.SUPPORT)
      {
         this.currentStepCommand.set(stepCommand);
         this.stepCommandIsValid.set(true);
      }
   }

   public void adjustStep(FramePoint3DReadOnly newGoalPosition)
   {
      this.currentStepCommand.setGoalPosition(newGoalPosition);
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
      // Note Sylvain 2018/03/23: the controller is sensitive to the call order on the doAction and doTransitions.
      // Inverting the ordering will break some tests, such as QuadrupedXGaitFlatGroundTrotTest.testTrottingInAForwardLeftCircle().
      footStateMachine.doAction();
      footStateMachine.doTransitions();
   }

   public void getDesiredSoleForce(FrameVector3D soleForceCommandToPack)
   {
      // Pack sole force command result.
      ReferenceFrame originalFrame = soleForceCommandToPack.getReferenceFrame();
      soleForceCommandToPack.setIncludingFrame(footStateMachine.getCurrentState().getSoleForceCommand());
      soleForceCommandToPack.changeFrame(originalFrame);
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

   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      return footStateMachine.getCurrentState().getFeedbackControlCommand();
   }
}
