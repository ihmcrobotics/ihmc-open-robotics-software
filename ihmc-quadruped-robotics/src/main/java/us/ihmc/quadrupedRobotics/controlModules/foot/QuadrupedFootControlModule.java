package us.ihmc.quadrupedRobotics.controlModules.foot;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerToolbox;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedSolePositionController;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedStepTransitionCallback;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedTaskSpaceEstimates;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedWaypointCallback;
import us.ihmc.quadrupedRobotics.planning.ContactState;
import us.ihmc.quadrupedRobotics.planning.QuadrupedSoleWaypointList;
import us.ihmc.quadrupedRobotics.planning.QuadrupedTimedStep;
import us.ihmc.quadrupedRobotics.planning.YoQuadrupedTimedStep;
import us.ihmc.robotics.stateMachines.eventBasedStateMachine.FiniteStateMachine;
import us.ihmc.robotics.stateMachines.eventBasedStateMachine.FiniteStateMachineBuilder;
import us.ihmc.robotics.stateMachines.eventBasedStateMachine.FiniteStateMachineStateChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class QuadrupedFootControlModule
{
   // control variables
   private final YoVariableRegistry registry;
   private final YoQuadrupedTimedStep stepCommand;
   private final YoBoolean stepCommandIsValid;

   // foot state machine
   public enum FootEvent { TIMEOUT }
   public enum QuadrupedFootRequest { REQUEST_SUPPORT, REQUEST_SWING, REQUEST_MOVE_VIA_WAYPOINTS, REQUEST_HOLD }

   private final QuadrupedSolePositionController solePositionController;

   private final QuadrupedMoveViaWaypointsState moveViaWaypointsState;
   private final FiniteStateMachine<QuadrupedFootStates, FootEvent, QuadrupedFootState> footStateMachine;

   public QuadrupedFootControlModule(RobotQuadrant robotQuadrant, QuadrupedForceControllerToolbox toolbox, YoVariableRegistry parentRegistry)
   {
      // control variables
      String prefix = robotQuadrant.getCamelCaseName();
      this.registry = new YoVariableRegistry(robotQuadrant.getPascalCaseName() + getClass().getSimpleName());
      this.stepCommand = new YoQuadrupedTimedStep(prefix + "StepCommand", registry);
      this.stepCommandIsValid = new YoBoolean(prefix + "StepCommandIsValid", registry);

      // position controller
      solePositionController = new QuadrupedSolePositionController(robotQuadrant, toolbox, registry);

      // state machine
      QuadrupedSupportState supportState = new QuadrupedSupportState(robotQuadrant, stepCommandIsValid, toolbox.getRuntimeEnvironment().getRobotTimestamp(), stepCommand);
      QuadrupedSwingState swingState = new QuadrupedSwingState(robotQuadrant, toolbox, solePositionController, stepCommandIsValid, stepCommand, registry);
      moveViaWaypointsState = new QuadrupedMoveViaWaypointsState(robotQuadrant, toolbox, solePositionController, registry);
      QuadrupedHoldPositionState holdState = new QuadrupedHoldPositionState(robotQuadrant, toolbox, solePositionController, registry);

      FiniteStateMachineBuilder<QuadrupedFootStates, FootEvent, QuadrupedFootState> stateMachineBuilder = new FiniteStateMachineBuilder<>(QuadrupedFootStates.class, FootEvent.class,
                                                                                                                                          prefix + "QuadrupedFootStates", registry);
      stateMachineBuilder.addState(QuadrupedFootStates.SUPPORT, supportState);
      stateMachineBuilder.addState(QuadrupedFootStates.SWING, swingState);
      stateMachineBuilder.addState(QuadrupedFootStates.MOVE_VIA_WAYPOINTS, moveViaWaypointsState);
      stateMachineBuilder.addState(QuadrupedFootStates.HOLD, holdState);

      stateMachineBuilder.addTransition(FootEvent.TIMEOUT, QuadrupedFootStates.SUPPORT, QuadrupedFootStates.SWING);
      stateMachineBuilder.addTransition(FootEvent.TIMEOUT, QuadrupedFootStates.SWING, QuadrupedFootStates.SUPPORT);
      stateMachineBuilder.addTransition(FootEvent.TIMEOUT, QuadrupedFootStates.MOVE_VIA_WAYPOINTS, QuadrupedFootStates.HOLD);

      stateMachineBuilder.addTransition(QuadrupedFootRequest.class, QuadrupedFootRequest.REQUEST_SUPPORT, QuadrupedFootStates.SWING, QuadrupedFootStates.SUPPORT);
      stateMachineBuilder.addTransition(QuadrupedFootRequest.class, QuadrupedFootRequest.REQUEST_SUPPORT, QuadrupedFootStates.MOVE_VIA_WAYPOINTS, QuadrupedFootStates.SUPPORT);
      stateMachineBuilder.addTransition(QuadrupedFootRequest.class, QuadrupedFootRequest.REQUEST_SUPPORT, QuadrupedFootStates.HOLD, QuadrupedFootStates.SUPPORT);
      stateMachineBuilder.addTransition(QuadrupedFootRequest.class, QuadrupedFootRequest.REQUEST_MOVE_VIA_WAYPOINTS, QuadrupedFootStates.SUPPORT, QuadrupedFootStates.MOVE_VIA_WAYPOINTS);
      stateMachineBuilder.addTransition(QuadrupedFootRequest.class, QuadrupedFootRequest.REQUEST_MOVE_VIA_WAYPOINTS, QuadrupedFootStates.SWING, QuadrupedFootStates.MOVE_VIA_WAYPOINTS);
      stateMachineBuilder.addTransition(QuadrupedFootRequest.class, QuadrupedFootRequest.REQUEST_MOVE_VIA_WAYPOINTS, QuadrupedFootStates.HOLD, QuadrupedFootStates.MOVE_VIA_WAYPOINTS);
      stateMachineBuilder.addTransition(QuadrupedFootRequest.class, QuadrupedFootRequest.REQUEST_SWING, QuadrupedFootStates.SUPPORT, QuadrupedFootStates.SWING);
      stateMachineBuilder.addTransition(QuadrupedFootRequest.class, QuadrupedFootRequest.REQUEST_SWING, QuadrupedFootStates.MOVE_VIA_WAYPOINTS, QuadrupedFootStates.SWING);
      stateMachineBuilder.addTransition(QuadrupedFootRequest.class, QuadrupedFootRequest.REQUEST_SWING, QuadrupedFootStates.HOLD, QuadrupedFootStates.SWING);
      stateMachineBuilder.addTransition(QuadrupedFootRequest.class, QuadrupedFootRequest.REQUEST_HOLD, QuadrupedFootStates.SUPPORT, QuadrupedFootStates.HOLD);
      stateMachineBuilder.addTransition(QuadrupedFootRequest.class, QuadrupedFootRequest.REQUEST_HOLD, QuadrupedFootStates.MOVE_VIA_WAYPOINTS, QuadrupedFootStates.HOLD);
      stateMachineBuilder.addTransition(QuadrupedFootRequest.class, QuadrupedFootRequest.REQUEST_HOLD, QuadrupedFootStates.SWING, QuadrupedFootStates.HOLD);

      footStateMachine = stateMachineBuilder.build(QuadrupedFootStates.MOVE_VIA_WAYPOINTS);

      parentRegistry.addChild(registry);
   }

   public QuadrupedSolePositionController getSolePositionController()
   {
      return solePositionController;
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

   public void attachStateChangedListener(FiniteStateMachineStateChangedListener stateChangedListener)
   {
      footStateMachine.attachStateChangedListener(stateChangedListener);
   }

   public void initializeWaypointTrajectory(QuadrupedSoleWaypointList quadrupedSoleWaypointList, QuadrupedTaskSpaceEstimates taskSpaceEstimates,
                                            boolean useInitialSoleForceAsFeedforwardTerm)
   {
      moveViaWaypointsState.handleWaypointList(quadrupedSoleWaypointList);
      moveViaWaypointsState.updateEstimates(taskSpaceEstimates);
      moveViaWaypointsState.initialize(useInitialSoleForceAsFeedforwardTerm);
   }

   public void requestSupport()
   {
      footStateMachine.trigger(QuadrupedFootRequest.class, QuadrupedFootRequest.REQUEST_SUPPORT);
   }

   public void requestSwing()
   {
      footStateMachine.trigger(QuadrupedFootRequest.class, QuadrupedFootRequest.REQUEST_SWING);
   }

   public void requestMoveViaWaypoints()
   {
      footStateMachine.trigger(QuadrupedFootRequest.class, QuadrupedFootRequest.REQUEST_MOVE_VIA_WAYPOINTS);
   }

   public void requestHold()
   {
      footStateMachine.trigger(QuadrupedFootRequest.class, QuadrupedFootRequest.REQUEST_HOLD);
   }

   public void reset()
   {
      stepCommandIsValid.set(false);
      footStateMachine.reset();
   }

   public void triggerStep(QuadrupedTimedStep stepCommand)
   {
      if (footStateMachine.getCurrentStateEnum() == QuadrupedFootStates.SUPPORT)
      {
         this.stepCommand.set(stepCommand);
         this.stepCommandIsValid.set(true);
      }
   }

   public void adjustStep(FramePoint3DReadOnly newGoalPosition)
   {
      this.stepCommand.setGoalPosition(newGoalPosition);
   }

   public ContactState getContactState()
   {
      if (footStateMachine.getCurrentStateEnum() == QuadrupedFootStates.SUPPORT)
         return ContactState.IN_CONTACT;
      else
         return ContactState.NO_CONTACT;
   }

   public void compute(FrameVector3D soleForceCommandToPack, QuadrupedTaskSpaceEstimates taskSpaceEstimates)
   {
      // Update estimates.
      footStateMachine.getCurrentState().updateEstimates(taskSpaceEstimates);

      // Update foot state machine.
      footStateMachine.process();

      // Pack sole force command result.
      ReferenceFrame originalFrame = soleForceCommandToPack.getReferenceFrame();
      soleForceCommandToPack.setIncludingFrame(footStateMachine.getCurrentState().getSoleForceCommand());
      soleForceCommandToPack.changeFrame(originalFrame);
   }
}
