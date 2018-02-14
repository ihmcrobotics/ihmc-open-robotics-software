package us.ihmc.quadrupedRobotics.controlModules.foot;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerToolbox;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedSolePositionController;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedStepTransitionCallback;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedTaskSpaceEstimates;
import us.ihmc.quadrupedRobotics.planning.ContactState;
import us.ihmc.quadrupedRobotics.planning.QuadrupedTimedStep;
import us.ihmc.quadrupedRobotics.planning.YoQuadrupedTimedStep;
import us.ihmc.robotics.stateMachines.eventBasedStateMachine.FiniteStateMachine;
import us.ihmc.robotics.stateMachines.eventBasedStateMachine.FiniteStateMachineBuilder;
import us.ihmc.robotics.stateMachines.eventBasedStateMachine.FiniteStateMachineStateChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class QuadrupedFootControlModule
{
   // control variables
   private final YoVariableRegistry registry;
   private final YoQuadrupedTimedStep stepCommand;
   private final YoBoolean stepCommandIsValid;

   // foot state machine
   public enum FootEvent
   {
      TIMEOUT
   }

   private final FiniteStateMachine<QuadrupedFootStates, FootEvent, QuadrupedFootState> footStateMachine;

   public QuadrupedFootControlModule(RobotQuadrant robotQuadrant, QuadrupedForceControllerToolbox toolbox, QuadrupedSolePositionController solePositionController,
                                     YoVariableRegistry parentRegistry)
   {
      // control variables
      String prefix = robotQuadrant.getCamelCaseName();
      this.registry = new YoVariableRegistry(robotQuadrant.getPascalCaseName() + getClass().getSimpleName());
      this.stepCommand = new YoQuadrupedTimedStep(prefix + "StepCommand", registry);
      this.stepCommandIsValid = new YoBoolean(prefix + "StepCommandIsValid", registry);
      // state machine
      FiniteStateMachineBuilder<QuadrupedFootStates, FootEvent, QuadrupedFootState> stateMachineBuilder = new FiniteStateMachineBuilder<>(QuadrupedFootStates.class, FootEvent.class,
            prefix + "QuadrupedFootStates", registry);
      QuadrupedSupportState supportState = new QuadrupedSupportState(robotQuadrant, stepCommandIsValid, toolbox.getRuntimeEnvironment().getRobotTimestamp(), stepCommand);
      QuadrupedSwingState swingState = new QuadrupedSwingState(robotQuadrant, toolbox, solePositionController, stepCommandIsValid, stepCommand, registry);
      stateMachineBuilder.addState(QuadrupedFootStates.SUPPORT, supportState);
      stateMachineBuilder.addState(QuadrupedFootStates.SWING, swingState);
      stateMachineBuilder.addTransition(FootEvent.TIMEOUT, QuadrupedFootStates.SUPPORT, QuadrupedFootStates.SWING);
      stateMachineBuilder.addTransition(FootEvent.TIMEOUT, QuadrupedFootStates.SWING, QuadrupedFootStates.SUPPORT);
      footStateMachine = stateMachineBuilder.build(QuadrupedFootStates.SUPPORT);

      parentRegistry.addChild(registry);
   }

   public void registerStepTransitionCallback(QuadrupedStepTransitionCallback stepTransitionCallback)
   {
      for (QuadrupedFootStates footState : QuadrupedFootStates.values)
         footStateMachine.getState(footState).registerStepTransitionCallback(stepTransitionCallback);
   }

   public void attachStateChangedListener(FiniteStateMachineStateChangedListener stateChangedListener)
   {
      footStateMachine.attachStateChangedListener(stateChangedListener);
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

   public void compute(FrameVector3D soleForceCommand, QuadrupedTaskSpaceEstimates taskSpaceEstimates)
   {
      // Update estimates.
      footStateMachine.getCurrentState().updateEstimates(taskSpaceEstimates);

      // Update foot state machine.
      footStateMachine.process();

      // Pack sole force command result.
      soleForceCommand.set(footStateMachine.getCurrentState().getSoleForceCommand());
   }
}
