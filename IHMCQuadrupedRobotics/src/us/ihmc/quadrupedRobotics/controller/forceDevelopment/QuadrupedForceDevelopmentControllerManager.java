package us.ihmc.quadrupedRobotics.controller.forceDevelopment;

import java.io.IOException;

import us.ihmc.quadrupedRobotics.controller.ControllerEvent;
import us.ihmc.quadrupedRobotics.controller.QuadrupedController;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControllerManager;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerToolbox;
import us.ihmc.quadrupedRobotics.controller.force.states.QuadrupedForceBasedJointInitializationController;
import us.ihmc.quadrupedRobotics.controller.forceDevelopment.states.QuadrupedVMCForceMultiGaitController;
import us.ihmc.quadrupedRobotics.model.QuadrupedPhysicalProperties;
import us.ihmc.quadrupedRobotics.model.QuadrupedRuntimeEnvironment;
import us.ihmc.quadrupedRobotics.planning.ContactState;
import us.ihmc.robotics.stateMachines.eventBasedStateMachine.FiniteStateMachine;
import us.ihmc.robotics.stateMachines.eventBasedStateMachine.FiniteStateMachineBuilder;
import us.ihmc.robotics.stateMachines.eventBasedStateMachine.FiniteStateMachineYoVariableTrigger;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.sensorProcessing.model.RobotMotionStatusHolder;

/**
 * A {@link RobotController} for switching between other robot controllers according to an internal finite state machine.
 * <p/>
 * Users can manually fire events on the {@code userTrigger} YoVariable.
 */
public class QuadrupedForceDevelopmentControllerManager implements QuadrupedControllerManager
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final RobotMotionStatusHolder motionStatusHolder = new RobotMotionStatusHolder();

   private final FiniteStateMachine<QuadrupedForceDevelopmentControllerState, ControllerEvent> stateMachine;
   private final FiniteStateMachineYoVariableTrigger<QuadrupedForceDevelopmentControllerRequestedEvent> userEventTrigger;
   private final QuadrupedRuntimeEnvironment runtimeEnvironment;
   private final QuadrupedForceControllerToolbox controllerToolbox;

   public QuadrupedForceDevelopmentControllerManager(QuadrupedRuntimeEnvironment runtimeEnvironment, QuadrupedPhysicalProperties physicalProperties)
         throws IOException
   {
      this.controllerToolbox = new QuadrupedForceControllerToolbox(runtimeEnvironment, physicalProperties, registry);
      this.stateMachine = buildStateMachine(runtimeEnvironment, physicalProperties);
      this.userEventTrigger = new FiniteStateMachineYoVariableTrigger<>(stateMachine, "userTrigger", registry, QuadrupedForceDevelopmentControllerRequestedEvent.class);
      this.runtimeEnvironment = runtimeEnvironment;
   }

   @Override
   public void initialize()
   {

   }

   @Override
   public void doControl()
   {
      stateMachine.process();

      // update contact state used for state estimation
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         if (controllerToolbox.getTaskSpaceController().getContactState(robotQuadrant) == ContactState.IN_CONTACT)
         {
            runtimeEnvironment.getFootSwitches().get(robotQuadrant).setFootContactState(true);
         }
         else
         {
            runtimeEnvironment.getFootSwitches().get(robotQuadrant).setFootContactState(false);
         }
      }
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return getClass().getSimpleName();
   }

   @Override
   public String getDescription()
   {
      return "A proxy controller for switching between multiple subcontrollers";
   }

   public RobotMotionStatusHolder getMotionStatusHolder()
   {
      return motionStatusHolder;
   }

   private FiniteStateMachine<QuadrupedForceDevelopmentControllerState, ControllerEvent> buildStateMachine(QuadrupedRuntimeEnvironment runtimeEnvironment,
         QuadrupedPhysicalProperties physicalProperties)
   {
      // Initialize controllers.
      QuadrupedController jointInitializationController = new QuadrupedForceBasedJointInitializationController(runtimeEnvironment);
//      QuadrupedController standPrepController = new QuadrupedForceBasedStandPrepController(runtimeEnvironment, controllerToolbox);
//      QuadrupedController standReadyController = new QuadrupedForceBasedFreezeController(runtimeEnvironment, controllerToolbox);
      QuadrupedController trotWalkController = new QuadrupedVMCForceMultiGaitController(physicalProperties, runtimeEnvironment.getFullRobotModel(),
            runtimeEnvironment.getFootSwitches(), runtimeEnvironment.getControlDT(), runtimeEnvironment.getRobotTimestamp(), registry,
            runtimeEnvironment.getGraphicsListRegistry());

      FiniteStateMachineBuilder<QuadrupedForceDevelopmentControllerState, ControllerEvent> builder = new FiniteStateMachineBuilder<>(
            QuadrupedForceDevelopmentControllerState.class, ControllerEvent.class, "forceDevelopmentControllerState", registry);

      builder.addState(QuadrupedForceDevelopmentControllerState.JOINT_INITIALIZATION, jointInitializationController);
//      builder.addState(QuadrupedForceDevelopmentControllerState.STAND_PREP, standPrepController);
//      builder.addState(QuadrupedForceDevelopmentControllerState.STAND_READY, standReadyController);
      builder.addState(QuadrupedForceDevelopmentControllerState.TROT_WALK, trotWalkController);

      // Add automatic transitions that lead into the stand state.
      builder.addTransition(ControllerEvent.DONE, QuadrupedForceDevelopmentControllerState.JOINT_INITIALIZATION, QuadrupedForceDevelopmentControllerState.TROT_WALK);
      builder.addTransition(ControllerEvent.DONE, QuadrupedForceDevelopmentControllerState.STAND_PREP, QuadrupedForceDevelopmentControllerState.STAND_READY);

      // Manually triggered events to transition to main controllers.
      builder.addTransition(QuadrupedForceDevelopmentControllerRequestedEvent.class, QuadrupedForceDevelopmentControllerRequestedEvent.REQUEST_TROT_WALK,
            QuadrupedForceDevelopmentControllerState.STAND_READY, QuadrupedForceDevelopmentControllerState.TROT_WALK);

      // Transitions from controllers back to stand prep.
      builder.addTransition(QuadrupedForceDevelopmentControllerRequestedEvent.class, QuadrupedForceDevelopmentControllerRequestedEvent.REQUEST_STAND_PREP,
            QuadrupedForceDevelopmentControllerState.TROT_WALK, QuadrupedForceDevelopmentControllerState.STAND_PREP);

      return builder.build(QuadrupedForceDevelopmentControllerState.JOINT_INITIALIZATION);
   }
}
