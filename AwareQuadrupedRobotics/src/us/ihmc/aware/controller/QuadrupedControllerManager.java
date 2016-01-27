package us.ihmc.aware.controller;

import us.ihmc.aware.vmc.QuadrupedVirtualModelController;
import us.ihmc.aware.parameters.QuadrupedRuntimeEnvironment;
import us.ihmc.aware.state.StateMachine;
import us.ihmc.aware.state.StateMachineBuilder;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedRobotParameters;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.sensorProcessing.model.RobotMotionStatusHolder;
import us.ihmc.simulationconstructionset.robotController.RobotController;

public class QuadrupedControllerManager implements RobotController
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final RobotMotionStatusHolder motionStatusHolder = new RobotMotionStatusHolder();

   private final StateMachine<QuadrupedController, QuadrupedControllerEvent> stateMachine;

   private final EnumYoVariable<QuadrupedControllerEvent> userEvent = new EnumYoVariable<>("userEvent", registry, QuadrupedControllerEvent.class, true);

   public QuadrupedControllerManager(QuadrupedRuntimeEnvironment runtimeEnvironment, QuadrupedRobotParameters parameters)
   {
      QuadrupedDoNothingController doNothingController = new QuadrupedDoNothingController(runtimeEnvironment);
      QuadrupedStandPrepController standPrepController = new QuadrupedStandPrepController(runtimeEnvironment, parameters);
      // QuadrupedVirtualModelController virtualModelController = new QuadrupedVirtualModelController(runtimeEnvironment.getFullRobotModel(), parameters, registry, runtimeEnvironment.getGraphicsListRegistry());
      // QuadrupedVirtualModelBasedStepController virtualModelBasedStepController = new QuadrupedVirtualModelBasedStepController(runtimeEnvironment, parameters, virtualModelController);

      // TODO: Define more state transitions.
      StateMachineBuilder<QuadrupedController, QuadrupedControllerEvent> builder = new StateMachineBuilder<>();
      builder.addTransition(QuadrupedControllerEvent.JOINTS_INITIALIZED, doNothingController, standPrepController);
      //builder.addTransition(QuadrupedControllerEvent.JOINTS_INITIALIZED, virtualModelBasedStepController, standPrepController);

      stateMachine = builder.build(doNothingController);
   }

   @Override
   public void initialize()
   {

   }

   @Override
   public void doControl()
   {
      if (userEvent.getEnumValue() != null)
      {
         stateMachine.trigger(userEvent.getEnumValue());
         userEvent.set(null);
      }

      stateMachine.process();
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
}
