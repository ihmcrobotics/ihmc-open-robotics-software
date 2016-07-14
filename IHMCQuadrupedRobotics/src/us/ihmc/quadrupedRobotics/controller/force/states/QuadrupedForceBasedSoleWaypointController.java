package us.ihmc.quadrupedRobotics.controller.force.states;

import us.ihmc.quadrupedRobotics.controller.ControllerEvent;
import us.ihmc.quadrupedRobotics.controller.QuadrupedController;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerToolbox;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedTaskSpaceController;
import us.ihmc.quadrupedRobotics.model.QuadrupedRuntimeEnvironment;
import us.ihmc.quadrupedRobotics.params.DoubleArrayParameter;
import us.ihmc.quadrupedRobotics.params.DoubleParameter;
import us.ihmc.quadrupedRobotics.params.ParameterFactory;
import us.ihmc.quadrupedRobotics.planning.ContactState;
import us.ihmc.robotics.controllers.YoEuclideanPositionGains;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.quadrupedRobotics.providers.QuadrupedSoleWaypointInputProvider;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedSoleWaypointController;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class QuadrupedForceBasedSoleWaypointController implements QuadrupedController
{
   // Yo variables
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoEuclideanPositionGains yoPositionControllerGains;

   // Parameters
   private final ParameterFactory parameterFactory = ParameterFactory.createWithRegistry(getClass(), registry);
   private final DoubleParameter jointDampingParameter = parameterFactory.createDouble("jointDamping", 15.0);
   private final DoubleParameter jointPositionLimitDampingParameter = parameterFactory.createDouble("jointPositionLimitDamping", 10);
   private final DoubleParameter jointPositionLimitStiffnessParameter = parameterFactory.createDouble("jointPositionLimitStiffness", 100);
   private final DoubleArrayParameter solePositionProportionalGainsParameter = parameterFactory
         .createDoubleArray("solePositionProportionalGains", 10000, 10000, 10000);
   private final DoubleArrayParameter solePositionDerivativeGainsParameter = parameterFactory.createDoubleArray("solePositionDerivativeGains", 100, 100, 100);
   private final DoubleArrayParameter solePositionIntegralGainsParameter = parameterFactory.createDoubleArray("solePositionIntegralGains", 0, 0, 0);
   private final DoubleParameter solePositionMaxIntegralErrorParameter = parameterFactory.createDouble("solePositionMaxIntegralError", 0);

   // Task space controller
   private final QuadrupedTaskSpaceController.Commands taskSpaceControllerCommands;
   private final QuadrupedTaskSpaceController.Settings taskSpaceControllerSettings;
   private final QuadrupedTaskSpaceController taskSpaceController;

   private final QuadrupedSoleWaypointInputProvider soleWaypointInputProvider;
   private final QuadrupedSoleWaypointController quadrupedSoleWaypointController;

   public QuadrupedForceBasedSoleWaypointController(QuadrupedRuntimeEnvironment environment, QuadrupedForceControllerToolbox controllerToolbox,
         QuadrupedSoleWaypointInputProvider inputProvider)
   {
      soleWaypointInputProvider = inputProvider;
      quadrupedSoleWaypointController = controllerToolbox.getSoleWaypointController();
      taskSpaceControllerCommands = new QuadrupedTaskSpaceController.Commands();
      taskSpaceControllerSettings = new QuadrupedTaskSpaceController.Settings();
      this.taskSpaceController = controllerToolbox.getTaskSpaceController();
      yoPositionControllerGains = new YoEuclideanPositionGains("positionControllerGains", registry);
      environment.getParentRegistry().addChild(registry);
   }

   @Override
   public void onEntry()
   {
      // Initialize task space controller
      taskSpaceControllerSettings.initialize();
      taskSpaceControllerSettings.getVirtualModelControllerSettings().setJointDamping(jointDampingParameter.get());
      taskSpaceControllerSettings.getVirtualModelControllerSettings().setJointPositionLimitDamping(jointPositionLimitDampingParameter.get());
      taskSpaceControllerSettings.getVirtualModelControllerSettings().setJointPositionLimitStiffness(jointPositionLimitStiffnessParameter.get());
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         taskSpaceControllerSettings.setContactState(robotQuadrant, ContactState.NO_CONTACT);
      }
      taskSpaceController.reset();
      updateGains();
      quadrupedSoleWaypointController.initialize(soleWaypointInputProvider.get(), yoPositionControllerGains);
   }

   @Override
   public ControllerEvent process()
   {
      boolean success = quadrupedSoleWaypointController.compute(taskSpaceControllerCommands.getSoleForce());
      taskSpaceController.compute(taskSpaceControllerSettings, taskSpaceControllerCommands);
      return success ? null : ControllerEvent.DONE;
   }

   @Override
   public void onExit()
   {
   }

   private void updateGains()
   {
      yoPositionControllerGains.setProportionalGains(solePositionProportionalGainsParameter.get());
      yoPositionControllerGains.setIntegralGains(solePositionIntegralGainsParameter.get(), solePositionMaxIntegralErrorParameter.get());
      yoPositionControllerGains.setDerivativeGains(solePositionDerivativeGainsParameter.get());
   }

}
