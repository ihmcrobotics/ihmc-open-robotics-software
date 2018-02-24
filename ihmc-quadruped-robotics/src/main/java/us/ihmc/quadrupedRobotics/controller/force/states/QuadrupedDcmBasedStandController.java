package us.ihmc.quadrupedRobotics.controller.force.states;

import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.quadrupedRobotics.controlModules.QuadrupedBalanceManager;
import us.ihmc.quadrupedRobotics.controlModules.QuadrupedBodyOrientationManager;
import us.ihmc.quadrupedRobotics.controlModules.QuadrupedControlManagerFactory;
import us.ihmc.quadrupedRobotics.controlModules.foot.QuadrupedFeetManager;
import us.ihmc.quadrupedRobotics.controller.ControllerEvent;
import us.ihmc.quadrupedRobotics.controller.QuadrupedController;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerToolbox;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.*;
import us.ihmc.quadrupedRobotics.estimator.GroundPlaneEstimator;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.robotics.dataStructures.parameter.DoubleArrayParameter;
import us.ihmc.robotics.dataStructures.parameter.DoubleParameter;
import us.ihmc.robotics.dataStructures.parameter.ParameterFactory;
import us.ihmc.quadrupedRobotics.planning.ContactState;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class QuadrupedDcmBasedStandController implements QuadrupedController
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   // parameters
   private final ParameterFactory parameterFactory = ParameterFactory.createWithRegistry(getClass(), registry);
   private final DoubleArrayParameter comForceCommandWeightsParameter = parameterFactory.createDoubleArray("comForceCommandWeights", 1, 1, 1);
   private final DoubleArrayParameter comTorqueCommandWeightsParameter = parameterFactory.createDoubleArray("comTorqueCommandWeights", 1, 1, 1);
   private final DoubleParameter jointDampingParameter = parameterFactory.createDouble("jointDamping", 2);
   private final DoubleParameter jointPositionLimitDampingParameter = parameterFactory.createDouble("jointPositionLimitDamping", 10);
   private final DoubleParameter jointPositionLimitStiffnessParameter = parameterFactory.createDouble("jointPositionLimitStiffness", 100);
   private final DoubleParameter contactPressureLowerLimitParameter = parameterFactory.createDouble("contactPressureLowerLimit", 50);
   private final DoubleParameter coefficientOfFrictionParameter = parameterFactory.createDouble("coefficientOfFriction", 0.5);

   // frames
   private final ReferenceFrame supportFrame;

   // feedback controllers
   private final QuadrupedBodyOrientationManager bodyOrientationManager;
   private final QuadrupedFeetManager feetManager;
   private final QuadrupedBalanceManager balanceManager;

   // task space controller
   private final QuadrupedTaskSpaceController.Commands taskSpaceControllerCommands;
   private final QuadrupedTaskSpaceController.Settings taskSpaceControllerSettings;
   private final QuadrupedTaskSpaceController taskSpaceController;

   // planning
   private final GroundPlaneEstimator groundPlaneEstimator;

   private final FrameQuaternion desiredBodyOrientation = new FrameQuaternion();
   private final QuadrupedForceControllerToolbox controllerToolbox;

   public QuadrupedDcmBasedStandController(QuadrupedForceControllerToolbox controllerToolbox, QuadrupedControlManagerFactory controlManagerFactory, YoVariableRegistry parentRegistry)
   {
      this.controllerToolbox = controllerToolbox;

      // frames
      QuadrupedReferenceFrames referenceFrames = controllerToolbox.getReferenceFrames();
      supportFrame = referenceFrames.getCenterOfFeetZUpFrameAveragingLowestZHeightsAcrossEnds();

      // feedback controllers
      feetManager = controlManagerFactory.getOrCreateFeetManager();
      bodyOrientationManager = controlManagerFactory.getOrCreateBodyOrientationManager();
      balanceManager = controlManagerFactory.getOrCreateBalanceManager();

      // task space controllers
      taskSpaceControllerCommands = new QuadrupedTaskSpaceController.Commands();
      taskSpaceControllerSettings = new QuadrupedTaskSpaceController.Settings();
      taskSpaceController = controllerToolbox.getTaskSpaceController();

      // planning
      groundPlaneEstimator = controllerToolbox.getGroundPlaneEstimator();

      parentRegistry.addChild(registry);
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   private void updateGains()
   {
      taskSpaceControllerSettings.getContactForceLimits().setCoefficientOfFriction(coefficientOfFrictionParameter.get());
      taskSpaceControllerSettings.getVirtualModelControllerSettings().setJointDamping(jointDampingParameter.get());
      taskSpaceControllerSettings.getVirtualModelControllerSettings().setJointPositionLimitDamping(jointPositionLimitDampingParameter.get());
      taskSpaceControllerSettings.getVirtualModelControllerSettings().setJointPositionLimitStiffness(jointPositionLimitStiffnessParameter.get());
      taskSpaceControllerSettings.getContactForceOptimizationSettings().setComForceCommandWeights(comForceCommandWeightsParameter.get());
      taskSpaceControllerSettings.getContactForceOptimizationSettings().setComTorqueCommandWeights(comTorqueCommandWeightsParameter.get());
      taskSpaceControllerSettings.getContactForceLimits().setPressureLowerLimit(contactPressureLowerLimitParameter.get());
   }

   @Override
   public ControllerEvent process()
   {
      updateGains();

      controllerToolbox.update();
      QuadrupedTaskSpaceEstimates taskSpaceEstimates = controllerToolbox.getTaskSpaceEstimates();

      // update ground plane estimate
      groundPlaneEstimator.compute(taskSpaceEstimates.getSolePosition());

      // update desired dcm, com position
      balanceManager.compute(taskSpaceControllerCommands.getComForce(), taskSpaceEstimates, taskSpaceControllerSettings);

      // update desired body orientation and angular rate
      desiredBodyOrientation.setToZero(supportFrame);
      bodyOrientationManager.compute(taskSpaceControllerCommands.getComTorque(), desiredBodyOrientation, taskSpaceEstimates);

      // update joint setpoints
      taskSpaceController.compute(taskSpaceControllerSettings, taskSpaceControllerCommands);

      return null;
   }

   @Override
   public void onEntry()
   {
      // update task space estimates
      controllerToolbox.update();
      QuadrupedTaskSpaceEstimates taskSpaceEstimates = controllerToolbox.getTaskSpaceEstimates();

      // update ground plane estimate
      groundPlaneEstimator.compute(taskSpaceEstimates.getSolePosition());

      // initialize feedback controllers
      balanceManager.initialize(taskSpaceEstimates);

      bodyOrientationManager.initialize(taskSpaceEstimates);

      feetManager.requestFullContact();

      // initialize task space controller
      taskSpaceControllerSettings.initialize();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         taskSpaceControllerSettings.getContactForceOptimizationSettings().setContactForceCommandWeights(robotQuadrant, 0.0, 0.0, 0.0);
         taskSpaceControllerSettings.setContactState(robotQuadrant, ContactState.IN_CONTACT);
      }
      taskSpaceController.reset();
   }

   @Override
   public void onExit()
   {
   }
}
