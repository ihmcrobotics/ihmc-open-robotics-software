package us.ihmc.quadrupedRobotics.controller.force.states;

import us.ihmc.euclid.Axis;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.quadrupedRobotics.controlModules.QuadrupedBalanceManager;
import us.ihmc.quadrupedRobotics.controlModules.QuadrupedBodyOrientationManager;
import us.ihmc.quadrupedRobotics.controlModules.QuadrupedControlManagerFactory;
import us.ihmc.quadrupedRobotics.controlModules.QuadrupedJointSpaceManager;
import us.ihmc.quadrupedRobotics.controlModules.foot.QuadrupedFeetManager;
import us.ihmc.quadrupedRobotics.controller.ControllerEvent;
import us.ihmc.quadrupedRobotics.controller.QuadrupedController;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerToolbox;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.*;
import us.ihmc.quadrupedRobotics.estimator.GroundPlaneEstimator;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.planning.ContactState;
import us.ihmc.robotics.dataStructures.parameters.ParameterVector3D;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class QuadrupedStandController implements QuadrupedController
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final ParameterVector3D comForceCommandWeightsParameter;
   private final ParameterVector3D comTorqueCommandWeightsParameter;

   private final DoubleParameter jointDampingParameter = new DoubleParameter("jointDamping", registry, 2.0);
   private final DoubleParameter jointPositionLimitDampingParameter = new DoubleParameter("jointPositionLimitDampingParameter", registry, 10);
   private final DoubleParameter jointPositionLimitStiffnessParameter = new DoubleParameter("jointPositionLimitStiffnessParameter", registry, 100);
   private final DoubleParameter contactPressureLowerLimitParameter = new DoubleParameter("contactPressureLowerLimitParameter", registry, 50);
   private final DoubleParameter coefficientOfFrictionParameter = new DoubleParameter("coefficientOfFrictionParameter", registry, 0.5);

   // frames
   private final ReferenceFrame supportFrame;

   // feedback controllers
   private final QuadrupedBodyOrientationManager bodyOrientationManager;
   private final QuadrupedFeetManager feetManager;
   private final QuadrupedBalanceManager balanceManager;
   private final QuadrupedJointSpaceManager jointSpaceManager;

   // task space controller
   private final QuadrupedTaskSpaceController.Commands taskSpaceControllerCommands;
   private final QuadrupedTaskSpaceController.Settings taskSpaceControllerSettings;
   private final QuadrupedTaskSpaceController taskSpaceController;

   // planning
   private final GroundPlaneEstimator groundPlaneEstimator;

   private final FrameQuaternion desiredBodyOrientation = new FrameQuaternion();
   private final QuadrupedForceControllerToolbox controllerToolbox;

   private final QuadrantDependentList<FramePoint3D> solePositions;

   public QuadrupedStandController(QuadrupedForceControllerToolbox controllerToolbox, QuadrupedControlManagerFactory controlManagerFactory, YoVariableRegistry parentRegistry)
   {
      this.controllerToolbox = controllerToolbox;

      // frames
      QuadrupedReferenceFrames referenceFrames = controllerToolbox.getReferenceFrames();
      supportFrame = referenceFrames.getCenterOfFeetZUpFrameAveragingLowestZHeightsAcrossEnds();

      // feedback controllers
      feetManager = controlManagerFactory.getOrCreateFeetManager();
      bodyOrientationManager = controlManagerFactory.getOrCreateBodyOrientationManager();
      balanceManager = controlManagerFactory.getOrCreateBalanceManager();
      jointSpaceManager = controlManagerFactory.getOrCreateJointSpaceManager();

      // task space controllers
      taskSpaceControllerCommands = new QuadrupedTaskSpaceController.Commands();
      taskSpaceControllerSettings = new QuadrupedTaskSpaceController.Settings();
      taskSpaceController = controllerToolbox.getTaskSpaceController();

      // planning
      groundPlaneEstimator = controllerToolbox.getGroundPlaneEstimator();

      solePositions = controllerToolbox.getTaskSpaceEstimates().getSolePositions();

      Vector3D defaultComForceCommandWeights = new Vector3D(1.0, 1.0, 1.0);
      Vector3D defaultComTorqueCommandWeights = new Vector3D(1.0, 1.0, 1.0);

      comForceCommandWeightsParameter = new ParameterVector3D("comForceCommandWeight", defaultComForceCommandWeights, registry);
      comTorqueCommandWeightsParameter = new ParameterVector3D("comTorqueCommandWeight", defaultComTorqueCommandWeights, registry);

      parentRegistry.addChild(registry);
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   private void updateGains()
   {
      taskSpaceControllerSettings.getContactForceLimits().setCoefficientOfFriction(coefficientOfFrictionParameter.getValue());
      taskSpaceControllerSettings.getVirtualModelControllerSettings().setJointDamping(jointDampingParameter.getValue());
      taskSpaceControllerSettings.getVirtualModelControllerSettings().setJointPositionLimitDamping(jointPositionLimitDampingParameter.getValue());
      taskSpaceControllerSettings.getVirtualModelControllerSettings().setJointPositionLimitStiffness(jointPositionLimitStiffnessParameter.getValue());
      taskSpaceControllerSettings.getContactForceOptimizationSettings().setComForceCommandWeights(comForceCommandWeightsParameter);
      taskSpaceControllerSettings.getContactForceOptimizationSettings().setComTorqueCommandWeights(comTorqueCommandWeightsParameter);
      taskSpaceControllerSettings.getContactForceLimits().setPressureLowerLimit(contactPressureLowerLimitParameter.getValue());
   }

   @Override
   public ControllerEvent process()
   {
      updateGains();

      controllerToolbox.update();
      feetManager.updateSupportPolygon();

      // update ground plane estimate
      groundPlaneEstimator.compute(solePositions);

      // update desired dcm, com position
      balanceManager.compute(taskSpaceControllerCommands.getComForce(), taskSpaceControllerSettings);

      // update desired body orientation and angular rate
      desiredBodyOrientation.setToZero(supportFrame);
      bodyOrientationManager.compute(taskSpaceControllerCommands.getComTorque(), desiredBodyOrientation);

      jointSpaceManager.compute();

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
      groundPlaneEstimator.compute(solePositions);

      // initialize feedback controllers
      balanceManager.initializeForStanding(taskSpaceControllerSettings);
      bodyOrientationManager.initialize(taskSpaceEstimates.getBodyOrientation());

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
