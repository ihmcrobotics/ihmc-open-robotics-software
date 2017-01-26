package us.ihmc.quadrupedRobotics.controller.forceDevelopment.states;

import us.ihmc.quadrupedRobotics.controller.ControllerEvent;
import us.ihmc.quadrupedRobotics.controller.QuadrupedController;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerToolbox;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.*;
import us.ihmc.quadrupedRobotics.controller.forceDevelopment.QuadrupedTimedStepController;
import us.ihmc.quadrupedRobotics.estimator.GroundPlaneEstimator;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.model.QuadrupedRuntimeEnvironment;
import us.ihmc.quadrupedRobotics.optimization.modelPredictiveControl.QuadrupedMpcOptimizationWithLaneChange;
import us.ihmc.quadrupedRobotics.optimization.modelPredictiveControl.QuadrupedDcmBasedMpcOptimizationWithLaneChange;
import us.ihmc.quadrupedRobotics.optimization.modelPredictiveControl.QuadrupedMpcOptimizationWithLaneChangeSettings;
import us.ihmc.robotics.dataStructures.parameter.DoubleArrayParameter;
import us.ihmc.robotics.dataStructures.parameter.DoubleParameter;
import us.ihmc.robotics.dataStructures.parameter.ParameterFactory;
import us.ihmc.quadrupedRobotics.planning.*;
import us.ihmc.quadrupedRobotics.providers.QuadrupedPostureInputProviderInterface;
import us.ihmc.quadrupedRobotics.providers.QuadrupedPlanarVelocityInputProvider;
import us.ihmc.quadrupedRobotics.providers.QuadrupedXGaitSettingsInputProvider;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.*;
import javax.vecmath.Vector3d;
import java.util.ArrayList;

public class QuadrupedMpcBasedXGaitController implements QuadrupedController, QuadrupedStepTransitionCallback
{
   private final QuadrupedPostureInputProviderInterface postureProvider;
   private final QuadrupedPlanarVelocityInputProvider planarVelocityProvider;
   private final QuadrupedXGaitSettingsInputProvider xGaitSettingsProvider;
   private final DoubleYoVariable robotTimestamp;
   private final double controlDT;
   private final double gravity;
   private final double mass;
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   // parameters
   private final ParameterFactory parameterFactory = ParameterFactory.createWithRegistry(getClass(), registry);
   private final DoubleParameter mpcMaximumPreviewTimeParameter = parameterFactory.createDouble("mpcMaximumPreviewTime", 5);
   private final DoubleParameter mpcStepAdjustmentCostParameter = parameterFactory.createDouble("mpcStepAdjustmentCost", 100000);
   private final DoubleParameter mpcCopAdjustmentCostParameter = parameterFactory.createDouble("mpcCopAdjustmentCost", 1);
   private final DoubleParameter mpcMinimumNormalizedContactPressureParameter = parameterFactory.createDouble("mpcMinimumNormalizedContactPressure", 0.1);
   private final DoubleArrayParameter bodyOrientationProportionalGainsParameter = parameterFactory
         .createDoubleArray("bodyOrientationProportionalGains", 5000, 5000, 5000);
   private final DoubleArrayParameter bodyOrientationDerivativeGainsParameter = parameterFactory
         .createDoubleArray("bodyOrientationDerivativeGains", 750, 750, 750);
   private final DoubleArrayParameter bodyOrientationIntegralGainsParameter = parameterFactory.createDoubleArray("bodyOrientationIntegralGains", 0, 0, 0);
   private final DoubleParameter bodyOrientationMaxIntegralErrorParameter = parameterFactory.createDouble("bodyOrientationMaxIntegralError", 0);
   private final DoubleArrayParameter comPositionProportionalGainsParameter = parameterFactory.createDoubleArray("comPositionProportionalGains", 0, 0, 5000);
   private final DoubleArrayParameter comPositionDerivativeGainsParameter = parameterFactory.createDoubleArray("comPositionDerivativeGains", 0, 0, 750);
   private final DoubleArrayParameter comPositionIntegralGainsParameter = parameterFactory.createDoubleArray("comPositionIntegralGains", 0, 0, 0);
   private final DoubleParameter comPositionMaxIntegralErrorParameter = parameterFactory.createDouble("comPositionMaxIntegralError", 0);
   private final DoubleParameter comPositionGravityCompensationParameter = parameterFactory.createDouble("comPositionGravityCompensation", 1);
   private final DoubleParameter jointDampingParameter = parameterFactory.createDouble("jointDamping", 1);
   private final DoubleParameter jointPositionLimitDampingParameter = parameterFactory.createDouble("jointPositionLimitDamping", 10);
   private final DoubleParameter jointPositionLimitStiffnessParameter = parameterFactory.createDouble("jointPositionLimitStiffness", 100);
   private final DoubleParameter initialTransitionDurationParameter = parameterFactory.createDouble("initialTransitionDuration", 0.25);
   private final DoubleParameter haltTransitionDurationParameter = parameterFactory.createDouble("haltTransitionDuration", 1.0);
   private final DoubleParameter minimumStepClearanceParameter = parameterFactory.createDouble("minimumStepClearance", 0.075);
   private final DoubleParameter maximumStepStrideParameter = parameterFactory.createDouble("maximumStepStride", 1.0);

   // frames
   private final ReferenceFrame supportFrame;
   private final ReferenceFrame worldFrame;

   // model
   private final LinearInvertedPendulumModel lipModel;

   // feedback controllers
   private final FramePoint cmpPositionSetpoint;
   private final QuadrupedComPositionController.Setpoints comPositionControllerSetpoints;
   private final QuadrupedComPositionController comPositionController;
   private final QuadrupedBodyOrientationController.Setpoints bodyOrientationControllerSetpoints;
   private final QuadrupedBodyOrientationController bodyOrientationController;
   private final QuadrupedTimedStepController timedStepController;
   private final QuadrupedMpcOptimizationWithLaneChange mpcOptimization;
   private final QuadrupedMpcOptimizationWithLaneChangeSettings mpcSettings;

   // task space controller
   private final QuadrupedTaskSpaceEstimator.Estimates taskSpaceEstimates;
   private final QuadrupedTaskSpaceEstimator taskSpaceEstimator;
   private final QuadrupedTaskSpaceController.Commands taskSpaceControllerCommands;
   private final QuadrupedTaskSpaceController.Settings taskSpaceControllerSettings;
   private final QuadrupedTaskSpaceController taskSpaceController;

   // planning
   private static int NUMBER_OF_PREVIEW_STEPS = 16;
   private double bodyYawSetpoint;
   private final GroundPlaneEstimator groundPlaneEstimator;
   private final QuadrantDependentList<FramePoint> groundPlanePositions;
   private final QuadrupedXGaitSettings xGaitSettings;
   private final QuadrupedXGaitPlanner xGaitStepPlanner;
   private final ArrayList<QuadrupedTimedStep> xGaitPreviewSteps;
   private final EndDependentList<QuadrupedTimedStep> xGaitCurrentSteps;
   private final QuadrupedStepCrossoverProjection crossoverProjection;
   private final FramePoint supportCentroid;
   private final FrameVector stepAdjustmentVector;
   private final FramePoint stepGoalPosition;

   // inputs
   private final DoubleYoVariable haltTime = new DoubleYoVariable("haltTime", registry);
   private final BooleanYoVariable haltFlag = new BooleanYoVariable("haltFlag", registry);

   public QuadrupedMpcBasedXGaitController(QuadrupedRuntimeEnvironment runtimeEnvironment, QuadrupedForceControllerToolbox controllerToolbox,
         QuadrupedPostureInputProviderInterface postureProvider, QuadrupedPlanarVelocityInputProvider planarVelocityProvider,
         QuadrupedXGaitSettingsInputProvider xGaitSettingsProvider)
   {
      this.postureProvider = postureProvider;
      this.planarVelocityProvider = planarVelocityProvider;
      this.xGaitSettingsProvider = xGaitSettingsProvider;
      this.robotTimestamp = runtimeEnvironment.getRobotTimestamp();
      this.controlDT = runtimeEnvironment.getControlDT();
      this.gravity = 9.81;
      this.mass = runtimeEnvironment.getFullRobotModel().getTotalMass();

      // frames
      QuadrupedReferenceFrames referenceFrames = controllerToolbox.getReferenceFrames();
      supportFrame = referenceFrames.getCenterOfFeetZUpFrameAveragingLowestZHeightsAcrossEnds();
      worldFrame = ReferenceFrame.getWorldFrame();

      // model
      lipModel = controllerToolbox.getLinearInvertedPendulumModel();

      // feedback controllers
      cmpPositionSetpoint = new FramePoint();
      comPositionControllerSetpoints = new QuadrupedComPositionController.Setpoints();
      comPositionController = controllerToolbox.getComPositionController();
      bodyOrientationControllerSetpoints = new QuadrupedBodyOrientationController.Setpoints();
      bodyOrientationController = controllerToolbox.getBodyOrientationController();
      timedStepController = new QuadrupedTimedStepController(controllerToolbox.getSolePositionController(), runtimeEnvironment.getRobotTimestamp(), registry,
            runtimeEnvironment.getGraphicsListRegistry());
      mpcOptimization = new QuadrupedDcmBasedMpcOptimizationWithLaneChange(controllerToolbox.getDcmPositionEstimator(), NUMBER_OF_PREVIEW_STEPS, registry,
            runtimeEnvironment.getGraphicsListRegistry());
      mpcSettings = new QuadrupedMpcOptimizationWithLaneChangeSettings(mpcMaximumPreviewTimeParameter.get(), mpcStepAdjustmentCostParameter.get(),
            mpcCopAdjustmentCostParameter.get(), mpcMinimumNormalizedContactPressureParameter.get());

      // task space controllers
      taskSpaceEstimates = new QuadrupedTaskSpaceEstimator.Estimates();
      taskSpaceEstimator = controllerToolbox.getTaskSpaceEstimator();
      taskSpaceControllerCommands = new QuadrupedTaskSpaceController.Commands();
      taskSpaceControllerSettings = new QuadrupedTaskSpaceController.Settings();
      taskSpaceController = controllerToolbox.getTaskSpaceController();

      // planning
      groundPlaneEstimator = controllerToolbox.getGroundPlaneEstimator();
      groundPlanePositions = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         groundPlanePositions.set(robotQuadrant, new FramePoint());
      }
      xGaitSettings = new QuadrupedXGaitSettings();
      xGaitStepPlanner = new QuadrupedXGaitPlanner();
      xGaitPreviewSteps = new ArrayList<>(NUMBER_OF_PREVIEW_STEPS);
      for (int i = 0; i < NUMBER_OF_PREVIEW_STEPS; i++)
      {
         xGaitPreviewSteps.add(new QuadrupedTimedStep());
      }
      xGaitCurrentSteps = new EndDependentList<>();
      for (RobotEnd robotEnd : RobotEnd.values)
      {
         xGaitCurrentSteps.set(robotEnd, new QuadrupedTimedStep());
      }
      stepAdjustmentVector = new FrameVector();
      stepGoalPosition = new FramePoint();
      crossoverProjection = new QuadrupedStepCrossoverProjection(referenceFrames.getBodyZUpFrame(), minimumStepClearanceParameter.get(),
            maximumStepStrideParameter.get());
      supportCentroid = new FramePoint();

      runtimeEnvironment.getParentRegistry().addChild(registry);
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   public void halt()
   {
      haltFlag.set(true);
      haltTime.set(robotTimestamp.getDoubleValue() + haltTransitionDurationParameter.get());
   }

   private void updateGains()
   {
      mpcSettings.setMaximumPreviewTime(mpcMaximumPreviewTimeParameter.get());
      mpcSettings.setStepAdjustmentCost(mpcStepAdjustmentCostParameter.get());
      mpcSettings.setCopAdjustmentCost(mpcCopAdjustmentCostParameter.get());
      mpcSettings.setMinimumNormalizedContactPressure(mpcMinimumNormalizedContactPressureParameter.get());
      comPositionController.getGains().setProportionalGains(comPositionProportionalGainsParameter.get());
      comPositionController.getGains().setIntegralGains(comPositionIntegralGainsParameter.get(), comPositionMaxIntegralErrorParameter.get());
      comPositionController.getGains().setDerivativeGains(comPositionDerivativeGainsParameter.get());
      bodyOrientationController.getGains().setProportionalGains(bodyOrientationProportionalGainsParameter.get());
      bodyOrientationController.getGains().setIntegralGains(bodyOrientationIntegralGainsParameter.get(), bodyOrientationMaxIntegralErrorParameter.get());
      bodyOrientationController.getGains().setDerivativeGains(bodyOrientationDerivativeGainsParameter.get());
      taskSpaceControllerSettings.getVirtualModelControllerSettings().setJointDamping(jointDampingParameter.get());
      taskSpaceControllerSettings.getVirtualModelControllerSettings().setJointPositionLimitDamping(jointPositionLimitDampingParameter.get());
      taskSpaceControllerSettings.getVirtualModelControllerSettings().setJointPositionLimitStiffness(jointPositionLimitStiffnessParameter.get());
   }

   private void updateEstimates()
   {
      // update model
      lipModel.setComHeight(postureProvider.getComPositionInput().getZ());

      // update task space estimates
      taskSpaceEstimator.compute(taskSpaceEstimates);
   }

   private void updateSetpoints()
   {
      // update desired horizontal com forces
      lipModel.computeComForce(taskSpaceControllerCommands.getComForce(), cmpPositionSetpoint);
      taskSpaceControllerCommands.getComForce().changeFrame(supportFrame);

      // update desired com position, velocity, and vertical force
      comPositionControllerSetpoints.getComPosition().changeFrame(supportFrame);
      comPositionControllerSetpoints.getComPosition().set(postureProvider.getComPositionInput());
      comPositionControllerSetpoints.getComVelocity().changeFrame(supportFrame);
      comPositionControllerSetpoints.getComVelocity().set(postureProvider.getComVelocityInput());
      comPositionControllerSetpoints.getComForceFeedforward().changeFrame(supportFrame);
      comPositionControllerSetpoints.getComForceFeedforward().set(taskSpaceControllerCommands.getComForce());
      comPositionControllerSetpoints.getComForceFeedforward().setZ(comPositionGravityCompensationParameter.get() * mass * gravity);
      comPositionController.compute(taskSpaceControllerCommands.getComForce(), comPositionControllerSetpoints, taskSpaceEstimates);

      // update desired body orientation, angular velocity, and torque
      bodyYawSetpoint += planarVelocityProvider.get().getZ() * controlDT;
      bodyOrientationControllerSetpoints.getBodyOrientation().changeFrame(worldFrame);
      bodyOrientationControllerSetpoints.getBodyOrientation()
            .setYawPitchRoll(RotationTools.computeYaw(postureProvider.getBodyOrientationInput()) + bodyYawSetpoint,
                  RotationTools.computePitch(postureProvider.getBodyOrientationInput()) + groundPlaneEstimator.getPitch(bodyYawSetpoint),
                  RotationTools.computeRoll(postureProvider.getBodyOrientationInput()));
      bodyOrientationControllerSetpoints.getBodyAngularVelocity().setToZero();
      bodyOrientationControllerSetpoints.getComTorqueFeedforward().setToZero();
      bodyOrientationController.compute(taskSpaceControllerCommands.getComTorque(), bodyOrientationControllerSetpoints, taskSpaceEstimates);

      // update desired contact state and sole forces
      timedStepController.compute(taskSpaceControllerSettings.getContactState(), taskSpaceControllerSettings.getContactForceLimits(), taskSpaceControllerCommands.getSoleForce(), taskSpaceEstimates);

      // update joint setpoints
      taskSpaceController.compute(taskSpaceControllerSettings, taskSpaceControllerCommands);
   }

   private void updateXGaitSettings()
   {
      xGaitSettingsProvider.getSettings(xGaitSettings);

      // increase stance dimensions to prevent self collisions
      double strideRotation = planarVelocityProvider.get().getZ() * xGaitSettings.getStepDuration();
      double strideLength = Math.abs(2 * planarVelocityProvider.get().getX() * xGaitSettings.getStepDuration());
      double strideWidth = Math.abs(2 * planarVelocityProvider.get().getY() * xGaitSettings.getStepDuration());
      strideLength += Math.abs(xGaitSettings.getStanceWidth() / 2 * Math.sin(2 * strideRotation));
      strideWidth += Math.abs(xGaitSettings.getStanceLength() / 2 * Math.sin(2 * strideRotation));
      xGaitSettings.setStanceLength(Math.max(xGaitSettings.getStanceLength(), strideLength / 2 + minimumStepClearanceParameter.get()));
      xGaitSettings.setStanceWidth(Math.max(xGaitSettings.getStanceWidth(), strideWidth / 2 + minimumStepClearanceParameter.get()));
   }

   private void updateStepPlan()
   {
      // compute xgait step plan
      double currentTime = robotTimestamp.getDoubleValue();
      Vector3d inputVelocity = planarVelocityProvider.get();
      xGaitStepPlanner.computeOnlinePlan(xGaitPreviewSteps, xGaitCurrentSteps, inputVelocity, currentTime, bodyYawSetpoint, xGaitSettings);
   }

   private void updateStepQueue()
   {
      // update step controller queue
      timedStepController.removeSteps();
      for (RobotEnd robotEnd : RobotEnd.values)
      {
         if (timedStepController.getCurrentStep(robotEnd) != null)
            timedStepController.getCurrentStep(robotEnd).set(xGaitCurrentSteps.get(robotEnd));
         else
            timedStepController.addStep(xGaitCurrentSteps.get(robotEnd));
      }
      for (int i = 0; i < xGaitPreviewSteps.size(); i++)
      {
         if (!haltFlag.getBooleanValue() || xGaitPreviewSteps.get(i).getTimeInterval().getEndTime() < haltTime.getDoubleValue())
            timedStepController.addStep(xGaitPreviewSteps.get(i));
      }

      // compute cmp position and step adjustment
      if (timedStepController.getStepSequenceSize() > 0)
      {
         computeStepAdjustmentAndCmpPosition();
      }
   }

   private void computeStepAdjustmentAndCmpPosition()
   {
      double currentTime = robotTimestamp.getDoubleValue();

      // solve for step adjustment and cmp position
      mpcOptimization.compute(stepAdjustmentVector, cmpPositionSetpoint, timedStepController.getStepSequence(), taskSpaceEstimates.getSolePosition(),
            taskSpaceControllerSettings.getContactState(), taskSpaceEstimates.getComPosition(), taskSpaceEstimates.getComVelocity(), currentTime, mpcSettings);

      // adjust goal positions in step controller queue
      stepAdjustmentVector.changeFrame(worldFrame);
      for (int i = 0; i < timedStepController.getStepSequence().size(); i++)
      {
         QuadrupedTimedStep step = timedStepController.getStepSequence().get(i);
         step.getGoalPosition(stepGoalPosition);
         stepGoalPosition.changeFrame(worldFrame);
         stepGoalPosition.add(stepAdjustmentVector);
         if (step.getTimeInterval().getStartTime() <= currentTime)
         {
            crossoverProjection.project(stepGoalPosition, taskSpaceEstimates.getSolePosition(), step.getRobotQuadrant());
         }
         groundPlaneEstimator.projectZ(stepGoalPosition);
         step.setGoalPosition(stepGoalPosition);
      }
   }

   @Override
   public void onLiftOff(RobotQuadrant thisStepQuadrant)
   {
      // update ground plane estimate
      groundPlanePositions.get(thisStepQuadrant).setIncludingFrame(taskSpaceEstimates.getSolePosition(thisStepQuadrant));
      groundPlanePositions.get(thisStepQuadrant).changeFrame(ReferenceFrame.getWorldFrame());
      groundPlaneEstimator.compute(groundPlanePositions);

      // update current step
      RobotEnd thisStepEnd = thisStepQuadrant.getEnd();
      xGaitCurrentSteps.get(thisStepEnd).set(timedStepController.getCurrentStep(thisStepQuadrant));
   }

   @Override
   public void onTouchDown(RobotQuadrant thisStepQuadrant)
   {
      // update current step goal position
      RobotEnd thisStepEnd = thisStepQuadrant.getEnd();
      if (thisStepQuadrant == xGaitCurrentSteps.get(thisStepEnd).getRobotQuadrant())
         xGaitCurrentSteps.get(thisStepEnd).setGoalPosition(taskSpaceEstimates.getSolePosition(thisStepQuadrant));
   }

   @Override
   public void onEntry()
   {
      haltFlag.set(false);

      // initialize estimates
      updateEstimates();
      supportCentroid.setToZero(supportFrame);

      // initialize feedback controllers
      comPositionControllerSetpoints.initialize(taskSpaceEstimates);
      comPositionController.reset();
      bodyOrientationControllerSetpoints.initialize(taskSpaceEstimates);
      bodyOrientationController.reset();
      timedStepController.reset();
      timedStepController.registerStepTransitionCallback(this);

      // initialize task space controller
      taskSpaceControllerSettings.initialize();
      taskSpaceControllerSettings.getContactForceOptimizationSettings().setComForceCommandWeights(1.0, 1.0, 1.0);
      taskSpaceControllerSettings.getContactForceOptimizationSettings().setComTorqueCommandWeights(1.0, 1.0, 1.0);
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         taskSpaceControllerSettings.getContactForceOptimizationSettings().setContactForceCommandWeights(robotQuadrant, 0.0, 0.0, 0.0);
         taskSpaceControllerSettings.setContactState(robotQuadrant, ContactState.IN_CONTACT);
      }
      taskSpaceController.reset();

      // initialize mpc optimization
      mpcOptimization.initialize();

      // initialize body yaw trajectory
      taskSpaceEstimates.getBodyOrientation().changeFrame(worldFrame);
      bodyYawSetpoint = taskSpaceEstimates.getBodyOrientation().getYaw();

      // initialize ground plane
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         groundPlanePositions.get(robotQuadrant).setIncludingFrame(taskSpaceEstimates.getSolePosition(robotQuadrant));
         groundPlanePositions.get(robotQuadrant).changeFrame(ReferenceFrame.getWorldFrame());
      }
      groundPlaneEstimator.compute(groundPlanePositions);

      // initialize step plan
      updateXGaitSettings();
      double initialTime = robotTimestamp.getDoubleValue() + initialTransitionDurationParameter.get();
      Vector3d initialVelocity = planarVelocityProvider.get();
      RobotQuadrant initialQuadrant = (xGaitSettings.getEndPhaseShift() < 90) ? RobotQuadrant.HIND_LEFT : RobotQuadrant.FRONT_LEFT;
      xGaitStepPlanner.computeInitialPlan(xGaitPreviewSteps, initialVelocity, initialQuadrant, supportCentroid, initialTime, bodyYawSetpoint, xGaitSettings);
      for (int i = 0; i < 2; i++)
      {
         RobotEnd robotEnd = xGaitPreviewSteps.get(i).getRobotQuadrant().getEnd();
         xGaitCurrentSteps.get(robotEnd).set(xGaitPreviewSteps.get(i));
      }

      // initialize step controller queue
      for (int i = 0; i < xGaitPreviewSteps.size(); i++)
      {
         timedStepController.addStep(xGaitPreviewSteps.get(i));
      }

      // initialize cmp position and step adjustment
      computeStepAdjustmentAndCmpPosition();
   }

   @Override
   public ControllerEvent process()
   {
      if (timedStepController.getStepSequenceSize() == 0)
      {
         return ControllerEvent.DONE;
      }
      else
      {
         updateGains();
         updateXGaitSettings();
         updateEstimates();
         updateSetpoints();
         updateStepPlan();
         updateStepQueue();
         return null;
      }
   }

   @Override
   public void onExit()
   {
      timedStepController.removeSteps();
      timedStepController.registerStepTransitionCallback(null);
   }
}
