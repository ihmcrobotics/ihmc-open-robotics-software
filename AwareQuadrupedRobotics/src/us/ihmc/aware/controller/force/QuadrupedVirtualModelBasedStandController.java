package us.ihmc.aware.controller.force;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.aware.controller.toolbox.*;
import us.ihmc.aware.params.DoubleArrayParameter;
import us.ihmc.aware.params.DoubleParameter;
import us.ihmc.aware.params.ParameterFactory;
import us.ihmc.aware.planning.GroundPlaneEstimator;
import us.ihmc.aware.util.ContactState;
import us.ihmc.aware.parameters.QuadrupedRuntimeEnvironment;
import us.ihmc.quadrupedRobotics.dataProviders.QuadrupedControllerInputProviderInterface;
import us.ihmc.quadrupedRobotics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class QuadrupedVirtualModelBasedStandController implements QuadrupedForceController
{
   private final SDFFullRobotModel fullRobotModel;
   private final DoubleYoVariable robotTimestamp;
   private final double controlDT;
   private final double gravity;
   private final double mass;
   private final QuadrupedControllerInputProviderInterface inputProvider;
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   // parameters
   private final ParameterFactory parameterFactory = new ParameterFactory(getClass().getName());
   private final DoubleParameter jointDampingParameter = parameterFactory.createDouble("jointDamping", 2);
   private final DoubleArrayParameter bodyOrientationProportionalGainsParameter = parameterFactory.createDoubleArray("bodyOrientationProportionalGains", 5000, 5000, 2500);
   private final DoubleArrayParameter bodyOrientationDerivativeGainsParameter = parameterFactory.createDoubleArray("bodyOrientationDerivativeGains", 750, 750, 500);
   private final DoubleArrayParameter bodyOrientationIntegralGainsParameter = parameterFactory.createDoubleArray("bodyOrientationIntegralGains", 0, 0, 0);
   private final DoubleParameter bodyOrientationMaxIntegralErrorParameter = parameterFactory.createDouble("bodyOrientationMaxIntegralError", 0);
   private final DoubleArrayParameter comPositionProportionalGainsParameter = parameterFactory.createDoubleArray("comPositionProportionalGains", 0, 0, 5000);
   private final DoubleArrayParameter comPositionDerivativeGainsParameter = parameterFactory.createDoubleArray("comPositionDerivativeGains", 0, 0, 750);
   private final DoubleArrayParameter comPositionIntegralGainsParameter = parameterFactory.createDoubleArray("comPositionIntegralGains", 0, 0, 0);
   private final DoubleParameter comPositionMaxIntegralErrorParameter = parameterFactory.createDouble("comPositionMaxIntegralError", 0);
   private final DoubleArrayParameter dcmPositionProportionalGainsParameter = parameterFactory.createDoubleArray("dcmPositionProportionalGains", 2, 2, 0);
   private final DoubleArrayParameter dcmPositionDerivativeGainsParameter = parameterFactory.createDoubleArray("dcmPositionDerivativeGains", 1, 1, 0);
   private final DoubleArrayParameter dcmPositionIntegralGainsParameter = parameterFactory.createDoubleArray("dcmPositionIntegralGains", 0, 0, 0);
   private final DoubleParameter dcmPositionMaxIntegralErrorParameter = parameterFactory.createDouble("dcmPositionMaxIntegralError", 0);

   // frames
   private final ReferenceFrame supportFrame;
   private final ReferenceFrame worldFrame;

   // feedback controllers
   private final FramePoint dcmPositionEstimate;
   private final DivergentComponentOfMotionController.Setpoints dcmPositionControllerSetpoints;
   private final DivergentComponentOfMotionController dcmPositionController;
   private final QuadrupedComPositionController.Setpoints comPositionControllerSetpoints;
   private final QuadrupedComPositionController comPositionController;
   private final QuadrupedBodyOrientationController.Setpoints bodyOrientationControllerSetpoints;
   private final QuadrupedBodyOrientationController bodyOrientationController;

   // task space controller
   private final QuadrupedTaskSpaceEstimator.Estimates taskSpaceEstimates;
   private final QuadrupedTaskSpaceEstimator taskSpaceEstimator;
   private final QuadrupedTaskSpaceController.Commands taskSpaceControllerCommands;
   private final QuadrupedTaskSpaceController.Settings taskSpaceControllerSettings;
   private final QuadrupedTaskSpaceController taskSpaceController;

   // planning
   private final GroundPlaneEstimator groundPlaneEstimator;

   public QuadrupedVirtualModelBasedStandController(QuadrupedRuntimeEnvironment runtimeEnvironment, QuadrupedControllerInputProviderInterface inputProvider,
         QuadrupedForceControllerToolbox controllerToolbox)
   {
      this.fullRobotModel = runtimeEnvironment.getFullRobotModel();
      this.robotTimestamp = runtimeEnvironment.getRobotTimestamp();
      this.controlDT = runtimeEnvironment.getControlDT();
      this.gravity = 9.81;
      this.mass = fullRobotModel.getTotalMass();
      this.inputProvider = inputProvider;

      // frames
      QuadrupedReferenceFrames referenceFrames = controllerToolbox.getReferenceFrames();
      supportFrame = referenceFrames.getCenterOfFeetZUpFrameAveragingLowestZHeightsAcrossEnds();
      worldFrame = ReferenceFrame.getWorldFrame();

      // feedback controllers
      dcmPositionEstimate = new FramePoint();
      dcmPositionControllerSetpoints = new DivergentComponentOfMotionController.Setpoints();
      dcmPositionController = controllerToolbox.getDcmPositionController();
      comPositionControllerSetpoints = new QuadrupedComPositionController.Setpoints();
      comPositionController = controllerToolbox.getComPositionController();
      bodyOrientationControllerSetpoints = new QuadrupedBodyOrientationController.Setpoints();
      bodyOrientationController = controllerToolbox.getBodyOrientationController();

      // task space controllers
      taskSpaceEstimates = new QuadrupedTaskSpaceEstimator.Estimates();
      taskSpaceEstimator = controllerToolbox.getTaskSpaceEstimator();
      taskSpaceControllerCommands = new QuadrupedTaskSpaceController.Commands();
      taskSpaceControllerSettings = new QuadrupedTaskSpaceController.Settings();
      taskSpaceController = controllerToolbox.getTaskSpaceController();

      // planning
      groundPlaneEstimator = new GroundPlaneEstimator();

      runtimeEnvironment.getParentRegistry().addChild(registry);
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   private void updateEstimates()
   {
      // update task space estimates
      taskSpaceEstimator.compute(taskSpaceEstimates);

      // update dcm estimate
      taskSpaceEstimates.getComPosition().changeFrame(worldFrame);
      taskSpaceEstimates.getComVelocity().changeFrame(worldFrame);
      dcmPositionEstimate.changeFrame(worldFrame);
      dcmPositionEstimate.set(taskSpaceEstimates.getComVelocity());
      dcmPositionEstimate.scale(1.0 / dcmPositionController.getNaturalFrequency());
      dcmPositionEstimate.add(taskSpaceEstimates.getComPosition());

      // update ground plane estimate
      groundPlaneEstimator.compute(taskSpaceEstimates.getSolePosition());
   }

   private void updateSetpoints()
   {
      taskSpaceControllerCommands.initialize();

      // update desired dcm position
      dcmPositionControllerSetpoints.getDcmPosition().changeFrame(supportFrame);
      dcmPositionControllerSetpoints.getDcmPosition().set(inputProvider.getComVelocityInput());
      dcmPositionControllerSetpoints.getDcmPosition().scale(1.0 / dcmPositionController.getNaturalFrequency());
      dcmPositionControllerSetpoints.getDcmPosition().add(inputProvider.getComPositionInput());
      dcmPositionControllerSetpoints.getDcmVelocity().setToZero(supportFrame);
      dcmPositionController.compute(taskSpaceControllerCommands.getComForce(), dcmPositionControllerSetpoints, dcmPositionEstimate);

      // update desired com position and velocity
      comPositionControllerSetpoints.getComPosition().changeFrame(supportFrame);
      comPositionControllerSetpoints.getComPosition().set(inputProvider.getComPositionInput());
      comPositionControllerSetpoints.getComVelocity().changeFrame(supportFrame);
      comPositionControllerSetpoints.getComVelocity().set(inputProvider.getComVelocityInput());
      comPositionControllerSetpoints.getComForceFeedforward().setIncludingFrame(taskSpaceControllerCommands.getComForce());
      comPositionControllerSetpoints.getComForceFeedforward().changeFrame(supportFrame);
      comPositionControllerSetpoints.getComForceFeedforward().setZ(mass * gravity);
      comPositionController.compute(taskSpaceControllerCommands.getComForce(), comPositionControllerSetpoints, taskSpaceEstimates);

      // update desired body orientation and angular rate
      bodyOrientationControllerSetpoints.getBodyOrientation().changeFrame(supportFrame);
      bodyOrientationControllerSetpoints.getBodyOrientation().set(inputProvider.getBodyOrientationInput());
      bodyOrientationControllerSetpoints.getBodyOrientation().changeFrame(worldFrame);
      double bodyYaw = bodyOrientationControllerSetpoints.getBodyOrientation().getYaw();
      double bodyPitch = bodyOrientationControllerSetpoints.getBodyOrientation().getPitch() + groundPlaneEstimator.getPitch(bodyYaw);
      double bodyRoll = bodyOrientationControllerSetpoints.getBodyOrientation().getRoll();
      bodyOrientationControllerSetpoints.getBodyOrientation().setYawPitchRoll(bodyYaw, bodyPitch, bodyRoll);
      bodyOrientationControllerSetpoints.getBodyAngularVelocity().set(inputProvider.getBodyAngularRateInput());
      bodyOrientationController.compute(taskSpaceControllerCommands.getComTorque(), bodyOrientationControllerSetpoints, taskSpaceEstimates);

      // update joint setpoints
      taskSpaceController.compute(taskSpaceControllerSettings, taskSpaceControllerCommands);
   }

   @Override public QuadrupedForceControllerEvent process()
   {
      dcmPositionController.setComHeight(inputProvider.getComPositionInput().getZ());
      updateEstimates();
      updateSetpoints();
      return null;
   }

   @Override public void onEntry()
   {
      // initialize estimates
      updateEstimates();

      // initialize feedback controllers
      dcmPositionControllerSetpoints.initialize(dcmPositionEstimate);
      dcmPositionController.reset();
      dcmPositionController.setComHeight(inputProvider.getComPositionInput().getZ());
      dcmPositionController.getGains().setProportionalGains(dcmPositionProportionalGainsParameter.get());
      dcmPositionController.getGains().setIntegralGains(dcmPositionIntegralGainsParameter.get(), dcmPositionMaxIntegralErrorParameter.get());
      dcmPositionController.getGains().setDerivativeGains(dcmPositionDerivativeGainsParameter.get());
      comPositionControllerSetpoints.initialize(taskSpaceEstimates);
      comPositionController.reset();
      comPositionController.getGains().setProportionalGains(comPositionProportionalGainsParameter.get());
      comPositionController.getGains().setIntegralGains(comPositionIntegralGainsParameter.get(), comPositionMaxIntegralErrorParameter.get());
      comPositionController.getGains().setDerivativeGains(comPositionDerivativeGainsParameter.get());
      bodyOrientationControllerSetpoints.initialize(taskSpaceEstimates);
      bodyOrientationController.reset();
      bodyOrientationController.getGains().setProportionalGains(bodyOrientationProportionalGainsParameter.get());
      bodyOrientationController.getGains().setIntegralGains(bodyOrientationIntegralGainsParameter.get(), bodyOrientationMaxIntegralErrorParameter.get());
      bodyOrientationController.getGains().setDerivativeGains(bodyOrientationDerivativeGainsParameter.get());

      // initialize task space controller
      taskSpaceControllerSettings.initialize();
      taskSpaceControllerSettings.getVirtualModelControllerSettings().setJointDamping(jointDampingParameter.get());
      taskSpaceControllerSettings.getContactForceOptimizationSettings().setComForceCommandWeights(1.0, 1.0, 1.0);
      taskSpaceControllerSettings.getContactForceOptimizationSettings().setComTorqueCommandWeights(1.0, 1.0, 1.0);
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         taskSpaceControllerSettings.getContactForceOptimizationSettings().setContactForceCommandWeights(robotQuadrant, 0.0, 0.0, 0.0);
         taskSpaceControllerSettings.setContactState(robotQuadrant, ContactState.IN_CONTACT);
      }
      taskSpaceController.reset();
   }

   @Override public void onExit()
   {
   }
}
