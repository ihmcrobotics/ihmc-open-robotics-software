package us.ihmc.aware.controller.force;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.aware.params.DoubleArrayParameter;
import us.ihmc.aware.params.DoubleParameter;
import us.ihmc.aware.params.ParameterFactory;
import us.ihmc.aware.controller.common.DivergentComponentOfMotionController;
import us.ihmc.aware.controller.common.GroundPlaneEstimator;
import us.ihmc.aware.controller.force.taskSpaceController.QuadrupedTaskSpaceCommands;
import us.ihmc.aware.controller.force.taskSpaceController.QuadrupedTaskSpaceController;
import us.ihmc.aware.controller.force.taskSpaceController.QuadrupedTaskSpaceControllerSettings;
import us.ihmc.aware.controller.force.taskSpaceController.QuadrupedTaskSpaceEstimates;
import us.ihmc.aware.controller.force.taskSpaceController.QuadrupedTaskSpaceEstimator;
import us.ihmc.aware.controller.force.taskSpaceController.QuadrupedTaskSpaceSetpoints;
import us.ihmc.aware.parameters.QuadrupedRuntimeEnvironment;
import us.ihmc.aware.util.ContactState;
import us.ihmc.quadrupedRobotics.dataProviders.QuadrupedControllerInputProviderInterface;
import us.ihmc.quadrupedRobotics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
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
   private final ParameterFactory parameterFactory = new ParameterFactory(QuadrupedVirtualModelBasedStandController.class.getName());
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
   private final DoubleArrayParameter dcmPositionDerivativeGainsParameter = parameterFactory.createDoubleArray("dcmPositionDerivativeGains", 0, 0, 0);
   private final DoubleArrayParameter dcmPositionIntegralGainsParameter = parameterFactory.createDoubleArray("dcmPositionIntegralGains", 0, 0, 0);
   private final DoubleParameter dcmPositionMaxIntegralErrorParameter = parameterFactory.createDouble("dcmPositionMaxIntegralError", 0);

   // frames
   private final ReferenceFrame supportFrame;
   private final ReferenceFrame worldFrame;

   // dcm controller
   private final FramePoint dcmPositionEstimate;
   private final FramePoint dcmPositionSetpoint;
   private final FrameVector dcmVelocitySetpoint;
   private final DivergentComponentOfMotionController dcmPositionController;

   // task space controller
   private final QuadrupedTaskSpaceCommands taskSpaceCommands;
   private final QuadrupedTaskSpaceSetpoints taskSpaceSetpoints;
   private final QuadrupedTaskSpaceEstimates taskSpaceEstimates;
   private final QuadrupedTaskSpaceEstimator taskSpaceEstimator;
   private final QuadrupedTaskSpaceController taskSpaceController;
   private final QuadrupedTaskSpaceControllerSettings taskSpaceControllerSettings;

   // planning
   private final GroundPlaneEstimator groundPlaneEstimator;

   public QuadrupedVirtualModelBasedStandController(QuadrupedRuntimeEnvironment runtimeEnvironment, QuadrupedControllerInputProviderInterface inputProvider,
         QuadrupedForceControllerContext controllerContext)
   {
      this.fullRobotModel = runtimeEnvironment.getFullRobotModel();
      this.robotTimestamp = runtimeEnvironment.getRobotTimestamp();
      this.controlDT = runtimeEnvironment.getControlDT();
      this.gravity = 9.81;
      this.mass = fullRobotModel.getTotalMass();
      this.inputProvider = inputProvider;

      // frames
      QuadrupedReferenceFrames referenceFrames = controllerContext.getReferenceFrames();
      supportFrame = referenceFrames.getCenterOfFeetZUpFrameAveragingLowestZHeightsAcrossEnds();
      worldFrame = ReferenceFrame.getWorldFrame();

      // dcm controller
      dcmPositionEstimate = new FramePoint();
      dcmPositionSetpoint = new FramePoint();
      dcmVelocitySetpoint = new FrameVector();
      dcmPositionController = controllerContext.getDcmPositionController();

      // task space controllers
      taskSpaceCommands = new QuadrupedTaskSpaceCommands();
      taskSpaceSetpoints = new QuadrupedTaskSpaceSetpoints();
      taskSpaceEstimates = new QuadrupedTaskSpaceEstimates();
      taskSpaceControllerSettings = new QuadrupedTaskSpaceControllerSettings();
      taskSpaceEstimator = controllerContext.getTaskSpaceEstimator();
      taskSpaceController = controllerContext.getTaskSpaceController();

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
      // update desired dcm position
      dcmPositionSetpoint.changeFrame(supportFrame);
      dcmPositionSetpoint.set(inputProvider.getComVelocityInput());
      dcmPositionSetpoint.scale(1.0 / dcmPositionController.getNaturalFrequency());
      dcmPositionSetpoint.add(inputProvider.getComPositionInput());
      dcmVelocitySetpoint.setToZero(supportFrame);
      dcmPositionController.compute(taskSpaceSetpoints.getComForceFeedforward(), dcmPositionSetpoint, dcmVelocitySetpoint, dcmPositionEstimate);

      // update desired com position and velocity
      taskSpaceSetpoints.getComPosition().changeFrame(supportFrame);
      taskSpaceSetpoints.getComPosition().set(inputProvider.getComPositionInput());
      taskSpaceSetpoints.getComVelocity().changeFrame(supportFrame);
      taskSpaceSetpoints.getComVelocity().set(inputProvider.getComVelocityInput());
      taskSpaceSetpoints.getComForceFeedforward().changeFrame(supportFrame);
      taskSpaceSetpoints.getComForceFeedforward().setZ(mass * gravity);

      // update desired body orientation and angular rate
      taskSpaceSetpoints.getBodyOrientation().changeFrame(supportFrame);
      taskSpaceSetpoints.getBodyOrientation().set(inputProvider.getBodyOrientationInput());
      taskSpaceSetpoints.getBodyOrientation().changeFrame(worldFrame);
      double bodyYaw = taskSpaceSetpoints.getBodyOrientation().getYaw();
      double bodyPitch = taskSpaceSetpoints.getBodyOrientation().getPitch() + groundPlaneEstimator.getPitch(bodyYaw);
      double bodyRoll = taskSpaceSetpoints.getBodyOrientation().getRoll();
      taskSpaceSetpoints.getBodyOrientation().setYawPitchRoll(bodyYaw, bodyPitch, bodyRoll);
      taskSpaceSetpoints.getBodyAngularVelocity().set(inputProvider.getBodyAngularRateInput());

      // update joint setpoints
      taskSpaceController.compute(taskSpaceControllerSettings, taskSpaceSetpoints, taskSpaceEstimates, taskSpaceCommands);
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
      // initialize dcm controller settings
      dcmPositionController.setGains(
            dcmPositionProportionalGainsParameter.get(),
            dcmPositionDerivativeGainsParameter.get(),
            dcmPositionIntegralGainsParameter.get(),
            dcmPositionMaxIntegralErrorParameter.get()
      );
      dcmPositionController.reset();

      // initialize task space controller
      taskSpaceEstimator.compute(taskSpaceEstimates);
      taskSpaceSetpoints.initialize(taskSpaceEstimates);
      taskSpaceControllerSettings.initialize();
      taskSpaceControllerSettings.setJointDamping(jointDampingParameter.get());
      taskSpaceControllerSettings.setComForceCommandWeights(1.0, 1.0, 1.0);
      taskSpaceControllerSettings.setComTorqueCommandWeights(1.0, 1.0, 1.0);
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         taskSpaceControllerSettings.setSoleForceCommandWeights(robotQuadrant, 0.0, 0.0, 0.0);
         taskSpaceControllerSettings.setContactState(robotQuadrant, ContactState.IN_CONTACT);
      }
      taskSpaceControllerSettings.setBodyOrientationFeedbackGains(
            bodyOrientationProportionalGainsParameter.get(),
            bodyOrientationDerivativeGainsParameter.get(),
            bodyOrientationIntegralGainsParameter.get(),
            bodyOrientationMaxIntegralErrorParameter.get()
      );
      taskSpaceControllerSettings.setComPositionFeedbackGains(
            comPositionProportionalGainsParameter.get(),
            comPositionDerivativeGainsParameter.get(),
            comPositionIntegralGainsParameter.get(),
            comPositionMaxIntegralErrorParameter.get()
      );
      taskSpaceController.reset();
   }

   @Override public void onExit()
   {
   }
}
