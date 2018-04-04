package us.ihmc.quadrupedRobotics.controlModules;

import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.OrientationFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.MomentumRateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualModelControlCommand;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.QuadrupedBodyOrientationCommand;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerToolbox;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedBodyOrientationController;
import us.ihmc.quadrupedRobotics.estimator.GroundPlaneEstimator;
import us.ihmc.quadrupedRobotics.providers.QuadrupedPostureInputProviderInterface;
import us.ihmc.robotics.controllers.pidGains.GainCoupling;
import us.ihmc.robotics.controllers.pidGains.YoPID3DGains;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultPID3DGains;
import us.ihmc.robotics.controllers.pidGains.implementations.ParameterizedPID3DGains;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsOrientationTrajectoryGenerator;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class QuadrupedBodyOrientationManager
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final QuadrupedBodyOrientationController.Setpoints setpoints = new QuadrupedBodyOrientationController.Setpoints();
   private final QuadrupedBodyOrientationController controller;
   private final YoPID3DGains gains;

   private final ParameterizedPID3DGains bodyOrientationGainsParameter;

   private final MultipleWaypointsOrientationTrajectoryGenerator bodyOrientationTrajectory;

   private final GroundPlaneEstimator groundPlaneEstimator;

   private final FrameQuaternion desiredBodyOrientation;
   private final FrameQuaternion desiredBodyOrientationOffset;
   private final FrameVector3D desiredBodyAngularVelocity;
   private final FrameVector3D desiredBodyAngularAcceleration;
   private final FrameVector3D desiredBodyFeedForwardTorque;

   private final YoDouble robotTimestamp;
   private final QuadrupedForceControllerToolbox controllerToolbox;

   private final MomentumRateCommand angularMomentumCommand = new MomentumRateCommand();
   private final YoFrameVector bodyAngularWeight = new YoFrameVector("bodyAngularWeight", worldFrame, registry);

   private final FrameVector3D desiredAngularMomentumRate = new FrameVector3D();

   public QuadrupedBodyOrientationManager(QuadrupedForceControllerToolbox controllerToolbox, QuadrupedPostureInputProviderInterface postureProvider,
                                          YoVariableRegistry parentRegistry)
   {
      this.controllerToolbox = controllerToolbox;

      robotTimestamp = controllerToolbox.getRuntimeEnvironment().getRobotTimestamp();

      DefaultPID3DGains bodyOrientationDefaultGains = new DefaultPID3DGains();
      bodyOrientationDefaultGains.setProportionalGains(1000.0, 1000.0, 1000.0);
      bodyOrientationDefaultGains.setDerivativeGains(250.0, 250.0, 250.0);
      bodyOrientationDefaultGains.setIntegralGains(0.0, 0.0, 0.0, 0.0);
      bodyOrientationGainsParameter = new ParameterizedPID3DGains("_bodyOrientation", GainCoupling.NONE, false, bodyOrientationDefaultGains, registry);

      controller = new QuadrupedBodyOrientationController(controllerToolbox, registry);
      groundPlaneEstimator = controllerToolbox.getGroundPlaneEstimator();
      gains = controller.getGains();

      bodyOrientationTrajectory = new MultipleWaypointsOrientationTrajectoryGenerator("bodyTrajectory", worldFrame, registry);

      desiredBodyOrientation = new FrameQuaternion();
      desiredBodyOrientationOffset = new FrameQuaternion();
      desiredBodyAngularVelocity = new FrameVector3D();
      desiredBodyAngularAcceleration = new FrameVector3D();

      desiredBodyFeedForwardTorque = new FrameVector3D();

      bodyAngularWeight.set(2.5, 2.5, 1.0);
      angularMomentumCommand.setAngularWeights(bodyAngularWeight);
      angularMomentumCommand.setSelectionMatrixForAngularControl();

      parentRegistry.addChild(registry);
   }

   public void initialize(FrameQuaternionReadOnly bodyOrientationEstimate)
   {
      setpoints.initialize(bodyOrientationEstimate);
      controller.reset();
   }

   public void handleBodyOrientationCommand(QuadrupedBodyOrientationCommand command)
   {
      double currentTime = robotTimestamp.getDoubleValue();
      double timeShift = command.isExpressedInAbsoluteTime() ? 0.0 : currentTime;
      command.getSO3Trajectory().getTrajectoryPointList().addTimeOffset(timeShift);

      bodyOrientationTrajectory.appendWaypoints(command.getSO3Trajectory().getTrajectoryPointList());
   }

   public void compute(FrameQuaternionReadOnly bodyOrientationDesired)
   {
      gains.set(bodyOrientationGainsParameter);

      bodyOrientationTrajectory.compute(robotTimestamp.getDoubleValue());

      desiredBodyOrientation.setIncludingFrame(bodyOrientationDesired);
      desiredBodyOrientation.changeFrame(worldFrame);

      bodyOrientationTrajectory.getAngularData(desiredBodyOrientationOffset, desiredBodyAngularVelocity, desiredBodyAngularAcceleration);
      desiredBodyOrientation.append(desiredBodyOrientationOffset);

      double bodyOrientationYaw = desiredBodyOrientation.getYaw();
      double bodyOrientationPitch = desiredBodyOrientation.getPitch() + groundPlaneEstimator.getPitch(bodyOrientationYaw);
      double bodyOrientationRoll = desiredBodyOrientation.getRoll();
      desiredBodyOrientation.setYawPitchRoll(bodyOrientationYaw, bodyOrientationPitch, bodyOrientationRoll);

      desiredBodyFeedForwardTorque.setToZero();

      controller.compute(desiredAngularMomentumRate, desiredBodyOrientation, desiredBodyAngularVelocity, desiredBodyFeedForwardTorque,
                         controllerToolbox.getTaskSpaceEstimates().getBodyAngularVelocity());

      desiredAngularMomentumRate.changeFrame(worldFrame);
      angularMomentumCommand.setAngularMomentumRate(desiredAngularMomentumRate);
      angularMomentumCommand.setAngularWeights(bodyAngularWeight);
   }

   public FeedbackControlCommand<?> createFeedbackControlTemplate()
   {
      return getFeedbackControlCommand();
   }

   public OrientationFeedbackControlCommand getFeedbackControlCommand()
   {
      return null;
   }

   public VirtualModelControlCommand<?> getVirtualModelControlCommand()
   {
      return angularMomentumCommand;
   }
}
