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
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.SO3TrajectoryControllerCommand;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerToolbox;
import us.ihmc.quadrupedRobotics.estimator.GroundPlaneEstimator;
import us.ihmc.robotics.controllers.AxisAngleOrientationController;
import us.ihmc.robotics.controllers.pidGains.GainCoupling;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultPID3DGains;
import us.ihmc.robotics.controllers.pidGains.implementations.ParameterizedPID3DGains;
import us.ihmc.robotics.math.frames.YoFrameOrientation;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsOrientationTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsPositionTrajectoryGenerator;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class QuadrupedBodyOrientationManager
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final ParameterizedPID3DGains bodyOrientationGainsParameter;

   private final YoBoolean useAbsoluteBodyOrientationTrajectory = new YoBoolean("useBaseBodyOrientationTrajectory", registry);
   private final MultipleWaypointsOrientationTrajectoryGenerator offsetBodyOrientationTrajectory;
   private final MultipleWaypointsOrientationTrajectoryGenerator absoluteBodyOrientationTrajectory;

   private final GroundPlaneEstimator groundPlaneEstimator;

   private final ReferenceFrame bodyFrame;
   private final AxisAngleOrientationController bodyOrientationController;
   private final YoFrameOrientation yoBodyOrientationSetpoint;
   private final YoFrameVector yoBodyAngularVelocitySetpoint;
   private final YoFrameVector yoComTorqueFeedforwardSetpoint;

   private final FrameQuaternion desiredBodyOrientation;
   private final FrameQuaternion desiredBodyOrientationOffset;
   private final FrameVector3D desiredBodyAngularVelocity;
   private final FrameVector3D desiredBodyAngularAcceleration;
   private final FrameVector3D desiredBodyFeedForwardTorque;

   private ReferenceFrame desiredFrameToHold;

   private final YoDouble robotTimestamp;
   private final QuadrupedForceControllerToolbox controllerToolbox;

   private final MomentumRateCommand angularMomentumCommand = new MomentumRateCommand();
   private final YoFrameVector bodyAngularWeight = new YoFrameVector("bodyAngularWeight", worldFrame, registry);

   private final FrameVector3D desiredAngularMomentumRate = new FrameVector3D();

   public QuadrupedBodyOrientationManager(QuadrupedForceControllerToolbox controllerToolbox, YoVariableRegistry parentRegistry)
   {
      this.controllerToolbox = controllerToolbox;

      robotTimestamp = controllerToolbox.getRuntimeEnvironment().getRobotTimestamp();

      DefaultPID3DGains bodyOrientationDefaultGains = new DefaultPID3DGains();
      bodyOrientationDefaultGains.setProportionalGains(1000.0, 1000.0, 1000.0);
      bodyOrientationDefaultGains.setDerivativeGains(250.0, 250.0, 250.0);
      bodyOrientationDefaultGains.setIntegralGains(0.0, 0.0, 0.0, 0.0);
      bodyOrientationGainsParameter = new ParameterizedPID3DGains("_bodyOrientation", GainCoupling.NONE, false, bodyOrientationDefaultGains, registry);

      bodyFrame = controllerToolbox.getReferenceFrames().getBodyFrame();
      bodyOrientationController = new AxisAngleOrientationController("bodyOrientation", bodyFrame, controllerToolbox.getRuntimeEnvironment().getControlDT(), registry);
      yoBodyOrientationSetpoint = new YoFrameOrientation("bodyOrientationSetpoint", worldFrame, registry);
      yoBodyAngularVelocitySetpoint = new YoFrameVector("bodyAngularVelocitySetpoint", worldFrame, registry);
      yoComTorqueFeedforwardSetpoint = new YoFrameVector("comTorqueFeedforwardSetpoint", worldFrame, registry);

      groundPlaneEstimator = controllerToolbox.getGroundPlaneEstimator();

      offsetBodyOrientationTrajectory = new MultipleWaypointsOrientationTrajectoryGenerator("offsetBodyOrientationTrajectory", worldFrame, registry);
      absoluteBodyOrientationTrajectory = new MultipleWaypointsOrientationTrajectoryGenerator("baseBodyOrientationTrajectory", worldFrame, registry);

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
      desiredBodyOrientation.setIncludingFrame(bodyOrientationEstimate);
      desiredBodyAngularVelocity.setToZero();
      desiredBodyAngularAcceleration.setToZero();
      desiredBodyFeedForwardTorque.setToZero();

      double currentTime = robotTimestamp.getDoubleValue();

      offsetBodyOrientationTrajectory.appendWaypoint(currentTime, desiredBodyOrientation, desiredBodyAngularVelocity);
      offsetBodyOrientationTrajectory.initialize();

      bodyOrientationController.reset();
      bodyOrientationController.resetIntegrator();
      useAbsoluteBodyOrientationTrajectory.set(false);
   }

   public void handleBodyOrientationCommand(QuadrupedBodyOrientationCommand command)
   {
      double currentTime = robotTimestamp.getDoubleValue();
      double timeShift = command.isExpressedInAbsoluteTime() ? 0.0 : currentTime;
      SO3TrajectoryControllerCommand so3Trajectory = command.getSO3Trajectory();
      so3Trajectory.getTrajectoryPointList().addTimeOffset(timeShift);

      if(command.isAnOffsetOrientation())
      {
         handleOrientationTrajectory(so3Trajectory, offsetBodyOrientationTrajectory);
         useAbsoluteBodyOrientationTrajectory.set(false);
      }
      else
      {
         handleOrientationTrajectory(so3Trajectory, absoluteBodyOrientationTrajectory);
         useAbsoluteBodyOrientationTrajectory.set(true);
      }
   }

   private void handleOrientationTrajectory(SO3TrajectoryControllerCommand so3Trajectory, MultipleWaypointsOrientationTrajectoryGenerator trajectoryToSet)
   {
      double currentTime = robotTimestamp.getDoubleValue();

      if (so3Trajectory.getTrajectoryPoint(0).getTime() > 1.0e-5 + currentTime)
      {
         trajectoryToSet.getOrientation(desiredBodyOrientation);
         desiredBodyOrientation.changeFrame(worldFrame);
         desiredBodyAngularVelocity.setToZero(worldFrame);

         trajectoryToSet.clear();
         trajectoryToSet.appendWaypoint(currentTime, desiredBodyOrientation, desiredBodyAngularVelocity);
      }
      else
      {
         trajectoryToSet.clear();
      }

      trajectoryToSet.appendWaypoints(so3Trajectory.getTrajectoryPointList());
      trajectoryToSet.initialize();
   }

   public void setDesiredFrameToHoldPosition(ReferenceFrame desiredFrameToHold)
   {
      this.desiredFrameToHold = desiredFrameToHold;
   }

   public void compute()
   {
      if (useAbsoluteBodyOrientationTrajectory.getBooleanValue() && absoluteBodyOrientationTrajectory.isDone())
      {
         useAbsoluteBodyOrientationTrajectory.set(false);
         absoluteBodyOrientationTrajectory.clear();
      }

      if(useAbsoluteBodyOrientationTrajectory.getBooleanValue())
      {
         computeSetpoints(worldFrame, absoluteBodyOrientationTrajectory);
      }
      else
      {
         computeSetpoints(desiredFrameToHold, offsetBodyOrientationTrajectory);
      }

      doControl();

      // update log variables
      yoBodyOrientationSetpoint.setAndMatchFrame(desiredBodyOrientation);
      yoBodyAngularVelocitySetpoint.setMatchingFrame(desiredBodyAngularVelocity);
      yoComTorqueFeedforwardSetpoint.setMatchingFrame(desiredBodyFeedForwardTorque);

      desiredAngularMomentumRate.changeFrame(worldFrame);
      angularMomentumCommand.setAngularMomentumRate(desiredAngularMomentumRate);
      angularMomentumCommand.setAngularWeights(bodyAngularWeight);
   }

   private void computeSetpoints(ReferenceFrame baseFrame, MultipleWaypointsOrientationTrajectoryGenerator orientationTrajectory)
   {
      orientationTrajectory.compute(robotTimestamp.getDoubleValue());

      desiredBodyOrientation.setToZero(baseFrame);
      desiredBodyOrientation.changeFrame(worldFrame);

      orientationTrajectory.getAngularData(desiredBodyOrientationOffset, desiredBodyAngularVelocity, desiredBodyAngularAcceleration);
      desiredBodyOrientation.append(desiredBodyOrientationOffset);

      double bodyOrientationYaw = desiredBodyOrientation.getYaw();
      double bodyOrientationPitch = desiredBodyOrientation.getPitch() + groundPlaneEstimator.getPitch(bodyOrientationYaw);
      double bodyOrientationRoll = desiredBodyOrientation.getRoll();
      desiredBodyOrientation.setYawPitchRoll(bodyOrientationYaw, bodyOrientationPitch, bodyOrientationRoll);
      desiredBodyFeedForwardTorque.setToZero();
   }

   private void doControl()
   {
      FrameVector3D estimatedVelocity = controllerToolbox.getTaskSpaceEstimates().getBodyAngularVelocity();

      // compute body torque
      desiredAngularMomentumRate.setToZero(bodyFrame);
      desiredBodyOrientation.changeFrame(bodyFrame);
      desiredBodyAngularVelocity.changeFrame(bodyFrame);
      estimatedVelocity.changeFrame(bodyFrame);
      desiredBodyFeedForwardTorque.changeFrame(bodyFrame);
      bodyOrientationController.setGains(bodyOrientationGainsParameter);
      bodyOrientationController
            .compute(desiredAngularMomentumRate, desiredBodyOrientation, desiredBodyAngularVelocity, estimatedVelocity, desiredBodyFeedForwardTorque);

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
