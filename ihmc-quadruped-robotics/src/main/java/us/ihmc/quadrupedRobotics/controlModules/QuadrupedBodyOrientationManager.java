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
import us.ihmc.quadrupedRobotics.controller.QuadrupedControllerToolbox;
import us.ihmc.quadrupedRobotics.estimator.GroundPlaneEstimator;
import us.ihmc.robotics.controllers.AxisAngleOrientationController;
import us.ihmc.robotics.controllers.pidGains.GainCoupling;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultPID3DGains;
import us.ihmc.robotics.controllers.pidGains.implementations.ParameterizedPID3DGains;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsOrientationTrajectoryGenerator;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFrameQuaternion;
import us.ihmc.yoVariables.variable.YoFrameVector3D;
import us.ihmc.yoVariables.variable.YoFrameYawPitchRoll;

public class QuadrupedBodyOrientationManager
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final ParameterizedPID3DGains bodyOrientationGainsParameter;

   private final MultipleWaypointsOrientationTrajectoryGenerator bodyOrientationTrajectory;

   private final GroundPlaneEstimator groundPlaneEstimator;

   private final YoFrameYawPitchRoll yoBodyOrientationSetpoint;
   private final YoFrameVector3D yoBodyAngularVelocitySetpoint;
   private final YoFrameVector3D yoComTorqueFeedforwardSetpoint;

   private final FrameQuaternion desiredBodyOrientation;
   private final FrameQuaternion desiredBodyOrientationOffset;
   private final FrameVector3D desiredBodyAngularVelocity;
   private final FrameVector3D desiredBodyAngularAcceleration;

   private ReferenceFrame desiredFrameToHold;

   private final YoDouble robotTimestamp;

   private final OrientationFeedbackControlCommand feedbackControlCommand = new OrientationFeedbackControlCommand();
   private final YoFrameVector3D bodyAngularWeight = new YoFrameVector3D("bodyAngularWeight", worldFrame, registry);

   public QuadrupedBodyOrientationManager(QuadrupedControllerToolbox controllerToolbox, YoVariableRegistry parentRegistry)
   {

      robotTimestamp = controllerToolbox.getRuntimeEnvironment().getRobotTimestamp();

      DefaultPID3DGains bodyOrientationDefaultGains = new DefaultPID3DGains();
      bodyOrientationDefaultGains.setProportionalGains(1000.0, 1000.0, 1000.0);
      bodyOrientationDefaultGains.setDerivativeGains(250.0, 250.0, 250.0);
      bodyOrientationDefaultGains.setIntegralGains(0.0, 0.0, 0.0, 0.0);
      bodyOrientationGainsParameter = new ParameterizedPID3DGains("_bodyOrientation", GainCoupling.NONE, false, bodyOrientationDefaultGains, registry);

      ReferenceFrame bodyFrame = controllerToolbox.getReferenceFrames().getBodyFrame();
      yoBodyOrientationSetpoint = new YoFrameYawPitchRoll("bodyOrientationSetpoint", worldFrame, registry);
      yoBodyAngularVelocitySetpoint = new YoFrameVector3D("bodyAngularVelocitySetpoint", worldFrame, registry);
      yoComTorqueFeedforwardSetpoint = new YoFrameVector3D("comTorqueFeedforwardSetpoint", worldFrame, registry);


      feedbackControlCommand.setGains(bodyOrientationDefaultGains);
      feedbackControlCommand.set(controllerToolbox.getFullRobotModel().getElevator(), controllerToolbox.getFullRobotModel().getBody());
      feedbackControlCommand.setGainsFrame(bodyFrame);

      groundPlaneEstimator = controllerToolbox.getGroundPlaneEstimator();

      bodyOrientationTrajectory = new MultipleWaypointsOrientationTrajectoryGenerator("bodyTrajectory", worldFrame, registry);

      desiredBodyOrientation = new FrameQuaternion();
      desiredBodyOrientationOffset = new FrameQuaternion();
      desiredBodyAngularVelocity = new FrameVector3D();
      desiredBodyAngularAcceleration = new FrameVector3D();

      bodyAngularWeight.set(2.5, 2.5, 1.0);

      parentRegistry.addChild(registry);
   }

   public void initialize(FrameQuaternionReadOnly bodyOrientationEstimate)
   {
      desiredBodyOrientation.setIncludingFrame(bodyOrientationEstimate);
      desiredBodyAngularVelocity.setToZero();
      desiredBodyAngularAcceleration.setToZero();

      double currentTime = robotTimestamp.getDoubleValue();

      bodyOrientationTrajectory.appendWaypoint(currentTime, desiredBodyOrientation, desiredBodyAngularVelocity);
      bodyOrientationTrajectory.initialize();
   }

   public void handleBodyOrientationCommand(QuadrupedBodyOrientationCommand command)
   {
      double currentTime = robotTimestamp.getDoubleValue();
      double timeShift = command.isExpressedInAbsoluteTime() ? 0.0 : currentTime;
      SO3TrajectoryControllerCommand so3Trajectory = command.getSO3Trajectory();
      so3Trajectory.getTrajectoryPointList().addTimeOffset(timeShift);

      if (so3Trajectory.getTrajectoryPoint(0).getTime() > 1.0e-5 + currentTime)
      {
         bodyOrientationTrajectory.getOrientation(desiredBodyOrientation);
         desiredBodyOrientation.changeFrame(worldFrame);
         desiredBodyAngularVelocity.setToZero(worldFrame);

         bodyOrientationTrajectory.clear();
         bodyOrientationTrajectory.appendWaypoint(currentTime, desiredBodyOrientation, desiredBodyAngularVelocity);
      }
      else
      {
         bodyOrientationTrajectory.clear();
      }

      bodyOrientationTrajectory.appendWaypoints(so3Trajectory.getTrajectoryPointList());
      bodyOrientationTrajectory.initialize();
   }

   public void setDesiredFrameToHoldPosition(ReferenceFrame desiredFrameToHold)
   {
      this.desiredFrameToHold = desiredFrameToHold;
   }

   public void compute()
   {
      bodyOrientationTrajectory.compute(robotTimestamp.getDoubleValue());

      desiredBodyOrientation.setToZero(desiredFrameToHold);
      desiredBodyOrientation.changeFrame(worldFrame);

      bodyOrientationTrajectory.getAngularData(desiredBodyOrientationOffset, desiredBodyAngularVelocity, desiredBodyAngularAcceleration);
      desiredBodyOrientation.append(desiredBodyOrientationOffset);

      double bodyOrientationYaw = desiredBodyOrientation.getYaw();
      double bodyOrientationPitch = desiredBodyOrientation.getPitch() + groundPlaneEstimator.getPitch(bodyOrientationYaw);
      double bodyOrientationRoll = desiredBodyOrientation.getRoll();
      desiredBodyOrientation.setYawPitchRoll(bodyOrientationYaw, bodyOrientationPitch, bodyOrientationRoll);

      yoComTorqueFeedforwardSetpoint.setToZero();

      feedbackControlCommand.setGains(bodyOrientationGainsParameter);
      feedbackControlCommand.setFeedForwardAction(yoComTorqueFeedforwardSetpoint);
      feedbackControlCommand.set(desiredBodyOrientation, desiredBodyAngularVelocity);
      feedbackControlCommand.setWeightsForSolver(bodyAngularWeight);

      yoBodyOrientationSetpoint.set(desiredBodyOrientation);
      yoBodyAngularVelocitySetpoint.set(desiredBodyAngularVelocity);
   }


   public FeedbackControlCommand<?> createFeedbackControlTemplate()
   {
      return getFeedbackControlCommand();
   }

   public OrientationFeedbackControlCommand getFeedbackControlCommand()
   {
      return feedbackControlCommand;
   }

   public VirtualModelControlCommand<?> getVirtualModelControlCommand()
   {
      return null;
   }
}
