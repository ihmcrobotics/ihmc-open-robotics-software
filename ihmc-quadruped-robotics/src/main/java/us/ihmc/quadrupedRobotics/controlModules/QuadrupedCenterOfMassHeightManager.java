package us.ihmc.quadrupedRobotics.controlModules;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.EuclideanTrajectoryControllerCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.QuadrupedBodyHeightCommand;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerToolbox;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.model.QuadrupedPhysicalProperties;
import us.ihmc.robotics.controllers.PIDController;
import us.ihmc.robotics.controllers.pidGains.implementations.PIDGains;
import us.ihmc.robotics.controllers.pidGains.implementations.ParameterizedPIDGains;
import us.ihmc.robotics.math.trajectories.waypoints.FrameEuclideanTrajectoryPoint;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsPositionTrajectoryGenerator;
import us.ihmc.robotics.screwTheory.CenterOfMassJacobian;
import us.ihmc.robotics.screwTheory.MovingReferenceFrame;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class QuadrupedCenterOfMassHeightManager
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoDouble robotTimestamp;

   private final YoBoolean controlBodyHeight = new YoBoolean("controlBodyHeight", registry);

   private final MovingReferenceFrame bodyFrame;
   private final ReferenceFrame centerOfMassFrame;

   private final CenterOfMassJacobian centerOfMassJacobian;

   private final ReferenceFrame supportFrame;
   private final MultipleWaypointsPositionTrajectoryGenerator centerOfMassHeightTrajectory;

   private final PIDController linearMomentumZPDController;

   private final FramePoint3D currentPosition = new FramePoint3D();
   private final FrameVector3D currentVelocity = new FrameVector3D();

   private final FramePoint3D desiredPosition = new FramePoint3D();
   private final FrameVector3D desiredVelocity = new FrameVector3D();
   private final FrameVector3D desiredAcceleration = new FrameVector3D();

   private final ParameterizedPIDGains comPositionGainsParameter;

   private final YoDouble currentHeightInWorld;
   private final YoDouble desiredHeightInWorld;
   private final YoDouble desiredVelocityInWorld;
   private final YoDouble currentVelocityInWorld;

   private final double controlDT;

   public QuadrupedCenterOfMassHeightManager(QuadrupedForceControllerToolbox controllerToolbox, QuadrupedPhysicalProperties physicalProperties,
                                             YoVariableRegistry parentRegistry)
   {
      this.robotTimestamp = controllerToolbox.getRuntimeEnvironment().getRobotTimestamp();
      this.controlDT = controllerToolbox.getRuntimeEnvironment().getControlDT();

      QuadrupedReferenceFrames referenceFrames = controllerToolbox.getReferenceFrames();
      supportFrame = referenceFrames.getCenterOfFeetZUpFrameAveragingLowestZHeightsAcrossEnds();
      bodyFrame = referenceFrames.getBodyFrame();
      centerOfMassFrame = referenceFrames.getCenterOfMassFrame();

      centerOfMassJacobian = controllerToolbox.getCenterOfMassJacobian();

      PIDGains defaultComPositionGains = new PIDGains();
      defaultComPositionGains.setKp(50.0);
      defaultComPositionGains.setKd(5.0);
      comPositionGainsParameter = new ParameterizedPIDGains("_comHeight", defaultComPositionGains, registry);

      linearMomentumZPDController = new PIDController("linearMomentumZPDController", registry);

      FramePoint3D initialPosition = new FramePoint3D(supportFrame, 0.0, 0.0, physicalProperties.getNominalCoMHeight());
      FrameVector3D initialVelocity = new FrameVector3D(supportFrame);
      centerOfMassHeightTrajectory = new MultipleWaypointsPositionTrajectoryGenerator("centerOfMassHeight", supportFrame, registry);
      centerOfMassHeightTrajectory.clear();
      centerOfMassHeightTrajectory.appendWaypoint(0.0, initialPosition, initialVelocity);
      centerOfMassHeightTrajectory.initialize();

      currentHeightInWorld = new YoDouble("currentHeightInWorld", registry);
      currentVelocityInWorld = new YoDouble("currentVelocityInWorld", registry);
      desiredHeightInWorld = new YoDouble("desiredHeightInWorld", registry);
      desiredVelocityInWorld = new YoDouble("desiredVelocityInWorld", registry);

      parentRegistry.addChild(registry);
   }

   public void handleBodyHeightCommand(QuadrupedBodyHeightCommand command)
   {
      controlBodyHeight.set(command.controlBodyHeight());

      double currentTime = robotTimestamp.getDoubleValue();
      double timeShift = command.isExpressedInAbsoluteTime() ? 0.0 : currentTime;
      EuclideanTrajectoryControllerCommand euclideanTrajectory = command.getEuclideanTrajectory();
      euclideanTrajectory.getTrajectoryPointList().addTimeOffset(timeShift);

      if (euclideanTrajectory.getTrajectoryPoint(0).getTime() > 1.0e-5 + currentTime)
      {
         centerOfMassHeightTrajectory.getPosition(desiredPosition);
         desiredVelocity.setToZero(worldFrame);

         desiredPosition.changeFrame(supportFrame);
         desiredVelocity.changeFrame(supportFrame);

         centerOfMassHeightTrajectory.clear();
         centerOfMassHeightTrajectory.appendWaypoint(currentTime, desiredPosition, desiredVelocity);
      }
      else
      {
         centerOfMassHeightTrajectory.clear();
      }

      for (int i = 0; i < euclideanTrajectory.getNumberOfTrajectoryPoints(); i++)
      {
         FrameEuclideanTrajectoryPoint trajectoryPoint = euclideanTrajectory.getTrajectoryPoint(i);
         trajectoryPoint.changeFrame(supportFrame);

         centerOfMassHeightTrajectory.appendWaypoint(trajectoryPoint);
      }

      centerOfMassHeightTrajectory.initialize();
   }

   public void update()
   {
      centerOfMassHeightTrajectory.compute(robotTimestamp.getDoubleValue());
      centerOfMassHeightTrajectory.getLinearData(desiredPosition, desiredVelocity, desiredAcceleration);

      desiredPosition.changeFrame(worldFrame);
      desiredVelocity.changeFrame(worldFrame);
      desiredHeightInWorld.set(desiredPosition.getZ());
      desiredVelocityInWorld.set(desiredVelocity.getZ());
   }

   public double getDesiredHeight(ReferenceFrame referenceFrame)
   {
      desiredPosition.changeFrame(referenceFrame);
      return desiredPosition.getZ();
   }

   public double computeDesiredCenterOfMassHeightAcceleration()
   {
      if (controlBodyHeight.getBooleanValue())
      {
         currentPosition.setToZero(bodyFrame);
         bodyFrame.getTwistOfFrame().getLinearVelocityOfPointFixedInBodyFrame(currentVelocity, currentPosition);
      }
      else
      {
         currentPosition.setToZero(centerOfMassFrame);
         centerOfMassJacobian.getCenterOfMassVelocity(currentVelocity);
      }

      currentPosition.changeFrame(worldFrame);
      currentVelocity.changeFrame(worldFrame);

      currentHeightInWorld.set(currentPosition.getZ());
      currentVelocityInWorld.set(currentVelocity.getZ());

      linearMomentumZPDController.setGains(comPositionGainsParameter);
      return linearMomentumZPDController.compute(currentPosition.getZ(), desiredPosition.getZ(), currentVelocity.getZ(), desiredVelocity.getZ(), controlDT);
   }

}
