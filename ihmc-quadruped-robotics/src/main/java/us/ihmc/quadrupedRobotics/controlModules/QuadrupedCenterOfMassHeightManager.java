package us.ihmc.quadrupedRobotics.controlModules;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.EuclideanTrajectoryControllerCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.QuadrupedBodyHeightCommand;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControllerToolbox;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.model.QuadrupedPhysicalProperties;
import us.ihmc.robotics.controllers.PIDController;
import us.ihmc.robotics.controllers.pidGains.implementations.PIDGains;
import us.ihmc.robotics.controllers.pidGains.implementations.ParameterizedPIDGains;
import us.ihmc.robotics.math.trajectories.waypoints.FrameEuclideanTrajectoryPoint;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsPositionTrajectoryGenerator;
import us.ihmc.robotics.screwTheory.MovingReferenceFrame;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class QuadrupedCenterOfMassHeightManager
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final DoubleParameter initializationDuration = new DoubleParameter("heightInitializationDuration", registry, 0.5);

   private final YoDouble controllerTime;

   private final YoBoolean controlBodyHeight = new YoBoolean("controlBodyHeight", registry);
   private final YoBoolean heightCommandHasBeenReceived = new YoBoolean("heightCommandHasBeenReceived", registry);

   private final MovingReferenceFrame bodyFrame;
   private final ReferenceFrame centerOfMassFrame;

   private final QuadrupedControllerToolbox controllerToolbox;

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

   private final FramePoint3D nominalPosition;
   private final FrameVector3D nominalVelocity;

   private final double controlDT;

   public QuadrupedCenterOfMassHeightManager(QuadrupedControllerToolbox controllerToolbox, QuadrupedPhysicalProperties physicalProperties,
                                             YoVariableRegistry parentRegistry)
   {
      this.controllerToolbox = controllerToolbox;
      this.controllerTime = controllerToolbox.getRuntimeEnvironment().getRobotTimestamp();
      this.controlDT = controllerToolbox.getRuntimeEnvironment().getControlDT();

      QuadrupedReferenceFrames referenceFrames = controllerToolbox.getReferenceFrames();
      supportFrame = referenceFrames.getCenterOfFeetZUpFrameAveragingLowestZHeightsAcrossEnds();
      bodyFrame = referenceFrames.getBodyFrame();
      centerOfMassFrame = referenceFrames.getCenterOfMassFrame();

      PIDGains defaultComPositionGains = new PIDGains();
      defaultComPositionGains.setKp(50.0);
      defaultComPositionGains.setKd(5.0);
      comPositionGainsParameter = new ParameterizedPIDGains("_comHeight", defaultComPositionGains, registry);
      linearMomentumZPDController = new PIDController("linearMomentumZPDController", registry);

      controlBodyHeight.set(true);
      heightCommandHasBeenReceived.set(false);

      nominalPosition = new FramePoint3D(supportFrame, 0.0, 0.0, physicalProperties.getNominalCoMHeight());
      nominalVelocity = new FrameVector3D(supportFrame);

      centerOfMassHeightTrajectory = new MultipleWaypointsPositionTrajectoryGenerator("centerOfMassHeight", supportFrame, registry);

      currentHeightInWorld = new YoDouble("currentHeightInWorld", registry);
      currentVelocityInWorld = new YoDouble("currentVelocityInWorld", registry);
      desiredHeightInWorld = new YoDouble("desiredHeightInWorld", registry);
      desiredVelocityInWorld = new YoDouble("desiredVelocityInWorld", registry);

      parentRegistry.addChild(registry);
   }

   public void handleBodyHeightCommand(QuadrupedBodyHeightCommand command)
   {
      controlBodyHeight.set(command.controlBodyHeight());

      double currentTime = controllerTime.getDoubleValue();
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
      heightCommandHasBeenReceived.set(true);
   }

   public void initialize()
   {
      if(!heightCommandHasBeenReceived.getBooleanValue())
      {
         computeCurrentState();

         currentPosition.changeFrame(supportFrame);
         currentVelocity.changeFrame(supportFrame);

         double startTime = controllerTime.getDoubleValue();
         double endTime = startTime + initializationDuration.getValue();
         centerOfMassHeightTrajectory.clear();
         centerOfMassHeightTrajectory.appendWaypoint(startTime, currentPosition, currentVelocity);
         centerOfMassHeightTrajectory.appendWaypoint(endTime, nominalPosition, nominalVelocity);
         centerOfMassHeightTrajectory.initialize();
      }
   }

   public void update()
   {
      centerOfMassHeightTrajectory.compute(controllerTime.getDoubleValue());
      centerOfMassHeightTrajectory.getLinearData(desiredPosition, desiredVelocity, desiredAcceleration);

      desiredPosition.changeFrame(worldFrame);
      desiredVelocity.changeFrame(worldFrame);
      desiredHeightInWorld.set(desiredPosition.getZ());
      desiredVelocityInWorld.set(desiredVelocity.getZ());

      computeCurrentState();
   }

   public double getDesiredHeight(ReferenceFrame referenceFrame)
   {
      desiredPosition.changeFrame(referenceFrame);
      return desiredPosition.getZ();
   }

   public double computeDesiredCenterOfMassHeightAcceleration()
   {
      linearMomentumZPDController.setGains(comPositionGainsParameter);
      return linearMomentumZPDController
            .compute(currentHeightInWorld.getDoubleValue(), desiredHeightInWorld.getDoubleValue(), currentVelocityInWorld.getDoubleValue(),
                     desiredVelocityInWorld.getDoubleValue(), controlDT);
   }

   private void computeCurrentState()
   {
      if (controlBodyHeight.getBooleanValue())
      {
         currentPosition.setToZero(bodyFrame);
         bodyFrame.getTwistOfFrame().getLinearVelocityOfPointFixedInBodyFrame(currentVelocity, currentPosition);
      }
      else
      {
         currentPosition.setToZero(centerOfMassFrame);
         currentVelocity.setIncludingFrame(controllerToolbox.getCoMVelocityEstimate());
      }

      currentPosition.changeFrame(worldFrame);
      currentVelocity.changeFrame(worldFrame);

      currentHeightInWorld.set(currentPosition.getZ());
      currentVelocityInWorld.set(currentVelocity.getZ());
   }
}
