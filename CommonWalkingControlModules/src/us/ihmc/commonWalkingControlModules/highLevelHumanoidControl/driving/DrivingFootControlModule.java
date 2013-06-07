package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.driving;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.AxisAngleOrientationController;
import com.yobotics.simulationconstructionset.util.EuclideanPositionController;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;
import com.yobotics.simulationconstructionset.util.trajectory.*;
import org.ejml.data.DenseMatrix64F;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.calculators.GainCalculator;
import us.ihmc.commonWalkingControlModules.desiredFootStep.DesiredFootstepCalculatorTools;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.TaskspaceConstraintData;
import us.ihmc.commonWalkingControlModules.trajectories.StraightLinePositionTrajectoryGenerator;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.*;

import javax.media.j3d.Transform3D;
import java.util.List;

/**
 * @author twan
 *         Date: 6/6/13
 */
public class DrivingFootControlModule
{
   private final double AVERAGE_VELOCITY = 0.3;

   private final YoVariableRegistry registry;
   private final GeometricJacobian footJacobian;
   private final FramePoint toePoint;
   private final EuclideanPositionController toePointPositionController;
   private final MomentumBasedController momentumBasedController;

   private final FramePoint desiredPosition = new FramePoint();
   private final FrameVector desiredVelocity = new FrameVector();
   private final FrameVector feedForward = new FrameVector();

   private final FrameVector currentVelocity = new FrameVector();
   private final FrameVector currentAngularVelocity = new FrameVector();

   private final PositionTrajectoryGenerator positionTrajectoryGenerator;
   private final YoFramePoint targetPosition;

   private final DoubleYoVariable trajectoryInitializationTime;
   private final DoubleYoVariable time;

   private final AxisAngleOrientationController orientationController;
   private final DenseMatrix64F footOrientationSelectionMatrix;
   private final SpatialAccelerationVector footRollSpatialAccelerationVector;
   private final TaskspaceConstraintData footOrientationTaskspaceConstraintData = new TaskspaceConstraintData();

   private final FrameOrientation desiredOrientation = new FrameOrientation(ReferenceFrame.getWorldFrame());
   private final FrameVector desiredAngularVelocity = new FrameVector();
   private final FrameVector feedForwardAngularAcceleration = new FrameVector();

   private final TwistCalculator twistCalculator;
   private final Twist currentTwist = new Twist();
   private final FramePoint toePointInBase = new FramePoint();
   private final ReferenceFrame toePointFrame;

   private final DrivingReferenceFrames drivingReferenceFrames;

   private final RigidBody foot;
   private final RigidBody elevator;

   private final DenseMatrix64F nullspaceMultipliers = new DenseMatrix64F(0, 1);

   public DrivingFootControlModule(RigidBody elevator, ContactablePlaneBody contactablePlaneFoot, MomentumBasedController momentumBasedController,
                                   DrivingReferenceFrames drivingReferenceFrames, DoubleYoVariable yoTime, TwistCalculator twistCalculator, YoVariableRegistry parentRegistry)
   {
      this.foot = contactablePlaneFoot.getRigidBody();
      this.elevator = elevator;
      registry = new YoVariableRegistry(foot.getName() + getClass().getSimpleName());
      footJacobian = new GeometricJacobian(elevator, foot, elevator.getBodyFixedFrame());
      toePoint = getCenterToePoint(contactablePlaneFoot);
      String toePointName = foot.getName() + "ToePoint";
      Transform3D transform = new Transform3D();
      transform.set(toePoint.getVectorCopy());
      toePointFrame = ReferenceFrame.constructBodyFrameWithUnchangingTransformToParent(toePointName, toePoint.getReferenceFrame(), transform);
      this.drivingReferenceFrames = drivingReferenceFrames;
      toePointPositionController = new EuclideanPositionController(toePointName, toePointFrame, registry);
      this.momentumBasedController = momentumBasedController;
      this.time = yoTime;
      trajectoryInitializationTime = new DoubleYoVariable(toePointName + "InitializationTime", registry);

      ReferenceFrame vehicleFrame = drivingReferenceFrames.getVehicleFrame();
      targetPosition = new YoFramePoint(toePointName + "Target", vehicleFrame, registry);

      PositionProvider initialPositionProvider = new ConstantPositionProvider(toePoint);
      PositionProvider finalPositionProvider = new YoPositionProvider(targetPosition);
      DoubleProvider trajectoryTimeProvider = new AverageVelocityTrajectoryTimeProvider(initialPositionProvider, finalPositionProvider, AVERAGE_VELOCITY, 1e-3);
      this.positionTrajectoryGenerator = new StraightLinePositionTrajectoryGenerator(toePointName + "Trajectory", vehicleFrame, trajectoryTimeProvider,
              initialPositionProvider, finalPositionProvider, registry);

      double kP = 100.0;
      double dampingRatio = 1.0;
      double kD = GainCalculator.computeDerivativeGain(kP, dampingRatio);
      toePointPositionController.setProportionalGains(kP, kP, kP);
      toePointPositionController.setDerivativeGains(kD, kD, kD);


      orientationController = new AxisAngleOrientationController(foot.getName() + "PD", foot.getBodyFixedFrame(), registry);
      double kPOrientation = 15.0;
      double kDOrientation = GainCalculator.computeDerivativeGain(kPOrientation, dampingRatio);
      orientationController.setProportionalGains(kPOrientation, kPOrientation, kPOrientation);
      orientationController.setDerivativeGains(kDOrientation, kDOrientation, kDOrientation);

      FrameVector rollAxis = new FrameVector(contactablePlaneFoot.getPlaneFrame(), 1.0, 0.0, 0.0);
      FrameVector yawAxis = new FrameVector(contactablePlaneFoot.getPlaneFrame(), 0.0, 0.0, 1.0);
      rollAxis.changeFrame(foot.getBodyFixedFrame());
      yawAxis.changeFrame(foot.getBodyFixedFrame());

      footOrientationSelectionMatrix = new DenseMatrix64F(2, SpatialMotionVector.SIZE);
      footOrientationSelectionMatrix.set(0, 0, rollAxis.getX());
      footOrientationSelectionMatrix.set(0, 1, rollAxis.getY());
      footOrientationSelectionMatrix.set(0, 2, rollAxis.getZ());
      footOrientationSelectionMatrix.set(1, 0, yawAxis.getX());
      footOrientationSelectionMatrix.set(1, 1, yawAxis.getY());
      footOrientationSelectionMatrix.set(1, 2, yawAxis.getZ());

      footRollSpatialAccelerationVector = new SpatialAccelerationVector();

      this.twistCalculator = twistCalculator;

      parentRegistry.addChild(registry);
   }

   public void holdPosition()
   {
      targetPosition.set(toePoint.changeFrameCopy(targetPosition.getReferenceFrame()));
      initializeTrajectory();
   }

   public void moveToPositionInGasPedalFrame(double z)
   {
      FramePoint target = new FramePoint(drivingReferenceFrames.getObjectFrame(VehicleObject.GAS_PEDAL), 0.0, 0.0, z);
      target.changeFrame(targetPosition.getReferenceFrame());
      targetPosition.set(target);
      initializeTrajectory();
   }

   public void moveToPositionInBrakePedalFrame(double z)
   {
      FramePoint target = new FramePoint(drivingReferenceFrames.getObjectFrame(VehicleObject.BRAKE_PEDAL), 0.0, 0.0, z);
      target.changeFrame(targetPosition.getReferenceFrame());
      targetPosition.set(target);
      initializeTrajectory();
   }

   public void doControl()
   {
      footJacobian.compute();
      updateCurrentVelocity();
      doToePositionControl();
      doFootOrientationControl();
   }

   private void doToePositionControl()
   {
      positionTrajectoryGenerator.compute(time.getDoubleValue() - trajectoryInitializationTime.getDoubleValue());
      positionTrajectoryGenerator.get(desiredPosition);
      positionTrajectoryGenerator.packVelocity(desiredVelocity);
      positionTrajectoryGenerator.packAcceleration(feedForward);

      FrameVector output = new FrameVector(toePointFrame);
      toePointPositionController.compute(output, desiredPosition, desiredVelocity, currentVelocity, feedForward);
      momentumBasedController.setDesiredPointAcceleration(footJacobian, toePoint, output);
   }

   private void doFootOrientationControl()
   {
      desiredOrientation.set(drivingReferenceFrames.getVehicleFrame());
      desiredAngularVelocity.setToZero(toePointFrame);
      feedForwardAngularAcceleration.setToZero(toePointFrame);

      FrameVector output = new FrameVector(toePointFrame);
      orientationController.compute(output, desiredOrientation, desiredAngularVelocity, currentAngularVelocity, feedForwardAngularAcceleration);

      footRollSpatialAccelerationVector.setToZero(foot.getBodyFixedFrame(), elevator.getBodyFixedFrame(), foot.getBodyFixedFrame());
      footRollSpatialAccelerationVector.setAngularPart(output.getVector());
      footOrientationTaskspaceConstraintData.set(footRollSpatialAccelerationVector, nullspaceMultipliers, footOrientationSelectionMatrix);
      momentumBasedController.setDesiredSpatialAcceleration(footJacobian, footOrientationTaskspaceConstraintData);
   }

   private void updateCurrentVelocity()
   {
      twistCalculator.packRelativeTwist(currentTwist, footJacobian.getBase(), footJacobian.getEndEffector());
      currentTwist.packAngularPart(currentAngularVelocity);
      currentTwist.changeFrame(footJacobian.getBaseFrame());
      toePointInBase.setAndChangeFrame(toePoint);
      toePointInBase.changeFrame(footJacobian.getBaseFrame());
      currentTwist.packVelocityOfPointFixedInBodyFrame(currentVelocity, toePointInBase);
   }

   private void initializeTrajectory()
   {
      positionTrajectoryGenerator.initialize();
      trajectoryInitializationTime.set(time.getDoubleValue());
   }

   private static FramePoint getCenterToePoint(ContactablePlaneBody foot)
   {
      FrameVector forward = new FrameVector(foot.getPlaneFrame(), 1.0, 0.0, 0.0);
      int nToePoints = 2;
      List<FramePoint> toePoints = DesiredFootstepCalculatorTools.computeMaximumPointsInDirection(foot.getContactPoints(), forward, nToePoints);
      FramePoint centerToePoint = FramePoint.average(toePoints);

      return centerToePoint;
   }
}
