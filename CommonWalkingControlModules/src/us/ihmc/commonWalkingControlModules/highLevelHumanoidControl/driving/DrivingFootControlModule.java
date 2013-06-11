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
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.taskExecutor.Task;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.taskExecutor.TaskExecutor;
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
   private final double AVERAGE_VELOCITY = 1.0;

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
   private final DoubleYoVariable footPitch;

   private final DoubleYoVariable trajectoryInitializationTime;
   private final DoubleYoVariable time;

   private final AxisAngleOrientationController orientationController;
   private final DenseMatrix64F footOrientationSelectionMatrix;
   private final DenseMatrix64F footOrientationNullspaceMultipliers = new DenseMatrix64F(0, 1);

   private final SpatialAccelerationVector footRollSpatialAccelerationVector;
   private final TaskspaceConstraintData footOrientationTaskspaceConstraintData = new TaskspaceConstraintData();

   private final FrameOrientation desiredOrientation;
   private final FrameVector desiredAngularVelocity;
   private final FrameVector feedForwardAngularAcceleration;

   private final TwistCalculator twistCalculator;
   private final Twist currentTwist = new Twist();
   private final FramePoint toePointInBase = new FramePoint();
   private final ReferenceFrame toePointFrame;

   private final DrivingReferenceFrames drivingReferenceFrames;

   private final RigidBody foot;
   private final RigidBody elevator;

   private final TaskExecutor taskExecutor = new TaskExecutor();
   private final double pedalY = 0.02;

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
      footPitch = new DoubleYoVariable("footPitch", registry);

      ReferenceFrame vehicleFrame = drivingReferenceFrames.getVehicleFrame();

      desiredOrientation = new FrameOrientation(vehicleFrame);
      desiredAngularVelocity = new FrameVector(vehicleFrame);
      feedForwardAngularAcceleration = new FrameVector(vehicleFrame);

      targetPosition = new YoFramePoint(toePointName + "Target", vehicleFrame, registry);

      PositionProvider initialPositionProvider = new ConstantPositionProvider(toePoint);
      PositionProvider finalPositionProvider = new YoPositionProvider(targetPosition);
      DoubleProvider trajectoryTimeProvider = new AverageVelocityTrajectoryTimeProvider(initialPositionProvider, finalPositionProvider, AVERAGE_VELOCITY, 1e-3);
      this.positionTrajectoryGenerator = new StraightLinePositionTrajectoryGenerator(toePointName + "Trajectory", vehicleFrame, trajectoryTimeProvider,
              initialPositionProvider, finalPositionProvider, registry);

      double kP = 300.0;
      double dampingRatio = 1.0;
      double kD = GainCalculator.computeDerivativeGain(kP, dampingRatio);
      toePointPositionController.setProportionalGains(kP, kP, kP);
      toePointPositionController.setDerivativeGains(kD, kD, kD);


      orientationController = new AxisAngleOrientationController(foot.getName() + "PD", foot.getBodyFixedFrame(), registry);
      double kPOrientationYZ = 100.0;
      double kDOrientationYZ = GainCalculator.computeDerivativeGain(kPOrientationYZ, dampingRatio);

      double kPOrientationX = 200.0;
      double kDOrientationX = GainCalculator.computeDerivativeGain(kPOrientationX, dampingRatio);

      orientationController.setProportionalGains(kPOrientationX, kPOrientationYZ, kPOrientationYZ);
      orientationController.setDerivativeGains(kDOrientationX, kDOrientationYZ, kDOrientationYZ);

      footOrientationSelectionMatrix = new DenseMatrix64F(3, SpatialMotionVector.SIZE);
      footOrientationSelectionMatrix.zero();
      footOrientationSelectionMatrix.set(0, 0, 1.0);
      footOrientationSelectionMatrix.set(1, 1, 1.0);
      footOrientationSelectionMatrix.set(2, 2, 1.0);

      footPitch.set(0.0);

      footRollSpatialAccelerationVector = new SpatialAccelerationVector();

      this.twistCalculator = twistCalculator;

      parentRegistry.addChild(registry);
   }

   public void holdPosition()
   {
      FramePoint target = new FramePoint(toePoint);
      moveToPosition(target);
   }

   public void moveToPositionInGasPedalFrame(double z)
   {
      FramePoint target = new FramePoint(drivingReferenceFrames.getObjectFrame(VehicleObject.GAS_PEDAL), 0.0, pedalY, z);
      moveToPosition(target);
   }

   public void moveToPositionInBrakePedalFrame(double z)
   {
      FramePoint target = new FramePoint(drivingReferenceFrames.getObjectFrame(VehicleObject.BRAKE_PEDAL), 0.0, pedalY, z);
      moveToPosition(target);
   }

   public void doControl()
   {
      taskExecutor.doControl();
      footJacobian.compute();
      updateCurrentVelocity();
      doToePositionControl();
      doFootOrientationControl();
   }

   private void moveToPosition(FramePoint target)
   {
      Task task = new FootControlTask(target);
      taskExecutor.submit(task);
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
      desiredOrientation.setYawPitchRoll(0.0, footPitch.getDoubleValue(), 0.0);
      desiredAngularVelocity.setToZero(toePointFrame);
      feedForwardAngularAcceleration.setToZero(toePointFrame);

      FrameVector output = new FrameVector(toePointFrame);
      orientationController.compute(output, desiredOrientation, desiredAngularVelocity, currentAngularVelocity, feedForwardAngularAcceleration);

      footRollSpatialAccelerationVector.setToZero(foot.getBodyFixedFrame(), elevator.getBodyFixedFrame(), foot.getBodyFixedFrame());
      footRollSpatialAccelerationVector.setAngularPart(output.getVector());
      footOrientationTaskspaceConstraintData.set(footRollSpatialAccelerationVector, footOrientationNullspaceMultipliers, footOrientationSelectionMatrix);
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

   private static FramePoint getCenterToePoint(ContactablePlaneBody foot)
   {
      FrameVector forward = new FrameVector(foot.getPlaneFrame(), 1.0, 0.0, 0.0);
      int nToePoints = 2;
      List<FramePoint> toePoints = DesiredFootstepCalculatorTools.computeMaximumPointsInDirection(foot.getContactPoints(), forward, nToePoints);
      FramePoint centerToePoint = FramePoint.average(toePoints);

      return centerToePoint;
   }

   private class FootControlTask implements Task
   {
      private final FramePoint targetPosition;

      public FootControlTask(FramePoint targetPosition)
      {
         this.targetPosition = targetPosition;
      }

      public void doTransitionIntoAction()
      {
         DrivingFootControlModule.this.targetPosition.set(targetPosition.changeFrameCopy(DrivingFootControlModule.this.targetPosition.getReferenceFrame()));
         positionTrajectoryGenerator.initialize();
         trajectoryInitializationTime.set(time.getDoubleValue());
      }

      public void doAction()
      {
      }

      public void doTransitionOutOfAction()
      {
      }

      public boolean isDone()
      {
         return positionTrajectoryGenerator.isDone();
      }
   }
}
