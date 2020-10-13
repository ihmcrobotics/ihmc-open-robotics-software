package us.ihmc.robotics.math.trajectories;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class BlendedOrientationTrajectoryGenerator implements OrientationTrajectoryGenerator
{
   private final OrientationTrajectoryGenerator trajectory;
   private final HermiteCurveBasedOrientationTrajectoryGenerator initialConstraintTrajectory;
   private final HermiteCurveBasedOrientationTrajectoryGenerator finalConstraintTrajectory;
   private final ReferenceFrame trajectoryFrame;
   private final YoDouble initialBlendStartTime;
   private final YoDouble initialBlendEndTime;
   private final YoDouble finalBlendStartTime;
   private final YoDouble finalBlendEndTime;

   private final Quaternion initialConstraintOrientationError = new Quaternion();
   private final Vector3D initialConstraintAngularVelocityError = new Vector3D();
   private final Quaternion finalConstraintOrientationError = new Quaternion();
   private final Vector3D finalConstraintAngularVelocityError = new Vector3D();

   private final FrameQuaternion initialConstraintOrientationOffset = new FrameQuaternion();
   private final FrameVector3D initialConstraintAngularVelocityOffset = new FrameVector3D();
   private final FrameVector3D initialConstraintAngularAccelerationOffset = new FrameVector3D();
   private final FrameQuaternion finalConstraintOrientationOffset = new FrameQuaternion();
   private final FrameVector3D finalConstraintAngularVelocityOffset = new FrameVector3D();
   private final FrameVector3D finalConstraintAngularAccelerationOffset = new FrameVector3D();

   private final DMatrixRMaj rotationMatrix = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj rotationMatrixDerivative = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj relativeConstraintAngularVelocityOffset = new DMatrixRMaj(3, 1);
   private final DMatrixRMaj relativeConstraintAngularVelocityOffsetSkewSymmetricMatrix = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj constraintAngularAccelerationOffsetDueToVelocity = new DMatrixRMaj(3, 1);

   private final FrameQuaternion orientation = new FrameQuaternion();
   private final FrameVector3D angularVelocity = new FrameVector3D();
   private final FrameVector3D angularAcceleration = new FrameVector3D();
   private final FrameQuaternion tempOrientation = new FrameQuaternion();
   private final FrameVector3D tempAngularVelocity = new FrameVector3D();
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();

   public BlendedOrientationTrajectoryGenerator(String prefix, OrientationTrajectoryGenerator trajectory, ReferenceFrame trajectoryFrame,
         YoRegistry parentRegistry)
   {
      this.trajectory = trajectory;
      this.trajectoryFrame = trajectoryFrame;
      this.initialConstraintTrajectory = new HermiteCurveBasedOrientationTrajectoryGenerator(prefix + "InitialConstraintTrajectory", trajectoryFrame, parentRegistry);
      this.finalConstraintTrajectory = new HermiteCurveBasedOrientationTrajectoryGenerator(prefix + "FinalConstraintTrajectory", trajectoryFrame, parentRegistry);
      this.initialBlendStartTime = new YoDouble(prefix + "InitialBlendStartTime", parentRegistry);
      this.initialBlendEndTime = new YoDouble(prefix + "InitialBlendEndTime", parentRegistry);
      this.finalBlendStartTime = new YoDouble(prefix + "FinalBlendStartTime", parentRegistry);
      this.finalBlendEndTime = new YoDouble(prefix + "FinalBlendEndTime", parentRegistry);
      this.initialConstraintOrientationOffset.changeFrame(trajectoryFrame);
      this.initialConstraintAngularVelocityOffset.changeFrame(trajectoryFrame);
      this.initialConstraintAngularAccelerationOffset.changeFrame(trajectoryFrame);
      this.finalConstraintOrientationOffset.changeFrame(trajectoryFrame);
      this.finalConstraintAngularVelocityOffset.changeFrame(trajectoryFrame);
      this.finalConstraintAngularAccelerationOffset.changeFrame(trajectoryFrame);
      this.orientation.changeFrame(trajectoryFrame);
      this.angularVelocity.changeFrame(trajectoryFrame);
      this.angularAcceleration.changeFrame(trajectoryFrame);
      this.tempOrientation.changeFrame(trajectoryFrame);
      this.tempAngularVelocity.changeFrame(trajectoryFrame);
      clear();
   }

   public void clear()
   {
      clearInitialConstraint();
      clearFinalConstraint();
   }

   public void clearInitialConstraint()
   {
      initialConstraintOrientationError.setToZero();
      initialConstraintAngularVelocityError.setToZero();
      tempOrientation.set(initialConstraintOrientationError);
      tempAngularVelocity.set(initialConstraintAngularVelocityError);
      initialConstraintTrajectory.setTrajectoryTime(0.0);
      initialConstraintTrajectory.setInitialConditions(tempOrientation, tempAngularVelocity);
      initialConstraintTrajectory.setFinalConditions(tempOrientation, tempAngularVelocity);
      initialConstraintTrajectory.initialize();
   }

   public void clearFinalConstraint()
   {
      finalConstraintOrientationError.setToZero();
      finalConstraintAngularVelocityError.setToZero();
      tempOrientation.set(finalConstraintOrientationError);
      tempAngularVelocity.set(finalConstraintAngularVelocityError);
      finalConstraintTrajectory.setTrajectoryTime(0.0);
      finalConstraintTrajectory.setInitialConditions(tempOrientation, tempAngularVelocity);
      finalConstraintTrajectory.setFinalConditions(tempOrientation, tempAngularVelocity);
      finalConstraintTrajectory.initialize();
   }

   public void blendInitialConstraint(FrameQuaternionReadOnly initialPose, double initialTime, double blendDuration)
   {
      clearInitialConstraint();
      computeInitialConstraintError(initialPose, initialTime);
      computeInitialConstraintTrajectory(initialTime, blendDuration);
   }

   public void blendInitialConstraint(FrameQuaternionReadOnly initialPose, FrameVector3D initialAngularVelocity, double initialTime, double blendDuration)
   {
      clearInitialConstraint();
      computeInitialConstraintError(initialPose, initialAngularVelocity, initialTime);
      computeInitialConstraintTrajectory(initialTime, blendDuration);
   }

   public void blendFinalConstraint(FrameQuaternionReadOnly finalOrientation, double finalTime, double blendDuration)
   {
      clearFinalConstraint();
      computeFinalConstraintError(finalOrientation, finalTime);
      computeFinalConstraintTrajectory(finalTime, blendDuration);
   }

   public void blendFinalConstraint(FrameQuaternionReadOnly finalOrientation, FrameVector3DReadOnly finalAngularVelocity, double finalTime, double blendDuration)
   {
      clearFinalConstraint();
      computeFinalConstraintError(finalOrientation, finalAngularVelocity, finalTime);
      computeFinalConstraintTrajectory(finalTime, blendDuration);
   }

   @Override
   public void getOrientation(FrameQuaternion orientationToPack)
   {
      orientationToPack.setIncludingFrame(orientation);
   }

   @Override
   public void getAngularVelocity(FrameVector3D angularVelocityToPack)
   {
      angularVelocityToPack.setIncludingFrame(angularVelocity);
   }

   @Override
   public void getAngularAcceleration(FrameVector3D angularAccelerationToPack)
   {
      angularAccelerationToPack.setIncludingFrame(angularAcceleration);
   }

   @Override
   public void initialize()
   {
      trajectory.initialize();
   }

   @Override
   public void compute(double time)
   {
      trajectory.compute(time);
      trajectory.getOrientation(orientation);
      trajectory.getAngularVelocity(angularVelocity);
      trajectory.getAngularAcceleration(angularAcceleration);
      orientation.changeFrame(trajectoryFrame);
      angularVelocity.changeFrame(trajectoryFrame);
      angularAcceleration.changeFrame(trajectoryFrame);

      tempTransform.getRotation().set(orientation);
      tempTransform.getRotation().get(rotationMatrix);
      MatrixTools.vectorToSkewSymmetricMatrix(relativeConstraintAngularVelocityOffsetSkewSymmetricMatrix, angularVelocity);
      CommonOps_DDRM.mult(relativeConstraintAngularVelocityOffsetSkewSymmetricMatrix, rotationMatrix, rotationMatrixDerivative);

      computeInitialConstraintOffset(time, rotationMatrix, rotationMatrixDerivative);
      orientation.multiply(initialConstraintOrientationOffset);
      angularVelocity.add(initialConstraintAngularVelocityOffset);
      angularAcceleration.add(initialConstraintAngularAccelerationOffset);

      computeFinalConstraintOffset(time, rotationMatrix, rotationMatrixDerivative);
      orientation.multiply(finalConstraintOrientationOffset);
      angularVelocity.add(finalConstraintAngularVelocityOffset);
      angularAcceleration.add(finalConstraintAngularAccelerationOffset);
   }

   @Override
   public boolean isDone()
   {
      return trajectory.isDone();
   }

   private void computeInitialConstraintError(FrameQuaternionReadOnly initialOrientation, double initialTime)
   {
      trajectory.compute(initialTime);
      trajectoryFrame.checkReferenceFrameMatch(initialOrientation.getReferenceFrame());

      trajectory.getOrientation(tempOrientation);
      tempOrientation.changeFrame(trajectoryFrame);
      initialConstraintOrientationError.difference(tempOrientation, initialOrientation);
   }

   private void computeInitialConstraintError(FrameQuaternionReadOnly initialOrientation, FrameVector3DReadOnly initialAngularVelocity, double initialTime)
   {
      computeInitialConstraintError(initialOrientation, initialTime);
      trajectoryFrame.checkReferenceFrameMatch(initialAngularVelocity.getReferenceFrame());

      trajectory.getAngularVelocity(tempAngularVelocity);
      tempAngularVelocity.changeFrame(trajectoryFrame);
      initialConstraintAngularVelocityError.set(initialAngularVelocity);
      initialConstraintAngularVelocityError.sub(tempAngularVelocity);
   }

   private void computeFinalConstraintError(FrameQuaternionReadOnly finalOrientation, double finalTime)
   {
      trajectory.compute(finalTime);
      trajectoryFrame.checkReferenceFrameMatch(finalOrientation.getReferenceFrame());

      trajectory.getOrientation(tempOrientation);
      tempOrientation.changeFrame(trajectoryFrame);
      finalConstraintOrientationError.difference(tempOrientation, finalOrientation);
   }

   private void computeFinalConstraintError(FrameQuaternionReadOnly finalOrientation, FrameVector3DReadOnly finalAngularVelocity, double finalTime)
   {
      computeFinalConstraintError(finalOrientation, finalTime);
      trajectoryFrame.checkReferenceFrameMatch(finalAngularVelocity.getReferenceFrame());

      trajectory.getAngularVelocity(tempAngularVelocity);
      tempAngularVelocity.changeFrame(trajectoryFrame);
      finalConstraintAngularVelocityError.set(finalAngularVelocity);
      finalConstraintAngularVelocityError.sub(tempAngularVelocity);
   }

   private void computeInitialConstraintTrajectory(double initialTime, double blendDuration)
   {
      initialBlendStartTime.set(initialTime);
      initialBlendEndTime.set(initialTime + blendDuration);
      initialConstraintTrajectory.setTrajectoryTime(blendDuration);

      trajectory.compute(initialTime);
      trajectory.getOrientation(tempOrientation);
      tempTransform.getRotation().set(tempOrientation);

      tempOrientation.set(initialConstraintOrientationError);
      tempAngularVelocity.set(initialConstraintAngularVelocityError);
      tempTransform.inverseTransform(tempAngularVelocity);
      initialConstraintTrajectory.setInitialConditions(tempOrientation, tempAngularVelocity);

      tempOrientation.setToZero();
      tempAngularVelocity.setToZero();
      initialConstraintTrajectory.setFinalConditions(tempOrientation, tempAngularVelocity);
      initialConstraintTrajectory.initialize();
   }

   private void computeFinalConstraintTrajectory(double finalTime, double blendDuration)
   {
      finalBlendStartTime.set(finalTime - blendDuration);
      finalBlendEndTime.set(finalTime);
      finalConstraintTrajectory.setTrajectoryTime(blendDuration);

      trajectory.compute(finalTime);
      trajectory.getOrientation(tempOrientation);
      tempTransform.getRotation().set(tempOrientation);

      tempOrientation.set(finalConstraintOrientationError);
      tempAngularVelocity.set(finalConstraintAngularVelocityError);
      tempTransform.inverseTransform(tempAngularVelocity);
      finalConstraintTrajectory.setFinalConditions(tempOrientation, tempAngularVelocity);

      tempOrientation.setToZero();
      tempAngularVelocity.setToZero();
      finalConstraintTrajectory.setInitialConditions(tempOrientation, tempAngularVelocity);
      finalConstraintTrajectory.initialize();
   }

   private void computeInitialConstraintOffset(double time, DMatrixRMaj rotationMatrix, DMatrixRMaj rotationMatrixDerivative)
   {
      double startTime = initialBlendStartTime.getDoubleValue();
      initialConstraintTrajectory.compute(time - startTime);
      initialConstraintTrajectory.getOrientation(initialConstraintOrientationOffset);
      initialConstraintTrajectory.getAngularVelocity(initialConstraintAngularVelocityOffset);
      initialConstraintTrajectory.getAngularAcceleration(initialConstraintAngularAccelerationOffset);

      tempTransform.getRotation().set(rotationMatrix);
      tempTransform.getTranslation().set(0.0, 0.0, 0.0);
      initialConstraintOrientationOffset.changeFrame(trajectoryFrame);

      initialConstraintAngularVelocityOffset.changeFrame(trajectoryFrame);
      initialConstraintAngularVelocityOffset.get(relativeConstraintAngularVelocityOffset);
      initialConstraintAngularVelocityOffset.applyTransform(tempTransform);

      CommonOps_DDRM.mult(rotationMatrixDerivative, relativeConstraintAngularVelocityOffset, constraintAngularAccelerationOffsetDueToVelocity);
      initialConstraintAngularAccelerationOffset.changeFrame(trajectoryFrame);
      initialConstraintAngularAccelerationOffset.applyTransform(tempTransform);
      initialConstraintAngularAccelerationOffset.addX(constraintAngularAccelerationOffsetDueToVelocity.get(0, 0));
      initialConstraintAngularAccelerationOffset.addY(constraintAngularAccelerationOffsetDueToVelocity.get(1, 0));
      initialConstraintAngularAccelerationOffset.addZ(constraintAngularAccelerationOffsetDueToVelocity.get(2, 0));
   }

   private void computeFinalConstraintOffset(double time, DMatrixRMaj rotationMatrix, DMatrixRMaj rotationMatrixDerivative)
   {
      double startTime = finalBlendStartTime.getDoubleValue();
      finalConstraintTrajectory.compute(time - startTime);
      finalConstraintTrajectory.getOrientation(finalConstraintOrientationOffset);
      finalConstraintTrajectory.getAngularVelocity(finalConstraintAngularVelocityOffset);
      finalConstraintTrajectory.getAngularAcceleration(finalConstraintAngularAccelerationOffset);

      tempTransform.getRotation().set(rotationMatrix);
      tempTransform.getTranslation().set(0.0, 0.0, 0.0);
      finalConstraintOrientationOffset.changeFrame(trajectoryFrame);

      finalConstraintAngularVelocityOffset.changeFrame(trajectoryFrame);
      finalConstraintAngularVelocityOffset.get(relativeConstraintAngularVelocityOffset);
      finalConstraintAngularVelocityOffset.applyTransform(tempTransform);

      CommonOps_DDRM.mult(rotationMatrixDerivative, relativeConstraintAngularVelocityOffset, constraintAngularAccelerationOffsetDueToVelocity);
      finalConstraintAngularAccelerationOffset.changeFrame(trajectoryFrame);
      finalConstraintAngularAccelerationOffset.applyTransform(tempTransform);
      finalConstraintAngularAccelerationOffset.addX(constraintAngularAccelerationOffsetDueToVelocity.get(0, 0));
      finalConstraintAngularAccelerationOffset.addY(constraintAngularAccelerationOffsetDueToVelocity.get(1, 0));
      finalConstraintAngularAccelerationOffset.addZ(constraintAngularAccelerationOffsetDueToVelocity.get(2, 0));
   }

   public HermiteCurveBasedOrientationTrajectoryGenerator getInitialConstraintTrajectory()
   {
      return initialConstraintTrajectory;
   }
}
