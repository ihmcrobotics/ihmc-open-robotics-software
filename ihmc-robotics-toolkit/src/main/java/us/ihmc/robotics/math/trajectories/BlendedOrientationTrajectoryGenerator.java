package us.ihmc.robotics.math.trajectories;

import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class BlendedOrientationTrajectoryGenerator implements OrientationTrajectoryGenerator
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

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

   private final FrameQuaternion orientation = new FrameQuaternion();
   private final FrameVector3D angularVelocity = new FrameVector3D();
   private final FrameVector3D angularAcceleration = new FrameVector3D();
   private final FrameQuaternion tempOrientation = new FrameQuaternion();
   private final FrameVector3D tempAngularVelocity = new FrameVector3D();
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();

   public BlendedOrientationTrajectoryGenerator(String prefix, OrientationTrajectoryGenerator trajectory, ReferenceFrame trajectoryFrame,
         YoVariableRegistry parentRegistry)
   {
      this.trajectory = trajectory;
      this.trajectoryFrame = trajectoryFrame;
      this.initialConstraintTrajectory = new HermiteCurveBasedOrientationTrajectoryGenerator(prefix + "InitialConstraintTrajectory", trajectoryFrame, registry);
      this.finalConstraintTrajectory = new HermiteCurveBasedOrientationTrajectoryGenerator(prefix + "FinalConstraintTrajectory", trajectoryFrame, registry);
      this.initialBlendStartTime = new YoDouble(prefix + "InitialBlendStartTime", registry);
      this.initialBlendEndTime = new YoDouble(prefix + "InitialBlendEndTime", registry);
      this.finalBlendStartTime = new YoDouble(prefix + "FinalBlendStartTime", registry);
      this.finalBlendEndTime = new YoDouble(prefix + "FinalBlendEndTime", registry);
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
      parentRegistry.addChild(registry);
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

      computeInitialConstraintOffset(time);
      tempTransform.setRotation(orientation);
      initialConstraintOrientationOffset.changeFrame(trajectoryFrame);
      initialConstraintAngularVelocityOffset.changeFrame(trajectoryFrame);
      initialConstraintAngularVelocityOffset.applyTransform(tempTransform);
      initialConstraintAngularAccelerationOffset.changeFrame(trajectoryFrame);
      initialConstraintAngularAccelerationOffset.applyTransform(tempTransform);
      orientation.multiply(initialConstraintOrientationOffset);
      angularVelocity.add(initialConstraintAngularVelocityOffset);
      angularAcceleration.add(initialConstraintAngularAccelerationOffset);

      computeFinalConstraintOffset(time);
      tempTransform.setRotation(orientation);
      finalConstraintOrientationOffset.changeFrame(trajectoryFrame);
      finalConstraintAngularVelocityOffset.changeFrame(trajectoryFrame);
      finalConstraintAngularVelocityOffset.applyTransform(tempTransform);
      finalConstraintAngularAccelerationOffset.changeFrame(trajectoryFrame);
      finalConstraintAngularAccelerationOffset.applyTransform(tempTransform);
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
      tempTransform.setRotation(tempOrientation);

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
      tempTransform.setRotation(tempOrientation);

      tempOrientation.set(finalConstraintOrientationError);
      tempAngularVelocity.set(finalConstraintAngularVelocityError);
      tempTransform.inverseTransform(tempAngularVelocity);
      finalConstraintTrajectory.setFinalConditions(tempOrientation, tempAngularVelocity);

      tempOrientation.setToZero();
      tempAngularVelocity.setToZero();
      finalConstraintTrajectory.setInitialConditions(tempOrientation, tempAngularVelocity);
      finalConstraintTrajectory.initialize();
   }

   private void computeInitialConstraintOffset(double time)
   {
      double startTime = initialBlendStartTime.getDoubleValue();
      initialConstraintTrajectory.compute(time - startTime);
      initialConstraintTrajectory.getOrientation(initialConstraintOrientationOffset);
      initialConstraintTrajectory.getAngularVelocity(initialConstraintAngularVelocityOffset);
      initialConstraintTrajectory.getAngularAcceleration(initialConstraintAngularAccelerationOffset);
   }

   private void computeFinalConstraintOffset(double time)
   {
      double startTime = finalBlendStartTime.getDoubleValue();
      finalConstraintTrajectory.compute(time - startTime);
      finalConstraintTrajectory.getOrientation(finalConstraintOrientationOffset);
      finalConstraintTrajectory.getAngularVelocity(finalConstraintAngularVelocityOffset);
      finalConstraintTrajectory.getAngularAcceleration(finalConstraintAngularAccelerationOffset);
   }
}
