package us.ihmc.robotics.math.trajectories;

import us.ihmc.euclid.rotationConversion.RotationVectorConversion;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class BlendedOrientationTrajectoryGenerator implements OrientationTrajectoryGenerator
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final OrientationTrajectoryGenerator trajectory;
   private final ReferenceFrame trajectoryFrame;
   private final YoPolynomial[] initialConstraintPolynomial = new YoPolynomial[3];
   private final YoPolynomial[] finalConstraintPolynomial = new YoPolynomial[3];
   private final DoubleYoVariable initialBlendStartTime;
   private final DoubleYoVariable initialBlendEndTime;
   private final DoubleYoVariable finalBlendStartTime;
   private final DoubleYoVariable finalBlendEndTime;

   private final Vector3D initialConstraintOrientationError = new Vector3D();
   private final Vector3D initialConstraintAngularVelocityError = new Vector3D();
   private final Vector3D initialConstraintAngularAccelerationError = new Vector3D();
   private final Vector3D finalConstraintOrientationError = new Vector3D();
   private final Vector3D finalConstraintAngularVelocityError = new Vector3D();
   private final Vector3D finalConstraintAngularAccelerationError = new Vector3D();

   private final Vector3D initialConstraintOrientationOffset = new Vector3D();
   private final Vector3D initialConstraintAngularVelocityOffset = new Vector3D();
   private final Vector3D initialConstraintAngularAccelerationOffset = new Vector3D();
   private final Vector3D finalConstraintOrientationOffset = new Vector3D();
   private final Vector3D finalConstraintAngularVelocityOffset = new Vector3D();
   private final Vector3D finalConstraintAngularAccelerationOffset = new Vector3D();

   private final FrameOrientation orientation = new FrameOrientation();
   private final FrameVector angularVelocity = new FrameVector();
   private final FrameVector angularAcceleration = new FrameVector();

   private final FrameOrientation tempOrientation = new FrameOrientation();
   private final FrameVector tempAngularVelocity = new FrameVector();
   private final FrameVector tempAngularAcceleration = new FrameVector();
   private final Quaternion tempQuaternion = new Quaternion();

   public BlendedOrientationTrajectoryGenerator(String prefix, OrientationTrajectoryGenerator trajectory, ReferenceFrame trajectoryFrame,
         YoVariableRegistry parentRegistry)
   {
      this.trajectory = trajectory;
      this.trajectoryFrame = trajectoryFrame;
      this.initialConstraintPolynomial[0] = new YoPolynomial(prefix + "InitialConstraintPolynomialX", 6, registry);
      this.initialConstraintPolynomial[1] = new YoPolynomial(prefix + "InitialConstraintPolynomialY", 6, registry);
      this.initialConstraintPolynomial[2] = new YoPolynomial(prefix + "InitialConstraintPolynomialZ", 6, registry);
      this.finalConstraintPolynomial[0] = new YoPolynomial(prefix + "FinalConstraintPolynomialX", 6, registry);
      this.finalConstraintPolynomial[1] = new YoPolynomial(prefix + "FinalConstraintPolynomialY", 6, registry);
      this.finalConstraintPolynomial[2] = new YoPolynomial(prefix + "FinalConstraintPolynomialZ", 6, registry);
      this.initialBlendStartTime = new DoubleYoVariable(prefix + "InitialBlendStartTime", registry);
      this.initialBlendEndTime = new DoubleYoVariable(prefix + "InitialBlendEndTime", registry);
      this.finalBlendStartTime = new DoubleYoVariable(prefix + "FinalBlendStartTime", registry);
      this.finalBlendEndTime = new DoubleYoVariable(prefix + "FinalBlendEndTime", registry);
      this.orientation.changeFrame(trajectoryFrame);
      this.angularVelocity.changeFrame(trajectoryFrame);
      this.angularAcceleration.changeFrame(trajectoryFrame);
      this.tempOrientation.changeFrame(trajectoryFrame);
      this.tempAngularVelocity.changeFrame(trajectoryFrame);
      this.tempAngularAcceleration.changeFrame(trajectoryFrame);
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
      for (int i = 0; i < 3; i++)
      {
         initialConstraintOrientationError.setToZero();
         initialConstraintAngularVelocityError.setToZero();
         initialConstraintAngularAccelerationError.setToZero();
         initialConstraintPolynomial[i].setConstant(0.0);
      }
   }

   public void clearFinalConstraint()
   {
      for (int i = 0; i < 3; i++)
      {
         finalConstraintOrientationError.setToZero();
         finalConstraintAngularVelocityError.setToZero();
         finalConstraintAngularAccelerationError.setToZero();
         finalConstraintPolynomial[i].setConstant(0.0);
      }
   }

   public void blendInitialConstraint(FrameOrientation initialPose, double initialTime, double blendDuration)
   {
      clearInitialConstraint();
      computeInitialConstraintError(initialPose, initialTime);
      computeInitialConstraintPolynomial(initialTime, blendDuration);
   }

   public void blendInitialConstraint(FrameOrientation initialPose, FrameVector initialAngularVelocity, double initialTime, double blendDuration)
   {
      clearInitialConstraint();
      computeInitialConstraintError(initialPose, initialAngularVelocity, initialTime);
      computeInitialConstraintPolynomial(initialTime, blendDuration);
   }

   public void blendInitialConstraint(FrameOrientation initialOrientation, FrameVector initialAngularVelocity, FrameVector initialAngularAcceleration,
         double initialTime, double blendDuration)
   {
      clearInitialConstraint();
      computeInitialConstraintError(initialOrientation, initialAngularVelocity, initialAngularAcceleration, initialTime);
      computeInitialConstraintPolynomial(initialTime, blendDuration);
   }

   public void blendFinalConstraint(FrameOrientation finalOrientation, double finalTime, double blendDuration)
   {
      clearFinalConstraint();
      computeFinalConstraintError(finalOrientation, finalTime);
      computeFinalConstraintPolynomial(finalTime, blendDuration);
   }

   public void blendFinalConstraint(FrameOrientation finalOrientation, FrameVector finalAngularVelocity, double finalTime, double blendDuration)
   {
      clearFinalConstraint();
      computeFinalConstraintError(finalOrientation, finalAngularVelocity, finalTime);
      computeFinalConstraintPolynomial(finalTime, blendDuration);
   }

   public void blendFinalConstraint(FrameOrientation finalOrientation, FrameVector finalAngularVelocity, FrameVector finalAngularAcceleration, double finalTime,
         double blendDuration)
   {
      clearFinalConstraint();
      computeFinalConstraintError(finalOrientation, finalAngularVelocity, finalAngularAcceleration, finalTime);
      computeFinalConstraintPolynomial(finalTime, blendDuration);
   }

   @Override
   public void getOrientation(FrameOrientation orientationToPack)
   {
      orientationToPack.setIncludingFrame(orientation);
   }

   @Override
   public void getAngularVelocity(FrameVector angularVelocityToPack)
   {
      angularVelocityToPack.setIncludingFrame(angularVelocity);
   }

   @Override
   public void getAngularAcceleration(FrameVector angularAccelerationToPack)
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
      tempQuaternion.set(initialConstraintOrientationOffset);
      orientation.multiply(tempQuaternion);
      angularVelocity.add(initialConstraintAngularVelocityOffset);
      angularAcceleration.add(initialConstraintAngularAccelerationOffset);

      computeFinalConstraintOffset(time);
      tempQuaternion.set(finalConstraintOrientationOffset);
      orientation.multiply(tempQuaternion);
      angularVelocity.add(finalConstraintAngularVelocityOffset);
      angularAcceleration.add(finalConstraintAngularAccelerationOffset);
   }

   @Override
   public boolean isDone()
   {
      return trajectory.isDone();
   }

   private void computeInitialConstraintError(FrameOrientation initialOrientation, double initialTime)
   {
      trajectory.compute(initialTime);
      trajectoryFrame.checkReferenceFrameMatch(initialOrientation.getReferenceFrame());

      trajectory.getOrientation(tempOrientation);
      tempOrientation.changeFrame(trajectoryFrame);
      tempQuaternion.difference(tempOrientation.getQuaternion(), initialOrientation.getQuaternion());
      RotationVectorConversion.convertQuaternionToRotationVector(tempQuaternion, initialConstraintOrientationError);
   }

   private void computeInitialConstraintError(FrameOrientation initialOrientation, FrameVector initialAngularVelocity, double initialTime)
   {
      computeInitialConstraintError(initialOrientation, initialTime);
      trajectoryFrame.checkReferenceFrameMatch(initialAngularVelocity.getReferenceFrame());

      trajectory.getAngularVelocity(tempAngularVelocity);
      tempAngularVelocity.changeFrame(trajectoryFrame);
      initialConstraintAngularVelocityError.set(initialAngularVelocity.getVector());
      initialConstraintAngularVelocityError.sub(tempAngularVelocity.getVector());
   }

   private void computeInitialConstraintError(FrameOrientation initialOrientation, FrameVector initialAngularVelocity, FrameVector initialAngularAcceleration,
         double initialTime)
   {
      computeInitialConstraintError(initialOrientation, initialAngularVelocity, initialTime);
      trajectoryFrame.checkReferenceFrameMatch(initialAngularAcceleration.getReferenceFrame());

      trajectory.getAngularAcceleration(tempAngularAcceleration);
      tempAngularAcceleration.changeFrame(trajectoryFrame);
      initialConstraintAngularAccelerationError.set(initialAngularAcceleration.getVector());
      initialConstraintAngularAccelerationError.sub(tempAngularAcceleration.getVector());
   }

   private void computeFinalConstraintError(FrameOrientation finalOrientation, double finalTime)
   {
      trajectory.compute(finalTime);
      trajectoryFrame.checkReferenceFrameMatch(finalOrientation.getReferenceFrame());

      trajectory.getOrientation(tempOrientation);
      tempOrientation.changeFrame(trajectoryFrame);
      tempQuaternion.difference(tempOrientation.getQuaternion(), finalOrientation.getQuaternion());
      RotationVectorConversion.convertQuaternionToRotationVector(tempQuaternion, finalConstraintOrientationError);
   }

   private void computeFinalConstraintError(FrameOrientation finalOrientation, FrameVector finalAngularVelocity, double finalTime)
   {
      computeFinalConstraintError(finalOrientation, finalTime);
      trajectoryFrame.checkReferenceFrameMatch(finalAngularVelocity.getReferenceFrame());

      trajectory.getAngularVelocity(tempAngularVelocity);
      tempAngularVelocity.changeFrame(trajectoryFrame);
      finalConstraintAngularVelocityError.set(finalAngularVelocity.getVector());
      finalConstraintAngularVelocityError.sub(tempAngularVelocity.getVector());
   }

   private void computeFinalConstraintError(FrameOrientation finalOrientation, FrameVector finalAngularVelocity, FrameVector finalAngularAcceleration,
         double finalTime)
   {
      computeFinalConstraintError(finalOrientation, finalAngularVelocity, finalTime);
      trajectoryFrame.checkReferenceFrameMatch(finalAngularAcceleration.getReferenceFrame());

      trajectory.getAngularAcceleration(tempAngularAcceleration);
      tempAngularAcceleration.changeFrame(trajectoryFrame);
      finalConstraintAngularAccelerationError.set(finalAngularAcceleration.getVector());
      finalConstraintAngularAccelerationError.sub(tempAngularAcceleration.getVector());
   }

   private void computeInitialConstraintPolynomial(double initialTime, double blendDuration)
   {
      initialBlendStartTime.set(initialTime);
      initialBlendEndTime.set(initialTime + blendDuration);

      for (int i = 0; i < 3; i++)
      {
         double startTime = initialBlendStartTime.getDoubleValue();
         double endTime = initialBlendEndTime.getDoubleValue();

         double orientationError = initialConstraintOrientationError.getElement(i);
         double angularVelocityError = initialConstraintAngularVelocityError.getElement(i);
         double angularAccelerationError = initialConstraintAngularAccelerationError.getElement(i);
         initialConstraintPolynomial[i].setQuintic(startTime, endTime, orientationError, angularVelocityError, angularAccelerationError, 0, 0, 0);
      }
   }

   private void computeFinalConstraintPolynomial(double finalTime, double blendDuration)
   {
      finalBlendStartTime.set(finalTime - blendDuration);
      finalBlendEndTime.set(finalTime);

      for (int i = 0; i < 3; i++)
      {
         double startTime = finalBlendStartTime.getDoubleValue();
         double endTime = finalBlendEndTime.getDoubleValue();

         double orientationError = finalConstraintOrientationError.getElement(i);
         double angularVelocityError = finalConstraintAngularVelocityError.getElement(i);
         double angularAccelerationError = finalConstraintAngularAccelerationError.getElement(i);
         finalConstraintPolynomial[i].setQuintic(startTime, endTime, 0, 0, 0, orientationError, angularVelocityError, angularAccelerationError);
      }
   }

   private void computeInitialConstraintOffset(double time)
   {
      double startTime = initialBlendStartTime.getDoubleValue();
      double endTime = initialBlendEndTime.getDoubleValue();
      time = MathTools.clamp(time, startTime, endTime);

      for (int i = 0; i < 3; i++)
      {
         initialConstraintPolynomial[i].compute(time);
         initialConstraintOrientationOffset.setElement(i, initialConstraintPolynomial[i].getPosition());
         initialConstraintAngularVelocityOffset.setElement(i, initialConstraintPolynomial[i].getVelocity());
         initialConstraintAngularAccelerationOffset.setElement(i, initialConstraintPolynomial[i].getAcceleration());
      }
   }

   private void computeFinalConstraintOffset(double time)
   {
      double startTime = finalBlendStartTime.getDoubleValue();
      double endTime = finalBlendEndTime.getDoubleValue();
      time = MathTools.clamp(time, startTime, endTime);

      for (int i = 0; i < 3; i++)
      {
         finalConstraintPolynomial[i].compute(time);
         finalConstraintOrientationOffset.setElement(i, finalConstraintPolynomial[i].getPosition());
         finalConstraintAngularVelocityOffset.setElement(i, finalConstraintPolynomial[i].getVelocity());
         finalConstraintAngularAccelerationOffset.setElement(i, finalConstraintPolynomial[i].getAcceleration());
      }
   }
}
