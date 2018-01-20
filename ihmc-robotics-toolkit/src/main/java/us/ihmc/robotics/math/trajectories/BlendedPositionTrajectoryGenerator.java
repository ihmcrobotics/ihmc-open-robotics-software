package us.ihmc.robotics.math.trajectories;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.commons.MathTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class BlendedPositionTrajectoryGenerator implements PositionTrajectoryGenerator
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final PositionTrajectoryGenerator trajectory;
   private final ReferenceFrame trajectoryFrame;
   private final YoPolynomial[] initialConstraintPolynomial = new YoPolynomial[3];
   private final YoPolynomial[] finalConstraintPolynomial = new YoPolynomial[3];
   private final YoDouble initialBlendStartTime;
   private final YoDouble initialBlendEndTime;
   private final YoDouble finalBlendStartTime;
   private final YoDouble finalBlendEndTime;

   private final Vector3D initialConstraintPositionError = new Vector3D();
   private final Vector3D initialConstraintVelocityError = new Vector3D();
   private final Vector3D finalConstraintPositionError = new Vector3D();
   private final Vector3D finalConstraintVelocityError = new Vector3D();

   private final Vector3D initialConstraintPositionOffset = new Vector3D();
   private final Vector3D initialConstraintVelocityOffset = new Vector3D();
   private final Vector3D initialConstraintAccelerationOffset = new Vector3D();
   private final Vector3D finalConstraintPositionOffset = new Vector3D();
   private final Vector3D finalConstraintVelocityOffset = new Vector3D();
   private final Vector3D finalConstraintAccelerationOffset = new Vector3D();

   private final FramePoint3D position = new FramePoint3D();
   private final FrameVector3D velocity = new FrameVector3D();
   private final FrameVector3D acceleration = new FrameVector3D();
   private final FramePoint3D tempPosition = new FramePoint3D();
   private final FrameVector3D tempVelocity = new FrameVector3D();
   private final FrameVector3D tempAcceleration = new FrameVector3D();

   public BlendedPositionTrajectoryGenerator(String prefix, PositionTrajectoryGenerator trajectory, ReferenceFrame trajectoryFrame,
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
      this.initialBlendStartTime = new YoDouble(prefix + "InitialBlendStartTime", registry);
      this.initialBlendEndTime = new YoDouble(prefix + "InitialBlendEndTime", registry);
      this.finalBlendStartTime = new YoDouble(prefix + "FinalBlendStartTime", registry);
      this.finalBlendEndTime = new YoDouble(prefix + "FinalBlendEndTime", registry);
      this.position.changeFrame(trajectoryFrame);
      this.velocity.changeFrame(trajectoryFrame);
      this.acceleration.changeFrame(trajectoryFrame);
      this.tempPosition.changeFrame(trajectoryFrame);
      this.tempVelocity.changeFrame(trajectoryFrame);
      this.tempAcceleration.changeFrame(trajectoryFrame);
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
         initialConstraintPositionError.setToZero();
         initialConstraintVelocityError.setToZero();
         initialConstraintPolynomial[i].setConstant(0.0);
      }
   }

   public void clearFinalConstraint()
   {
      for (int i = 0; i < 3; i++)
      {
         finalConstraintPositionError.setToZero();
         finalConstraintVelocityError.setToZero();
         finalConstraintPolynomial[i].setConstant(0.0);
      }
   }

   public void blendInitialConstraint(FramePoint3DReadOnly initialPosition, double initialTime, double blendDuration)
   {
      clearInitialConstraint();
      computeInitialConstraintError(initialPosition, initialTime);
      computeInitialConstraintPolynomial(initialTime, blendDuration);
   }

   public void blendInitialConstraint(FramePoint3DReadOnly initialPosition, FrameVector3D initialVelocity, double initialTime, double blendDuration)
   {
      clearInitialConstraint();
      computeInitialConstraintError(initialPosition, initialVelocity, initialTime);
      computeInitialConstraintPolynomial(initialTime, blendDuration);
   }

   public void blendFinalConstraint(FramePoint3DReadOnly finalPosition, double finalTime, double blendDuration)
   {
      clearFinalConstraint();
      computeFinalConstraintError(finalPosition, finalTime);
      computeFinalConstraintPolynomial(finalTime, blendDuration);
   }

   public void blendFinalConstraint(FramePoint3DReadOnly finalPosition, FrameVector3DReadOnly finalVelocity, double finalTime, double blendDuration)
   {
      clearFinalConstraint();
      computeFinalConstraintError(finalPosition, finalVelocity, finalTime);
      computeFinalConstraintPolynomial(finalTime, blendDuration);
   }

   @Override
   public void getPosition(FramePoint3D positionToPack)
   {
      positionToPack.setIncludingFrame(position);
   }

   @Override
   public void getVelocity(FrameVector3D velocityToPack)
   {
      velocityToPack.setIncludingFrame(velocity);
   }

   @Override
   public void getAcceleration(FrameVector3D accelerationToPack)
   {
      accelerationToPack.setIncludingFrame(acceleration);
   }

   @Override
   public void showVisualization()
   {
      trajectory.showVisualization();
   }

   @Override
   public void hideVisualization()
   {
      trajectory.hideVisualization();
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
      trajectory.getPosition(position);
      trajectory.getVelocity(velocity);
      trajectory.getAcceleration(acceleration);

      position.changeFrame(trajectoryFrame);
      velocity.changeFrame(trajectoryFrame);
      acceleration.changeFrame(trajectoryFrame);

      computeInitialConstraintOffset(time);
      position.add(initialConstraintPositionOffset);
      velocity.add(initialConstraintVelocityOffset);
      acceleration.add(initialConstraintAccelerationOffset);

      computeFinalConstraintOffset(time);
      position.add(finalConstraintPositionOffset);
      velocity.add(finalConstraintVelocityOffset);
      acceleration.add(finalConstraintAccelerationOffset);
   }

   @Override
   public boolean isDone()
   {
      return trajectory.isDone();
   }

   private void computeInitialConstraintError(FramePoint3DReadOnly initialPosition, double initialTime)
   {
      trajectory.compute(initialTime);
      trajectoryFrame.checkReferenceFrameMatch(initialPosition.getReferenceFrame());

      trajectory.getPosition(tempPosition);
      tempPosition.changeFrame(trajectoryFrame);
      initialConstraintPositionError.set(initialPosition);
      initialConstraintPositionError.sub(tempPosition);
   }

   private void computeInitialConstraintError(FramePoint3DReadOnly initialPosition, FrameVector3DReadOnly initialVelocity, double initialTime)
   {
      computeInitialConstraintError(initialPosition, initialTime);
      trajectoryFrame.checkReferenceFrameMatch(initialVelocity.getReferenceFrame());

      trajectory.getVelocity(tempVelocity);
      tempVelocity.changeFrame(trajectoryFrame);
      initialConstraintVelocityError.set(initialVelocity);
      initialConstraintVelocityError.sub(tempVelocity);
   }

   private void computeFinalConstraintError(FramePoint3DReadOnly finalPosition, double finalTime)
   {
      trajectory.compute(finalTime);
      trajectoryFrame.checkReferenceFrameMatch(finalPosition.getReferenceFrame());

      trajectory.getPosition(tempPosition);
      tempPosition.changeFrame(trajectoryFrame);
      finalConstraintPositionError.set(finalPosition);
      finalConstraintPositionError.sub(tempPosition);
   }

   private void computeFinalConstraintError(FramePoint3DReadOnly finalPosition, FrameVector3DReadOnly finalVelocity, double finalTime)
   {
      computeFinalConstraintError(finalPosition, finalTime);
      trajectoryFrame.checkReferenceFrameMatch(finalVelocity.getReferenceFrame());

      trajectory.getVelocity(tempVelocity);
      tempVelocity.changeFrame(trajectoryFrame);
      finalConstraintVelocityError.set(finalVelocity);
      finalConstraintVelocityError.sub(tempVelocity);
   }

   private void computeInitialConstraintPolynomial(double initialTime, double blendDuration)
   {
      initialBlendStartTime.set(initialTime);
      initialBlendEndTime.set(initialTime + blendDuration);

      for (int i = 0; i < 3; i++)
      {
         double startTime = initialBlendStartTime.getDoubleValue();
         double endTime = initialBlendEndTime.getDoubleValue();

         double positionError = initialConstraintPositionError.getElement(i);
         double velocityError = initialConstraintVelocityError.getElement(i);
         initialConstraintPolynomial[i].setQuintic(startTime, endTime, positionError, velocityError, 0, 0, 0, 0);
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

         double positionError = finalConstraintPositionError.getElement(i);
         double velocityError = finalConstraintVelocityError.getElement(i);
         finalConstraintPolynomial[i].setQuintic(startTime, endTime, 0, 0, 0, positionError, velocityError, 0);
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
         initialConstraintPositionOffset.setElement(i, initialConstraintPolynomial[i].getPosition());
         initialConstraintVelocityOffset.setElement(i, initialConstraintPolynomial[i].getVelocity());
         initialConstraintAccelerationOffset.setElement(i, initialConstraintPolynomial[i].getAcceleration());
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
         finalConstraintPositionOffset.setElement(i, finalConstraintPolynomial[i].getPosition());
         finalConstraintVelocityOffset.setElement(i, finalConstraintPolynomial[i].getVelocity());
         finalConstraintAccelerationOffset.setElement(i, finalConstraintPolynomial[i].getAcceleration());
      }
   }
}
