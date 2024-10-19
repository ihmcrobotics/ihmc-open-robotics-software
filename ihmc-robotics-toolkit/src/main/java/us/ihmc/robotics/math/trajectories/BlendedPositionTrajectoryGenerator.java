package us.ihmc.robotics.math.trajectories;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.commons.trajectories.interfaces.FixedFramePositionTrajectoryGenerator;
import us.ihmc.commons.trajectories.yoVariables.YoPolynomial3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class BlendedPositionTrajectoryGenerator implements FixedFramePositionTrajectoryGenerator
{
   private final FixedFramePositionTrajectoryGenerator trajectory;
   private final ReferenceFrame trajectoryFrame;
   private final YoPolynomial3D initialConstraintPolynomial;
   private final YoPolynomial3D finalConstraintPolynomial;
   private final YoDouble initialBlendTimeOffset;
   private final YoDouble initialBlendStartTime;
   private final YoDouble initialBlendEndTime;
   private final YoDouble finalBlendTimeOffset;
   private final YoDouble finalBlendStartTime;
   private final YoDouble finalBlendEndTime;

   private final Vector3D initialConstraintPositionError = new Vector3D();
   private final Vector3D initialConstraintVelocityError = new Vector3D();

   private final Vector3D finalConstraintPositionError = new Vector3D();
   private final Vector3D finalConstraintVelocityError = new Vector3D();

   private static final Vector3DReadOnly zeroVector = EuclidCoreTools.zeroVector3D;

   private final FramePoint3D position = new FramePoint3D();
   private final FrameVector3D velocity = new FrameVector3D();
   private final FrameVector3D acceleration = new FrameVector3D();
   private final FramePoint3D tempPosition = new FramePoint3D();
   private final FrameVector3D tempVelocity = new FrameVector3D();

   public BlendedPositionTrajectoryGenerator(String prefix,
                                             FixedFramePositionTrajectoryGenerator trajectory,
                                             ReferenceFrame trajectoryFrame,
                                             YoRegistry parentRegistry)
   {
      this.trajectory = trajectory;
      this.trajectoryFrame = trajectoryFrame;

      this.initialConstraintPolynomial = new YoPolynomial3D(prefix + "InitialConstraintPolynomial", 6, parentRegistry);
      this.finalConstraintPolynomial = new YoPolynomial3D(prefix + "FinalConstraintPolynomial", 6, parentRegistry);

      this.initialBlendTimeOffset = new YoDouble(prefix + "InitialBlendTimeOffset", parentRegistry);
      this.initialBlendStartTime = new YoDouble(prefix + "InitialBlendStartTime", parentRegistry);
      this.initialBlendEndTime = new YoDouble(prefix + "InitialBlendEndTime", parentRegistry);
      this.finalBlendTimeOffset = new YoDouble(prefix + "FinalBlendTimeOffset", parentRegistry);
      this.finalBlendStartTime = new YoDouble(prefix + "FinalBlendStartTime", parentRegistry);
      this.finalBlendEndTime = new YoDouble(prefix + "FinalBlendEndTime", parentRegistry);

      this.position.changeFrame(trajectoryFrame);
      this.velocity.changeFrame(trajectoryFrame);
      this.acceleration.changeFrame(trajectoryFrame);
      this.tempPosition.changeFrame(trajectoryFrame);
      this.tempVelocity.changeFrame(trajectoryFrame);
      clear();
   }

   public void clear()
   {
      clearInitialConstraint();
      clearFinalConstraint();
   }

   public void clearInitialConstraint()
   {
      initialConstraintPositionError.setToZero();
      initialConstraintVelocityError.setToZero();
      initialConstraintPolynomial.setZero();
   }

   public void clearFinalConstraint()
   {
      finalConstraintPositionError.setToZero();
      finalConstraintVelocityError.setToZero();
      finalConstraintPolynomial.setZero();
   }

   public void blendInitialConstraint(FramePoint3DReadOnly initialPosition, double initialTime, double blendDuration)
   {
      clearInitialConstraint();
      computeInitialConstraintError(initialPosition, initialTime);
      computeInitialConstraintPolynomial(initialTime, blendDuration);
   }

   public void blendInitialConstraint(FramePoint3DReadOnly initialPosition, FrameVector3DReadOnly initialVelocity, double initialTime, double blendDuration)
   {
      clearInitialConstraint();
      computeInitialConstraintError(initialPosition, initialVelocity, initialTime);
      computeInitialConstraintPolynomial(initialTime, blendDuration);
   }

   public void blendFinalConstraint(FramePoint3DReadOnly finalPosition, FrameVector3DReadOnly finalVelocity, double finalTime, double blendDuration)
   {
      clearFinalConstraint();
      computeFinalConstraintError(finalPosition, finalVelocity, finalTime);
      computeFinalConstraintPolynomial(finalTime, blendDuration);
   }

   public void blendFinalConstraint(FramePoint3DReadOnly finalPosition, double finalTime, double blendDuration)
   {
      clearFinalConstraint();
      computeFinalConstraintError(finalPosition, finalTime);
      computeFinalConstraintPolynomial(finalTime, blendDuration);
   }

   public void initializeTrajectory()
   {
      trajectory.initialize();
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return trajectoryFrame;
   }

   @Override
   public FramePoint3DReadOnly getPosition()
   {
      return position;
   }

   @Override
   public FrameVector3DReadOnly getVelocity()
   {
      return velocity;
   }

   @Override
   public FrameVector3DReadOnly getAcceleration()
   {
      return acceleration;
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
      position.setIncludingFrame(trajectory.getPosition());
      velocity.setIncludingFrame(trajectory.getVelocity());
      acceleration.setIncludingFrame(trajectory.getAcceleration());

      position.changeFrame(trajectoryFrame);
      velocity.changeFrame(trajectoryFrame);
      acceleration.changeFrame(trajectoryFrame);

      computeInitialConstraintOffset(time);
      position.add(initialConstraintPolynomial.getPosition());
      velocity.add(initialConstraintPolynomial.getVelocity());
      acceleration.add(initialConstraintPolynomial.getAcceleration());

      computeFinalConstraintOffset(time);
      position.add(finalConstraintPolynomial.getPosition());
      velocity.add(finalConstraintPolynomial.getVelocity());
      acceleration.add(finalConstraintPolynomial.getAcceleration());
   }

   @Override
   public boolean isDone()
   {
      return trajectory.isDone();
   }

   private void computeInitialConstraintError(FramePoint3DReadOnly initialPosition, double initialTime)
   {
      computeConstraintPositionError(initialPosition, initialTime, initialConstraintPositionError);
   }


   private void computeInitialConstraintError(FramePoint3DReadOnly initialPosition, FrameVector3DReadOnly initialVelocity, double initialTime)
   {
      computeInitialConstraintError(initialPosition, initialTime);
      computeConstraintVelocityError(initialVelocity, initialConstraintVelocityError);
   }


   private void computeFinalConstraintError(FramePoint3DReadOnly finalPosition, double finalTime)
   {
      computeConstraintPositionError(finalPosition, finalTime, finalConstraintPositionError);
   }

   private void computeFinalConstraintError(FramePoint3DReadOnly finalPosition, FrameVector3DReadOnly finalVelocity, double finalTime)
   {
      computeFinalConstraintError(finalPosition, finalTime);
      computeConstraintVelocityError(finalVelocity, finalConstraintVelocityError);
   }

   private void computeConstraintPositionError(FramePoint3DReadOnly desiredPosition, double time, Vector3DBasics positionErrorToPack)
   {
      trajectory.compute(time);
      trajectoryFrame.checkReferenceFrameMatch(desiredPosition.getReferenceFrame());

      tempPosition.setIncludingFrame(trajectory.getPosition());
      tempPosition.changeFrame(trajectoryFrame);
      positionErrorToPack.sub(desiredPosition, tempPosition);
   }

   private void computeConstraintVelocityError(FrameVector3DReadOnly desiredVelocity, Vector3DBasics velocityErrorToPack)
   {
      trajectoryFrame.checkReferenceFrameMatch(desiredVelocity.getReferenceFrame());

      tempVelocity.setIncludingFrame(trajectory.getVelocity());
      tempVelocity.changeFrame(trajectoryFrame);
      velocityErrorToPack.sub(desiredVelocity, tempVelocity);
   }

   private void computeInitialConstraintPolynomial(double initialTime, double blendDuration)
   {
      initialBlendTimeOffset.set(initialTime);
      initialBlendStartTime.set(0.0);
      initialBlendEndTime.set(blendDuration);

      initialConstraintPolynomial.setQuinticWithZeroTerminalAcceleration(initialBlendStartTime.getDoubleValue(),
                                                                         initialBlendEndTime.getDoubleValue(),
                                                                         initialConstraintPositionError,
                                                                         initialConstraintVelocityError,
                                                                         zeroVector,
                                                                         zeroVector);
   }

   private void computeFinalConstraintPolynomial(double finalTime, double blendDuration)
   {
      finalBlendTimeOffset.set(finalTime);
      finalBlendStartTime.set(-blendDuration);
      finalBlendEndTime.set(0.0);

      finalConstraintPolynomial.setQuinticWithZeroTerminalAcceleration(finalBlendStartTime.getDoubleValue(),
                                                                       finalBlendEndTime.getDoubleValue(),
                                                                       zeroVector,
                                                                       zeroVector,
                                                                       finalConstraintPositionError,
                                                                       finalConstraintVelocityError);
   }

   private void computeInitialConstraintOffset(double time)
   {
      double startTime = initialBlendStartTime.getDoubleValue();
      double endTime = initialBlendEndTime.getDoubleValue();
      time = MathTools.clamp(time - initialBlendTimeOffset.getValue(), startTime, endTime);

      initialConstraintPolynomial.compute(time);
   }

   private void computeFinalConstraintOffset(double time)
   {
      double startTime = finalBlendStartTime.getDoubleValue();
      double endTime = finalBlendEndTime.getDoubleValue();
      time = MathTools.clamp(time - finalBlendTimeOffset.getValue(), startTime, endTime);

      finalConstraintPolynomial.compute(time);
   }
}
