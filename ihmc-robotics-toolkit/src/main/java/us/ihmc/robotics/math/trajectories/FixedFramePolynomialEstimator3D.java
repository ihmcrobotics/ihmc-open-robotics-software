package us.ihmc.robotics.math.trajectories;

import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.robotics.math.trajectories.core.Polynomial3DFrameFactories;
import us.ihmc.robotics.math.trajectories.interfaces.FixedFramePositionTrajectoryGenerator;
import us.ihmc.commons.time.TimeIntervalBasics;
import us.ihmc.commons.time.TimeIntervalProvider;

public class FixedFramePolynomialEstimator3D implements FixedFramePositionTrajectoryGenerator, TimeIntervalProvider, Settable<FixedFramePolynomialEstimator3D>
{
   private final FramePoint3DReadOnly position;
   private final FrameVector3DReadOnly velocity;
   private final FrameVector3DReadOnly acceleration;

   private double currentTime = 0.0;

   private final PolynomialEstimator3D estimator = new PolynomialEstimator3D();

   private final ReferenceFrame referenceFrame;

   public FixedFramePolynomialEstimator3D(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;

      position = Polynomial3DFrameFactories.newLinkedFramePoint3DReadOnly(this, estimator.getPosition());
      velocity = Polynomial3DFrameFactories.newLinkedFrameVector3DReadOnly(this, estimator.getVelocity());
      acceleration = Polynomial3DFrameFactories.newLinkedFrameVector3DReadOnly(this, estimator.getAcceleration());
   }


   public void reset()
   {
      currentTime = Double.NaN;
      estimator.reset();
   }

   public void reshape(int order)
   {
      estimator.reshape(order);
   }

   @Override
   public void set(FixedFramePolynomialEstimator3D other)
   {
      checkReferenceFrameMatch(other);

      estimator.set(other.estimator);
   }

   @Override
   public TimeIntervalBasics getTimeInterval()
   {
      return estimator.getTimeInterval();
   }

   public void addObjectivePosition(double time, FrameTuple3DReadOnly value)
   {
      checkReferenceFrameMatch(value);
      estimator.addObjectivePosition(time, value);
   }

   public void addObjectivePosition(double weight, double time, FrameTuple3DReadOnly value)
   {
      checkReferenceFrameMatch(value);
      estimator.addObjectivePosition(weight, time, value);
   }

   public void addObjectiveVelocity(double time, FrameTuple3DReadOnly value)
   {
      checkReferenceFrameMatch(value);
      estimator.addObjectiveVelocity(time, value);
   }

   public void addObjectiveVelocity(double weight, double time, FrameTuple3DReadOnly value)
   {
      checkReferenceFrameMatch(value);
      estimator.addObjectiveVelocity(weight, time, value);
   }

   public void addConstraintPosition(double time, FrameTuple3DReadOnly value)
   {
      checkReferenceFrameMatch(value);
      estimator.addConstraintPosition(time, value);
   }

   public void addConstraintVelocity(double time, FrameTuple3DReadOnly value)
   {
      checkReferenceFrameMatch(value);
      estimator.addConstraintVelocity(time, value);
   }

   @Override
   public void initialize()
   {
      estimator.initialize();
   }

   @Override
   public void compute(double time)
   {
      estimator.compute(time);
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
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
   public boolean isDone()
   {
      return currentTime >= getTimeInterval().getEndTime();
   }

   @Override
   public void showVisualization()
   {
   }

   @Override
   public void hideVisualization()
   {
   }
}
