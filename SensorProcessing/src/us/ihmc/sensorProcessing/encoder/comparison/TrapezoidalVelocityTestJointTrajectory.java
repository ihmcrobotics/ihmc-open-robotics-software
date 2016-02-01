package us.ihmc.sensorProcessing.encoder.comparison;

import us.ihmc.robotics.trajectories.TrapezoidalVelocityTrajectory;

public class TrapezoidalVelocityTestJointTrajectory implements EncoderProcessorEvaluationTrajectory
{
   private final TrapezoidalVelocityTrajectory trajectory;
   private double q;
   private double qd;

   public TrapezoidalVelocityTestJointTrajectory(double x0, double xF, double v0, double vF, double vMax, double aMax)
   {
      trajectory = new TrapezoidalVelocityTrajectory(0.0, x0, xF, v0, vF, vMax, aMax);
   }

   public void update(double time)
   {
      q = trajectory.getPosition(time);
      qd = trajectory.getVelocity(time);
   }

   public double getPosition()
   {
      return q;
   }

   public double getVelocity()
   {
      return qd;
   }
}
