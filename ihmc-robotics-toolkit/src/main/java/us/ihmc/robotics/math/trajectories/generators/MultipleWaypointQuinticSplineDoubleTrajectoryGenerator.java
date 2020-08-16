package us.ihmc.robotics.math.trajectories.generators;

import us.ihmc.robotics.math.interpolators.QuinticSplineInterpolator;
import us.ihmc.robotics.math.trajectories.DoubleTrajectoryGenerator;
import us.ihmc.yoVariables.registry.YoRegistry;

/**
 * Helper class to create a one degree of freedom trajectory using the quintic spline interpolator
 * 
 * Not realtime safe. 
 * 
 */
public class MultipleWaypointQuinticSplineDoubleTrajectoryGenerator implements DoubleTrajectoryGenerator
{
   private final QuinticSplineInterpolator interpolator;

   private int numberOfPoints = 0;
   private double v0, vf, a0, af;
   private final double[] time;
   private final double[] positions;

   public MultipleWaypointQuinticSplineDoubleTrajectoryGenerator(String name, int maximumNumberOfPoints, YoRegistry parentRegistry)
   {
      this.interpolator = new QuinticSplineInterpolator(name, maximumNumberOfPoints, 1, parentRegistry);
      this.time = new double[maximumNumberOfPoints];
      this.positions = new double[maximumNumberOfPoints];
      
      clear();
   }

   /**
    * Clear all waypoints out of this trajectory
    */
   public void clear()
   {
      this.numberOfPoints = 0;
      this.v0 = 0.0;
      this.vf = 0.0;
      this.a0 = 0.0;
      this.af = 0.0;
   }
   
   
   public void addWaypoint(double time, double position)
   {
      if(this.numberOfPoints >= this.interpolator.getMaximumNumberOfWaypoints())
      {
         throw new RuntimeException("Number of waypoints exceeds maximum number of waypoints");
      }
      
      this.time[this.numberOfPoints] = time;
      this.positions[this.numberOfPoints] = position;
      this.numberOfPoints++;
   }
   
   public void setInitialConditions(double initialVelocity, double initialAcceleration)
   {
      this.v0 = initialVelocity;
      this.a0 = initialAcceleration;
   }
   
   public void setFinalConditions(double finalVelocity, double finalAcceleration)
   {
      this.vf = finalVelocity;
      this.af = finalAcceleration;
   }
   
   @Override
   public void initialize()
   {
      if (time.length != positions.length)
      {
         throw new RuntimeException("Time and positions lengths do not match");
      }
      this.interpolator.initialize(this.numberOfPoints, time);

      this.interpolator.determineCoefficients(0, positions, v0, vf, a0, af);
   }

   @Override
   public void compute(double time)
   {
      this.interpolator.compute(time);
   }

   @Override
   public boolean isDone()
   {
      return this.interpolator.isDone();
   }

   @Override
   public double getValue()
   {
      return getPosition();
   }

   public double getPosition()
   {
      return this.interpolator.getPosition(0);
   }

   @Override
   public double getVelocity()
   {
      return this.interpolator.getVelocity(0);
   }

   @Override
   public double getAcceleration()
   {
      return this.interpolator.getAcceleration(0);
   }
}
