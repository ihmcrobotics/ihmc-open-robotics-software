package us.ihmc.robotics.math.trajectories.waypoints;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.math.trajectories.YoPolynomial;

public class WaypointTrajectory1DCalculator
{
   private static final double EPSILON = 1.0e-7;
   private final SimpleTrajectoryWaypoint1DData trajectory = new SimpleTrajectoryWaypoint1DData();
   private final YoPolynomial polynomial = new YoPolynomial("polynomial", 6, new YoVariableRegistry("Dummy"));

   public WaypointTrajectory1DCalculator()
   {
      clear();
   }

   public void clear()
   {
      trajectory.clear();
   }

   public void appendWaypoint(Waypoint1DInterface<?> waypoint)
   {
      trajectory.addWaypoint(waypoint);
   }

   public void appendWaypoint(double position)
   {
      appendWaypoint(Double.NaN, position);
   }

   public void appendWaypoint(double time, double position)
   {
      appendWaypoint(time, position, Double.NaN);
   }

   public void appendWaypoint(double time, double position, double velocity)
   {
      trajectory.addWaypoint(time, position, velocity);
   }

   public void computeWaypointTimes(double firstWaypointTime, double trajectoryTime)
   {
      int numberOfWaypoints = getNumberOfWaypoints();
      if (numberOfWaypoints == 0)
         throw new RuntimeException("There is no waypoint.");

      if (numberOfWaypoints == 1)
      {
         trajectory.getWaypoint(0).setTime(trajectoryTime);
         return;
      }

      double totalLength = 0.0;

      for (int i = 0; i < numberOfWaypoints - 1; i++)
         totalLength += Math.abs(trajectory.getWaypoint(i + 1).getPosition() - trajectory.getWaypoint(i).getPosition());

      trajectory.getWaypoint(0).setTime(firstWaypointTime);
      trajectory.getWaypoint(trajectory.getNumberOfWaypoints() - 1).setTime(firstWaypointTime + trajectoryTime);
      double time = firstWaypointTime;

      if (totalLength > EPSILON * getNumberOfWaypoints())
      {
         for (int i = 1; i < numberOfWaypoints - 1; i++)
         {
            double subLength = Math.abs(trajectory.getWaypoint(i).getPosition() - trajectory.getWaypoint(i - 1).getPosition());
            time += trajectoryTime * (subLength / totalLength);
            trajectory.getWaypoint(i).setTime(time);
         }
      }
      else
      {
         for (int i = 1; i < numberOfWaypoints - 1; i++)
         {
            time += trajectoryTime / getNumberOfWaypoints();
            trajectory.getWaypoint(i).setTime(time);
         }
      }
   }

   public void computeWaypointVelocities(boolean startAndFinishWithZeroVelocity)
   {
      int numberOfWaypoints = getNumberOfWaypoints();
      if (numberOfWaypoints < 3)
         throw new RuntimeException("Need at least 3 waypoints.");

      SimpleWaypoint1D firstWaypoint;
      SimpleWaypoint1D secondWaypoint;
      SimpleWaypoint1D thirdWaypoint;

      if (startAndFinishWithZeroVelocity)
      {
         trajectory.getWaypoint(0).setVelocity(0.0);
         trajectory.getWaypoint(numberOfWaypoints - 1).setVelocity(0.0);

         if (numberOfWaypoints == 3)
         {
            firstWaypoint = trajectory.getWaypoint(0);
            secondWaypoint = trajectory.getWaypoint(1);
            thirdWaypoint = trajectory.getWaypoint(2);
            secondWaypoint.setVelocity(compute2ndWaypointVelocityWithVelocityConstraint(firstWaypoint, secondWaypoint, thirdWaypoint));
            return;
         }
      }
      else
      {
         firstWaypoint = trajectory.getWaypoint(0);
         secondWaypoint = trajectory.getWaypoint(1);
         thirdWaypoint = trajectory.getWaypoint(2);
         firstWaypoint.setVelocity(computeWaypointVelocity(firstWaypoint, secondWaypoint, thirdWaypoint, Waypoint.FIRST));

         firstWaypoint = trajectory.getWaypoint(numberOfWaypoints - 3);
         secondWaypoint = trajectory.getWaypoint(numberOfWaypoints - 2);
         thirdWaypoint = trajectory.getWaypoint(numberOfWaypoints - 1);
         thirdWaypoint.setVelocity(computeWaypointVelocity(firstWaypoint, secondWaypoint, thirdWaypoint, Waypoint.THIRD));
      }

      for (int i = 1; i < numberOfWaypoints - 1; i++)
      {
         firstWaypoint = trajectory.getWaypoint(i - 1);
         secondWaypoint = trajectory.getWaypoint(i);
         thirdWaypoint = trajectory.getWaypoint(i + 1);
         secondWaypoint.setVelocity(computeWaypointVelocity(firstWaypoint, secondWaypoint, thirdWaypoint, Waypoint.SECOND));
      }
   }

   public boolean shouldVelocityBeZero(Waypoint1DInterface<?> firstWaypoint, Waypoint1DInterface<?> secondWaypoint)
   {
      double deltaPosition = Math.abs(secondWaypoint.getPosition() - firstWaypoint.getPosition());
      double deltaTime = Math.abs(secondWaypoint.getTime() - firstWaypoint.getTime());
      return MathTools.epsilonEquals(0.0, deltaPosition / deltaTime, 1.0e-7);
   }

   private double compute2ndWaypointVelocityWithVelocityConstraint(Waypoint1DInterface<?> firstWaypoint, Waypoint1DInterface<?> secondWaypoint,
         Waypoint1DInterface<?> thirdWaypoint)
   {
      double t0 = firstWaypoint.getTime();
      double z0 = firstWaypoint.getPosition();
      double zd0 = firstWaypoint.getVelocity();

      double tIntermediate = secondWaypoint.getTime();
      double zIntermediate = secondWaypoint.getPosition();

      double tf = thirdWaypoint.getTime();
      double zf = thirdWaypoint.getPosition();
      double zdf = thirdWaypoint.getVelocity();

      if (MathTools.epsilonEquals(tf, t0, EPSILON))
         return 0.0;
      else if (MathTools.epsilonEquals(t0, tIntermediate, EPSILON))
         tIntermediate += 0.001 * (tf - t0);
      else if (MathTools.epsilonEquals(tIntermediate, tf, EPSILON))
         tIntermediate -= 0.001 * (tf - t0);

      polynomial.setQuarticUsingWayPoint(t0, tIntermediate, tf, z0, zd0, zIntermediate, zf, zdf);
      polynomial.compute(tIntermediate);

      return polynomial.getVelocity();
   }

   private enum Waypoint
   {
      FIRST, SECOND, THIRD
   };

   private double computeWaypointVelocity(Waypoint1DInterface<?> firstWaypoint, Waypoint1DInterface<?> secondWaypoint, Waypoint1DInterface<?> thirdWaypoint,
         Waypoint waypointToComputeVelocityOf)
   {
      double t0 = firstWaypoint.getTime();
      double z0 = firstWaypoint.getPosition();

      double tIntermediate = secondWaypoint.getTime();
      double zIntermediate = secondWaypoint.getPosition();

      double tf = thirdWaypoint.getTime();
      double zf = thirdWaypoint.getPosition();

      if (MathTools.epsilonEquals(tf, t0, EPSILON))
         return 0.0;
      else if (MathTools.epsilonEquals(t0, tIntermediate, EPSILON))
         tIntermediate += 0.001 * (tf - t0);
      else if (MathTools.epsilonEquals(tIntermediate, tf, EPSILON))
         tIntermediate -= 0.001 * (tf - t0);

      polynomial.setQuadraticUsingIntermediatePoint(t0, tIntermediate, tf, z0, zIntermediate, zf);
      switch (waypointToComputeVelocityOf)
      {
      case FIRST:
         polynomial.compute(t0);
         break;
      case SECOND:
         polynomial.compute(tIntermediate);
         break;
      case THIRD:
         polynomial.compute(tf);
      default:
         break;
      }

      return polynomial.getVelocity();
   }

   public int getNumberOfWaypoints()
   {
      return trajectory.getNumberOfWaypoints();
   }

   public RecyclingArrayList<? extends Waypoint1DInterface<?>> getWaypoints()
   {
      return trajectory.getWaypoints();
   }

   public TrajectoryWaypointDataInterface<?, ? extends Waypoint1DInterface<?>> getTrajectoryData()
   {
      return trajectory;
   }
}
