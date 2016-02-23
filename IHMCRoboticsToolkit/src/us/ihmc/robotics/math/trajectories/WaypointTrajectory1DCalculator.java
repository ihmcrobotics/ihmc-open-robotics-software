package us.ihmc.robotics.math.trajectories;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.lists.RecyclingArrayList;

public class WaypointTrajectory1DCalculator
{
   private final RecyclingArrayList<SimpleWaypoint1D> waypoints = new RecyclingArrayList<>(20, SimpleWaypoint1D.class);
   private final YoPolynomial polynomial = new YoPolynomial("polynomial", 6, new YoVariableRegistry("Dummy"));

   public WaypointTrajectory1DCalculator()
   {
      clear();
   }

   public void clear()
   {
      waypoints.clear();
   }

   public void appendWaypoint(Waypoint1DInterface waypoint)
   {
      waypoints.add().set(waypoint);
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
      waypoints.add().set(time, position, velocity);
   }

   public void computeWaypointTimes(double firstWaypointTime, double trajectoryTime)
   {
      int numberOfWaypoints = getNumberOfWaypoints();
      if (numberOfWaypoints == 0)
         throw new RuntimeException("There is no waypoint.");

      if (numberOfWaypoints == 1)
      {
         waypoints.get(0).setTime(trajectoryTime);
         return;
      }

      double totalLength = 0.0;

      for (int i = 0; i < numberOfWaypoints - 1; i++)
         totalLength += Math.abs(waypoints.get(i + 1).getPosition() - waypoints.get(i).getPosition());

      waypoints.get(0).setTime(firstWaypointTime);
      waypoints.get(waypoints.size() - 1).setTime(firstWaypointTime + trajectoryTime);
      double time = firstWaypointTime;

      for (int i = 1; i < numberOfWaypoints - 1; i++)
      {
         double subLength = Math.abs(waypoints.get(i).getPosition() - waypoints.get(i - 1).getPosition());
         time += trajectoryTime * (subLength / totalLength);
         waypoints.get(i).setTime(time);
      }
   }

   public void computeWaypointVelocities(boolean startAndFinishWithZeroVelocity)
   {
      int numberOfWaypoints = getNumberOfWaypoints();
      if (numberOfWaypoints < 3)
         throw new RuntimeException("Need at least 3 waypoints.");

      if (startAndFinishWithZeroVelocity)
      {
         waypoints.get(0).setVelocity(0.0);
         waypoints.get(numberOfWaypoints - 1).setVelocity(0.0);

         if (numberOfWaypoints == 3)
         {
            waypoints.get(1).setVelocity(compute2ndWaypointVelocityWithVelocityConstraint(waypoints.get(0), waypoints.get(1), waypoints.get(2)));
            return;
         }
      }
      else
      {
         waypoints.get(0).setVelocity(computeWaypointVelocity(waypoints.get(0), waypoints.get(1), waypoints.get(2), Waypoint.FIRST));
         waypoints.get(numberOfWaypoints - 1).setVelocity(computeWaypointVelocity(waypoints.get(numberOfWaypoints - 3), waypoints.get(numberOfWaypoints - 2),
               waypoints.get(numberOfWaypoints - 1), Waypoint.THIRD));
      }

      for (int i = 1; i < numberOfWaypoints - 1; i++)
      {
         Waypoint1DInterface firstWaypoint = waypoints.get(i - 1);
         Waypoint1DInterface secondWaypoint = waypoints.get(i);
         Waypoint1DInterface thirdWaypoint = waypoints.get(i + 1);
         waypoints.get(i).setVelocity(computeWaypointVelocity(firstWaypoint, secondWaypoint, thirdWaypoint, Waypoint.SECOND));
      }
   }

   private double compute2ndWaypointVelocityWithVelocityConstraint(Waypoint1DInterface firstWaypoint, Waypoint1DInterface secondWaypoint,
         Waypoint1DInterface thirdWaypoint)
   {
      double t0 = firstWaypoint.getTime();
      double z0 = firstWaypoint.getPosition();
      double zd0 = firstWaypoint.getVelocity();

      double tIntermediate = secondWaypoint.getTime();
      double zIntermediate = secondWaypoint.getPosition();

      double tf = thirdWaypoint.getTime();
      double zf = thirdWaypoint.getPosition();
      double zdf = thirdWaypoint.getVelocity();

      polynomial.setQuarticUsingWayPoint(t0, tIntermediate, tf, z0, zd0, zIntermediate, zf, zdf);
      polynomial.compute(tIntermediate);

      return polynomial.getVelocity();
   }

   private enum Waypoint
   {
      FIRST, SECOND, THIRD
   };

   private double computeWaypointVelocity(Waypoint1DInterface firstWaypoint, Waypoint1DInterface secondWaypoint, Waypoint1DInterface thirdWaypoint,
         Waypoint waypointToComputeVelocityOf)
   {
      double t0 = firstWaypoint.getTime();
      double z0 = firstWaypoint.getPosition();

      double tIntermediate = secondWaypoint.getTime();
      double zIntermediate = secondWaypoint.getPosition();

      double tf = thirdWaypoint.getTime();
      double zf = thirdWaypoint.getPosition();

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
      return waypoints.size();
   }

   public RecyclingArrayList<? extends Waypoint1DInterface> getWaypoints()
   {
      return waypoints;
   }
}
