package us.ihmc.robotics.math.trajectories.generators;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.math3.util.Precision;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.robotics.math.trajectories.trajectorypoints.lists.OneDoFTrajectoryPointList;

public class OneDoFTrajectoryPointCalculator
{
   private static final int maxIterations = 2000;
   private final TrajectoryPointOptimizer trajectoryPointOptimizer = new TrajectoryPointOptimizer(1);

   private final OneDoFTrajectoryPointList trajectory = new OneDoFTrajectoryPointList();

   private final TDoubleArrayList positions = new TDoubleArrayList();
   private final TDoubleArrayList velocities = new TDoubleArrayList();
   private final TDoubleArrayList times = new TDoubleArrayList();

   public void clear()
   {
      positions.clear();
      times.clear();
      velocities.clear();
      trajectory.clear();
   }

   public void appendTrajectoryPoint(double position)
   {
      positions.add(position);
   }

   public void appendTrajectoryPoint(double time, double position)
   {
      positions.add(position);
      times.add(time);
   }

   public void compute(double trajectoryTime)
   {
      if (!velocities.isEmpty())
      {
         throw new RuntimeException("Was already computed. Need to clear and readd points before computing again.");
      }

      TDoubleArrayList startPosition = new TDoubleArrayList(new double[] {positions.get(0)});
      TDoubleArrayList startVelocity = new TDoubleArrayList(new double[] {0.0});
      TDoubleArrayList finalPosition = new TDoubleArrayList(new double[] {positions.get(positions.size() - 1)});
      TDoubleArrayList finalVelocity = new TDoubleArrayList(new double[] {0.0});

      List<TDoubleArrayList> waypoints = new ArrayList<>();
      for (int i = 1; i < positions.size() - 1; i++)
      {
         waypoints.add(new TDoubleArrayList(new double[] {positions.get(i)}));
      }

      trajectoryPointOptimizer.setEndPoints(startPosition, startVelocity, finalPosition, finalVelocity);
      trajectoryPointOptimizer.setWaypoints(waypoints);

      if (times.isEmpty())
      {
         computeIncludingTimes();
      }
      else
      {
         computeForFixedTime(trajectoryTime);
      }
      
      times.clear();
      times.add(0.0);
      velocities.add(0.0);
      TDoubleArrayList velocityToPack = new TDoubleArrayList();
      for (int i = 0; i < waypoints.size(); i++)
      {
         times.add(trajectoryPointOptimizer.getWaypointTime(i) * trajectoryTime);
         trajectoryPointOptimizer.getWaypointVelocity(velocityToPack, i);
         velocities.add(velocityToPack.get(0) / trajectoryTime);
      }
      times.add(trajectoryTime);
      velocities.add(0.0);

      for (int i = 0; i < positions.size(); i++)
      {
         double time = times.get(i);
         double position = positions.get(i);
         double velocity = velocities.get(i);
         trajectory.addTrajectoryPoint(time, position, velocity);
      }
   }

   private void computeForFixedTime(double trajectoryTime)
   {
      if (times.size() != positions.size())
      {
         throw new RuntimeException("If providing times provide one for each position waypoint!");
      }
      if (!Precision.equals(times.get(0), 0.0, Double.MIN_VALUE))
      {
         throw new RuntimeException("First time must be zero. Offset your trajectory later!");
      }
      if (!Precision.equals(times.get(times.size() - 1), trajectoryTime, Double.MIN_VALUE))
      {
         throw new RuntimeException("Last waypoint time must match the trajectory time!");
      }

      TDoubleArrayList waypointTimes = new TDoubleArrayList(times.subList(1, times.size() - 1));
      waypointTimes.transformValues(time -> time / trajectoryTime);
      trajectoryPointOptimizer.computeForFixedTime(waypointTimes);

   }

   private void computeIncludingTimes()
   {
      trajectoryPointOptimizer.compute(maxIterations);
   }

   public int getNumberOfTrajectoryPoints()
   {
      return positions.size();
   }

   public OneDoFTrajectoryPointList getTrajectoryData()
   {
      return trajectory;
   }
}
