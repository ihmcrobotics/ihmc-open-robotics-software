package us.ihmc.robotics.math.trajectories.generators;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.math3.util.Precision;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.euclid.Axis;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.math.trajectories.trajectorypoints.FrameEuclideanTrajectoryPoint;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.EuclideanTrajectoryPointBasics;
import us.ihmc.robotics.math.trajectories.trajectorypoints.lists.FrameEuclideanTrajectoryPointList;

public class EuclideanTrajectoryPointCalculator
{
   private static final int dimension = Axis.values.length;
   private static final int maxIterations = 2000;
   private final TrajectoryPointOptimizer trajectoryPointOptimizer = new TrajectoryPointOptimizer(dimension);

   private final FrameEuclideanTrajectoryPointList trajectoryPoints = new FrameEuclideanTrajectoryPointList();
   private final TDoubleArrayList times = new TDoubleArrayList();

   public void clear()
   {
      trajectoryPoints.clear();
      times.clear();
   }

   public void appendTrajectoryPoint(EuclideanTrajectoryPointBasics trajectoryPoint)
   {
      trajectoryPoints.addTrajectoryPoint(trajectoryPoint);
   }

   public void appendTrajectoryPoint(Point3DBasics position)
   {
      FrameEuclideanTrajectoryPoint newTrajectoryPoint = new FrameEuclideanTrajectoryPoint();
      newTrajectoryPoint.setToZero(trajectoryPoints.getReferenceFrame());
      newTrajectoryPoint.setTimeToNaN();
      newTrajectoryPoint.setPosition(position);
      newTrajectoryPoint.setLinearVelocityToNaN();
      trajectoryPoints.addTrajectoryPoint(newTrajectoryPoint);
   }

   public void appendTrajectoryPoint(double time, Point3DBasics position)
   {
      FrameEuclideanTrajectoryPoint newTrajectoryPoint = new FrameEuclideanTrajectoryPoint();
      newTrajectoryPoint.setToZero(trajectoryPoints.getReferenceFrame());
      newTrajectoryPoint.setTime(time);
      newTrajectoryPoint.setPosition(position);
      newTrajectoryPoint.setLinearVelocityToNaN();
      trajectoryPoints.addTrajectoryPoint(newTrajectoryPoint);
      times.add(time);
   }

   public void appendTrajectoryPoint(double time, Point3DBasics position, Vector3DBasics linearVelocity)
   {
      FrameEuclideanTrajectoryPoint newTrajectoryPoint = new FrameEuclideanTrajectoryPoint();
      newTrajectoryPoint.setToZero(trajectoryPoints.getReferenceFrame());
      newTrajectoryPoint.setTime(time);
      newTrajectoryPoint.setPosition(position);
      newTrajectoryPoint.setLinearVelocity(linearVelocity);
      trajectoryPoints.addTrajectoryPoint(newTrajectoryPoint);
      times.add(time);
   }

   public void changeFrame(ReferenceFrame referenceFrame)
   {
      for (int i = 0; i < getNumberOfTrajectoryPoints(); i++)
         trajectoryPoints.getTrajectoryPoint(i).changeFrame(referenceFrame);
      trajectoryPoints.setReferenceFrame(referenceFrame);
   }

   public void compute(double trajectoryTime)
   {
      TDoubleArrayList startPosition = new TDoubleArrayList();
      TDoubleArrayList startVelocity = new TDoubleArrayList();
      TDoubleArrayList finalPosition = new TDoubleArrayList();
      TDoubleArrayList finalVelocity = new TDoubleArrayList();

      FrameEuclideanTrajectoryPoint first = trajectoryPoints.getTrajectoryPoint(0);
      FrameEuclideanTrajectoryPoint last = trajectoryPoints.getLastTrajectoryPoint();

      if (first.getLinearVelocity().containsNaN())
         first.setLinearVelocity(0.0, 0.0, 0.0);
      if (last.getLinearVelocity().containsNaN())
         last.setLinearVelocity(0.0, 0.0, 0.0);

      for (int i = 0; i < dimension; i++)
      {
         startPosition.add(first.getPosition().getElement(i));
         startVelocity.add(first.getLinearVelocity().getElement(i));
         finalPosition.add(last.getPosition().getElement(i));
         finalVelocity.add(last.getLinearVelocity().getElement(i));
      }

      List<TDoubleArrayList> waypoints = new ArrayList<TDoubleArrayList>();
      for (int i = 1; i < trajectoryPoints.getNumberOfTrajectoryPoints() - 1; i++)
      {
         TDoubleArrayList waypointXYZ = new TDoubleArrayList();
         for (int j = 0; j < dimension; j++)
         {
            waypointXYZ.add(trajectoryPoints.getTrajectoryPoint(i).getPosition().getElement(j));
         }
         waypoints.add(waypointXYZ);
      }

      trajectoryPointOptimizer.setEndPoints(startPosition, startVelocity, finalPosition, finalVelocity);
      trajectoryPointOptimizer.setWaypoints(waypoints);

      if (times.size() == 0)
      {
         computeIncludingTimes();
      }
      else
      {
         computeForFixedTime(trajectoryTime);
      }

      times.clear();
      times.add(0.0);
      for (int i = 0; i < waypoints.size(); i++)
         times.add(trajectoryPointOptimizer.getWaypointTime(i) * trajectoryTime);
      times.add(trajectoryTime);

      for (int i = 0; i < trajectoryPoints.getNumberOfTrajectoryPoints(); i++)
         trajectoryPoints.getTrajectoryPoint(i).setTime(times.get(i));

      for (int i = 0; i < waypoints.size(); i++)
      {
         TDoubleArrayList velocityToPack = new TDoubleArrayList();
         trajectoryPointOptimizer.getWaypointVelocity(velocityToPack, i);
         Vector3D waypointVelocity = new Vector3D();

         for (int j = 0; j < velocityToPack.size(); j++)
            waypointVelocity.setElement(j, velocityToPack.get(j) / trajectoryTime);

         trajectoryPoints.getTrajectoryPoint(i + 1).setLinearVelocity(waypointVelocity);
      }
   }

   private void computeForFixedTime(double trajectoryTime)
   {
      if (times.size() != trajectoryPoints.getNumberOfTrajectoryPoints())
      {
         LogTools.warn("If providing times provide one for each position waypoint!");
         throw new RuntimeException("If providing times provide one for each position waypoint!");
      }
      if (!Precision.equals(times.get(0), 0.0, Double.MIN_VALUE))
      {
         LogTools.warn("First time must be zero. Offset your trajectory later!");
         throw new RuntimeException("First time must be zero. Offset your trajectory later!");
      }
      if (!Precision.equals(times.get(times.size() - 1), trajectoryTime, Double.MIN_VALUE))
      {
         LogTools.warn("Last waypoint time must match the trajectory time!");
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
      return trajectoryPoints.getNumberOfTrajectoryPoints();
   }

   public FrameEuclideanTrajectoryPoint getTrajectoryPoint(int i)
   {
      return trajectoryPoints.getTrajectoryPoint(i);
   }

   public FrameEuclideanTrajectoryPointList getTrajectoryPoints()
   {
      return trajectoryPoints;
   }
}
