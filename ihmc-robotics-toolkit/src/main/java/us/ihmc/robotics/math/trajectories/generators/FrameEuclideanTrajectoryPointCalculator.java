package us.ihmc.robotics.math.trajectories.generators;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.math3.util.Precision;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.Axis;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.robotics.math.trajectories.trajectorypoints.FrameEuclideanTrajectoryPoint;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.EuclideanTrajectoryPointBasics;

public class FrameEuclideanTrajectoryPointCalculator
{
   private ReferenceFrame referenceFrame = ReferenceFrame.getWorldFrame();

   private static final int dimension = Axis.values.length;
   private static final int maxIterations = 2000;
   private final TrajectoryPointOptimizer trajectoryPointOptimizer = new TrajectoryPointOptimizer(1);

   private final RecyclingArrayList<FrameEuclideanTrajectoryPoint> trajectoryPoints = new RecyclingArrayList<>(20, FrameEuclideanTrajectoryPoint.class);
   private final TDoubleArrayList times = new TDoubleArrayList();

   public void clear()
   {
      trajectoryPoints.clear();
      times.clear();
   }

   public void appendTrajectoryPoint(EuclideanTrajectoryPointBasics trajectoryPoint)
   {
      trajectoryPoints.add().set(trajectoryPoint);
   }

   public void appendTrajectoryPoint(Point3DBasics position)
   {
      FrameEuclideanTrajectoryPoint newTrajectoryPoint = trajectoryPoints.add();
      newTrajectoryPoint.setToZero(referenceFrame);
      newTrajectoryPoint.setTimeToNaN();
      newTrajectoryPoint.setPosition(position);
      newTrajectoryPoint.setLinearVelocityToNaN();
   }

   public void appendTrajectoryPoint(double time, Point3DBasics position)
   {
      FrameEuclideanTrajectoryPoint newTrajectoryPoint = trajectoryPoints.add();
      newTrajectoryPoint.setToZero(referenceFrame);
      newTrajectoryPoint.setTime(time);
      newTrajectoryPoint.setPosition(position);
      newTrajectoryPoint.setLinearVelocityToNaN();
   }

   public void appendTrajectoryPoint(double time, Point3DBasics position, Vector3DBasics linearVelocity)
   {
      FrameEuclideanTrajectoryPoint newTrajectoryPoint = trajectoryPoints.add();
      newTrajectoryPoint.setToZero(referenceFrame);
      newTrajectoryPoint.setTime(time);
      newTrajectoryPoint.setPosition(position);
      newTrajectoryPoint.setLinearVelocity(linearVelocity);
   }

   public void changeFrame(ReferenceFrame referenceFrame)
   {
      for (int i = 0; i < getNumberOfTrajectoryPoints(); i++)
         trajectoryPoints.get(i).changeFrame(referenceFrame);
      this.referenceFrame = referenceFrame;
   }

   public void compute(double trajectoryTime)
   {
      TDoubleArrayList startPosition = new TDoubleArrayList();
      TDoubleArrayList startVelocity = new TDoubleArrayList();
      TDoubleArrayList finalPosition = new TDoubleArrayList();
      TDoubleArrayList finalVelocity = new TDoubleArrayList();

      FrameEuclideanTrajectoryPoint first = trajectoryPoints.getFirst();
      FrameEuclideanTrajectoryPoint last = trajectoryPoints.getLast();
      for (int i = 0; i < dimension; i++)
      {
         startPosition.add(first.getPosition().getElement(i));
         startVelocity.add(first.getLinearVelocity().getElement(i));
         finalPosition.add(last.getPosition().getElement(i));
         finalVelocity.add(last.getLinearVelocity().getElement(i));
      }

      List<TDoubleArrayList> waypoints = new ArrayList<>();
      for (int i = 1; i < trajectoryPoints.size() - 1; i++)
      {
         TDoubleArrayList waypointXYZ = new TDoubleArrayList();
         for (int j = 0; j < dimension; j++)
         {
            waypointXYZ.add(trajectoryPoints.get(i).getPosition().getElement(j));
         }
         waypoints.add(waypointXYZ);
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

      times.add(0.0);
      for (int i = 0; i < waypoints.size(); i++)
         times.add(trajectoryPointOptimizer.getWaypointTime(i) * trajectoryTime);
      times.add(trajectoryTime);

      for (int i = 0; i < trajectoryPoints.size(); i++)
      {
         TDoubleArrayList velocityToPack = new TDoubleArrayList();
         trajectoryPointOptimizer.getWaypointVelocity(velocityToPack, i);
         Vector3D waypointVelocity = new Vector3D();

         for (int j = 0; j < velocityToPack.size(); j++)
         {
            waypointVelocity.setElement(j, velocityToPack.get(j) / trajectoryTime);
         }
         trajectoryPoints.get(i).setLinearVelocity(waypointVelocity);
      }
   }

   private void computeForFixedTime(double trajectoryTime)
   {
      if (times.size() != trajectoryPoints.size())
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
      return trajectoryPoints.size();
   }

   public FrameEuclideanTrajectoryPoint getTrajectoryPoint(int i)
   {
      return trajectoryPoints.get(i);
   }

   public RecyclingArrayList<FrameEuclideanTrajectoryPoint> getTrajectoryPoints()
   {
      return trajectoryPoints;
   }
}
