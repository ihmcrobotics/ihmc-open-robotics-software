package us.ihmc.robotics.math.trajectories.generators;

import gnu.trove.list.array.TDoubleArrayList;
import org.apache.commons.math3.util.Precision;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.math.trajectories.trajectorypoints.FrameEuclideanTrajectoryPoint;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.EuclideanTrajectoryPointBasics;
import us.ihmc.robotics.math.trajectories.trajectorypoints.lists.FrameEuclideanTrajectoryPointList;

import java.util.ArrayList;
import java.util.List;

public class RealtimeEuclideanTrajectoryPointCalculator
{
   private static final int dimension = Axis3D.values.length;
   private static final int maxIterations = 2000;
   private final TrajectoryPointOptimizer trajectoryPointOptimizer = new TrajectoryPointOptimizer(dimension);

   private final FrameEuclideanTrajectoryPointList trajectoryPoints = new FrameEuclideanTrajectoryPointList();
   private final TDoubleArrayList times = new TDoubleArrayList();
   private final TDoubleArrayList transformedTimes = new TDoubleArrayList();

   private final TDoubleArrayList initialPositionArray = new TDoubleArrayList(dimension);
   private final TDoubleArrayList initialVelocityArray = new TDoubleArrayList(dimension);
   private final TDoubleArrayList finalPositionArray = new TDoubleArrayList(dimension);
   private final TDoubleArrayList finalVelocityArray = new TDoubleArrayList(dimension);
   private final TDoubleArrayList waypointVelocity = new TDoubleArrayList(dimension);

   private final RecyclingArrayList<TDoubleArrayList> waypointPositions;


   public RealtimeEuclideanTrajectoryPointCalculator()
   {
      waypointPositions = new RecyclingArrayList<>(0, () ->
      {
         TDoubleArrayList ret = new TDoubleArrayList(dimension);
         for (int i = 0; i < dimension; i++)
            ret.add(0.0);
         return ret;
      });
   }

   public void clear()
   {
      trajectoryPoints.clear();
      times.clear();
   }

   public void appendTrajectoryPoint(Point3DBasics position)
   {
      FrameEuclideanTrajectoryPoint newTrajectoryPoint = trajectoryPoints.addTrajectoryPoint();
      newTrajectoryPoint.setToZero(trajectoryPoints.getReferenceFrame());
      newTrajectoryPoint.setTimeToNaN();
      newTrajectoryPoint.setPosition(position);
      newTrajectoryPoint.setLinearVelocityToNaN();
   }

   public void appendTrajectoryPoint(double time, Point3DBasics position)
   {
      FrameEuclideanTrajectoryPoint newTrajectoryPoint = trajectoryPoints.addTrajectoryPoint();
      newTrajectoryPoint.setToZero(trajectoryPoints.getReferenceFrame());
      newTrajectoryPoint.setTime(time);
      newTrajectoryPoint.setPosition(position);
      newTrajectoryPoint.setLinearVelocityToNaN();
      times.add(time);
   }

   public void changeFrame(ReferenceFrame referenceFrame)
   {
      trajectoryPoints.changeFrame(referenceFrame);
   }

//   public boolean doOptimizationUpdate()
//   {
//      if (!hasConverged.getBooleanValue())
//      {
//         hasConverged.set(optimizer.doFullTimeUpdate());
//         updateVariablesFromOptimizer();
//      }
//
//      return !hasConverged();
//   }

   private final TDoubleArrayList velocityArrayTemp = new TDoubleArrayList();
   private final Vector3D velocityVectorTemp = new Vector3D();

   public void compute(double trajectoryTime)
   {
      FrameEuclideanTrajectoryPoint first = trajectoryPoints.getTrajectoryPoint(0);
      FrameEuclideanTrajectoryPoint last = trajectoryPoints.getLastTrajectoryPoint();

      if (first.getLinearVelocity().containsNaN())
         first.setLinearVelocity(0.0, 0.0, 0.0);
      if (last.getLinearVelocity().containsNaN())
         last.setLinearVelocity(0.0, 0.0, 0.0);

      for (int i = 0; i < dimension; i++)
      {
         initialPositionArray.set(i, first.getPosition().getElement(i));
         initialVelocityArray.set(i, first.getLinearVelocity().getElement(i));
         finalPositionArray.set(i, last.getPosition().getElement(i));
         finalVelocityArray.set(i, last.getLinearVelocity().getElement(i));
      }

      waypointPositions.clear();
      for (int i = 1; i < trajectoryPoints.getNumberOfTrajectoryPoints() - 1; i++)
      {
         TDoubleArrayList waypointXYZ = waypointPositions.add();
         for (int j = 0; j < dimension; j++)
         {
            waypointXYZ.add(trajectoryPoints.getTrajectoryPoint(i).getPosition().getElement(j));
         }
      }

      trajectoryPointOptimizer.setEndPoints(initialPositionArray, initialVelocityArray, finalPositionArray, finalVelocityArray);
      trajectoryPointOptimizer.setWaypoints(waypointPositions);

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
      for (int i = 0; i < waypointPositions.size(); i++)
         times.add(trajectoryPointOptimizer.getWaypointTime(i) * trajectoryTime);
      times.add(trajectoryTime);

      for (int i = 0; i < trajectoryPoints.getNumberOfTrajectoryPoints(); i++)
         trajectoryPoints.getTrajectoryPoint(i).setTime(times.get(i));

      for (int i = 0; i < waypointPositions.size(); i++)
      {
         velocityArrayTemp.clear();
         trajectoryPointOptimizer.getWaypointVelocity(velocityArrayTemp, i);

         for (int j = 0; j < velocityArrayTemp.size(); j++)
            velocityVectorTemp.setElement(j, velocityArrayTemp.get(j) / trajectoryTime);

         trajectoryPoints.getTrajectoryPoint(i + 1).setLinearVelocity(velocityVectorTemp);
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

      transformedTimes.clear();
      for (int i = 1; i < times.size() - 1; i++)
         transformedTimes.add(times.get(i));

      trajectoryPointOptimizer.computeForFixedTime(transformedTimes);
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
