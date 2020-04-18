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
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.ArrayList;
import java.util.List;

public class RealtimeEuclideanTrajectoryPointCalculator
{
   private final YoVariableRegistry registry;
   private static final int dimension = Axis3D.values.length;
   private static final int defaultMaxIterations = 2000;
   private final TrajectoryPointOptimizer trajectoryPointOptimizer;

   private final FrameEuclideanTrajectoryPointList trajectoryPoints = new FrameEuclideanTrajectoryPointList();
   private final TDoubleArrayList times = new TDoubleArrayList();
   private final TDoubleArrayList transformedTimes = new TDoubleArrayList();

   private final TDoubleArrayList initialPositionArray = new TDoubleArrayList(dimension);
   private final TDoubleArrayList initialVelocityArray = new TDoubleArrayList(dimension);
   private final TDoubleArrayList finalPositionArray = new TDoubleArrayList(dimension);
   private final TDoubleArrayList finalVelocityArray = new TDoubleArrayList(dimension);
   private final TDoubleArrayList waypointVelocity = new TDoubleArrayList(dimension);

   private final TDoubleArrayList velocityArrayTemp = new TDoubleArrayList();
   private final Vector3D velocityVectorTemp = new Vector3D();

   private final RecyclingArrayList<TDoubleArrayList> waypointPositions;

   private final YoInteger maxIterations;
   private final YoBoolean hasConverged;
   private final YoBoolean optimizeInOneTick;

   public RealtimeEuclideanTrajectoryPointCalculator(String namePrefix, YoVariableRegistry parentRegistry)
   {
      this(namePrefix, defaultMaxIterations, parentRegistry);
   }

   public RealtimeEuclideanTrajectoryPointCalculator(String namePrefix, int maxIterations, YoVariableRegistry parentRegistry)
   {
      registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());

      trajectoryPointOptimizer = new TrajectoryPointOptimizer(namePrefix, dimension, registry);
      hasConverged = new YoBoolean(namePrefix + "HasConverged", registry);
      optimizeInOneTick = new YoBoolean(namePrefix + "OptimizeInOneTick", registry);

      this.maxIterations = new YoInteger(namePrefix + "MaxIterations", registry);
      this.maxIterations.set(maxIterations);

      waypointPositions = new RecyclingArrayList<>(0, () ->
      {
         TDoubleArrayList ret = new TDoubleArrayList(dimension);
         for (int i = 0; i < dimension; i++)
            ret.add(0.0);
         return ret;
      });

      parentRegistry.addChild(registry);
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

   public boolean doOptimizationUpdate(double trajectoryDuration)
   {
      if (!hasConverged.getBooleanValue())
      {
         hasConverged.set(trajectoryPointOptimizer.doFullTimeUpdate());
         updateVariablesFromOptimizer(trajectoryDuration);
      }

      return !hasConverged();
   }

   public void initialize(double trajectoryDuration)
   {
      if (trajectoryPoints.getTrajectoryPoint(0).getPosition().containsNaN())
         throw new RuntimeException("Does not have valid endpoint conditions. Did you call setEndpointConditions?");

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


      if (optimizeInOneTick.getBooleanValue())
      {
         compute(maxIterations.getIntegerValue(), trajectoryDuration);
         hasConverged.set(true);
      }
      else
      {
         hasConverged.set(false);
         compute(0, trajectoryDuration);
         trajectoryPointOptimizer.compute(0);
      }

      updateVariablesFromOptimizer(trajectoryDuration);
   }

   private void compute(int maxIterations, double trajectoryDuration)
   {
      if (times.size() == 0)
      {
         computeIncludingTimes(maxIterations);
      }
      else
      {
         computeForFixedTime(maxIterations, trajectoryDuration);
      }
   }

   private void updateVariablesFromOptimizer(double trajectoryDuration)
   {
      times.clear();
      times.add(0.0);
      for (int i = 0; i < waypointPositions.size(); i++)
         times.add(trajectoryPointOptimizer.getWaypointTime(i) * trajectoryDuration);
      times.add(trajectoryDuration);

      for (int i = 0; i < trajectoryPoints.getNumberOfTrajectoryPoints(); i++)
         trajectoryPoints.getTrajectoryPoint(i).setTime(times.get(i));

      for (int i = 0; i < waypointPositions.size(); i++)
      {
         velocityArrayTemp.clear();
         trajectoryPointOptimizer.getWaypointVelocity(velocityArrayTemp, i);

         for (int j = 0; j < velocityArrayTemp.size(); j++)
            velocityVectorTemp.setElement(j, velocityArrayTemp.get(j) / trajectoryDuration);

         trajectoryPoints.getTrajectoryPoint(i + 1).setLinearVelocity(velocityVectorTemp);
      }
   }

   private void computeForFixedTime(int maxIterations, double trajectoryTime)
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

      trajectoryPointOptimizer.compute(maxIterations, transformedTimes);
   }

   private void computeIncludingTimes(int maxIterations)
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

   /**
    * Returns whether the trajectory optimization has converged or not. This is useful when continuously improving
    * the solution quality instead of waiting for the optimizer to finish in the initialize method.
    *
    * @return whether the optimizer has converged or not
    */
   public boolean hasConverged()
   {
      return hasConverged.getBooleanValue();
   }
}
