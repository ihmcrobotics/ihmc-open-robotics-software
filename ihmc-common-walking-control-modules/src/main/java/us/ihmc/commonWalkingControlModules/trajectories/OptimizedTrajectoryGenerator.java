package us.ihmc.commonWalkingControlModules.trajectories;

import gnu.trove.list.array.TDoubleArrayList;
import gnu.trove.map.TIntIntMap;
import gnu.trove.map.hash.TIntIntHashMap;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.robotics.math.trajectories.YoPolynomial;
import us.ihmc.robotics.math.trajectories.generators.TrajectoryPointOptimizer;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.ArrayList;

/**
 * This class is a wrapper for the TrajectoryPointOptimizer. It was made for trajectories in 1d
 * space and creates third order trajectories. It can be used either to generate waypoint times as
 * velocities, or it can serve as an actual trajectory with the optimization taking place when the
 * trajectory is initialized.
 *
 * The trajectory is continuous in acceleration but does not have zero initial and final
 * acceleration. The optimization finds waypoint times and velocities such that the overall squared
 * acceleration is minimized.
 *
 * @author rgriffin
 *
 */
public class OptimizedTrajectoryGenerator
{
   private static final int dimensions = 1;
   private final String namePrefix;

   private final TrajectoryPointOptimizer optimizer;
   private final YoInteger maxIterations;
   private final RecyclingArrayList<TDoubleArrayList> coefficients;
   private final ArrayList<YoPolynomial> trajectories = new ArrayList<>();
   private final double[] tempCoeffs = new double[TrajectoryPointOptimizer.coefficients];

   private double initialPosition;
   private double initialVelocity;
   private double finalPosition;
   private double finalVelocity;
   private final RecyclingArrayList<TDoubleArrayList> waypointPositions;
   private final TIntIntMap indexMap = new TIntIntHashMap(10, 0.5f, -1, -1);

   private final TDoubleArrayList initialPositionArray = new TDoubleArrayList(dimensions);
   private final TDoubleArrayList initialVelocityArray = new TDoubleArrayList(dimensions);
   private final TDoubleArrayList finalPositionArray = new TDoubleArrayList(dimensions);
   private final TDoubleArrayList finalVelocityArray = new TDoubleArrayList(dimensions);
   private final TDoubleArrayList waypointVelocity = new TDoubleArrayList(dimensions);
   private final TDoubleArrayList waypointTimesArray = new TDoubleArrayList();


   private final YoRegistry registry;
   private final YoBoolean isDone;
   private final YoBoolean optimizeInOneTick;
   private final YoBoolean hasConverged;
   private final YoInteger segments;
   private final YoInteger activeSegment;
   private final ArrayList<YoDouble> waypointTimes = new ArrayList<>();

   private final YoDouble desiredPosition;
   private final YoDouble desiredVelocity;
   private final YoDouble desiredAcceleration;
   private final YoDouble desiredJerk;

   private final YoDouble maxSpeed;
   private final YoDouble maxSpeedTime;

   public OptimizedTrajectoryGenerator(String namePrefix, int maxIterations, int maxWaypoints, YoRegistry parentRegistry)
   {
      this.namePrefix = namePrefix;

      coefficients = new RecyclingArrayList<>(0, () ->
      {
         TDoubleArrayList ret = new TDoubleArrayList(TrajectoryPointOptimizer.coefficients);
         for (int i = 0; i < TrajectoryPointOptimizer.coefficients; i++)
            ret.add(0.0);
         return ret;
      });

      waypointPositions = new RecyclingArrayList<>(0, () ->
      {
         TDoubleArrayList ret = new TDoubleArrayList(dimensions);
         for (int i = 0; i < dimensions; i++)
            ret.add(0.0);
         return ret;
      });

      registry = new YoRegistry(namePrefix + "Trajectory");
      optimizer = new TrajectoryPointOptimizer(namePrefix, dimensions, registry);
      this.maxIterations = new YoInteger(namePrefix + "MaxIterations", registry);
      this.maxIterations.set(maxIterations);
      isDone = new YoBoolean(namePrefix + "IsDone", registry);
      optimizeInOneTick = new YoBoolean(namePrefix + "OptimizeInOneTick", registry);
      hasConverged = new YoBoolean(namePrefix + "HasConverged", registry);
      segments = new YoInteger(namePrefix + "Segments", registry);
      activeSegment = new YoInteger(namePrefix + "ActiveSegment", registry);

      optimizeInOneTick.set(maxIterations >= 0);
      hasConverged.set(optimizeInOneTick.getBooleanValue());

      maxSpeed = new YoDouble("MaxVelocity", registry);
      maxSpeedTime = new YoDouble("MaxVelocityTime", registry);

      desiredPosition = new YoDouble(namePrefix + "DesiredPosition", registry);
      desiredVelocity = new YoDouble(namePrefix + "DesiredVelocity", registry);
      desiredAcceleration = new YoDouble(namePrefix + "DesiredAcceleration", registry);
      desiredJerk = new YoDouble(namePrefix + "DesiredJerk", registry);

      for (int i = 0; i < dimensions; i++)
      {
         initialPositionArray.add(0.0);
         initialVelocityArray.add(0.0);
         finalPositionArray.add(0.0);
         finalVelocityArray.add(0.0);
      }

      while (waypointTimes.size() <= maxWaypoints)
         extendBySegment(registry);

      reset();
      if (parentRegistry != null)
         parentRegistry.addChild(registry);
   }

   private void extendBySegment(YoRegistry registry)
   {
      int size = waypointTimes.size() + 1;
      trajectories.add(new YoPolynomial(namePrefix + "Segment" + size, TrajectoryPointOptimizer.coefficients, registry));
      waypointTimes.add(new YoDouble(namePrefix + "WaypointTime" + size, registry));
      waypointPositions.add();
   }

   /**
    * Resets the optimizer and removes all waypoints as well as previous start and end conditions.
    */
   public void reset()
   {
      initialPosition = Double.NaN;
      initialVelocity = Double.NaN;
      finalPosition = Double.NaN;
      finalVelocity = Double.NaN;
      segments.set(1);

      this.waypointPositions.clear();
      optimizer.setWaypoints(waypointPositions);
      coefficients.clear();
      coefficients.add();

      waypointTimesArray.reset();
      for (int i = 0; i < waypointTimes.size(); i++)
         waypointTimes.get(i).setToNaN();
   }

   /**
    * Set the desired position and velocity at the start and end points of the trajectory.
    *
    * @param initialPosition
    * @param initialVelocity
    * @param finalPosition
    * @param finalVelocity
    */
   public void setEndpointConditions(double initialPosition, double initialVelocity, double finalPosition, double finalVelocity)
   {
      this.initialPosition = initialPosition;
      this.initialVelocity = initialVelocity;

      this.finalPosition = finalPosition;
      this.finalVelocity = finalVelocity;

      initialPositionArray.set(0, this.initialPosition);
      initialVelocityArray.set(0, this.initialVelocity);
      finalPositionArray.set(0, this.finalPosition);
      finalVelocityArray.set(0, this.finalVelocity);

      optimizer.setEndPoints(initialPositionArray, initialVelocityArray, finalPositionArray, finalVelocityArray);
   }

   /**
    * Set the positions of the waypoints on the trajectory.
    *
    * @param waypointPositions
    */
   public void setWaypoints(TDoubleArrayList waypointPositions)
   {
      if (waypointPositions.size() > waypointTimes.size())
         throw new RuntimeException("Too many waypoints");

      this.waypointPositions.clear();
      coefficients.clear();
      indexMap.clear();

      coefficients.add();
      int optimizerIndex = 0;
      for (int i = 0; i < waypointPositions.size(); i++)
      {
         double waypointPosition = waypointPositions.get(i);

         indexMap.put(i, optimizerIndex);
         optimizerIndex++;

         TDoubleArrayList waypoint = this.waypointPositions.add();
         waypoint.set(0, waypointPosition);
         coefficients.add();
      }

      optimizer.setWaypoints(this.waypointPositions);
      segments.set(coefficients.size());
   }

   public void setWaypointTimes(TDoubleArrayList waypointTimes)
   {
      if (waypointTimes.size() > this.waypointTimes.size())
         throw new RuntimeException("Too many waypoints");

      for (int i = 0; i < waypointTimes.size() - 1; i++)
      {
         if (waypointTimes.get(i + 1) < waypointTimes.get(i))
            throw new RuntimeException("Waypoint times aren't in ascending order.");
      }

      for (int i = 0; i < waypointTimes.size(); i++)
      {
         double waypointTime = waypointTimes.get(i);
         this.waypointTimes.get(i).set(waypointTime);
         waypointTimesArray.add(waypointTime);
      }
   }

   /**
    * This method initialized the trajectory and does the optimization. This has to be called after
    * setting the end point conditions and waypoints. It has to be called regardless of whether is
    * class is used as an actual trajectory or just to compute optimal waypoint times and
    * velocities.
    */
   public void initialize()
   {
      if (Double.isNaN(initialPosition))
         throw new RuntimeException("Does not have valid endpoint conditions. Did you call setEndpointConditions?");

      if (optimizeInOneTick.getBooleanValue())
      {
         optimizer.compute(maxIterations.getIntegerValue(), waypointTimesArray);
         hasConverged.set(true);
      }
      else
      {
         hasConverged.set(false);
         optimizer.compute(0, waypointTimesArray);
      }

      updateVariablesFromOptimizer();
   }

   private void updateVariablesFromOptimizer()
   {
      for (int i = 0; i < segments.getIntegerValue() - 1; i++)
         waypointTimes.get(i).set(optimizer.getWaypointTime(i));

      waypointTimes.get(segments.getIntegerValue() - 1).set(1.0);

      for (int i = segments.getIntegerValue(); i < waypointTimes.size(); i++)
         waypointTimes.get(i).set(Double.NaN);

      optimizer.getPolynomialCoefficients(coefficients, 0);
      for (int i = 0; i < segments.getIntegerValue(); i++)
      {
         coefficients.get(i).toArray(tempCoeffs);
         trajectories.get(i).setDirectlyReverse(tempCoeffs);
      }

      isDone.set(false);
//      visualize();
   }

   /*
   private void visualize()
   {
      if (trajectoryViz == null)
         return;

      trajectoryViz.showGraphic();
   }
    */

   /**
    * Attempt at improving the trajectory if iterative improvement is desired.
    * @return whether an optimization step was done or not.
    */
   public boolean doOptimizationUpdate()
   {
      if (!hasConverged.getBooleanValue())
      {
         hasConverged.set(optimizer.doFullTimeUpdate());
         updateVariablesFromOptimizer();
      }

      return !hasConverged();
   }

   /**
    * Evaluates the trajectory at the given dimensionless time. Time is assumed to go from 0.0 at
    * the start of the trajectory to 1.0 at the end.
    *
    * @param time
    */
   public void compute(double time)
   {
      doOptimizationUpdate();
      isDone.set(time > 1.0);

      if (time < 0.0)
      {
         desiredPosition.set(initialPosition);
         desiredVelocity.set(0.0);
         desiredAcceleration.set(0.0);
         desiredJerk.set(0.0);
         return;
      }
      if (time > 1.0)
      {
         desiredPosition.set(finalPosition);
         desiredVelocity.set(0.0);
         desiredAcceleration.set(0.0);
         desiredJerk.set(0.0);
         return;
      }

      time = MathTools.clamp(time, 0.0, 1.0);

      int activeSegment = 0;
      for (int i = 0; i < segments.getIntegerValue() - 1; i++)
      {
         double waypointTime = waypointTimes.get(i).getDoubleValue();
         if (time > waypointTime)
            activeSegment = i + 1;
         else
            break;
      }
      this.activeSegment.set(activeSegment);

      YoPolynomial polynomial = trajectories.get(activeSegment);
      polynomial.compute(time);
      desiredPosition.set(polynomial.getPosition());
      desiredVelocity.set(polynomial.getVelocity());
      desiredAcceleration.set(polynomial.getAcceleration());
      desiredJerk.set(polynomial.getJerk());
   }

   /**
    * Call this function after initialize to retrieve the optimal waypoint time for a given waypoint
    * index.
    *
    * @param waypointIndex
    * @return
    */
   public double getWaypointTime(int waypointIndex)
   {
      return optimizer.getWaypointTime(indexMap.get(waypointIndex));
   }

   /**
    * Call this function after initialize to retrieve the optimal waypoint velocity for a given
    * waypoint index.
    *
    * @param waypointIndex
    */
   public double getWaypointVelocity(int waypointIndex)
   {
      optimizer.getWaypointVelocity(this.waypointVelocity, indexMap.get(waypointIndex));
      return this.waypointVelocity.get(0);
   }

   public boolean isDone()
   {
      return isDone.getBooleanValue();
   }

   public double getPosition()
   {
      return desiredPosition.getDoubleValue();
   }

   public double getVelocity()
   {
      double duration = waypointTimes.get(waypointTimes.size() - 1).getDoubleValue();
      return desiredVelocity.getDoubleValue() / duration;
   }

   public double getAcceleration()
   {
      double squaredDuration = MathTools.square(waypointTimes.get(waypointTimes.size() - 1).getDoubleValue());
      return desiredAcceleration.getDoubleValue() / squaredDuration;
   }

   public double getJerk()
   {
      double duration = waypointTimes.get(waypointTimes.size() - 1).getDoubleValue();
      double cubedDuration = MathTools.pow(duration, 3);
      return desiredJerk.getDoubleValue() / cubedDuration;
   }


   public void informDone()
   {
      desiredPosition.set(0.0);
      desiredVelocity.set(0.0);
      desiredAcceleration.set(0.0);
   }

   /*
   public void showVisualization()
   {
      if (trajectoryViz == null)
         return;
      trajectoryViz.showGraphic();
   }

   public void hideVisualization()
   {
      if (trajectoryViz == null)
         return;
      trajectoryViz.hideGraphic();
   }
    */

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

   /**
    * Numerically compute the maximum speed along the trajectory and the time at which this speed
    * occurs.
    */
   public void computeMaxSpeed()
   {
      computeMaxSpeed(1.0E-5);
   }

   /**
    * Numerically compute the maximum speed along the trajectory and the time at which this speed
    * occurs. The time precision can be specified.
    */
   public void computeMaxSpeed(double timeIncrement)
   {
      maxSpeed.set(Double.NEGATIVE_INFINITY);
      maxSpeedTime.set(Double.NaN);

      for (double time = 0.0; time <= 1.0; time += timeIncrement)
      {
         compute(time);
         double speed = getVelocity();
         if (speed > maxSpeed.getDoubleValue())
         {
            maxSpeed.set(speed);
            maxSpeedTime.set(time);
         }
      }
   }

   public double getMaxSpeed()
   {
      return maxSpeed.getDoubleValue();
   }

   public double getMaxSpeedTime()
   {
      return maxSpeedTime.getDoubleValue();
   }
}
