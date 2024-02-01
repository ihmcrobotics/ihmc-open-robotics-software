package us.ihmc.robotics.math.trajectories.generators;

import java.util.ArrayList;
import java.util.List;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

/**
 * The TrajectoryPointOptimizer computes an optimal (minimal integrated acceleration) trajectory.
 * <p>
 * Given start and end point position and velocity as well as waypoint positions (optional) this
 * optimizer finds velocities, accelerations, and times at the waypoints that minimize the overall
 * integrated acceleration along the trajectory.
 * </p>
 * <p>
 * If a third order trajectory is requested the acceleration will not be zero at start and end but
 * be continuous at the waypoints. For a fifth order polynomial the start and end accelerations will
 * be zero. Time is assumed to be without dimension and goes from 0.0 at the start to 1.0 at the end
 * of the trajectory.
 * </p>
 * <p>
 * The trajectory times are found through an iterative process using a gradient descent. The
 * compute() method can be called with an optional argument specifying the maximal amount of
 * iterations if the runtime is critical. The waypoint times are initialized to be evenly
 * distributed in the interval 0.0 to 1.0. The maximum number of time optimization steps can be set
 * to zero.
 * </p>
 * <p>
 * The class can return waypoint times, polynomial coefficients for the trajectory segments, and
 * optimal velocities and accelerations at the knot points of the trajectory.
 * </p>
 * <p>
 * Algorithm based on "Minimum Snap Trajectory Generation and Control for Quadrotors" - Mellinger
 * </p>
 * <p>
 * 
 * @author gwiedebach
 */
public class TrajectoryPointOptimizer
{
   public static final int maxWaypoints = 200;
   public static final int maxIterations = 200;

   private static final double epsilon = 1E-7;

   private static final double initialTimeGain = 0.001;
   private static final double costEpsilon = 0.01;

   public static final int coefficients = MultiCubicSpline1DSolver.coefficients;

   private final YoRegistry registry;

   private final YoInteger dimensions;
   private final YoInteger nWaypoints;
   private final YoInteger intervals;
   private final YoInteger problemSize;
   private final YoInteger iteration;

   private final TDoubleArrayList x0, x1, xd0, xd1;
   private final ArrayList<DMatrixRMaj> waypoints = new ArrayList<>();
   private final MultiCubicSpline1DSolver solver;
   private final TDoubleArrayList w0, w1, wd0, wd1;

   private final DMatrixRMaj intervalTimes = new DMatrixRMaj(1, 1);
   private final DMatrixRMaj saveIntervalTimes = new DMatrixRMaj(1, 1);
   private final TDoubleArrayList costs = new TDoubleArrayList(maxIterations + 1);

   private final RecyclingArrayList<DMatrixRMaj> x = new RecyclingArrayList<>(0, () -> new DMatrixRMaj(1, 1));

   private final DMatrixRMaj timeGradient = new DMatrixRMaj(1, 1);
   private final DMatrixRMaj timeUpdate = new DMatrixRMaj(1, 1);
   private final YoDouble timeGain;

   private final ExecutionTimer computeTimer;
   private final ExecutionTimer timeUpdateTimer;

   private final DMatrixRMaj tempCoeffs = new DMatrixRMaj(1, 1);
   private final DMatrixRMaj tempLine = new DMatrixRMaj(1, 1);

   public TrajectoryPointOptimizer(int dimensions, YoRegistry parentRegistry)
   {
      this("", dimensions, parentRegistry);
   }

   public TrajectoryPointOptimizer(String namePrefix, int dimensions, YoRegistry parentRegistry)
   {
      this(namePrefix, dimensions);
      parentRegistry.addChild(registry);
   }

   public TrajectoryPointOptimizer(int dimensions)
   {
      this("", dimensions);
   }

   public TrajectoryPointOptimizer(String namePrefix, int dimensions)
   {
      this.registry = new YoRegistry(namePrefix + getClass().getSimpleName());
      this.dimensions = new YoInteger(namePrefix + "Dimensions", registry);
      this.nWaypoints = new YoInteger(namePrefix + "NumberOfWaypoints", registry);
      this.intervals = new YoInteger(namePrefix + "NumberOfIntervals", registry);
      this.problemSize = new YoInteger(namePrefix + "ProblemSize", registry);
      this.iteration = new YoInteger(namePrefix + "Iteration", registry);
      this.computeTimer = new ExecutionTimer(namePrefix + "ComputeTimer", 0.0, registry);
      this.timeUpdateTimer = new ExecutionTimer(namePrefix + "TimeUpdateTimer", 0.0, registry);
      this.timeGain = new YoDouble(namePrefix + "TimeGain", registry);

      solver = new MultiCubicSpline1DSolver();

      dimensions = Math.max(dimensions, 0);
      this.dimensions.set(dimensions);
      timeGain.set(initialTimeGain);

      x0 = new TDoubleArrayList(dimensions);
      x1 = new TDoubleArrayList(dimensions);
      xd0 = new TDoubleArrayList(dimensions);
      xd1 = new TDoubleArrayList(dimensions);

      for (int i = 0; i < dimensions; i++)
      {
         x0.add(0.0);
         xd0.add(0.0);
         x1.add(0.0);
         xd1.add(0.0);
      }

      for (int i = 0; i < maxWaypoints; i++)
      {
         waypoints.add(new DMatrixRMaj(dimensions, 1));
      }

      w0 = new TDoubleArrayList(dimensions);
      w1 = new TDoubleArrayList(dimensions);
      wd0 = new TDoubleArrayList(dimensions);
      wd1 = new TDoubleArrayList(dimensions);
      clearWeights();

      tempCoeffs.reshape(coefficients, 1);
   }

   /**
    * Resets all weight to {@link Double#POSITIVE_INFINITY} such that, unless later modified, all the
    * inputs are solved as hard constraints.
    */
   public void clearWeights()
   {
      w0.fill(0, dimensions.getValue(), Double.POSITIVE_INFINITY);
      w1.fill(0, dimensions.getValue(), Double.POSITIVE_INFINITY);
      wd0.fill(0, dimensions.getValue(), Double.POSITIVE_INFINITY);
      wd1.fill(0, dimensions.getValue(), Double.POSITIVE_INFINITY);
   }

   /**
    * Set the conditions at trajectory start and end for a single dimension.
    * 
    * @param dimension      the dimension to update. Must be in [0, this.dimensions[.
    * @param startPosition  start position of the trajectory at time 0.0
    * @param startVelocity  start velocity of the trajectory at time 0.0
    * @param targetPosition final position of the trajectory at time 1.0
    * @param targetVelocity final velocity of the trajectory at time 1.0
    */
   public void setEndPoints(int dimension, double startPosition, double startVelocity, double targetPosition, double targetVelocity)
   {
      if (dimension < 0 || dimension >= dimensions.getValue())
         throw new IllegalArgumentException("Illegal dimension, expected to be in [0, " + dimensions.getValue() + "[, but was: " + dimension);

      x0.set(dimension, startPosition);
      xd0.set(dimension, startVelocity);
      x1.set(dimension, targetPosition);
      xd1.set(dimension, targetVelocity);
   }

   /**
    * Set the conditions at trajectory start and end. All arguments are expected to have a size equal
    * to the number of dimensions of the optimizer (e.g. 3 for a 3d trajectory)
    *
    * @param startPosition  start position of the trajectory at time 0.0
    * @param startVelocity  start velocity of the trajectory at time 0.0
    * @param targetPosition final position of the trajectory at time 1.0
    * @param targetVelocity final velocity of the trajectory at time 1.0
    */
   public void setEndPoints(TDoubleArrayList startPosition, TDoubleArrayList startVelocity, TDoubleArrayList targetPosition, TDoubleArrayList targetVelocity)
   {
      if (startPosition.size() != dimensions.getIntegerValue())
         throw new RuntimeException("Unexpected Size of Input");
      if (startVelocity.size() != dimensions.getIntegerValue())
         throw new RuntimeException("Unexpected Size of Input");
      if (targetPosition.size() != dimensions.getIntegerValue())
         throw new RuntimeException("Unexpected Size of Input");
      if (targetVelocity.size() != dimensions.getIntegerValue())
         throw new RuntimeException("Unexpected Size of Input");

      for (int i = 0; i < dimensions.getIntegerValue(); i++)
      {
         x0.set(i, startPosition.get(i));
         xd0.set(i, startVelocity.get(i));
         x1.set(i, targetPosition.get(i));
         xd1.set(i, targetVelocity.get(i));
      }
   }

   /**
    * Sets the weights for the initial and final conditions for a single dimension.
    * <p>
    * Weights should be in <tt>[0, &infin;[</tt>. A weight set to {@link Double#POSITIVE_INFINITY} will
    * set up a hard constraint while any other real value will set up an objective to constrain the
    * corresponding spline.
    * </p>
    * 
    * @param dimension            the dimension to update. Must be in [0, this.dimensions[.
    * @param startPositionWeight  the weight value to use for the start position condition.
    * @param startVelocityWeight  the weight value to use for the start velocity condition.
    * @param targetPositionWeight the weight value to use for the final position condition.
    * @param targetVelocityWeight the weight value to use for the final velocity condition.
    */
   public void setEndPointWeights(int dimension,
                                  double startPositionWeight,
                                  double startVelocityWeight,
                                  double targetPositionWeight,
                                  double targetVelocityWeight)
   {
      if (dimension < 0 || dimension >= dimensions.getValue())
         throw new IllegalArgumentException("Illegal dimension, expected to be in [0, " + dimensions.getValue() + "[, but was: " + dimension);

      w0.set(dimension, startPositionWeight);
      wd0.set(dimension, startVelocityWeight);
      w1.set(dimension, targetPositionWeight);
      wd1.set(dimension, targetVelocityWeight);
   }

   /**
    * Sets the weights for the initial and final conditions. All arguments are expected to have a size
    * equal to the number of dimensions of the optimizer (e.g. 3 for a 3d trajectory)
    * <p>
    * Weights should be in <tt>[0, &infin;[</tt>. A weight set to {@link Double#POSITIVE_INFINITY} will
    * set up a hard constraint while any other real value will set up an objective to constrain the
    * corresponding spline.
    * </p>
    * 
    * @param startPositionWeight  the weight value to use for the start position condition.
    * @param startVelocityWeight  the weight value to use for the start velocity condition.
    * @param targetPositionWeight the weight value to use for the final position condition.
    * @param targetVelocityWeight the weight value to use for the final velocity condition.
    */
   public void setEndPointWeights(TDoubleArrayList startPositionWeight,
                                  TDoubleArrayList startVelocityWeight,
                                  TDoubleArrayList targetPositionWeight,
                                  TDoubleArrayList targetVelocityWeight)
   {
      if (startPositionWeight != null && startPositionWeight.size() != dimensions.getIntegerValue())
         throw new RuntimeException("Unexpected Size of Input");
      if (startVelocityWeight != null && startVelocityWeight.size() != dimensions.getIntegerValue())
         throw new RuntimeException("Unexpected Size of Input");
      if (targetPositionWeight != null && targetPositionWeight.size() != dimensions.getIntegerValue())
         throw new RuntimeException("Unexpected Size of Input");
      if (targetVelocityWeight != null && targetVelocityWeight.size() != dimensions.getIntegerValue())
         throw new RuntimeException("Unexpected Size of Input");

      if (startPositionWeight == null)
         w0.fill(0, dimensions.getValue(), Double.POSITIVE_INFINITY);
      if (startVelocityWeight == null)
         wd0.fill(0, dimensions.getValue(), Double.POSITIVE_INFINITY);
      if (targetPositionWeight == null)
         w1.fill(0, dimensions.getValue(), Double.POSITIVE_INFINITY);
      if (targetVelocityWeight == null)
         wd1.fill(0, dimensions.getValue(), Double.POSITIVE_INFINITY);

      for (int i = 0; i < dimensions.getValue(); i++)
      {
         if (startPositionWeight != null)
            w0.set(i, startPositionWeight.get(i));
         if (startVelocityWeight != null)
            w1.set(i, targetPositionWeight.get(i));
         if (targetPositionWeight != null)
            wd0.set(i, startVelocityWeight.get(i));
         if (targetVelocityWeight != null)
            wd1.set(i, targetVelocityWeight.get(i));
      }
   }

   /**
    * Set the waypoint positions along the trajectory. Each waypoint is expected to have a size equal
    * to the number of dimensions of the optimizer (e.g. 3 for a 3d trajectory). Times and velocities
    * are not specified but found by the optimization algorithm.
    *
    * @param waypoints list of all waypoint positions in the trajectory
    */
   public void setWaypoints(List<TDoubleArrayList> waypoints)
   {
      if (waypoints.size() > maxWaypoints)
         throw new RuntimeException("Too Many Waypoints");
      nWaypoints.set(waypoints.size());

      for (int i = 0; i < nWaypoints.getIntegerValue(); i++)
      {
         if (waypoints.get(i).size() != dimensions.getIntegerValue())
            throw new RuntimeException("Unexpected Size of Input");

         waypoints.get(i).toArray(this.waypoints.get(i).data);
      }
   }

   /**
    * Run the optimization with the default number of maximum time updates. This assumes knot points of
    * the trajectory have been set using the methods setEndPoints and setWaypoints. It is possible to
    * specify no waypoints.
    */
   public void compute()
   {
      compute(maxIterations);
   }

   /**
    * Run the optimization with the given number of maximum time updates. This assumes knot points of
    * the trajectory have been set using the methods setEndPoints and setWaypoints. It is possible to
    * specify no waypoints.
    *
    * @param maxIterations maximum number of iterations for the waypoint time optimization
    */
   public void compute(int maxIterations)
   {
      intervals.set(nWaypoints.getIntegerValue() + 1);
      intervalTimes.reshape(intervals.getValue(), 1);
      CommonOps_DDRM.fill(intervalTimes, 1.0 / intervals.getValue());
      computeInternal(maxIterations);
   }

   /**
    * If the user would like to provide waypoint times and only solve for the waypoint velocities this
    * method can be used.
    *
    * @param waypointTimes the times at waypoints. Times must be between 0.0 and 1.0.
    */
   public void computeForFixedTime(TDoubleArrayList waypointTimes)
   {
      compute(0, waypointTimes);
   }

   /**
    * This is a compute method that provides an optional argument for the initial waypoint times. These
    * times will be used as an initial guess for the gradient descent that optimizes the waypoint
    * times.
    *
    * @param waypointTimes the times at waypoints. Times must be between 0.0 and 1.0.
    */
   public void compute(int maxIterations, TDoubleArrayList waypointTimes)
   {
      intervals.set(nWaypoints.getIntegerValue() + 1);
      setIntervalTimes(waypointTimes);
      computeInternal(maxIterations);
   }

   private void computeInternal(int maxIterations)
   {
      computeTimer.startMeasurement();
      timeGain.set(initialTimeGain);

      problemSize.set(dimensions.getIntegerValue() * coefficients * intervals.getValue());
      costs.reset();
      costs.add(solveMinAcceleration());
      iteration.set(0);

      for (int iteration = 0; iteration < maxIterations; iteration++)
      {
         if (doFullTimeUpdate())
            break;
      }

      computeTimer.stopMeasurement();
   }

   private void setIntervalTimes(TDoubleArrayList waypointTimes)
   {
      intervalTimes.reshape(intervals.getValue(), 1);

      if (waypointTimes.size() != nWaypoints.getValue())
      {
         throw new RuntimeException("Unexpected number of waypoint times. Need " + nWaypoints.getValue() + ", got " + waypointTimes.size() + ".");
      }

      for (int i = 0; i < intervals.getValue(); i++)
      {
         double previousWaypointTime = i == 0 ? 0.0 : waypointTimes.get(i - 1);
         double waypointTime = i == nWaypoints.getValue() ? 1.0 : waypointTimes.get(i);
         double intervalTime = waypointTime - previousWaypointTime;
         if (intervalTime < 0.0 || intervalTime > 1.0)
         {
            throw new RuntimeException("Time in this trajectory is from 0.0 to 1.0. Got invalid waypoint times:\n" + waypointTimes.toString());
         }
         intervalTimes.set(i, intervalTime);
      }
   }

   /**
    * Provides an alternative API to the optimizer. This method allows the user to run a single
    * gradient descent step at a time. Will return true if the optimization has converged.
    * <p>
    * If this is desired call compute(0) to initialize the optimizer and then call doFullTimeUpdate()
    * to improve waypoint timing iteratively.
    * </p>
    * 
    * @return whether the gradient descent has converged or not.
    */
   public boolean doFullTimeUpdate()
   {
      double oldCost = costs.get(iteration.getIntegerValue());
      double newCost = computeTimeUpdate(oldCost);

      costs.add(newCost);
      iteration.increment();

      if (Math.abs(oldCost - newCost) < costEpsilon)
         return true;
      return false;
   }

   private double computeTimeUpdate(double cost)
   {
      timeUpdateTimer.startMeasurement();

      int intervals = this.intervals.getIntegerValue();
      timeGradient.reshape(intervals, 1);
      saveIntervalTimes.set(intervalTimes);

      for (int i = 0; i < intervals; i++)
      {
         for (int j = 0; j < intervals; j++)
         {
            if (j == i)
            {
               intervalTimes.add(j, 0, epsilon);
            }
            else
            {
               intervalTimes.add(j, 0, -epsilon / (intervals - 1));
            }
         }

         double value = (solveMinAcceleration() - cost) / epsilon;
         timeGradient.set(i, value);

         intervalTimes.set(saveIntervalTimes);
      }

      double length = CommonOps_DDRM.elementSum(timeGradient);
      CommonOps_DDRM.add(timeGradient, -length / intervals);

      for (int i = 0; i < 10; i++)
      {
         double newCost = applyTimeUpdate();

         if (newCost > cost)
         {
            timeGain.set(timeGain.getDoubleValue() * 0.5);
            intervalTimes.set(saveIntervalTimes);
         }
         else
         {
            return newCost;
         }
      }

      double newCost = applyTimeUpdate();

      timeUpdateTimer.stopMeasurement();
      return newCost;
   }

   private double applyTimeUpdate()
   {
      timeUpdate.set(timeGradient);
      CommonOps_DDRM.scale(-timeGain.getDoubleValue(), timeUpdate);

      double maxUpdate = CommonOps_DDRM.elementMaxAbs(timeUpdate);
      double minIntervalTime = CommonOps_DDRM.elementMin(intervalTimes);
      if (maxUpdate > 0.4 * minIntervalTime)
      {
         CommonOps_DDRM.scale(0.4 * minIntervalTime / maxUpdate, timeUpdate);
      }

      for (int i = 0; i < intervals.getIntegerValue(); i++)
      {
         intervalTimes.add(i, 0, timeUpdate.get(i));
      }

      return solveMinAcceleration();
   }

   private double solveMinAcceleration()
   {
      double cost = 0;
      x.clear();
      for (int dimension = 0; dimension < dimensions.getValue(); dimension++)
      {
         cost += solveDimension(dimension, x.add());
      }
      return cost;
   }

   private double solveDimension(int dimension, DMatrixRMaj solutionToPack)
   {
      solver.setEndpoints(x0.get(dimension), xd0.get(dimension), x1.get(dimension), xd1.get(dimension));
      solver.setEndpointWeights(w0.get(dimension), wd0.get(dimension), w1.get(dimension), wd1.get(dimension));
      solver.clearWaypoints();

      double time = 0.0;

      for (int w = 0; w < nWaypoints.getValue(); w++)
      {
         time += intervalTimes.get(w);
         solver.addWaypoint(waypoints.get(w).get(dimension), time);
      }

      return solver.solveAndComputeCost(solutionToPack);
   }

   /**
    * Get the optimal times at the given waypoints.
    *
    * @param timesToPack modified - the optimal waypoint times are stored here
    */
   public void getWaypointTimes(TDoubleArrayList timesToPack)
   {
      timesToPack.reset();

      for (int i = 0; i < nWaypoints.getIntegerValue(); i++)
      {
         if (i == 0)
         {
            timesToPack.add(intervalTimes.get(0));
            continue;
         }
         timesToPack.add(timesToPack.get(i - 1) + intervalTimes.get(i));
      }
   }

   /**
    * Get the optimal time for a specific waypoint.
    *
    * @param waypoint the index of the waypoint of interest
    * @return the optimal time for this waypoint
    */
   public double getWaypointTime(int waypoint)
   {
      if (waypoint < 0)
         throw new RuntimeException("Unexpected Waypoint Index " + waypoint);
      if (waypoint > nWaypoints.getIntegerValue() - 1)
         throw new RuntimeException("Unexpected Waypoint Index " + waypoint);

      double time = intervalTimes.get(0);
      for (int i = 1; i < waypoint + 1; i++)
         time += intervalTimes.get(i);
      return time;
   }

   /**
    * Get the coefficients for the polynomials that describe the trajectory. Only the coefficients for
    * a single dimension are returned (e.g. for a 2d optimization problem this method can be called for
    * dimension 0 and 1). The coefficients are stored in the given list that is expected to have size
    * equal to the number of trajectory segments. The list will contain the coefficients for the
    * trajectory segments in order.
    *
    * @param coefficientsToPack modified - the polynomial coefficients are stored here
    * @param dimension          the dimension for which the polynomial coefficients are returned
    */
   public void getPolynomialCoefficients(List<TDoubleArrayList> coefficientsToPack, int dimension)
   {
      if (coefficientsToPack.size() != intervals.getIntegerValue())
         throw new RuntimeException("Unexpected Size of Output. Coefficients : " + coefficientsToPack.size() + ", intervals : " + intervals.getIntegerValue());
      if (dimension > dimensions.getIntegerValue() - 1 || dimension < 0)
         throw new RuntimeException("Unknown Dimension");

      DMatrixRMaj xDim = x.get(dimension);
      for (int i = 0; i < intervals.getIntegerValue(); i++)
      {
         int index = i * coefficients;
         CommonOps_DDRM.extract(xDim, index, index + coefficients, 0, 1, tempCoeffs, 0, 0);
         coefficientsToPack.get(i).reset();
         coefficientsToPack.get(i).add(tempCoeffs.getData());
      }
   }

   /**
    * Get the optimal velocity at a given waypoint.
    *
    * @param velocityToPack modified - the waypoint velocity is stored here
    * @param waypointIndex  index of the waypoint of interest
    */
   public void getWaypointVelocity(TDoubleArrayList velocityToPack, int waypointIndex)
   {
      double waypointTime = getWaypointTime(waypointIndex);
      tempLine.reshape(1, coefficients);
      MultiCubicSpline1DSolver.getVelocityConstraintABlock(waypointTime, 0, 0, tempLine);

      velocityToPack.reset();
      for (int dimension = 0; dimension < dimensions.getIntegerValue(); dimension++)
      {
         DMatrixRMaj xDim = x.get(dimension);
         int index = waypointIndex * coefficients;
         CommonOps_DDRM.extract(xDim, index, index + coefficients, 0, 1, tempCoeffs, 0, 0);
         velocityToPack.add(CommonOps_DDRM.dot(tempCoeffs, tempLine));
      }
   }

   /**
    * Computes from the optimal set of coefficients the position at the start.
    * <p>
    * This method is only useful when the start position is configured as an objective and can thus be
    * different from the given position.
    * </p>
    * 
    * @param positionToPack modified - the start position is stored here.
    */
   public void getStartPosition(TDoubleArrayList positionToPack)
   {
      tempLine.reshape(1, coefficients);
      MultiCubicSpline1DSolver.getPositionConstraintABlock(0.0, 0, 0, tempLine);

      positionToPack.reset();
      for (int dimension = 0; dimension < dimensions.getIntegerValue(); dimension++)
      {
         DMatrixRMaj xDim = x.get(dimension);
         CommonOps_DDRM.extract(xDim, 0, coefficients, 0, 1, tempCoeffs, 0, 0);
         positionToPack.add(CommonOps_DDRM.dot(tempCoeffs, tempLine));
      }
   }

   /**
    * Computes from the optimal set of coefficients the velocity at the start.
    * <p>
    * This method is only useful when the start velocity is configured as an objective and can thus be
    * different from the given velocity.
    * </p>
    * 
    * @param velocityToPack modified - the start velocity is stored here.
    */
   public void getStartVelocity(TDoubleArrayList velocityToPack)
   {
      tempLine.reshape(1, coefficients);
      MultiCubicSpline1DSolver.getVelocityConstraintABlock(0.0, 0, 0, tempLine);

      velocityToPack.reset();
      for (int dimension = 0; dimension < dimensions.getIntegerValue(); dimension++)
      {
         DMatrixRMaj xDim = x.get(dimension);
         CommonOps_DDRM.extract(xDim, 0, coefficients, 0, 1, tempCoeffs, 0, 0);
         velocityToPack.add(CommonOps_DDRM.dot(tempCoeffs, tempLine));
      }
   }

   /**
    * Computes from the optimal set of coefficients the position for the target.
    * <p>
    * This method is only useful when the target position is configured as an objective and can thus be
    * different from the given position.
    * </p>
    * 
    * @param positionToPack modified - the target position is stored here.
    */
   public void getTargetPosition(TDoubleArrayList positionToPack)
   {
      tempLine.reshape(1, coefficients);
      MultiCubicSpline1DSolver.getPositionConstraintABlock(1.0, 0, 0, tempLine);

      positionToPack.reset();
      int index = nWaypoints.getValue() * coefficients;

      for (int dimension = 0; dimension < dimensions.getIntegerValue(); dimension++)
      {
         DMatrixRMaj xDim = x.get(dimension);
         CommonOps_DDRM.extract(xDim, index, index + coefficients, 0, 1, tempCoeffs, 0, 0);
         positionToPack.add(CommonOps_DDRM.dot(tempCoeffs, tempLine));
      }
   }

   /**
    * Computes from the optimal set of coefficients the velocity for the target.
    * <p>
    * This method is only useful when the target velocity is configured as an objective and can thus be
    * different from the given velocity.
    * </p>
    * 
    * @param velocityToPack modified - the target velocity is stored here.
    */
   public void getTargetVelocity(TDoubleArrayList velocityToPack)
   {
      tempLine.reshape(1, coefficients);
      MultiCubicSpline1DSolver.getVelocityConstraintABlock(1.0, 0, 0, tempLine);

      velocityToPack.reset();
      int index = nWaypoints.getValue() * coefficients;

      for (int dimension = 0; dimension < dimensions.getIntegerValue(); dimension++)
      {
         DMatrixRMaj xDim = x.get(dimension);
         CommonOps_DDRM.extract(xDim, index, index + coefficients, 0, 1, tempCoeffs, 0, 0);
         velocityToPack.add(CommonOps_DDRM.dot(tempCoeffs, tempLine));
      }
   }
}
