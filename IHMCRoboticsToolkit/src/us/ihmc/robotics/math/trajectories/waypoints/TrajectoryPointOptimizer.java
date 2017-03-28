package us.ihmc.robotics.math.trajectories.waypoints;

import java.util.ArrayList;
import java.util.List;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.ops.CommonOps;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.time.ExecutionTimer;

/**
 * The TrajectoryPointOptimizer computes an optimal (minimal integrated acceleration) trajectory.
 *
 * Given start and end point position and velocity as well as waypoint positions (optional) this
 * optimizer finds velocities, accelerations, and times at the waypoints that minimize the overall
 * integrated acceleration along the trajectory.
 *
 * If a third order trajectory is requested the acceleration will not be zero at start and end
 * but be continuous at the waypoints. For a fifth order polynomial the start and end accelerations
 * will be zero. Time is assumed to be without dimension and goes from 0.0 at the start to 1.0 at
 * the end of the trajectory.
 *
 * The trajectory times are found through an iterative process using a gradient descent. The compute()
 * method can be called with an optional argument specifying the maximal amount of iterations if the
 * runtime is critical. The waypoint times are initialized to be evenly distributed in the interval
 * 0.0 to 1.0. The maximum number of time optimization steps can be set to zero.
 *
 * The class can return waypoint times, polynomial coefficients for the trajectory segments, and
 * optimal velocities and accelerations at the knot points of the trajectory.
 *
 * Algorithm based on "Minimum Snap Trajectory Generation and Control for Quadrotors" - Mellinger
 * @author gwiedebach
 *
 */
public class TrajectoryPointOptimizer
{
   public static final int maxWaypoints = 12;
   public static final int maxIterations = 20;

   private static final double regularizationWeight = 1E-10;
   private static final double epsilon = 1E-7;

   private static final double initialTimeGain = 0.001;
   private static final double costEpsilon = 0.01;

   private final YoVariableRegistry registry;

   private final PolynomialOrder order;

   private final IntegerYoVariable dimensions;
   private final IntegerYoVariable nWaypoints;
   private final IntegerYoVariable intervals;
   private final IntegerYoVariable coefficients;
   private final IntegerYoVariable problemSize;
   private final IntegerYoVariable inversionSize;
   private final IntegerYoVariable constraints;
   private final IntegerYoVariable iteration;

   private final TDoubleArrayList x0, x1, xd0, xd1;
   private final ArrayList<DenseMatrix64F> waypoints = new ArrayList<>();

   private final DenseMatrix64F intervalTimes = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F saveIntervalTimes = new DenseMatrix64F(1, 1);
   private final TDoubleArrayList costs = new TDoubleArrayList(maxIterations + 1);

   private final DenseMatrix64F H = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F x = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F f = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F A = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F ATranspose = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F b = new DenseMatrix64F(1, 1);

   private final DenseMatrix64F E = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F d = new DenseMatrix64F(1, 1);

   private final DenseMatrix64F hBlock = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F Ad = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F bd = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F AdLine = new DenseMatrix64F(1, 1);

   private final DenseMatrix64F timeGradient = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F timeUpdate = new DenseMatrix64F(1, 1);
   private final DoubleYoVariable timeGain;

   private final ExecutionTimer computeTimer;
   private final ExecutionTimer timeUpdateTimer;

   private final LinearSolver<DenseMatrix64F> solver = LinearSolverFactory.linear(0);

   private final DenseMatrix64F tempCoeffs = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F tempLine = new DenseMatrix64F(1, 1);

   public TrajectoryPointOptimizer(int dimensions, PolynomialOrder order, YoVariableRegistry parentRegistry)
   {
      this("", dimensions, order, parentRegistry);
   }

   public TrajectoryPointOptimizer(String namePrefix, int dimensions, PolynomialOrder order, YoVariableRegistry parentRegistry)
   {
      this(namePrefix, dimensions, order);
      parentRegistry.addChild(registry);
   }

   public TrajectoryPointOptimizer(int dimensions, PolynomialOrder order)
   {
      this("", dimensions, order);
   }

   public TrajectoryPointOptimizer(String namePrefix, int dimensions, PolynomialOrder order)
   {
      this.registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      this.dimensions = new IntegerYoVariable(namePrefix + "Dimensions", registry);
      this.nWaypoints = new IntegerYoVariable(namePrefix + "NumberOfWaypoints", registry);
      this.intervals = new IntegerYoVariable(namePrefix + "NumberOfIntervals", registry);
      this.coefficients = new IntegerYoVariable(namePrefix + "Coefficients", registry);
      this.problemSize = new IntegerYoVariable(namePrefix + "ProblemSize", registry);
      this.inversionSize = new IntegerYoVariable(namePrefix + "InversionSize", registry);
      this.constraints = new IntegerYoVariable(namePrefix + "Conditions", registry);
      this.iteration = new IntegerYoVariable(namePrefix + "Iteration", registry);
      this.computeTimer = new ExecutionTimer(namePrefix + "ComputeTimer", 0.0, registry);
      this.timeUpdateTimer = new ExecutionTimer(namePrefix + "TimeUpdateTimer", 0.0, registry);
      this.timeGain = new DoubleYoVariable(namePrefix + "TimeGain", registry);

      dimensions = Math.max(dimensions, 0);
      this.dimensions.set(dimensions);
      this.order = order;
      coefficients.set(order.getCoefficients());
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
         waypoints.add(new DenseMatrix64F(dimensions, 1));
      }

      tempCoeffs.reshape(order.getCoefficients(), 1);
   }

   /**
    * Set the conditions at trajectory start and end. All arguments are expected to have a size equal to the number
    * of dimensions of the optimizer (e.g. 3 for a 3d trajectory)
    *
    * @param startPosition      start position of the trajectory at time 0.0
    * @param startVelocity      start velocity of the trajectory at time 0.0
    * @param targetPosition     final position of the trajectory at time 1.0
    * @param targetVelocity     final velocity of the trajectory at time 1.0
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
    * Set the waypoint positions along the trajectory. Each waypoint is expected to have a size equal to the number
    * of dimensions of the optimizer (e.g. 3 for a 3d trajectory). Times and velocities are not specified but found
    * by the optimization algorithm.
    *
    * @param waypoints   list of all waypoint positions in the trajectory
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
    * Run the optimization with the default number of maximum time updates. This assumes knot points of the trajectory
    * have been set using the methods setEndPoints and setWaypoints. It is possible to specify no waypoints.
    */
   public void compute()
   {
      compute(maxIterations);
   }

   /**
    * Run the optimization with the given number of maximum time updates. This assumes knot points of the trajectory
    * have been set using the methods setEndPoints and setWaypoints. It is possible to specify no waypoints.
    *
    * @param maxIterations   maximum number of iterations for the waypoint time optimization
    */
   public void compute(int maxIterations)
   {
      computeTimer.startMeasurement();
      timeGain.set(initialTimeGain);

      int intervals = nWaypoints.getIntegerValue() + 1;
      this.intervals.set(intervals);
      intervalTimes.reshape(intervals, 1);
      CommonOps.fill(intervalTimes, 1.0 / intervals);

      problemSize.set(dimensions.getIntegerValue() * coefficients.getIntegerValue() * intervals);
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

   /**
    * Provides an alternative API to the optimizer. This method allows the user to run a single gradient descent step
    * at a time. Will return true if the optimization has converged.
    *
    * If this is desired call compute(0) to initialize the optimizer and then call doFullTimeUpdate() to improve
    * waypoint timing iteratively.
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

      double length = CommonOps.elementSum(timeGradient);
      CommonOps.add(timeGradient, -length / intervals);

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
      CommonOps.scale(-timeGain.getDoubleValue(), timeUpdate);

      double maxUpdate = CommonOps.elementMaxAbs(timeUpdate);
      double minIntervalTime = CommonOps.elementMin(intervalTimes);
      if (maxUpdate > 0.4 * minIntervalTime)
      {
         CommonOps.scale(0.4 * minIntervalTime / maxUpdate, timeUpdate);
      }

      for (int i = 0; i < intervals.getIntegerValue(); i++)
      {
         intervalTimes.add(i, 0, timeUpdate.get(i));
      }

      return solveMinAcceleration();
   }

   private double solveMinAcceleration()
   {
      buildCostMatrix();
      buildConstraintMatrices();

      int problemSize = this.problemSize.getIntegerValue();
      f.reshape(problemSize, 1);
      CommonOps.fill(f, regularizationWeight);

      // min 0.5*x'*H*x + f'*x
      // s.t. A*x == b

      int size = problemSize + constraints.getIntegerValue();
      this.inversionSize.set(size);
      E.reshape(size, size);
      d.reshape(size, 1);

      CommonOps.fill(E, 0.0);
      CommonOps.insert(H, E, 0, 0);
      CommonOps.insert(A, E, problemSize, 0);
      ATranspose.reshape(A.getNumCols(), A.getNumRows());
      CommonOps.transpose(A, ATranspose);
      CommonOps.insert(ATranspose, E, 0, problemSize);
      CommonOps.scale(-1.0, f);
      CommonOps.insert(f, d, 0, 0);
      CommonOps.insert(b, d, problemSize, 0);

      x.reshape(size, 1);
      if (solver.setA(E)) // TODO: if this does not work what then?
         solver.solve(d, x);
      x.reshape(problemSize, 1);

      d.reshape(problemSize, 1);
      b.reshape(1, 1);
      CommonOps.mult(H, x, d);
      CommonOps.multTransA(x, d, b);
      double cost = 0.5 * b.get(0, 0);
      return cost;
   }

   private void buildConstraintMatrices()
   {
      int dimensions = this.dimensions.getIntegerValue();
      int endpointConstraints = dimensions * order.getCoefficients();
      int waypointConstraints = nWaypoints.getIntegerValue() * dimensions * (2 + order.getCoefficients() / 2 - 1);
      constraints.set(endpointConstraints + waypointConstraints);

      int constraints = this.constraints.getIntegerValue();

      A.reshape(constraints, problemSize.getIntegerValue());
      b.reshape(constraints, 1);
      CommonOps.fill(A, 0.0);

      int dimensionConstraints = constraints / dimensions;
      int subProblemSize = problemSize.getIntegerValue() / dimensions;
      Ad.reshape(dimensionConstraints, subProblemSize);
      bd.reshape(dimensionConstraints, 1);
      CommonOps.fill(Ad, 0.0);

      for (int d = 0; d < dimensions; d++)
      {
         int line = 0;

         if (order.getPositionLine(0.0, AdLine))
         {
            CommonOps.insert(AdLine, Ad, line, 0);
            bd.set(line, x0.get(d));
            line++;
         }
         if (order.getVelocityLine(0.0, AdLine))
         {
            CommonOps.insert(AdLine, Ad, line, 0);
            bd.set(line, xd0.get(d));
            line++;
         }
         if (order.getAccelerationLine(0.0, AdLine))
         {
            CommonOps.insert(AdLine, Ad, line, 0);
            bd.set(line, 0.0);
            line++;
         }
         if (order.getJerkLine(0.0, AdLine))
         {
            CommonOps.insert(AdLine, Ad, line, 0);
            bd.set(line, 0.0);
            line++;
         }

         double t = 0.0;
         for (int w = 0; w < nWaypoints.getIntegerValue(); w++)
         {
            t += intervalTimes.get(w);
            int colOffset = w * order.getCoefficients();
            DenseMatrix64F waypoint = waypoints.get(w);

            order.getPositionLine(t, AdLine);
            CommonOps.insert(AdLine, Ad, line, colOffset);
            bd.set(line, waypoint.get(d));
            line++;
            CommonOps.insert(AdLine, Ad, line, colOffset + order.getCoefficients());
            bd.set(line, waypoint.get(d));
            line++;

            if (order.getVelocityLine(t, AdLine))
            {
               CommonOps.insert(AdLine, Ad, line, colOffset);
               CommonOps.scale(-1.0, AdLine);
               CommonOps.insert(AdLine, Ad, line, colOffset + order.getCoefficients());
               bd.set(line, 0.0);
               line++;
            }

            if (order.getAccelerationLine(t, AdLine))
            {
               CommonOps.insert(AdLine, Ad, line, colOffset);
               CommonOps.scale(-1.0, AdLine);
               CommonOps.insert(AdLine, Ad, line, colOffset + order.getCoefficients());
               bd.set(line, 0.0);
               line++;
            }

            if (order.getJerkLine(t, AdLine))
            {
               CommonOps.insert(AdLine, Ad, line, colOffset);
               CommonOps.scale(-1.0, AdLine);
               CommonOps.insert(AdLine, Ad, line, colOffset + order.getCoefficients());
               bd.set(line, 0.0);
               line++;
            }
         }

         if (order.getPositionLine(1.0, AdLine))
         {
            CommonOps.insert(AdLine, Ad, line, subProblemSize - order.getCoefficients());
            bd.set(line, x1.get(d));
            line++;
         }
         if (order.getVelocityLine(1.0, AdLine))
         {
            CommonOps.insert(AdLine, Ad, line, subProblemSize - order.getCoefficients());
            bd.set(line, xd1.get(d));
            line++;
         }
         if (order.getAccelerationLine(1.0, AdLine))
         {
            CommonOps.insert(AdLine, Ad, line, subProblemSize - order.getCoefficients());
            bd.set(line, 0.0);
            line++;
         }
         if (order.getJerkLine(1.0, AdLine))
         {
            CommonOps.insert(AdLine, Ad, line, subProblemSize - order.getCoefficients());
            bd.set(line, 0.0);
         }

         int rowOffset = d * dimensionConstraints;
         int colOffset = d * subProblemSize;
         CommonOps.insert(Ad, A, rowOffset, colOffset);
         CommonOps.insert(bd, b, rowOffset, 0);
      }
   }

   private void buildCostMatrix()
   {
      H.reshape(problemSize.getIntegerValue(), problemSize.getIntegerValue());
      CommonOps.fill(H, 0.0);

      double t0 = 0.0;
      double t1 = 0.0;
      for (int i = 0; i < intervals.getIntegerValue(); i++)
      {
         t0 = t1;
         t1 = t1 + intervalTimes.get(i);

         order.getHBlock(t0, t1, hBlock);
         for (int d = 0; d < dimensions.getIntegerValue(); d++)
         {
            int offset = (i + d * intervals.getIntegerValue()) * coefficients.getIntegerValue();
            CommonOps.insert(hBlock, H, offset, offset);
         }
      }
   }

   /**
    * Get the optimal times at the given waypoints.
    *
    * @param timesToPack    modified - the optimal waypoint times are stopred here
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
    * @param waypoint       the index of the waypoint of interest
    * @return               the optimal time for this waypoint
    */
   public double getWaypointTime(int waypoint)
   {
      if (waypoint < 0)
         throw new RuntimeException("Unexpected Waypoint Index");
      if (waypoint > nWaypoints.getIntegerValue() - 1)
         throw new RuntimeException("Unexpected Waypoint Index");

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
    * @param coefficientsToPack   modified - the polynomial coefficients are stored here
    * @param dimension            the dimension for which the polynomial coefficients are returned
    */
   public void getPolynomialCoefficients(List<TDoubleArrayList> coefficientsToPack, int dimension)
   {
      if (coefficientsToPack.size() != intervals.getIntegerValue())
         throw new RuntimeException("Unexpected Size of Output");
      if (dimension > dimensions.getIntegerValue() - 1 || dimension < 0)
         throw new RuntimeException("Unknown Dimension");

      for (int i = 0; i < intervals.getIntegerValue(); i++)
      {
         int index = i * order.getCoefficients() + dimension * order.getCoefficients() * intervals.getIntegerValue();
         CommonOps.extract(x, index, index + order.getCoefficients(), 0, 1, tempCoeffs, 0, 0);
         coefficientsToPack.get(i).reset();
         coefficientsToPack.get(i).add(tempCoeffs.getData());
      }
   }

   /**
    * Get the optimal velocity at a given waypoint.
    *
    * @param velocityToPack     modified - the waypoint velocity is stored here
    * @param waypointIndex      index of the waypoint of interest
    */
   public void getWaypointVelocity(TDoubleArrayList velocityToPack, int waypointIndex)
   {
      double waypointTime = getWaypointTime(waypointIndex);
      if (!order.getVelocityLine(waypointTime, tempLine))
         throw new RuntimeException("Waypoint velocity not available for this polynomial order");

      velocityToPack.reset();
      for (int d = 0; d < dimensions.getIntegerValue(); d++)
      {
         int index = waypointIndex * order.getCoefficients() + d * order.getCoefficients() * intervals.getIntegerValue();
         CommonOps.extract(x, index, index + order.getCoefficients(), 0, 1, tempCoeffs, 0, 0);
         velocityToPack.add(CommonOps.dot(tempCoeffs, tempLine));
      }
   }

   /**
    * Get the optimal acceleration at a given waypoint. This method will throw an exception
    * if the order of the trajectory is too low to specify the waypoint accelerations (e.g.
    * order 3).
    *
    * @param accelerationToPack     modified - the waypoint velocity is stored here
    * @param waypointIndex          index of the waypoint of interest
    */
   public void getWaypointAcceleration(TDoubleArrayList accelerationToPack, int waypointIndex)
   {
      double waypointTime = getWaypointTime(waypointIndex);
      if (!order.getAccelerationLine(waypointTime, tempLine))
         throw new RuntimeException("Waypoint acceleration not available for this polynomial order");

      accelerationToPack.reset();
      for (int d = 0; d < dimensions.getIntegerValue(); d++)
      {
         int index = waypointIndex * order.getCoefficients() + d * order.getCoefficients() * intervals.getIntegerValue();
         CommonOps.extract(x, index, index + order.getCoefficients(), 0, 1, tempCoeffs, 0, 0);
         accelerationToPack.add(CommonOps.dot(tempCoeffs, tempLine));
      }
   }

}
