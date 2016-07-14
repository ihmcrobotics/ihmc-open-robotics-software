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
 * This class can compute a optimal trajectory from a start point to a target point. Given position and
 * velocity at start and end point as well as waypoint positions this class computes velocities and times
 * at the waypoints such that the integral of the squared acceleration over the whole trajectory is
 * minimized. Time is dimensionless and goes from 0.0 at the start to 1.0 at the target. Optionally the
 * trajectory polynomials between the waypoints can be returned.
 *
 * @author shadylady
 *
 */
public class TrajectoryPointOptimizer
{
   private static final int maxWaypoints = 12;
   private static final int maxIterations = 20;

   private static final double regularizationWeight = 1E-10;
   private static final double epsilon = 1E-7;

   private static final double initialTimeGain = 0.002;
   private static final double costEpsilon = 0.1;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final PolynomialOrder order;

   private final IntegerYoVariable dimensions = new IntegerYoVariable("Dimensions", registry);
   private final IntegerYoVariable nWaypoints = new IntegerYoVariable("NumberOfWaypoints", registry);
   private final IntegerYoVariable intervals = new IntegerYoVariable("NumberOfIntervals", registry);
   private final IntegerYoVariable coefficients = new IntegerYoVariable("Coefficients", registry);
   private final IntegerYoVariable problemSize = new IntegerYoVariable("ProblemSize", registry);
   private final IntegerYoVariable constraints = new IntegerYoVariable("Conditions", registry);
   private final IntegerYoVariable iteration = new IntegerYoVariable("Iteration", registry);

   private final TDoubleArrayList x0, x1, xd0, xd1;
   private final ArrayList<DenseMatrix64F> waypoints = new ArrayList<>();

   private final DenseMatrix64F intervalTimes = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F saveIntervalTimes = new DenseMatrix64F(1, 1);
   private final TDoubleArrayList costs = new TDoubleArrayList(maxIterations+1);

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
   private final DoubleYoVariable timeGain = new DoubleYoVariable("TimeGain", registry);

   private final ExecutionTimer timer = new ExecutionTimer("TrajectoryOptimizationTimer", 0.5, registry);

   private final LinearSolver<DenseMatrix64F> solver = LinearSolverFactory.linear(0);

   private final DenseMatrix64F tempCoeffs = new DenseMatrix64F(1, 1);

   public TrajectoryPointOptimizer(int dimensions, PolynomialOrder order, YoVariableRegistry parentRegistry)
   {
      this(dimensions, order);
      parentRegistry.addChild(registry);
   }

   public TrajectoryPointOptimizer(int dimensions, PolynomialOrder order)
   {
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

   public void setEndPoints(TDoubleArrayList start, TDoubleArrayList startVel, TDoubleArrayList target, TDoubleArrayList targetVel)
   {
      if (start.size() != dimensions.getIntegerValue())
         throw new RuntimeException("Unexpected Size of Input");
      if (startVel.size() != dimensions.getIntegerValue())
         throw new RuntimeException("Unexpected Size of Input");
      if (target.size() != dimensions.getIntegerValue())
         throw new RuntimeException("Unexpected Size of Input");
      if (targetVel.size() != dimensions.getIntegerValue())
         throw new RuntimeException("Unexpected Size of Input");

      for (int i = 0; i < dimensions.getIntegerValue(); i++)
      {
         x0.set(i, start.get(i));
         xd0.set(i, startVel.get(i));
         x1.set(i, target.get(i));
         xd1.set(i, targetVel.get(i));
      }
   }

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

   public void compute()
   {
      compute(maxIterations);
   }

   public void compute(int maxIterations)
   {
      timer.startMeasurement();
      timeGain.set(initialTimeGain);

      int intervals = nWaypoints.getIntegerValue() + 1;
      this.intervals.set(intervals);
      intervalTimes.reshape(intervals, 1);
      CommonOps.fill(intervalTimes, 1.0/intervals);

      problemSize.set(dimensions.getIntegerValue() * coefficients.getIntegerValue() * intervals);
      costs.reset();
      costs.add(solveMinAcceleration());

      for (int iteration = 0; iteration < maxIterations; iteration++)
      {
         double newCost = computeTimeUpdate(costs.get(iteration));
         this.iteration.set(iteration+1);
         costs.add(newCost);

         if (Math.abs(costs.get(iteration) - newCost) < costEpsilon)
            break;

//         if (iteration == maxIterations-1)
//            System.err.println("Trajectory optimization max iteration.");
      }

      timer.stopMeasurement();
   }

   private double computeTimeUpdate(double cost)
   {
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
               intervalTimes.add(j, 0, -epsilon / (intervals-1));
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

      return applyTimeUpdate();
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

      if (solver.setA(E))
         solver.invert(E);
      x.reshape(size, 1);
      CommonOps.mult(E, d, x);
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
      int waypointConstraints = nWaypoints.getIntegerValue() * dimensions * (2 + order.getCoefficients()/2 - 1);
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
         for (int w = 0 ; w < nWaypoints.getIntegerValue(); w++)
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
         timesToPack.add(timesToPack.get(i-1) + intervalTimes.get(i));
      }
   }

   public double getWaypointTime(int waypoint)
   {
      if (waypoint < 0)
         throw new RuntimeException("Unexpected Waypoint Index");
      if (waypoint > nWaypoints.getIntegerValue()-1)
         throw new RuntimeException("Unexpected Waypoint Index");

      double time = intervalTimes.get(0);
      for (int i = 1; i < waypoint+1; i++)
         time += intervalTimes.get(i);
      return time;
   }

   public void getPolynomialCoefficients(List<TDoubleArrayList> coefficientsToPack, int dimension)
   {
      if (coefficientsToPack.size() != intervals.getIntegerValue())
         throw new RuntimeException("Unexpected Size of Output");
      if (dimension > dimensions.getIntegerValue()-1 || dimension < 0)
         throw new RuntimeException("Unknown Dimension");

      for (int i = 0; i < intervals.getIntegerValue(); i++)
      {
         int index = i * order.getCoefficients() + dimension * order.getCoefficients() * intervals.getIntegerValue();
         CommonOps.extract(x, index, index+order.getCoefficients(), 0, 1, tempCoeffs, 0, 0);
         coefficientsToPack.get(i).reset();
         coefficientsToPack.get(i).add(tempCoeffs.getData());
      }
   }

}
