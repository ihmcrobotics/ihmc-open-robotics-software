package us.ihmc.robotics.math.trajectories.waypoints;

import java.util.ArrayList;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;

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

   private static final double initialTimeGain = 0.001;
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

   private final DenseMatrix64F x0, x1, xd0, xd1;
   private final ArrayList<DenseMatrix64F> waypoints = new ArrayList<>();

   private final DenseMatrix64F intervalTimes = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F saveIntervalTimes = new DenseMatrix64F(1, 1);
   private final double[] costs = new double[maxIterations+1];

   private final DenseMatrix64F H = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F x = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F f = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F A = new DenseMatrix64F(1, 1);
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

   private final DoubleYoVariable computationTime = new DoubleYoVariable("ComputationTimeMS", registry);

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

      x0 = new DenseMatrix64F(dimensions, 1);
      x1 = new DenseMatrix64F(dimensions, 1);
      xd0 = new DenseMatrix64F(dimensions, 1);
      xd1 = new DenseMatrix64F(dimensions, 1);

      for (int i = 0; i < maxWaypoints; i++)
      {
         waypoints.add(new DenseMatrix64F(dimensions, 1));
      }
   }

   public void setEndPoints(double[] start, double startVel[], double[] target, double[] targetVel)
   {
      if (start.length != dimensions.getIntegerValue())
         throw new RuntimeException("Unexpected Size of Input");
      if (startVel.length != dimensions.getIntegerValue())
         throw new RuntimeException("Unexpected Size of Input");
      if (target.length != dimensions.getIntegerValue())
         throw new RuntimeException("Unexpected Size of Input");
      if (targetVel.length != dimensions.getIntegerValue())
         throw new RuntimeException("Unexpected Size of Input");

      x0.setData(start);
      xd0.setData(startVel);
      x1.setData(target);
      xd1.setData(targetVel);
   }

   public void setWaypoints(ArrayList<double[]> waypoints)
   {
      if (waypoints.size() > maxWaypoints)
         throw new RuntimeException("Too Many Waypoints");
      nWaypoints.set(waypoints.size());

      for (int i = 0; i < nWaypoints.getIntegerValue(); i++)
      {
         if (waypoints.get(i).length != dimensions.getIntegerValue())
            throw new RuntimeException("Unexpected Size of Input");
         this.waypoints.get(i).setData(waypoints.get(i));
      }
   }

   public void compute()
   {
      long startTime = System.nanoTime();

      int intervals = nWaypoints.getIntegerValue() + 1;
      this.intervals.set(intervals);
      intervalTimes.reshape(intervals, 1);
      CommonOps.fill(intervalTimes, 1.0/intervals);

      problemSize.set(dimensions.getIntegerValue() * coefficients.getIntegerValue() * intervals);
      costs[0] = solveMinAcceleration();

      for (int iteration = 0; iteration < maxIterations; iteration++)
      {
         double newCost = computeTimeUpdate(costs[iteration]);
         this.iteration.set(iteration+1);
         costs[iteration+1] = newCost;

         if (Math.abs(costs[iteration] - newCost) < costEpsilon)
            break;

         if (iteration == maxIterations-1)
            System.err.println("Trajectory optimization max iteration.");
      }

      long duration = System.nanoTime() - startTime;
      computationTime.set((double)duration / 10E6);
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
            timeGain.mul(0.5);
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
      CommonOps.transpose(A);
      CommonOps.insert(A, E, 0, problemSize);
      CommonOps.scale(-1.0, f);
      CommonOps.insert(f, d, 0, 0);
      CommonOps.insert(b, d, problemSize, 0);

      CommonOps.invert(E);
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
      CommonOps.fill(b, 0.0);

      int dimensionConstraints = constraints / dimensions;
      int subProblemSize = problemSize.getIntegerValue() / dimensions;
      Ad.reshape(dimensionConstraints, subProblemSize);
      bd.reshape(dimensionConstraints, 1);

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
            line++;
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

   public void getWaypointTimes(double[] timesToPack)
   {
      int n = nWaypoints.getIntegerValue();
      if (timesToPack.length != n)
         throw new RuntimeException("Unexpected Size of Output");
      for (int i = 0; i < n; i++)
      {
         if (i == 0)
         {
            timesToPack[0] = intervalTimes.get(0);
            continue;
         }
         timesToPack[i] = timesToPack[i-1] + intervalTimes.get(i);
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

   public void getPolynomialCoefficients(ArrayList<double[]> coefficientsToPack, int dimension)
   {
      if (coefficientsToPack.size() != intervals.getIntegerValue())
         throw new RuntimeException("Unexpected Size of Output");
      if (dimension > dimensions.getIntegerValue()-1 || dimension < 0)
         throw new RuntimeException("Unknown Dimension");

      for (int i = 0; i < intervals.getIntegerValue(); i++)
      {
         if (coefficientsToPack.get(i).length != order.getCoefficients())
            throw new RuntimeException("Unexpected Size of Output");

         int index = i * order.getCoefficients() + dimension * order.getCoefficients() * intervals.getIntegerValue();
         System.arraycopy(x.data, index, coefficientsToPack.get(i), 0, order.getCoefficients());
      }
   }

}
