package us.ihmc.robotics.math.trajectories.generators;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.matrixlib.NativeMatrix;
import us.ihmc.robotics.math.trajectories.interfaces.PolynomialBasics;

public class MultiSpline1DSolver
{
   private static final double regularizationWeight = 1E-10;
   public static final int defaultCoefficients = 4;

   public static class WaypointData
   {
      /** Time at which this waypoint should be reached. */
      private double t;
      /** The waypoint position. */
      private double x;
      /** The waypoint velocity. */
      private double xd;
      /**
       * The weight associated with the position.
       * <p>
       * When <tt>w={@value Double#POSITIVE_INFINITY}</tt>, a hard equality constraints is setup to
       * achieve {@code x}, otherwise it is setup as an objective.
       * </p>
       * <p>
       * When <tt>w=0</tt>, the position {@code x} is ignored.
       * </p>
       */
      private double w;
      /**
       * The weight associated with the velocity.
       * <p>
       * When <tt>wd={@value Double#POSITIVE_INFINITY}</tt>, a hard equality constraints is setup to
       * achieve {@code xd}, otherwise it is setup as an objective.
       * </p>
       * <p>
       * When <tt>wd=0</tt>, the velocity {@code xd} is ignored.
       * </p>
       */
      private double wd;
      /** The number of coefficients for the spline succeeding this waypoint. */
      private int numberOfCoefficients;

      public WaypointData()
      {
         clear();
      }

      public void clear()
      {
         t = Double.NaN;
         x = Double.NaN;
         xd = Double.NaN;
         w = 0.0;
         wd = 0.0;
         numberOfCoefficients = defaultCoefficients;
      }

      public void set(double time, double position)
      {
         set(time, position, Double.NaN, Double.POSITIVE_INFINITY, 0.0);
      }

      public void set(double time, double position, double velocity)
      {
         set(time, position, velocity, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
      }

      public void set(double time, double position, double velocity, double positionWeight, double velocityWeight)
      {
         t = time;
         x = position;
         xd = velocity;
         setPositionWeight(positionWeight);
         setVelocityWeight(velocityWeight);
      }

      public void setPositionWeight(double positionWeight)
      {
         checkWeightValue(positionWeight);
         w = positionWeight;
      }

      public void setVelocityWeight(double velocityWeight)
      {
         checkWeightValue(velocityWeight);
         wd = velocityWeight;
      }

      private static void checkWeightValue(double weight)
      {
         if (weight < 0.0)
            throw new IllegalArgumentException("A weight should be in [0, +Infinity[, was: " + weight);
      }

      public void setNumberOfCoefficients(int numberOfCoefficients)
      {
         this.numberOfCoefficients = numberOfCoefficients;
      }

      public double getTime()
      {
         return t;
      }

      public double getPosition()
      {
         return x;
      }

      public double getVelocity()
      {
         return xd;
      }

      public double getPositionWeight()
      {
         return w;
      }

      public double getVelocityWeight()
      {
         return wd;
      }

      public int getNumberOfCoefficients()
      {
         return numberOfCoefficients;
      }
   }

   public class Spline1DSegment
   {
      private WaypointData start, end;
      private int indexOffset;

      public Spline1DSegment()
      {
         clear();
      }

      private void clear()
      {
         start = null;
         end = null;
         indexOffset = -1;
      }

      private void set(WaypointData start, WaypointData end, int indexOffset)
      {
         this.start = start;
         this.end = end;
         this.indexOffset = indexOffset;
      }

      public WaypointData getStart()
      {
         return start;
      }

      public WaypointData getEnd()
      {
         return end;
      }

      public double getCoefficient(int i)
      {
         if (i < 0 || i >= getNumberOfCoefficients())
            throw new IllegalArgumentException("Index out of bounds: " + i + " should be in [0," + getNumberOfCoefficients() + "[.");

         return solution.get(indexOffset + i);
      }

      public int getNumberOfCoefficients()
      {
         return start.numberOfCoefficients;
      }

      public double computePosition(double time)
      {
         time = MathTools.clamp(time, start.t, end.t);

         int coefficientIndex = indexOffset + getNumberOfCoefficients() - 1;
         double tPower = 1.0;
         double x = 0.0;

         for (int i = 0; i < getNumberOfCoefficients(); i++)
         {
            x += solution.get(coefficientIndex--) * tPower;
            tPower *= time;
         }

         return x;
      }

      public double computeVelocity(double time)
      {
         time = MathTools.clamp(time, start.t, end.t);

         int coefficientIndex = indexOffset + getNumberOfCoefficients() - 2;
         double tPower = 1.0;
         double xd = 0.0;

         for (int i = 1; i < getNumberOfCoefficients(); i++)
         {
            xd += solution.get(coefficientIndex--) * tPower * i;
            tPower *= time;
         }

         return xd;
      }

      public double computeAcceleration(double time)
      {
         time = MathTools.clamp(time, start.t, end.t);

         int coefficientIndex = indexOffset + getNumberOfCoefficients() - 3;
         double tPower = 1.0;
         double xdd = 0.0;

         for (int i = 2; i < getNumberOfCoefficients(); i++)
         {
            xdd += solution.get(coefficientIndex--) * tPower * i * (i - 1.0);
            tPower *= time;
         }

         return xdd;
      }

      public void getPolynomial(PolynomialBasics polynomialToPack)
      {
         polynomialToPack.setDirectlyReverse(solution, indexOffset, getNumberOfCoefficients());
      }
   }

   private final RecyclingArrayList<WaypointData> waypoints = new RecyclingArrayList<>(WaypointData::new);
   private final RecyclingArrayList<Spline1DSegment> splineSegments = new RecyclingArrayList<>(Spline1DSegment::new);

   /**
    * Contains the Hessian only for minimizing the integrated acceleration. Allows to compute the
    * integrated acceleration and return it in {@link #solve(DMatrixRMaj)}.
    */
   private final NativeMatrix H_minAccel = new NativeMatrix(1, 1);
   private final DMatrixRMaj H = new DMatrixRMaj(1, 1);
   private final DMatrixRMaj f = new DMatrixRMaj(1, 1);
   private final DMatrixRMaj A = new DMatrixRMaj(1, 1);
   private final DMatrixRMaj ATranspose = new DMatrixRMaj(1, 1);
   private final DMatrixRMaj b = new DMatrixRMaj(1, 1);

   private int lastSize = 1; // E.getNumRows() is rather expensive, keeping track of it size will allow to avoid the method call.
   private final NativeMatrix E = new NativeMatrix(1, 1);
   private final NativeMatrix d = new NativeMatrix(1, 1);
   private final NativeMatrix nativeSolution = new NativeMatrix(1, 1);
   private final DMatrixRMaj solution = new DMatrixRMaj(1, 1);

   public MultiSpline1DSolver()
   {
   }

   /**
    * Clears all waypoint previously added.
    */
   public void clearWaypoints()
   {
      for (int i = 0; i < waypoints.size(); i++)
      {
         waypoints.get(i).clear();
      }
      waypoints.clear();
   }

   public WaypointData addWaypoint()
   {
      return waypoints.add();
   }

   /**
    * Adds a new waypoint to go through at a given time.
    * 
    * @param time     the time at which the position should be reached.
    * @param position the position to go through.
    * @throws IllegalArgumentException if the time is not increasing with respect to the previous
    *                                  waypoint if any.
    */
   public WaypointData addWaypointPosition(double time, double position)
   {
      if (!waypoints.isEmpty() && time <= waypoints.getLast().t)
         throw new IllegalArgumentException("The given time is not greater than the previous waypoint: time=" + time + ", previous waypoint time="
               + waypoints.getLast().t);

      WaypointData waypoint = addWaypoint();
      waypoint.set(time, position);
      return waypoint;
   }

   /**
    * Adds a new waypoint to go through at a given time.
    * <p>
    * Weights should be in <tt>[0, &infin;[</tt>. A weight set to {@link Double#POSITIVE_INFINITY} will
    * set up a hard constraint while any other real value will set up an objective to constrain the
    * corresponding spline.
    * </p>
    * 
    * @param time     the time at which the position should be reached.
    * @param position the position to go through.
    * @param weight   the weight associated with the position.
    * @throws IllegalArgumentException if the time is not increasing with respect to the previous
    *                                  waypoint if any.
    * @throws IllegalArgumentException if the weight is not in the range <tt>[0, &infin;[</tt>.
    */
   public WaypointData addWaypointPosition(double time, double position, double weight)
   {
      if (!waypoints.isEmpty() && time <= waypoints.getLast().t)
         throw new IllegalArgumentException("The given time is not greater than the previous waypoint: time=" + time + ", previous waypoint time="
               + waypoints.getLast().t);

      WaypointData waypoint = addWaypoint();
      waypoint.set(time, position);
      waypoint.setPositionWeight(weight);
      return waypoint;
   }

   /**
    * Adds a new waypoint to go through at a given time.
    * 
    * @param time     the time at which the position should be reached.
    * @param position the position to go through.
    * @return
    * @throws IllegalArgumentException if the time is not increasing with respect to the previous
    *                                  waypoint if any.
    */
   public WaypointData addWaypoint(double time, double position, double velocity)
   {
      if (!waypoints.isEmpty() && time <= waypoints.getLast().t)
         throw new IllegalArgumentException("The given time is not greater than the previous waypoint: time=" + time + ", previous waypoint time="
               + waypoints.getLast().t);

      WaypointData waypoint = addWaypoint();
      waypoint.set(time, position, velocity);
      return waypoint;
   }

   /**
    * Solves for the coefficients of the cubic splines.
    * 
    * @param solutionToPack the matrix used to store the splines coefficients as column vector. By
    *                       default, there is 4 coefficients per spline, for each spline they are
    *                       sorted from highest to lowest order. Modified.
    * @return the acceleration integrated over the trajectory time.
    */
   public double solveAndComputeCost()
   {
      solve();
      return computeCost();
   }

   /**
    * Solves for the coefficients of the cubic splines and returns the acceleration integrated over the
    * time <tt>[0, 1]</tt>.
    * 
    * @param solutionToPack the matrix used to store the cubic spline coefficients as column vector.
    *                       There is 4 coefficients per spline, for each spline they are sorted from
    *                       highest to lowest order. Modified.
    */
   public void solve()
   {
      if (waypoints.size() < 2)
         throw new IllegalStateException("Not enough waypoints.");

      buildCostFunction(H_minAccel, H, f);
      buildKnotEqualityConstraints(A, b);

      int subProblemSize = getProblemSize();
      int constraints = computeNumberOfEqualityConstraints();

      // min 0.5*x'*H*x + f'*x
      // s.t. A*x == b
      int size = subProblemSize + constraints;

      if (lastSize != size)
      {
         E.reshape(size, size);
         d.reshape(size, 1);
         E.zero();
         lastSize = size;
      }

      E.insert(H, 0, 0);
      E.insert(A, subProblemSize, 0);
      ATranspose.reshape(A.getNumCols(), A.getNumRows());
      CommonOps_DDRM.transpose(A, ATranspose);
      E.insert(ATranspose, 0, subProblemSize);
      CommonOps_DDRM.scale(-1.0, f);
      d.insert(f, 0, 0);
      d.insert(b, subProblemSize, 0);

      nativeSolution.solve(E, d);
      nativeSolution.reshape(subProblemSize, 1);
      nativeSolution.get(solution);

      int indexOffset = 0;

      for (int i = 0; i < waypoints.size() - 1; i++)
      {
         WaypointData start = waypoints.get(i);
         WaypointData end = waypoints.get(i + 1);

         Spline1DSegment segment = splineSegments.getAndGrowIfNeeded(i);
         segment.set(start, end, indexOffset);
         indexOffset += start.numberOfCoefficients;
      }

      while (splineSegments.size() > waypoints.size() - 1)
      {
         splineSegments.getLast().clear();
         splineSegments.remove(splineSegments.size() - 1);
      }
   }

   public DMatrixRMaj getSolution()
   {
      return solution;
   }

   public double computeCost()
   {
      // d = x^T H x
      d.multQuad(nativeSolution, H_minAccel);
      return 0.5 * d.get(0, 0);
   }

   public int getProblemSize()
   {
      int size = 0;

      for (int i = 0; i < waypoints.size() - 1; i++)
      {
         size += waypoints.get(i).getNumberOfCoefficients();
      }

      return size;
   }

   /**
    * Sets up the equality constraints used for the QP:
    * <ul>
    * <li>at <tt>t=0</tt>: position and velocity are equal to {@code x0} and {@code xd0}.
    * <li>at <tt>t=1</tt>: position and velocity are equal to {@code x1} and {@code xd1}.
    * <li>at <tt>t=t<sub>i</sub></tt> with <tt>i&in;[0;xi.size()[</tt>: position equals
    * <tt>x<sub>i</sub></tt>, and velocity of the 2 cubic segments joining at the waypoint are equal.
    * </ul>
    */
   private void buildKnotEqualityConstraints(DMatrixRMaj A, DMatrixRMaj b)
   {
      int constraints = computeNumberOfEqualityConstraints();
      int subProblemSize = getProblemSize();
      A.reshape(constraints, subProblemSize);
      b.reshape(constraints, 1);
      CommonOps_DDRM.fill(A, 0.0);

      int constraintIndex = 0;
      int splineOffset = 0;

      // add initial condition
      WaypointData waypoint = waypoints.getFirst();
      int nCoeffs = waypoint.numberOfCoefficients;

      if (waypoint.w == Double.POSITIVE_INFINITY)
      {
         getPositionConstraintABlock(waypoint.t, nCoeffs, constraintIndex, splineOffset, A);
         b.set(constraintIndex, waypoint.x);
         constraintIndex++;
      }

      if (waypoint.wd == Double.POSITIVE_INFINITY)
      {
         getVelocityConstraintABlock(waypoint.t, nCoeffs, constraintIndex, splineOffset, A);
         b.set(constraintIndex, waypoint.xd);
         constraintIndex++;
      }

      for (int i = 1; i < waypoints.size() - 1; i++)
      {
         nCoeffs = waypoint.numberOfCoefficients;
         waypoint = waypoints.get(i);
         int nextSplineOffset = splineOffset + nCoeffs;

         if (waypoint.w != Double.POSITIVE_INFINITY)
         {
            // Either:
            // - No position target
            // - The position target is setup as an objective (it's flexible)
            // For both case we need to enforce continuity by adding a position matching constraint between the 2 splines
            getPositionConstraintABlock(waypoint.t, nCoeffs, constraintIndex, splineOffset, A);
            MatrixTools.setMatrixBlock(A, constraintIndex, nextSplineOffset, A, constraintIndex, splineOffset, 1, nCoeffs, -1.0);
            b.set(constraintIndex, 0.0);
            constraintIndex++;
         }
         else
         {
            getPositionConstraintABlock(waypoint.t, nCoeffs, constraintIndex, splineOffset, A);
            b.set(constraintIndex, waypoint.x);
            CommonOps_DDRM.extract(A, constraintIndex, constraintIndex + 1, splineOffset, nextSplineOffset, A, constraintIndex + 1, nextSplineOffset);
            constraintIndex++;
            b.set(constraintIndex, waypoint.x);
            constraintIndex++;
         }

         if (waypoint.wd != Double.POSITIVE_INFINITY)
         { // Same idea as for the position
            getVelocityConstraintABlock(waypoint.t, nCoeffs, constraintIndex, splineOffset, A);
            MatrixTools.setMatrixBlock(A, constraintIndex, nextSplineOffset, A, constraintIndex, splineOffset, 1, nCoeffs, -1.0);
            b.set(constraintIndex, 0.0);
            constraintIndex++;
         }
         else
         {
            getVelocityConstraintABlock(waypoint.t, nCoeffs, constraintIndex, splineOffset, A);
            b.set(constraintIndex, waypoint.xd);
            CommonOps_DDRM.extract(A, constraintIndex, constraintIndex + 1, splineOffset, nextSplineOffset, A, constraintIndex + 1, nextSplineOffset);
            constraintIndex++;
            b.set(constraintIndex, waypoint.xd);
            constraintIndex++;
         }

         splineOffset = nextSplineOffset;
      }

      // add final condition
      waypoint = waypoints.getLast();

      if (waypoint.w == Double.POSITIVE_INFINITY)
      {
         getPositionConstraintABlock(waypoint.t, nCoeffs, constraintIndex, splineOffset, A);
         b.set(constraintIndex, waypoint.x);
         constraintIndex++;
      }
      if (waypoint.wd == Double.POSITIVE_INFINITY)
      {
         getVelocityConstraintABlock(waypoint.t, nCoeffs, constraintIndex, splineOffset, A);
         b.set(constraintIndex, waypoint.xd);
      }
   }

   /**
    * Evaluates and returns the actual number of equality constraints by looking at the value of each
    * individual weight.
    */
   private int computeNumberOfEqualityConstraints()
   {
      int constraints = 0;

      WaypointData waypoint = waypoints.getFirst();
      if (waypoint.w == Double.POSITIVE_INFINITY)
         constraints++;
      if (waypoint.wd == Double.POSITIVE_INFINITY)
         constraints++;

      for (int i = 1; i < waypoints.size() - 1; i++)
      {
         waypoint = waypoints.get(i);

         if (waypoint.w == Double.POSITIVE_INFINITY)
            constraints += 2;
         else
            constraints++;

         if (waypoint.wd == Double.POSITIVE_INFINITY)
            constraints += 2;
         else
            constraints++;
      }

      waypoint = waypoints.getLast();
      if (waypoint.w == Double.POSITIVE_INFINITY)
         constraints++;
      if (waypoint.wd == Double.POSITIVE_INFINITY)
         constraints++;

      return constraints;
   }

   /**
    * Assembles the cost function necessary to minimize the integrated acceleration and achieve the
    * given inputs if the weights are different from {@link Double#POSITIVE_INFINITY}.
    * 
    * @param H_minAccel can be used to compute the integrated acceleration.
    */
   private void buildCostFunction(NativeMatrix H_minAccel, DMatrixRMaj H, DMatrixRMaj f)
   {
      int size = getProblemSize();

      f.reshape(size, 1);
      CommonOps_DDRM.fill(f, regularizationWeight);

      H.reshape(size, size);
      H.zero();
      getMinAccelerationCostFunction(H);
      H_minAccel.set(H);

      addKnotsCostFunction(H, f);
   }

   private void getMinAccelerationCostFunction(DMatrixRMaj H)
   {
      int splineOffset = 0;

      for (int i = 0; i < waypoints.size() - 1; i++)
      {
         WaypointData w0 = waypoints.get(i);
         WaypointData w1 = waypoints.get(i + 1);
         getMinAccelerationHBlock(w0.t, w1.t, w0.numberOfCoefficients, splineOffset, splineOffset, H);
         splineOffset += w0.numberOfCoefficients;
      }
   }

   /**
    * Assembles as needed the objective function to satisfy each of the given inputs.
    */
   private void addKnotsCostFunction(DMatrixRMaj H, DMatrixRMaj f)
   {
      int splineOffset = 0;

      WaypointData waypoint = waypoints.getFirst();
      int nCoeffs = waypoint.numberOfCoefficients;

      if (waypoint.w != Double.POSITIVE_INFINITY && waypoint.w > 0.0)
         addPositionObjective(waypoint.t, waypoint.x, waypoint.w, nCoeffs, splineOffset, splineOffset, H, f);
      if (waypoint.wd != Double.POSITIVE_INFINITY && waypoint.wd > 0.0)
         addVelocityObjective(waypoint.t, waypoint.xd, waypoint.wd, nCoeffs, splineOffset, splineOffset, H, f);

      for (int i = 1; i < waypoints.size() - 1; i++)
      {
         nCoeffs = waypoint.numberOfCoefficients;
         waypoint = waypoints.get(i);

         if (waypoint.w != Double.POSITIVE_INFINITY && waypoint.w > 0.0)
         {
            addPositionObjective(waypoint.t, waypoint.x, waypoint.w, nCoeffs, splineOffset, splineOffset, H, f);
            // No need to add the objective for the next spline as we have continuity enforced as a constraint in buildKnotEqualityConstraints(...)
            // addPositionObjective(waypoint.t, waypoint.x, waypoint.w, waypoint.numberOfCoefficients, splineOffset + waypoint.numberOfCoefficients, splineOffset + waypoint.numberOfCoefficients, H, f);
         }

         if (waypoint.wd != Double.POSITIVE_INFINITY && waypoint.wd > 0.0)
         {
            addVelocityObjective(waypoint.t, waypoint.xd, waypoint.wd, nCoeffs, splineOffset, splineOffset, H, f);
            // No need to add the objective for the next spline as we have continuity enforced as a constraint in buildKnotEqualityConstraints(...)
            // addVelocityObjective(waypoint.t, waypoint.xd, waypoint.wd, waypoint.numberOfCoefficients, splineOffset + waypoint.numberOfCoefficients, splineOffset + waypoint.numberOfCoefficients, H, f);
         }

         splineOffset += nCoeffs;
      }

      waypoint = waypoints.getLast();

      if (waypoint.w != Double.POSITIVE_INFINITY && waypoint.w > 0.0)
         addPositionObjective(waypoint.t, waypoint.x, waypoint.w, nCoeffs, splineOffset, splineOffset, H, f);
      if (waypoint.wd != Double.POSITIVE_INFINITY && waypoint.wd > 0.0)
         addVelocityObjective(waypoint.t, waypoint.xd, waypoint.wd, nCoeffs, splineOffset, splineOffset, H, f);
   }

   public double computePosition(double time)
   {
      time = MathTools.clamp(time, waypoints.getFirst().t, waypoints.getLast().t);
      return findSplineSegment(time).computePosition(time);
   }

   public double computeVelocity(double time)
   {
      time = MathTools.clamp(time, waypoints.getFirst().t, waypoints.getLast().t);
      return findSplineSegment(time).computeVelocity(time);
   }

   public double computeAcceleration(double time)
   {
      time = MathTools.clamp(time, waypoints.getFirst().t, waypoints.getLast().t);
      return findSplineSegment(time).computeAcceleration(time);
   }

   public WaypointData getWaypoint(int index)
   {
      return waypoints.get(index);
   }

   public Spline1DSegment findSplineSegment(double time)
   {
      if (time < splineSegments.getFirst().getStart().t)
         return null;

      for (int i = 0; i < splineSegments.size(); i++)
      {
         Spline1DSegment segment = splineSegments.get(i);
         if (time <= segment.getEnd().t)
            return segment;
      }

      return null;
   }

   public Spline1DSegment getSplineSegment(int index)
   {
      return splineSegments.get(index);
   }

   /**
    * Inserts the following matrix block into {@code A} used for constraining the spline position at
    * time equals {@code t}. For {@code numberOfCoefficients == N}, we have:
    * 
    * <pre>
    * ABlock = [ t<sup>N-1</sup> t<sup>N-2</sup> ... t 1 ]
    * </pre>
    */
   static void getPositionConstraintABlock(double t, int numberOfCoefficients, int row, int startColumn, DMatrixRMaj A)
   {
      double tpow = 1.0;

      int column = startColumn + numberOfCoefficients - 1;

      for (int i = 0; i < numberOfCoefficients; i++)
      {
         A.set(row, column--, tpow);
         tpow *= t;
      }
   }

   /**
    * Inserts the following matrix block into {@code A} used for constraining the spline velocity at
    * time equals {@code t}. For {@code numberOfCoefficients == N}, we have:
    * 
    * <pre>
    * ABlock = [ (N-1)t<sup>N-2</sup> (N-2)t<sup>N-3</sup> ... 1 0 ]
    * </pre>
    */
   static void getVelocityConstraintABlock(double t, int numberOfCoefficients, int row, int startColumn, DMatrixRMaj A)
   {
      double tpow = 1.0;

      int column = startColumn + numberOfCoefficients - 2;

      for (int i = 1; i < numberOfCoefficients; i++)
      {
         A.set(row, column--, i * tpow);
         tpow *= t;
      }
   }

   /**
    * Inserts the matrix block into {@code H} for minimizing the integral of the squared acceleration
    * function for a spline segment.
    * <p>
    * For {@code numberOfCoefficients == 4}, we have:<br>
    * The integrated squared acceleration function from {@code t0} to {@code t1} is:<br>
    * <tt> &int; xDDot(t)<sup>2</sup> = 12 c0^2 (t1^3 - t0^3) + 12 c0 c1 (t1^2 - t0^2) + 4 c1^2 (t1 - t0)</tt>
    * 
    * <pre>
    * HBlock = / 12 * (t1<sup>3</sup> - t0<sup>3</sup>)    6 * (t1<sup>2</sup> - t0<sup>2</sup>) \
    *          \  6 * (t1<sup>2</sup> - t0<sup>2</sup>)    4 * (t1<sup> </sup> - t0<sup> </sup>) /
    * </pre>
    * </p>
    * <p>
    * For {@code numberOfCoefficients == 3}, we have:<br>
    * The integrated squared acceleration function from {@code t0} to {@code t1} is:<br>
    * <tt> &int; xDDot(t)<sup>2</sup> = 4 c0^2 (t1 - t0)</tt>
    * 
    * <pre>
    * HBlock(startRow, startColumn) = 4 * (t1 - t0)
    * </pre>
    * </p>
    * <p>
    * For {@code numberOfCoefficients <= 2} the acceleration is zero, {@code H} is unchanged.
    * </p>
    */
   static void getMinAccelerationHBlock(double t0, double t1, int numberOfCoefficients, int startRow, int startColumn, DMatrixRMaj H)
   {
      if (numberOfCoefficients == 4)
      {
         double t0pow = t0;
         double t1pow = t1;
         H.set(startRow + 1, startColumn + 1, 4.0 * (t1pow - t0pow));
         t0pow *= t0;
         t1pow *= t1;
         H.set(startRow + 1, startColumn + 0, 6.0 * (t1pow - t0pow));
         H.set(startRow + 0, startColumn + 1, 6.0 * (t1pow - t0pow));
         t0pow *= t0;
         t1pow *= t1;
         H.set(startRow + 0, startColumn + 0, 12.0 * (t1pow - t0pow));
      }
      else if (numberOfCoefficients == 3)
      {
         H.set(startRow, startColumn, 4.0 * (t1 - t0));
      }
      else if (numberOfCoefficients <= 2)
      {
         // The acceleration is 0, do nothing.
      }
      else
      {
         throw new UnsupportedOperationException("Implement me for the following number of coefficients: " + numberOfCoefficients);
      }
   }

   /**
    * Adds the following matrix blocks into {@code H} at [{@code startRow}, {@code startColumn}] and
    * into {@code f} at [{@code startRow}, {@code 0}]. For {@code numberOfCoefficients == 4}, we have:
    * 
    * <pre>
    *                   / t<sup>6</sup> t<sup>5</sup> t<sup>4</sup> t<sup>3</sup> \
    * HBlock = weight * | t<sup>5</sup> t<sup>4</sup> t<sup>3</sup> t<sup>2</sup> | ( = A<sup>T</sup> W A )
    *                   | t<sup>4</sup> t<sup>3</sup> t<sup>2</sup> t<sup> </sup> |
    *                   \ t<sup>3</sup> t<sup>2</sup> t<sup> </sup> 1<sup> </sup> /
    * 
    *                        / t<sup>3</sup> \
    * fBlock = -x * weight * | t<sup>2</sup> | ( = -A<sup>T</sup> W x )
    *                        | t<sup> </sup> |
    *                        \ 1<sup> </sup> /
    * </pre>
    */
   static void addPositionObjective(double t, double x, double weight, int numberOfCoefficients, int startRow, int startColumn, DMatrixRMaj H, DMatrixRMaj f)
   {
      double tpow = weight;

      // We traverse the H matrix in diagonals
      int numberOfDiagonals = 2 * numberOfCoefficients - 1;

      for (int diagonal = numberOfDiagonals - 1; diagonal >= 0; diagonal--)
      {
         int offsetCol = Math.max(0, diagonal - numberOfCoefficients + 1);
         int offsetRow = Math.min(diagonal, numberOfCoefficients - 1);
         int size = min(diagonal + 1, numberOfCoefficients - offsetCol, numberOfCoefficients);

         for (int i = 0; i < size; i++)
         {
            H.add(startRow + offsetRow, startColumn + offsetCol, tpow);
            offsetRow--;
            offsetCol++;
         }
         tpow *= t;
      }

      int f_index = startRow + numberOfCoefficients - 1;
      tpow = -weight * x;

      for (int i = 0; i < numberOfCoefficients; i++)
      {
         f.add(f_index--, 0, tpow);
         tpow *= t;
      }
   }

   /**
    * Inserts the following matrix blocks into {@code H} at [{@code startRow}, {@code startColumn}] and
    * into {@code f} at [{@code startRow}, {@code 0}]:
    * 
    * <pre>
    *                   / 9t<sup>4</sup> 6t<sup>3</sup> 3t<sup>2</sup> 0 \
    * HBlock = weight * | 6t<sup>3</sup> 4t<sup>2</sup> 2t<sup> </sup> 0 | ( = A<sup>T</sup> W A )
    *                   | 3t<sup>2</sup> 2t<sup> </sup>  1<sup> </sup> 0 |
    *                   \  0<sup> </sup>  0<sup> </sup>  0<sup> </sup> 0 /
    * 
    *                         / 3t<sup>2</sup> \
    * fBlock = -xd * weight * | 2t<sup> </sup> | ( = -A<sup>T</sup> W xd )
    *                         |  1<sup> </sup> |
    *                         \  0<sup> </sup> /
    * </pre>
    */
   static void addVelocityObjective(double t, double xd, double weight, int numberOfCoefficients, int startRow, int startColumn, DMatrixRMaj H, DMatrixRMaj f)
   {
      double tpow = weight;

      // We traverse the H matrix in diagonals, while skipping the last row and last column
      int blockSize = numberOfCoefficients - 1;
      int numberOfDiagonals = 2 * blockSize - 1;

      for (int diagonal = numberOfDiagonals - 1; diagonal >= 0; diagonal--)
      {
         int offsetCol = Math.max(0, diagonal - blockSize + 1);
         int offsetRow = Math.min(diagonal, blockSize - 1);
         int size = min(diagonal + 1, blockSize - offsetCol, blockSize);

         for (int i = 0; i < size; i++)
         {
            H.add(startRow + offsetRow, startColumn + offsetCol, tpow);
            offsetRow--;
            offsetCol++;
         }
         tpow *= t;
      }

      int f_index = startRow + blockSize - 1;
      tpow = -weight * xd;

      for (int i = 0; i < blockSize; i++)
      {
         f.add(f_index--, 0, tpow);
         tpow *= t;
      }
   }

   /**
    * Find and return the argument with the minimum value.
    *
    * @param a the first argument to compare.
    * @param b the second argument to compare.
    * @param c the third argument to compare.
    * @return the minimum value of the three arguments.
    */
   public static final int min(int a, int b, int c)
   {
      if (a < b)
         return a < c ? a : c;
      else
         return b < c ? b : c;
   }

   public static void main(String[] args)
   {
      int numberOfCoefficients = 4;

      DMatrixRMaj d = new DMatrixRMaj(4, 4);

      int numberOfDiagonals = 2 * numberOfCoefficients - 1;

      for (int diagonal = numberOfDiagonals; diagonal >= 0; diagonal--)
      {
         int offsetCol = Math.max(0, diagonal - numberOfCoefficients + 1);
         int offsetRow = Math.min(diagonal, numberOfCoefficients - 1);
         int size = min(diagonal + 1, numberOfCoefficients - offsetCol, numberOfCoefficients);

         for (int i = 0; i < size; i++)
         {
            System.out.println(offsetRow + ", " + offsetCol + ", " + size);
            d.set(offsetRow, offsetCol, diagonal);
            offsetRow--;
            offsetCol++;
         }
      }

      System.out.println(d);
   }
}
