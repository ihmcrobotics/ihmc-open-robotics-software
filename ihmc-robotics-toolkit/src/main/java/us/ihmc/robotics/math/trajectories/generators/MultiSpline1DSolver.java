package us.ihmc.robotics.math.trajectories.generators;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.commons.lists.SupplierBuilder;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.matrixlib.NativeMatrix;
import us.ihmc.robotics.math.trajectories.interfaces.PolynomialBasics;

/**
 * Similar to {@link MultiCubicSpline1DSolver}, this solver is more general in a few ways:
 * <ul>
 * <li>It can solve for spline segments that can be cubic or other order.
 * <li>The initial time is not assumed to be 0 and the final time is not assumed to be 1.
 * <li>Velocity at waypoints can be imposed.
 * </ul>
 */
public class MultiSpline1DSolver
{
   public static final int defaultCoefficients = 4;

   public class WaypointData
   {
      /** The index of this waypoint. */
      private final int index;
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
      /**
       * The weight associated with minimizing the acceleration at this waypoint.
       */
      private double wdd;

      private WaypointData(int index)
      {
         this.index = index;
         clear();
      }

      /**
       * Clears this waypoint internal data, intended for internal use.
       */
      public void clear()
      {
         t = Double.NaN;
         clearPosition();
         clearVelocity();
         wdd = 0.0;
      }

      /**
       * Clears the position constraint/objective, essentially freeing the position to be reached at this
       * waypoint's time.
       */
      public void clearPosition()
      {
         x = Double.NaN;
         w = 0.0;
      }

      /**
       * Clears the velocity constraint/objective, essentially freeing the velocity to be reached at this
       * waypoint's time.
       */
      public void clearVelocity()
      {
         xd = Double.NaN;
         wd = 0.0;
      }

      /**
       * Sets up this waypoint with time and a position constraint. The velocity is unconstrained.
       * 
       * @param time     this waypoint time.
       * @param position the position to reach for this waypoint. It is considered as a hard constraint.
       */
      public void set(double time, double position)
      {
         set(time, position, Double.NaN, Double.POSITIVE_INFINITY, 0.0);
      }

      /**
       * Sets up this waypoint with time, and position and velocity constraints.
       * 
       * @param time     this waypoint time.
       * @param position the position to reach for this waypoint. It is considered as a hard constraint.
       * @param velocity the velocity to reach for this waypoint. It is considered as a hard constraint.
       */
      public void set(double time, double position, double velocity)
      {
         set(time, position, velocity, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
      }

      /**
       * Sets up this waypoint with time, and position and velocity objectives/constraints.
       * 
       * @param time           this waypoint time.
       * @param position       the position to reach for this waypoint.
       * @param velocity       the velocity to reach for this waypoint.
       * @param positionWeight the weight associated with the position, {@code Double#POSITIVE_INFINITY}
       *                       to make it a hard constraint, or {@code 0} to free the position.
       * @param velocityWeight the weight associated with the velocity, {@code Double#POSITIVE_INFINITY}
       *                       to make it a hard constraint, or {@code 0} to free the velocity.
       */
      public void set(double time, double position, double velocity, double positionWeight, double velocityWeight)
      {
         t = time;
         x = position;
         xd = velocity;
         setPositionWeight(positionWeight);
         setVelocityWeight(velocityWeight);
      }

      /**
       * Sets the position for this waypoint.
       * 
       * @param position the new waypoint position.
       */
      public void setPosition(double position)
      {
         x = position;
      }

      /**
       * Sets the weight to use for the position.
       * 
       * @param positionWeight the weight associated with the position, {@code Double#POSITIVE_INFINITY}
       *                       to make it a hard constraint, or {@code 0} to free the position.
       */
      public void setPositionWeight(double positionWeight)
      {
         checkWeightValue(positionWeight);
         w = positionWeight;
      }

      /**
       * Sets the velocity for this waypoint.
       * 
       * @param velocity the new waypoint velocity.
       */
      public void setVelocity(double velocity)
      {
         xd = velocity;
      }

      /**
       * Sets the weight to use for the velocity.
       * 
       * @param velocityWeight the weight associated with the velocity, {@code Double#POSITIVE_INFINITY}
       *                       to make it a hard constraint, or {@code 0} to free the velocity.
       */
      public void setVelocityWeight(double velocityWeight)
      {
         checkWeightValue(velocityWeight);
         wd = velocityWeight;
      }

      public void setAccelerationWeight(double accelerationWeight)
      {
         if (accelerationWeight < 0.0 || !Double.isFinite(accelerationWeight))
            throw new IllegalArgumentException("The acceleration weight should be a finite positive number, was: " + accelerationWeight);
         wdd = accelerationWeight;
      }

      private static void checkWeightValue(double weight)
      {
         if (weight < 0.0)
            throw new IllegalArgumentException("A weight should be in [0, +Infinity[, was: " + weight);
      }

      public int getIndex()
      {
         return index;
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

      /**
       * Whether there is a waypoint after this one or this waypoint represents the final conditions.
       * 
       * @return {@code true} if there a next.
       */
      public boolean hasNext()
      {
         return index < waypoints.size() - 1;
      }

      /**
       * The next waypoint or {@code null} if this is the last.
       * 
       * @return the next waypoint.
       */
      public WaypointData next()
      {
         if (hasNext())
            return waypoints.get(index + 1);
         else
            return null;
      }

      /**
       * Whether there is a waypoint before this one or this waypoint represents the initial conditions.
       * 
       * @return {@code true} if there a previous.
       */
      public boolean hasPrevious()
      {
         return index > 0;
      }

      /**
       * The previous waypoint or {@code null} if this is the first.
       * 
       * @return the previous waypoint.
       */
      public WaypointData previous()
      {
         if (hasPrevious())
            return waypoints.get(index - 1);
         else
            return null;
      }

      /**
       * The spline segment that joins this waypoint to the next waypoint.
       * 
       * @return the next spline segment.
       */
      public Spline1DSegment getNextSpline()
      {
         if (index < splineSegments.size())
            return splineSegments.get(index);
         else
            return null;
      }

      /**
       * The spline segment that joins the previous waypoint to this waypoint.
       * 
       * @return the previous spline segment.
       */
      public Spline1DSegment getPreviousSpline()
      {
         if (hasPrevious())
            return previous().getNextSpline();
         else
            return null;
      }

      @Override
      public String toString()
      {
         return "[index=" + index + ", t=" + t + ", x=" + x + ", xd=" + xd + ", w=" + w + ", wd=" + wd + "]";
      }
   }

   public class Spline1DSegment
   {
      private final int index;
      private int indexFirstCoefficient;
      /** The number of coefficients for the spline succeeding this waypoint. */
      private int numberOfCoefficients;

      /**
       * Weight used to prioritize which segment acceleration should be minimize more.
       * <p>
       * This weight is used in the objective minimize the integral of the square acceleration of the
       * spline.
       * </p>
       */
      private double accelerationWeight = 1.0;

      public Spline1DSegment(int index)
      {
         this.index = index;
         clear();
      }

      /**
       * Rests this spline segment internal data, intended for internal use.
       */
      private void clear()
      {
         indexFirstCoefficient = -1;
         numberOfCoefficients = defaultCoefficients;
         accelerationWeight = 1.0;
      }

      private void update()
      {
         indexFirstCoefficient = 0;
         Spline1DSegment previous = previous();
         if (previous != null)
         {
            previous.update();
            indexFirstCoefficient = previous.getIndexFirstCoefficient() + previous.getNumberOfCoefficients();
         }
      }

      /**
       * Sets the order of the polynomial for this spline segment.
       * 
       * @param numberOfCoefficients the number of coefficients defining the polynomial order.
       */
      public void setNumberOfCoefficients(int numberOfCoefficients)
      {
         this.numberOfCoefficients = numberOfCoefficients;
         splineSegments.getLast().update(); // Update the index offsets of all spline segments.
      }

      public double getAccelerationWeight()
      {
         return accelerationWeight;
      }

      public void setAccelerationWeight(double accelerationWeight)
      {
         this.accelerationWeight = accelerationWeight;
      }

      /**
       * The waypoint from which this spline segment starts.
       * 
       * @return the start.
       */
      public WaypointData getStart()
      {
         return waypoints.get(index);
      }

      /**
       * The waypoint to which this spline segment ends.
       * 
       * @return the end.
       */
      public WaypointData getEnd()
      {
         return waypoints.get(index + 1);
      }

      /**
       * [Solver Output] - Gets the value of the i<sup>th</sup> coefficient for this spline segment. The
       * coefficients for the spline segment are order from highest order to lowest:
       * 
       * <pre>
       * p(x) = a<sup>N</sup> + a<sup>N-1</sup> x + a<sup>N-2</sup> x^2 + ... + a<sup>0</sup> x^N
       * </pre>
       * 
       * N+1 is the number of coefficients for this polynomial.
       * 
       * @param i the coefficient index, 0 the highest order, N is the polynomial constant.
       * @return
       */
      public double getCoefficient(int i)
      {
         if (i < 0 || i >= getNumberOfCoefficients())
            throw new IllegalArgumentException("Index out of bounds: " + i + " should be in [0," + getNumberOfCoefficients() + "[.");

         return solution.get(indexFirstCoefficient + i);
      }

      /**
       * Index of the first coefficient for this spline segment in
       * {@link MultiSpline1DSolver#getSolution()}.
       * 
       * @return the index of the first coefficient in the solver's solution matrix.
       */
      public int getIndexFirstCoefficient()
      {
         return indexFirstCoefficient;
      }

      /**
       * The number of coefficients for this spline segment.
       * 
       * @return the number of coefficients.
       */
      public int getNumberOfCoefficients()
      {
         return numberOfCoefficients;
      }

      public double computePosition(double time)
      {
         time = MathTools.clamp(time, getStart().t, getEnd().t);

         int coefficientIndex = indexFirstCoefficient + getNumberOfCoefficients() - 1;
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
         time = MathTools.clamp(time, getStart().t, getEnd().t);

         int coefficientIndex = indexFirstCoefficient + getNumberOfCoefficients() - 2;
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
         time = MathTools.clamp(time, getStart().t, getEnd().t);

         int coefficientIndex = indexFirstCoefficient + getNumberOfCoefficients() - 3;
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
         polynomialToPack.setDirectlyReverse(solution, indexFirstCoefficient, getNumberOfCoefficients());
      }

      public boolean hasNext()
      {
         return index < splineSegments.size() - 1;
      }

      public Spline1DSegment next()
      {
         if (hasNext())
            return splineSegments.get(index + 1);
         else
            return null;
      }

      public boolean hasPrevious()
      {
         return index > 0;
      }

      public Spline1DSegment previous()
      {
         if (hasPrevious())
            return splineSegments.get(index - 1);
         else
            return null;
      }

      @Override
      public String toString()
      {
         return "[index=" + index + ", numberOfCoefficients=" + numberOfCoefficients + "]";
      }
   }

   private final RecyclingArrayList<WaypointData> waypoints = new RecyclingArrayList<>(SupplierBuilder.indexedSupplier(WaypointData::new));
   private final RecyclingArrayList<Spline1DSegment> splineSegments = new RecyclingArrayList<>(SupplierBuilder.indexedSupplier(Spline1DSegment::new));

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

      for (int i = 0; i < splineSegments.size(); i++)
      {
         splineSegments.get(i).clear();
      }
      splineSegments.clear();
   }

   public WaypointData addWaypoint()
   {
      if (!waypoints.isEmpty())
         splineSegments.add().update();
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
      return integratedAccelerationSquared();
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
   }

   public DMatrixRMaj getSolution()
   {
      return solution;
   }

   public double integratedAccelerationSquared()
   {
      // d = x^T H x
      d.multQuad(nativeSolution, H_minAccel);
      return 0.5 * d.get(0, 0);
   }

   public int getProblemSize()
   {
      int size = 0;

      for (int i = 0; i < splineSegments.size(); i++)
      {
         size += splineSegments.get(i).getNumberOfCoefficients();
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

      for (int i = 0; i < waypoints.size(); i++)
      {
         WaypointData waypoint = waypoints.get(i);

         if (waypoint.w == Double.POSITIVE_INFINITY)
            constraintIndex = addDesiredWaypointPositionConstraint(waypoint, constraintIndex, A, b);
         else
            constraintIndex = addPositionContinuityConstraint(waypoint, constraintIndex, A, b);

         if (waypoint.wd == Double.POSITIVE_INFINITY)
            constraintIndex = addDesiredWaypointVelocityConstraint(waypoint, constraintIndex, A, b);
         else
            constraintIndex = addVelocityContinuityConstraint(waypoint, constraintIndex, A, b);
      }
   }

   /**
    * Adds an equality constraint such that the preceding and succeeding splines of the given
    * {@code waypoint} match in position at the waypoint time.
    */
   static int addPositionContinuityConstraint(WaypointData waypoint, int constraintIndex, DMatrixRMaj A, DMatrixRMaj b)
   {
      Spline1DSegment spline0 = waypoint.getPreviousSpline();
      Spline1DSegment spline1 = waypoint.getNextSpline();

      if (spline0 == null || spline1 == null)
         return constraintIndex;

      int nCoeffs0 = spline0.getNumberOfCoefficients();
      int nCoeffs1 = spline1.getNumberOfCoefficients();

      int i0 = spline0.getIndexFirstCoefficient();
      int i1 = spline1.getIndexFirstCoefficient();

      double t = waypoint.t;

      getPositionConstraintABlock(t, nCoeffs0, constraintIndex, i0, A);

      if (nCoeffs0 == nCoeffs1)
         MatrixTools.setMatrixBlock(A, constraintIndex, i1, A, constraintIndex, i0, 1, nCoeffs0, -1.0);
      else
         getPositionConstraintABlock(t, nCoeffs1, true, constraintIndex, i1, A);

      b.set(constraintIndex, 0.0);
      return constraintIndex + 1;
   }

   /**
    * Adds an equality constraint such that the preceding and succeeding splines of the given
    * {@code waypoint} match in velocity at the waypoint time.
    */
   static int addVelocityContinuityConstraint(WaypointData waypoint, int constraintIndex, DMatrixRMaj A, DMatrixRMaj b)
   {
      Spline1DSegment spline0 = waypoint.getPreviousSpline();
      Spline1DSegment spline1 = waypoint.getNextSpline();

      if (spline0 == null || spline1 == null)
         return constraintIndex;

      int i0 = spline0.getIndexFirstCoefficient();
      int i1 = spline1.getIndexFirstCoefficient();

      int n0 = spline0.getNumberOfCoefficients();
      int n1 = spline1.getNumberOfCoefficients();

      double t = waypoint.t;

      getVelocityConstraintABlock(t, n0, constraintIndex, i0, A);

      if (n0 == n1)
         MatrixTools.setMatrixBlock(A, constraintIndex, i1, A, constraintIndex, i0, 1, n0, -1.0);
      else
         getVelocityConstraintABlock(t, n1, true, constraintIndex, i1, A);

      b.set(constraintIndex, 0.0);
      return constraintIndex + 1;
   }

   /**
    * Adds an equality constraint such that the preceding and succeeding splines of the given
    * {@code waypoint} satisfy the waypoint position.
    */
   static int addDesiredWaypointPositionConstraint(WaypointData waypoint, int constraintIndex, DMatrixRMaj A, DMatrixRMaj b)
   {
      Spline1DSegment spline0 = waypoint.getPreviousSpline();
      Spline1DSegment spline1 = waypoint.getNextSpline();

      double t = waypoint.t;
      double x = waypoint.x;

      int i0 = -1;
      int n0 = -1;

      if (spline0 != null)
      {
         i0 = spline0.getIndexFirstCoefficient();
         n0 = spline0.getNumberOfCoefficients();

         getPositionConstraintABlock(t, n0, constraintIndex, i0, A);
         b.set(constraintIndex, x);
         constraintIndex++;
      }

      if (spline1 != null)
      {
         int i1 = spline1.getIndexFirstCoefficient();
         int n1 = spline1.getNumberOfCoefficients();

         if (n1 == n0)
            CommonOps_DDRM.extract(A, constraintIndex - 1, constraintIndex, i0, i1, A, constraintIndex, i1);
         else
            getPositionConstraintABlock(t, n1, constraintIndex, i1, A);
         b.set(constraintIndex, x);
         constraintIndex++;
      }
      return constraintIndex;
   }

   /**
    * Adds an equality constraint such that the preceding and succeeding splines of the given
    * {@code waypoint} satisfy the waypoint velocity.
    */
   static int addDesiredWaypointVelocityConstraint(WaypointData waypoint, int constraintIndex, DMatrixRMaj A, DMatrixRMaj b)
   {
      Spline1DSegment spline0 = waypoint.getPreviousSpline();
      Spline1DSegment spline1 = waypoint.getNextSpline();

      double t = waypoint.t;
      double xd = waypoint.xd;

      int i0 = -1;
      int n0 = -1;

      if (spline0 != null)
      {
         i0 = spline0.getIndexFirstCoefficient();
         n0 = spline0.getNumberOfCoefficients();

         getVelocityConstraintABlock(t, n0, constraintIndex, i0, A);
         b.set(constraintIndex, xd);
         constraintIndex++;
      }

      if (spline1 != null)
      {
         int i1 = spline1.getIndexFirstCoefficient();
         int n1 = spline1.getNumberOfCoefficients();

         if (n1 == n0)
            CommonOps_DDRM.extract(A, constraintIndex - 1, constraintIndex, i0, i1, A, constraintIndex, i1);
         else
            getVelocityConstraintABlock(t, n1, constraintIndex, i1, A);
         b.set(constraintIndex, xd);
         constraintIndex++;
      }
      return constraintIndex;
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
      f.zero();

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
         Spline1DSegment spline = w0.getNextSpline();
         getMinAccelerationHBlock(w0.t, w1.t, spline.numberOfCoefficients, spline.accelerationWeight, splineOffset, splineOffset, H);
         splineOffset += spline.numberOfCoefficients;
      }
   }

   /**
    * Assembles as needed the objective function to satisfy each of the given inputs.
    */
   private void addKnotsCostFunction(DMatrixRMaj H, DMatrixRMaj f)
   {
      for (int i = 0; i < waypoints.size(); i++)
      {
         WaypointData waypoint = waypoints.get(i);

         if (waypoint.w != Double.POSITIVE_INFINITY && waypoint.w > 0.0)
            addDesiredWaypointPositionObjective(waypoint, H, f);
         if (waypoint.wd != Double.POSITIVE_INFINITY && waypoint.wd > 0.0)
            addDesiredWaypointVelocityObjective(waypoint, H, f);
         if (waypoint.wdd > 0.0)
            addDesiredWaypointAccelerationObjective(waypoint, H, f);
      }
   }

   static void addDesiredWaypointPositionObjective(WaypointData waypoint, DMatrixRMaj H, DMatrixRMaj f)
   {
      Spline1DSegment spline = waypoint.getNextSpline();

      if (spline == null)
         spline = waypoint.getPreviousSpline();

      int i = spline.getIndexFirstCoefficient();
      int n = spline.getNumberOfCoefficients();
      addPositionObjective(waypoint.t, waypoint.x, waypoint.w, n, i, i, H, f);
   }

   static void addDesiredWaypointVelocityObjective(WaypointData waypoint, DMatrixRMaj H, DMatrixRMaj f)
   {
      Spline1DSegment spline = waypoint.getNextSpline();

      if (spline == null)
         spline = waypoint.getPreviousSpline();

      int i = spline.getIndexFirstCoefficient();
      int n = spline.getNumberOfCoefficients();
      addVelocityObjective(waypoint.t, waypoint.xd, waypoint.wd, n, i, i, H, f);
   }

   static void addDesiredWaypointAccelerationObjective(WaypointData waypoint, DMatrixRMaj H, DMatrixRMaj f)
   {
      Spline1DSegment spline = waypoint.getNextSpline();

      if (spline == null)
         spline = waypoint.getPreviousSpline();

      int i = spline.getIndexFirstCoefficient();
      int n = spline.getNumberOfCoefficients();
      addAccelerationObjective(waypoint.t, 0.0, waypoint.wdd, n, i, i, H, f);
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

   public int getNumberOfWaypoints()
   {
      return waypoints.size();
   }

   public WaypointData getWaypoint(int index)
   {
      return waypoints.get(index);
   }

   public WaypointData getFirstWaypoint()
   {
      return waypoints.getFirst();
   }

   public WaypointData getLastWaypoint()
   {
      return waypoints.getLast();
   }

   public RecyclingArrayList<WaypointData> getWaypoints()
   {
      return waypoints;
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
      getPositionConstraintABlock(t, numberOfCoefficients, false, row, startColumn, A);
   }

   static void getPositionConstraintABlock(double t, int numberOfCoefficients, boolean negate, int row, int startColumn, DMatrixRMaj A)
   {
      double tpow = negate ? -1.0 : 1.0;

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
      getVelocityConstraintABlock(t, numberOfCoefficients, false, row, startColumn, A);
   }

   static void getVelocityConstraintABlock(double t, int numberOfCoefficients, boolean negate, int row, int startColumn, DMatrixRMaj A)
   {
      double tpow = negate ? -1.0 : 1.0;

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
   static void getMinAccelerationHBlock(double t0, double t1, int numberOfCoefficients, double weight, int startRow, int startColumn, DMatrixRMaj H)
   {
      if (weight == 0.0)
         return;
      if (weight < 0.0)
         throw new IllegalArgumentException("Wight cannot be negative: " + weight);

      if (numberOfCoefficients == 4)
      {
         double t0pow = t0;
         double t1pow = t1;
         double h00 = weight * 4.0 * (t1pow - t0pow);
         t0pow *= t0;
         t1pow *= t1;
         double h01 = weight * 6.0 * (t1pow - t0pow);
         t0pow *= t0;
         t1pow *= t1;
         double h11 = weight * 12.0 * (t1pow - t0pow);

         H.set(startRow + 1, startColumn + 1, h00);
         H.set(startRow + 1, startColumn + 0, h01);
         H.set(startRow + 0, startColumn + 1, h01);
         H.set(startRow + 0, startColumn + 0, h11);
      }
      else if (numberOfCoefficients == 3)
      {
         H.set(startRow, startColumn, weight * 4.0 * (t1 - t0));
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
      if (weight == 0.0)
         return;

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

      if (x != 0.0)
      {
         int f_index = startRow + numberOfCoefficients - 1;
         tpow = -weight * x;

         for (int i = 0; i < numberOfCoefficients; i++)
         {
            f.add(f_index--, 0, tpow);
            tpow *= t;
         }
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
      if (weight == 0.0)
         return;

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
            double scale = (blockSize - offsetRow) * (blockSize - offsetCol);
            H.add(startRow + offsetRow, startColumn + offsetCol, scale * tpow);
            offsetRow--;
            offsetCol++;
         }
         tpow *= t;
      }

      if (xd != 0.0)
      {
         int f_index = startRow + blockSize - 1;
         tpow = -weight * xd;

         for (int i = 0; i < blockSize; i++)
         {
            f.add(f_index--, 0, (i + 1) * tpow);
            tpow *= t;
         }
      }
   }

   /**
    * Inserts the following matrix blocks into {@code H} at [{@code startRow}, {@code startColumn}] and
    * into {@code f} at [{@code startRow}, {@code 0}]:
    * 
    * <pre>
    *                   / 36t<sup>2</sup> 12t 0 0 \
    * HBlock = weight * | 12t<sup> </sup>  4  0 0 | ( = A<sup>T</sup> W A )
    *                   |   0<sup> </sup>  0  0 0 |
    *                   \   0<sup> </sup>  0  0 0 /
    * 
    *                         / 6t \
    * fBlock = -xd * weight * | 2  | ( = -A<sup>T</sup> W xd )
    *                         | 0  |
    *                         \ 0  /
    * </pre>
    */
   static void addAccelerationObjective(double t,
                                        double xdd,
                                        double weight,
                                        int numberOfCoefficients,
                                        int startRow,
                                        int startColumn,
                                        DMatrixRMaj H,
                                        DMatrixRMaj f)
   {
      if (weight == 0.0)
         return;

      double tpow = weight;

      // We traverse the H matrix in diagonals, while skipping the last 2 rows and last 2 columns
      int blockSize = numberOfCoefficients - 2;
      int numberOfDiagonals = 2 * blockSize - 1;

      for (int diagonal = numberOfDiagonals - 1; diagonal >= 0; diagonal--)
      {
         int offsetCol = Math.max(0, diagonal - blockSize + 1);
         int offsetRow = Math.min(diagonal, blockSize - 1);
         int size = min(diagonal + 1, blockSize - offsetCol, blockSize);

         for (int i = 0; i < size; i++)
         {
            double scale = (blockSize - offsetRow) * (blockSize - offsetRow + 1) * (blockSize - offsetCol) * (blockSize - offsetCol + 1);
            H.add(startRow + offsetRow, startColumn + offsetCol, scale * tpow);
            offsetRow--;
            offsetCol++;
         }
         tpow *= t;
      }

      if (xdd != 0.0)
      {
         int f_index = startRow + blockSize - 1;
         tpow = -weight * xdd;

         for (int i = 0; i < blockSize; i++)
         {
            f.add(f_index--, 0, (i + 2) * (i + 1) * tpow);
            tpow *= t;
         }
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
