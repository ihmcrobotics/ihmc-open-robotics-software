package us.ihmc.robotics.math.trajectories.generators;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.matrixlib.NativeCommonOps;

/**
 * This solver allows to compute coefficients for concatenated 1D cubic splines such that:
 * <ul>
 * <li>the initial cubic spline starts at a given position <tt>x0</tt> and velocity <tt>xd0</tt> at
 * <tt>t = 0</tt>
 * <li>the i<sup>th</sup> and i+1<sup>th</sup> cubic splines meet at a given position <tt>xi</tt> at
 * <tt>t = ti</tt> and such that their velocity match at that position.
 * <li>the final cubic spline ends at a given position and velocity at <tt>t = 1</tt>
 * </ul>
 * <p>
 * The solver computes the optimal spline coefficients by minimizing the overall integrated
 * acceleration.
 * </p>
 * <p>
 * The number of cubic splines to solve for is dictated by the number of waypoints. There's
 * <tt>N</tt> cubic splines where <tt>N = xi.size() + 1</tt>.
 * </p>
 * <p>
 * Any of the given positions and velocities can be solved for as hard constraints or as objectives
 * allowing to relax the trajectory.
 * </p>
 * <p>
 * The trajectory duration is always unitary, i.e. <tt>t &in; [0, 1]<tt>.
 * </p>
 */
public class MultiCubicSpline1DSolver
{
   public static final int coefficients = 4;
   private static final double regularizationWeight = 1E-10;

   /** Position to start from at {@code t = 0}. */
   private double x0;
   /** Position to end to at {@code t = 1}. */
   private double x1;
   /** Velocity to start from at {@code t = 0}. */
   private double xd0;
   /** Velocity to end to at {@code t = 1}. */
   private double xd1;
   /** Waypoints to go through at <tt>t = {ti}</tt>. */
   private final TDoubleArrayList xi = new TDoubleArrayList();
   /** Times to reach each waypoint. */
   private final TDoubleArrayList ti = new TDoubleArrayList();

   /**
    * Weight associated with the position {@link #x0}. When
    * <tt>w0={@value Double#POSITIVE_INFINITY}</tt>, a hard equality constraints is setup to achieve
    * {@code x0}, otherwise it is setup as an objective.
    */
   private double w0;
   /**
    * Weight associated with the position {@link #x1}. When
    * <tt>w1={@value Double#POSITIVE_INFINITY}</tt>, a hard equality constraints is setup to achieve
    * {@code x1}, otherwise it is setup as an objective.
    */
   private double w1;
   /**
    * Weight associated with the position {@link #xd0}. When
    * <tt>wd0={@value Double#POSITIVE_INFINITY}</tt>, a hard equality constraints is setup to achieve
    * {@code xd0}, otherwise it is setup as an objective.
    */
   private double wd0;
   /**
    * Weight associated with the position {@link #xd1}. When
    * <tt>wd1={@value Double#POSITIVE_INFINITY}</tt>, a hard equality constraints is setup to achieve
    * {@code xd1}, otherwise it is setup as an objective.
    */
   private double wd1;
   /**
    * Weight associated with each waypoint {@link #xi}. When
    * <tt>wi={@value Double#POSITIVE_INFINITY}</tt>, a hard equality constraints is setup to achieve
    * {@code xi}, otherwise it is setup as an objective.
    */
   private final TDoubleArrayList wi = new TDoubleArrayList();

   /**
    * Contains the Hessian only for minimizing the integrated acceleration. Allows to compute the
    * integrated acceleration and return it in {@link #solve(DMatrixRMaj)}.
    */
   private final DMatrixRMaj H_minAccel = new DMatrixRMaj(1, 1);
   private final DMatrixRMaj H = new DMatrixRMaj(1, 1);
   private final DMatrixRMaj f = new DMatrixRMaj(1, 1);
   private final DMatrixRMaj A = new DMatrixRMaj(1, 1);
   private final DMatrixRMaj ATranspose = new DMatrixRMaj(1, 1);
   private final DMatrixRMaj b = new DMatrixRMaj(1, 1);

   private final DMatrixRMaj E = new DMatrixRMaj(1, 1);
   private final DMatrixRMaj d = new DMatrixRMaj(1, 1);

   public MultiCubicSpline1DSolver()
   {
      clearWeights();
   }

   /**
    * Resets all weight to {@link Double#POSITIVE_INFINITY} such that, unless later modified, all the
    * inputs are solved as hard constraints.
    */
   public void clearWeights()
   {
      w0 = Double.POSITIVE_INFINITY;
      w1 = Double.POSITIVE_INFINITY;
      wd0 = Double.POSITIVE_INFINITY;
      wd1 = Double.POSITIVE_INFINITY;

      wi.fill(0, xi.size(), Double.POSITIVE_INFINITY);
   }

   /**
    * Sets the conditions at <tt>t = 0</tt> and <tt>t = 1</tt> used to constrain the first and last
    * cubic splines, respectively.
    * 
    * @param startPosition  position of the trajectory at <tt>t = 0</tt>.
    * @param startVelocity  velocity of the trajectory at <tt>t = 0</tt>.
    * @param targetPosition position of the trajectory at <tt>t = 1</tt>.
    * @param targetVelocity velocity of the trajectory at <tt>t = 1</tt>.
    */
   public void setEndpoints(double startPosition, double startVelocity, double targetPosition, double targetVelocity)
   {
      x0 = startPosition;
      x1 = targetPosition;
      xd0 = startVelocity;
      xd1 = targetVelocity;
   }

   /**
    * Sets the weights for the initial and final conditions.
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
    * @throws IllegalArgumentException if any of the weights is not in the range <tt>[0, &infin;[</tt>.
    */
   public void setEndpointWeights(double startPositionWeight, double startVelocityWeight, double targetPositionWeight, double targetVelocityWeight)
   {
      checkWeightValue(startPositionWeight);
      checkWeightValue(targetPositionWeight);
      checkWeightValue(startVelocityWeight);
      checkWeightValue(targetVelocityWeight);

      w0 = startPositionWeight;
      w1 = targetPositionWeight;
      wd0 = startVelocityWeight;
      wd1 = targetVelocityWeight;
   }

   /**
    * Clears all waypoint previously added.
    */
   public void clearWaypoints()
   {
      xi.reset();
      ti.reset();
      wi.clear();
   }

   /**
    * Adds a new waypoint to go through at a given time.
    * 
    * @param position the position to go through.
    * @param time     the time at which the position should be reached.
    * @throws IllegalArgumentException if the time is not in <tt>]0, 1[</tt>.
    */
   public void addWaypoint(double position, double time)
   {
      addWaypoint(position, time, Double.POSITIVE_INFINITY);
   }

   /**
    * Adds a new waypoint to go through at a given time.
    * <p>
    * Weights should be in <tt>[0, &infin;[</tt>. A weight set to {@link Double#POSITIVE_INFINITY} will
    * set up a hard constraint while any other real value will set up an objective to constrain the
    * corresponding spline.
    * </p>
    * 
    * @param position the position to go through.
    * @param time     the time at which the position should be reached.
    * @param weight   the weight associated with the position.
    * @throws IllegalArgumentException if the time is not in <tt>]0, 1[</tt>.
    * @throws IllegalArgumentException if the weight is not in the range <tt>[0, &infin;[</tt>.
    */
   public void addWaypoint(double position, double time, double weight)
   {
      if (time <= 0.0 || time >= 1.0)
         throw new IllegalArgumentException("The time for a waypoint should be in ]0, 1[, was: " + time);
      checkWeightValue(weight);

      xi.add(position);
      ti.add(time);
      wi.add(weight);
   }

   private static void checkWeightValue(double weight)
   {
      if (weight < 0.0)
         throw new IllegalArgumentException("A weight should be in [0, +Infinity[, was: " + weight);
   }

   /**
    * Solves for the coefficients of the cubic splines and returns the acceleration integrated over the
    * time <tt>[0, 1]</tt>.
    * <p>
    * The
    * </p>
    * 
    * @param solutionToPack the matrix used to store the cubic spline coefficients as column vector.
    *                       There is 4 coefficients per spline, for each spline they are sorted from
    *                       highest to lowest order. Modified.
    * @return the acceleration integrated over the time <tt>[0, 1]</tt>
    */
   public double solve(DMatrixRMaj solutionToPack)
   {
      buildCostFunction(H_minAccel, H, f);
      buildKnotEqualityConstraints(A, b);

      int subProblemSize = coefficients * (xi.size() + 1);
      int constraints = computeNumberOfEqualityConstraints();

      // min 0.5*x'*H*x + f'*x
      // s.t. A*x == b
      int size = subProblemSize + constraints;
      E.reshape(size, size);
      d.reshape(size, 1);

      CommonOps_DDRM.fill(E, 0.0);
      CommonOps_DDRM.insert(H, E, 0, 0);
      CommonOps_DDRM.insert(A, E, subProblemSize, 0);
      ATranspose.reshape(A.getNumCols(), A.getNumRows());
      CommonOps_DDRM.transpose(A, ATranspose);
      CommonOps_DDRM.insert(ATranspose, E, 0, subProblemSize);
      CommonOps_DDRM.scale(-1.0, f);
      CommonOps_DDRM.insert(f, d, 0, 0);
      CommonOps_DDRM.insert(b, d, subProblemSize, 0);

      NativeCommonOps.solve(E, d, solutionToPack);
      solutionToPack.reshape(subProblemSize, 1);
      //      NativeCommonOps.multQuad(solutionToPack, H, b);
      NativeCommonOps.multQuad(solutionToPack, H_minAccel, b);

      return 0.5 * b.get(0, 0);
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
      int subProblemSize = coefficients * (xi.size() + 1);
      A.reshape(constraints, subProblemSize);
      b.reshape(constraints, 1);
      CommonOps_DDRM.fill(A, 0.0);

      int line = 0;

      // add initial condition
      if (w0 == Double.POSITIVE_INFINITY)
      {
         getPositionConstraintABlock(0.0, line, 0, A);
         b.set(line, x0);
         line++;
      }
      if (wd0 == Double.POSITIVE_INFINITY)
      {
         getVelocityConstraintABlock(0.0, line, 0, A);
         b.set(line, xd0);
         line++;
      }

      for (int i = 0; i < xi.size(); i++)
      {
         int colOffset = i * coefficients;
         if (wi.get(i) == Double.POSITIVE_INFINITY)
         {
            getPositionConstraintABlock(ti.get(i), line, colOffset, A);
            b.set(line, xi.get(i));
            CommonOps_DDRM.extract(A, line, line + 1, colOffset, colOffset + coefficients, A, line + 1, colOffset + coefficients);
            line++;
            b.set(line, xi.get(i));
            line++;
         }

         getVelocityConstraintABlock(ti.get(i), line, colOffset, A);
         MatrixTools.setMatrixBlock(A, line, colOffset + coefficients, A, line, colOffset, 1, coefficients, -1.0);
         b.set(line, 0.0);
         line++;
      }

      // add final condition
      if (w1 == Double.POSITIVE_INFINITY)
      {
         getPositionConstraintABlock(1.0, line, subProblemSize - coefficients, A);
         b.set(line, x1);
         line++;
      }
      if (wd1 == Double.POSITIVE_INFINITY)
      {
         getVelocityConstraintABlock(1.0, line, subProblemSize - coefficients, A);
         b.set(line, xd1);
      }
   }

   /**
    * Evaluates and returns the actual number of equality constraints by looking at the value of each
    * individual weight.
    */
   private int computeNumberOfEqualityConstraints()
   {
      int constraints = 0;

      if (w0 == Double.POSITIVE_INFINITY)
         constraints++;
      if (w1 == Double.POSITIVE_INFINITY)
         constraints++;
      if (wd0 == Double.POSITIVE_INFINITY)
         constraints++;
      if (wd1 == Double.POSITIVE_INFINITY)
         constraints++;

      for (int i = 0; i < xi.size(); i++)
      {
         if (wi.get(i) == Double.POSITIVE_INFINITY)
            constraints += 2;
         constraints++; // Velocity constraint
      }

      return constraints;
   }

   /**
    * Assembles the cost function necessary to minimize the integrated acceleration and achieve the
    * given inputs if the weights are different from {@link Double#POSITIVE_INFINITY}.
    * 
    * @param H_minAccel can be used to compute the integrated acceleration.
    */
   private void buildCostFunction(DMatrixRMaj H_minAccel, DMatrixRMaj H, DMatrixRMaj f)
   {
      int size = coefficients * (xi.size() + 1);
      H_minAccel.reshape(size, size);
      CommonOps_DDRM.fill(H_minAccel, 0.0);

      f.reshape(size, 1);
      CommonOps_DDRM.fill(f, regularizationWeight);

      getMinAccelerationCostFunction(H_minAccel);
      H.set(H_minAccel);
      addKnotsCostFunction(H, f);
   }

   private void getMinAccelerationCostFunction(DMatrixRMaj H)
   {
      double t0 = 0.0;
      double t1 = 0.0;
      for (int i = 0; i < xi.size(); i++)
      {
         t0 = t1;
         t1 = ti.get(i);
         int offset = i * coefficients;
         getMinAccelerationHBlock(t0, t1, offset, offset, H);
      }

      t0 = t1;
      t1 = 1.0;
      int offset = xi.size() * coefficients;
      getMinAccelerationHBlock(t0, t1, offset, offset, H);
   }

   /**
    * Assembles as needed the objective function to satisfy each of the given inputs.
    */
   private void addKnotsCostFunction(DMatrixRMaj H, DMatrixRMaj f)
   {
      int offset = 0;

      if (w0 != Double.POSITIVE_INFINITY)
         addPositionObjective(0.0, x0, w0, offset, offset, H, f);
      if (wd0 != Double.POSITIVE_INFINITY)
         addVelocityObjective(0.0, xd0, wd0, offset, offset, H, f);

      for (int i = 0; i < xi.size(); i++)
      {
         if (wi.get(i) != Double.POSITIVE_INFINITY)
         {
            addPositionObjective(ti.get(i), xi.get(i), wi.get(i), offset, offset, H, f);
            offset += coefficients;
            addPositionObjective(ti.get(i), xi.get(i), wi.get(i), offset, offset, H, f);
         }
         else
         {
            offset += coefficients;
         }
      }

      if (w1 != Double.POSITIVE_INFINITY)
         addPositionObjective(1.0, x1, w1, offset, offset, H, f);
      if (wd1 != Double.POSITIVE_INFINITY)
         addVelocityObjective(1.0, xd1, wd1, offset, offset, H, f);
   }

   /**
    * Inserts the following matrix block into {@code A} at [{@code row}, {@code startColumn}]:
    * 
    * <pre>
    * ABlock = [ t<sup>3</sup> t<sup>2</sup> t 1 ]
    * </pre>
    */
   static void getPositionConstraintABlock(double t, int row, int startColumn, DMatrixRMaj A)
   {
      A.set(row, startColumn + 3, 1.0);
      double tpow = t;
      A.set(row, startColumn + 2, tpow);
      tpow *= t;
      A.set(row, startColumn + 1, tpow);
      tpow *= t;
      A.set(row, startColumn, tpow);
   }

   /**
    * Inserts the following matrix block into {@code A} at [{@code row}, {@code startColumn}]:
    * 
    * <pre>
    * ABlock = [ 3t<sup>2</sup> 2t 1 0 ]
    * </pre>
    */
   static void getVelocityConstraintABlock(double t, int row, int startColumn, DMatrixRMaj A)
   {
      A.set(row, startColumn + 3, 0.0);
      A.set(row, startColumn + 2, 1.0);
      double tpow = t;
      A.set(row, startColumn + 1, 2.0 * tpow);
      tpow *= t;
      A.set(row, startColumn, 3.0 * tpow);
   }

   /**
    * Inserts the following matrix block into {@code H} at [{@code startRow}, {@code startColumn}]:
    * 
    * <pre>
    * HBlock = / 12 * (t1<sup>3</sup> - t0<sup>3</sup>)    6 * (t1<sup>2</sup> - t0<sup>2</sup>) \
    *          \  6 * (t1<sup>2</sup> - t0<sup>2</sup>)    4 * (t1<sup> </sup> - t0<sup> </sup>) /
    * </pre>
    */
   static void getMinAccelerationHBlock(double t0, double t1, int startRow, int startColumn, DMatrixRMaj H)
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

   /**
    * Inserts the following matrix blocks into {@code H} at [{@code startRow}, {@code startColumn}] and
    * into {@code f} at [{@code startRow}, {@code 0}]:
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
   static void addPositionObjective(double t, double x, double weight, int startRow, int startColumn, DMatrixRMaj H, DMatrixRMaj f)
   {
      double tpow = weight;
      H.add(startRow + 3, startColumn + 3, tpow);
      tpow *= t;
      H.add(startRow + 3, startColumn + 2, tpow);
      H.add(startRow + 2, startColumn + 3, tpow);
      tpow *= t;
      H.add(startRow + 3, startColumn + 1, tpow);
      H.add(startRow + 2, startColumn + 2, tpow);
      H.add(startRow + 1, startColumn + 3, tpow);
      tpow *= t;
      H.add(startRow + 3, startColumn + 0, tpow);
      H.add(startRow + 2, startColumn + 1, tpow);
      H.add(startRow + 1, startColumn + 2, tpow);
      H.add(startRow + 0, startColumn + 3, tpow);
      tpow *= t;
      H.add(startRow + 2, startColumn + 0, tpow);
      H.add(startRow + 1, startColumn + 1, tpow);
      H.add(startRow + 0, startColumn + 2, tpow);
      tpow *= t;
      H.add(startRow + 1, startColumn + 0, tpow);
      H.add(startRow + 0, startColumn + 1, tpow);
      tpow *= t;
      H.add(startRow + 0, startColumn + 0, tpow);

      tpow = -weight * x;
      f.add(startRow + 3, 0, tpow);
      tpow *= t;
      f.add(startRow + 2, 0, tpow);
      tpow *= t;
      f.add(startRow + 1, 0, tpow);
      tpow *= t;
      f.add(startRow, 0, tpow);
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
   static void addVelocityObjective(double t, double xd, double weight, int startRow, int startColumn, DMatrixRMaj H, DMatrixRMaj f)
   {
      double tpow = weight;
      H.add(startRow + 2, startColumn + 2, tpow);
      tpow *= t;
      H.add(startRow + 2, startColumn + 1, 2.0 * tpow);
      H.add(startRow + 1, startColumn + 2, 2.0 * tpow);
      tpow *= t;
      H.add(startRow + 2, startColumn + 0, 3.0 * tpow);
      H.add(startRow + 1, startColumn + 1, 4.0 * tpow);
      H.add(startRow + 0, startColumn + 2, 3.0 * tpow);
      tpow *= t;
      H.add(startRow + 1, startColumn + 0, 6.0 * tpow);
      H.add(startRow + 0, startColumn + 1, 6.0 * tpow);
      tpow *= t;
      H.add(startRow + 0, startColumn + 0, 9.0 * tpow);

      tpow = -weight * xd;
      f.add(startRow + 2, 0, tpow);
      tpow *= t;
      f.add(startRow + 1, 0, 2.0 * tpow);
      tpow *= t;
      f.add(startRow, 0, 3.0 * tpow);
   }
}
