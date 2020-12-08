package us.ihmc.robotics.math.trajectories.generators;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.matrixlib.NativeCommonOps;

public class MultiCubicSpline1DSolver
{
   public static final int coefficients = 4;
   private static final double regularizationWeight = 1E-10;

   private double x0, x1, xd0, xd1;
   private final TDoubleArrayList xi = new TDoubleArrayList();
   private final TDoubleArrayList ti = new TDoubleArrayList();

   private final DMatrixRMaj H = new DMatrixRMaj(1, 1);
   private final DMatrixRMaj f = new DMatrixRMaj(1, 1);
   private final DMatrixRMaj A = new DMatrixRMaj(1, 1);
   private final DMatrixRMaj ATranspose = new DMatrixRMaj(1, 1);
   private final DMatrixRMaj b = new DMatrixRMaj(1, 1);

   private final DMatrixRMaj E = new DMatrixRMaj(1, 1);
   private final DMatrixRMaj d = new DMatrixRMaj(1, 1);

   private final DMatrixRMaj hBlock = new DMatrixRMaj(1, 1);
   private final DMatrixRMaj AdLine = new DMatrixRMaj(1, 1);

   public MultiCubicSpline1DSolver()
   {
   }

   public void setEndpoints(double startPosition, double startVelocity, double targetPosition, double targetVelocity)
   {
      x0 = startPosition;
      x1 = targetPosition;
      xd0 = startVelocity;
      xd1 = targetVelocity;
   }

   public void clearIntermediatePoints()
   {
      xi.reset();
      ti.reset();
   }

   public void addIntermediatePoint(double position, double time)
   {
      xi.add(position);
      ti.add(time);
   }

   public double solve(DMatrixRMaj solutionToPack)
   {
      buildCostMatrixForDimension(H);
      buildConstraintMatrixForDimension(A, b);

      int subProblemSize = coefficients * (xi.size() + 1);
      int constraints = 4 + 3 * xi.size();

      f.reshape(subProblemSize, 1);
      CommonOps_DDRM.fill(f, regularizationWeight);

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
      NativeCommonOps.multQuad(solutionToPack, H, b);

      return 0.5 * b.get(0, 0);
   }

   /**
    * Sets up the equality constraints used for the QP:
    * <ul>
    * <li>at <tt>t=0</tt>: position and velocity are equal to {@code x0} and {@code xd0}.
    * <li>at <tt>t=1</tt>: position and velocity are equal to {@code x1} and {@code xd1}.
    * <li>at <tt>t=t<sub>i</sub></tt> with <tt>i&in;[0;nWaypoints[</tt>: position equals
    * <tt>waypoints<sub>i</sub></tt>, and velocity of the 2 cubic segments joining at the waypoint are
    * equal.
    * </ul>
    */
   private void buildConstraintMatrixForDimension(DMatrixRMaj A, DMatrixRMaj b)
   {
      int constraints = 4 + 3 * xi.size();
      int subProblemSize = coefficients * (xi.size() + 1);
      A.reshape(constraints, subProblemSize);
      b.reshape(constraints, 1);
      CommonOps_DDRM.fill(A, 0.0);

      int line = 0;

      // add initial condition
      getPositionLine(0.0, AdLine);
      CommonOps_DDRM.insert(AdLine, A, line, 0);
      b.set(line, x0);
      line++;
      getVelocityLine(0.0, AdLine);
      CommonOps_DDRM.insert(AdLine, A, line, 0);
      b.set(line, xd0);
      line++;

      for (int w = 0; w < xi.size(); w++)
      {
         int colOffset = w * coefficients;
         getPositionLine(ti.get(w), AdLine);
         CommonOps_DDRM.insert(AdLine, A, line, colOffset);
         b.set(line, xi.get(w));
         line++;
         CommonOps_DDRM.insert(AdLine, A, line, colOffset + coefficients);
         b.set(line, xi.get(w));
         line++;

         getVelocityLine(ti.get(w), AdLine);
         CommonOps_DDRM.insert(AdLine, A, line, colOffset);
         CommonOps_DDRM.scale(-1.0, AdLine);
         CommonOps_DDRM.insert(AdLine, A, line, colOffset + coefficients);
         b.set(line, 0.0);
         line++;
      }

      // add final condition
      getPositionLine(1.0, AdLine);
      CommonOps_DDRM.insert(AdLine, A, line, subProblemSize - coefficients);
      b.set(line, x1);
      line++;
      getVelocityLine(1.0, AdLine);
      CommonOps_DDRM.insert(AdLine, A, line, subProblemSize - coefficients);
      b.set(line, xd1);
   }

   private void buildCostMatrixForDimension(DMatrixRMaj H)
   {
      int size = coefficients * (xi.size() + 1);
      H.reshape(size, size);
      CommonOps_DDRM.fill(H, 0.0);

      double t0 = 0.0;
      double t1 = 0.0;
      for (int i = 0; i < xi.size(); i++)
      {
         t0 = t1;
         t1 = ti.get(i);
         getHBlock(t0, t1, hBlock);
         int offset = i * coefficients;
         CommonOps_DDRM.insert(hBlock, H, offset, offset);
      }

      t0 = t1;
      t1 = 1.0;
      getHBlock(t0, t1, hBlock);
      int offset = xi.size() * coefficients;
      CommonOps_DDRM.insert(hBlock, H, offset, offset);
   }

   /**
    * Sets up the row vector as follows:
    * 
    * <pre>
    * lineToPack = [ t<sup>3</sup> t<sup>2</sup> t 1 ]
    * </pre>
    * 
    * @param t          current time.
    * @param lineToPack modified - used to store the row vector.
    */
   static void getPositionLine(double t, DMatrixRMaj lineToPack)
   {
      lineToPack.reshape(1, coefficients);
      lineToPack.set(0, 3, 1.0);
      double tpow = t;
      lineToPack.set(0, 2, tpow);
      tpow *= t;
      lineToPack.set(0, 1, tpow);
      tpow *= t;
      lineToPack.set(0, 0, tpow);
   }

   /**
    * Sets up the row vector as follows:
    * 
    * <pre>
    * lineToPack = [ 3t<sup>2</sup> 2t 1 0 ]
    * </pre>
    * 
    * @param t          current time.
    * @param lineToPack modified - used to store the row vector.
    */
   static void getVelocityLine(double t, DMatrixRMaj lineToPack)
   {
      lineToPack.reshape(1, coefficients);
      lineToPack.set(0, 3, 0.0);
      lineToPack.set(0, 2, 1.0);
      double tpow = t;
      lineToPack.set(0, 1, 2.0 * tpow);
      tpow *= t;
      lineToPack.set(0, 0, 3.0 * tpow);
   }

   /**
    * Sets up the 2-by-2 matrix as follows:
    * 
    * <pre>
    * hBlockToPak = / 12 * (t1<sup>3</sup> - t0<sup>3</sup>)    6 * (t1<sup>2</sup> - t0<sup>2</sup>) \
    *               \  6 * (t1<sup>2</sup> - t0<sup>2</sup>)    4 * (t1<sup> </sup> - t0<sup> </sup>) /
    * </pre>
    * 
    * @param t0           start time of the segment.
    * @param t1           end time of the segment.
    * @param hBlockToPack modified - used to store the 2-by-2 matrix.
    */
   static void getHBlock(double t0, double t1, DMatrixRMaj hBlockToPack)
   {
      hBlockToPack.reshape(2, 2);
      double t0pow = t0;
      double t1pow = t1;
      hBlockToPack.set(1, 1, 4.0 * (t1pow - t0pow));
      t0pow *= t0;
      t1pow *= t1;
      hBlockToPack.set(1, 0, 6.0 * (t1pow - t0pow));
      hBlockToPack.set(0, 1, 6.0 * (t1pow - t0pow));
      t0pow *= t0;
      t1pow *= t1;
      hBlockToPack.set(0, 0, 12.0 * (t1pow - t0pow));
   }
}
