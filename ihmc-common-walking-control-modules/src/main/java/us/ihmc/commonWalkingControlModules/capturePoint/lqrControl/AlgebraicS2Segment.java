package us.ihmc.commonWalkingControlModules.capturePoint.lqrControl;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.factory.LinearSolverFactory_DDRM;
import org.ejml.interfaces.linsol.LinearSolverDense;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.matrixlib.NativeCommonOps;
import us.ihmc.robotics.linearAlgebra.MatrixExponentialCalculator;
import us.ihmc.robotics.math.trajectories.Trajectory3D;

public class AlgebraicS2Segment implements S2Segment
{
   private final DMatrixRMaj A2 = new DMatrixRMaj(6, 6);
   private final DMatrixRMaj timeScaledA2 = new DMatrixRMaj(6, 6);
   private final DMatrixRMaj A2Exponential = new DMatrixRMaj(6, 6);
   private final DMatrixRMaj summedBetas = new DMatrixRMaj(6, 1);
   private final DMatrixRMaj coefficients = new DMatrixRMaj(3, 1);

   private final MatrixExponentialCalculator exponentialCalculator = new MatrixExponentialCalculator(6);
   private final LinearSolverDense<DMatrixRMaj> solver = LinearSolverFactory_DDRM.linear(3);

   private final DMatrixRMaj alpha = new DMatrixRMaj(6, 1);
   private final RecyclingArrayList<DMatrixRMaj> betas = new RecyclingArrayList<>(() -> new DMatrixRMaj(6, 1));

   public void set(DMatrixRMaj endValue, Trajectory3D vrpTrajectory, LQRCommonValues lqrCommonValues)
   {
      set(endValue, vrpTrajectory, lqrCommonValues.getA2(), lqrCommonValues.getA2Inverse(), lqrCommonValues.getA2InverseB2());
   }

   public void set(DMatrixRMaj endValue, Trajectory3D vrpTrajectory, DMatrixRMaj A2, DMatrixRMaj A2Inverse, DMatrixRMaj A2InverseB2)
   {
      this.A2.set(A2);

      int k = vrpTrajectory.getNumberOfCoefficients() - 1;
      betas.clear();
      for (int i = 0; i <= k; i++)
      {
         betas.add().zero();
      }

      // solve for betas
      vrpTrajectory.getCoefficients(k, coefficients);
      DMatrixRMaj betaLocal = betas.get(k);

      // betaJK = -A2inv B2 cJK
      CommonOps_DDRM.mult(-1.0, A2InverseB2, coefficients, betaLocal);

      DMatrixRMaj betaLocalPrevious = betaLocal;

      for (int i = k - 1; i >= 0; i--)
      {
         betaLocal = betas.get(i);
         vrpTrajectory.getCoefficients(i, coefficients);

         // betaJI = A2inv ((i + 1) betaJI+1 - B2 cJI)
         CommonOps_DDRM.mult(i + 1.0, A2Inverse, betaLocalPrevious, betaLocal);
         CommonOps_DDRM.multAdd(-1.0, A2InverseB2, coefficients, betaLocal);

         betaLocalPrevious = betaLocal;
      }

      double duration = Math.min(vrpTrajectory.getDuration(), 1e1);
      summedBetas.zero();
      for (int i = 0; i <= k; i++)
         CommonOps_DDRM.addEquals(summedBetas, -MathTools.pow(duration, i), betas.get(i));

      CommonOps_DDRM.scale(duration, A2, timeScaledA2);
      exponentialCalculator.compute(A2Exponential, timeScaledA2);

      CommonOps_DDRM.addEquals(summedBetas, endValue);
      solver.setA(A2Exponential);
      solver.solve(summedBetas, alpha);
   }

   public void compute(double timeInState, DMatrixRMaj s2ToPack)
   {
      CommonOps_DDRM.scale(timeInState, A2, timeScaledA2);
      exponentialCalculator.compute(A2Exponential, timeScaledA2);

      CommonOps_DDRM.mult(A2Exponential, alpha, s2ToPack);
      for (int i = 0; i < betas.size(); i++)
         CommonOps_DDRM.addEquals(s2ToPack, MathTools.pow(timeInState, i), betas.get(i));
   }

   public DMatrixRMaj getAlpha()
   {
      return alpha;
   }

   public DMatrixRMaj getBeta(int index)
   {
      return betas.get(index);
   }
}
