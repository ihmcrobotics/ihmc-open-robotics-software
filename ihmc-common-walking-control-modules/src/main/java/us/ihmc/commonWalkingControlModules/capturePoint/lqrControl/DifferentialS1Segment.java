package us.ihmc.commonWalkingControlModules.capturePoint.lqrControl;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.matrixlib.NativeCommonOps;

import java.util.ArrayList;
import java.util.List;

public class DifferentialS1Segment implements S1Function
{
   private final double dt;
   private final DMatrixRMaj NB = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj S1Dot = new DMatrixRMaj(6, 6);

   private final RecyclingArrayList<DMatrixRMaj> S1ReverseTrajectory = new RecyclingArrayList<>(() -> new DMatrixRMaj(6, 6));
   final List<DMatrixRMaj> S1Trajectory = new ArrayList<>();

   public DifferentialS1Segment(double dt)
   {
      this.dt = dt;
   }

   public void set(LQRCommonValues lqrCommonValues, DMatrixRMaj S1AtEnd, double duration)
   {
      set(lqrCommonValues.getQ1(),
          lqrCommonValues.getR1Inverse(),
          lqrCommonValues.getNTranspose(),
          lqrCommonValues.getA(),
          lqrCommonValues.getB(),
          S1AtEnd,
          duration);
   }

   public void set(DMatrixRMaj Q1, DMatrixRMaj R1Inverse, DMatrixRMaj NTranspose, DMatrixRMaj A, DMatrixRMaj B, DMatrixRMaj S1AtEnd, double duration)
   {
      S1Trajectory.clear();
      S1ReverseTrajectory.clear();
      S1ReverseTrajectory.add().set(S1AtEnd);

      for (double t = dt; t <= duration + dt / 10.0; t += dt)
      {
         DMatrixRMaj previousS1 = S1ReverseTrajectory.getLast();
         DMatrixRMaj newS1 = S1ReverseTrajectory.add();

         computeNB(B, NTranspose, previousS1);
         computeS1Dot(Q1, NB, R1Inverse, previousS1, A);

         CommonOps_DDRM.add(previousS1, -dt, S1Dot, newS1);
      }

      for (int i = S1ReverseTrajectory.size() - 1; i >= 0; i--)
      {
         S1Trajectory.add(S1ReverseTrajectory.get(i));
      }
   }

   @Override
   public void compute(double timeInState, DMatrixRMaj S1ToPack)
   {
      int startIndex = getStartIndex(timeInState);
      DMatrixRMaj start = S1Trajectory.get(startIndex);
      if (startIndex == S1Trajectory.size() - 1)
      {
         S1ToPack.set(S1Trajectory.get(S1Trajectory.size() - 1));
         return;
      }
      DMatrixRMaj end = S1Trajectory.get(startIndex + 1);
      interpolate(start, end, getAlphaBetweenSegments(timeInState), S1ToPack);
   }

   int getStartIndex(double timeInState)
   {
      return (int) Math.floor(timeInState / dt + dt / 10.0);
   }

   double getAlphaBetweenSegments(double timeInState)
   {
      return (timeInState % dt) / dt;
   }

   private static void interpolate(DMatrixRMaj start, DMatrixRMaj end, double alpha, DMatrixRMaj ret)
   {
      CommonOps_DDRM.scale(1.0 - alpha, start, ret);
      CommonOps_DDRM.addEquals(ret, alpha, end);
   }

   private void computeNB(DMatrixRMaj B, DMatrixRMaj NTranspose, DMatrixRMaj S1)
   {
      computeNB(B, NTranspose, S1, NB);
   }

   static void computeNB(DMatrixRMaj B, DMatrixRMaj NTranspose, DMatrixRMaj S1, DMatrixRMaj NbToPack)
   {
      CommonOps_DDRM.multTransA(B, S1, NbToPack);
      CommonOps_DDRM.addEquals(NbToPack, NTranspose);
   }

   private void computeS1Dot(DMatrixRMaj Q1, DMatrixRMaj NB, DMatrixRMaj R1Inverse, DMatrixRMaj S1, DMatrixRMaj A)
   {
      computeS1Dot(Q1, NB, R1Inverse, S1, A, S1Dot);
   }

   static void computeS1Dot(DMatrixRMaj Q1, DMatrixRMaj NB, DMatrixRMaj R1Inverse, DMatrixRMaj S1, DMatrixRMaj A, DMatrixRMaj S1DotToPack)
   {
      NativeCommonOps.multQuad(NB, R1Inverse, S1DotToPack);
      CommonOps_DDRM.addEquals(S1DotToPack, -1.0, Q1);
      CommonOps_DDRM.multAdd(-1.0, S1, A, S1DotToPack);
      CommonOps_DDRM.multAddTransA(-1.0, A, S1, S1DotToPack);
   }

}
