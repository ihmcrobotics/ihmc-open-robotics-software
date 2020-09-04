package us.ihmc.commonWalkingControlModules.capturePoint.lqrControl;

import com.google.common.collect.Lists;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.matrixlib.NativeCommonOps;
import us.ihmc.robotics.math.trajectories.Trajectory3D;

public class DifferentialS2Segment implements S2Segment
{
   private final double dt;
   private final DMatrixRMaj NB = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj s2Dot = new DMatrixRMaj(6, 1);
   private final DMatrixRMaj q2 = new DMatrixRMaj(6, 1);
   private final DMatrixRMaj r2 = new DMatrixRMaj(6, 1);
   private final DMatrixRMaj rs = new DMatrixRMaj(6, 1);

   private final DMatrixRMaj S1 = new DMatrixRMaj(6, 6);
   private final DMatrixRMaj referenceVRP = new DMatrixRMaj(3, 1);

   private final RecyclingArrayList<DMatrixRMaj> s2Trajectory = new RecyclingArrayList<>(() -> new DMatrixRMaj(6, 1));

   public DifferentialS2Segment(double dt)
   {
      this.dt = dt;
   }

   public void set(S1Function s1Segment,
                   Trajectory3D vrpTrajectory,
                   LQRCommonValues lqrCommonValues,
                   DMatrixRMaj s2AtEnd)
   {
      set(s1Segment,
          vrpTrajectory,
          lqrCommonValues.getQ(),
          lqrCommonValues.getR1Inverse(),
          lqrCommonValues.getNTranspose(),
          lqrCommonValues.getA(),
          lqrCommonValues.getB(),
          lqrCommonValues.getC(),
          lqrCommonValues.getD(),
          s2AtEnd);
   }

   public void set(S1Function s1Segment,
                   Trajectory3D vrpTrajectory,
                   DMatrixRMaj Q,
                   DMatrixRMaj R1Inverse,
                   DMatrixRMaj NTranspose,
                   DMatrixRMaj A,
                   DMatrixRMaj B,
                   DMatrixRMaj C,
                   DMatrixRMaj D,
                   DMatrixRMaj s2AtEnd)
   {

      s2Trajectory.clear();
      s2Trajectory.add().set(s2AtEnd);

      double duration = vrpTrajectory.getDuration();

      for (double t = dt; t <= duration; t += dt)
      {
         DMatrixRMaj previousS2 = s2Trajectory.getLast();
         DMatrixRMaj newS2 = s2Trajectory.add();

         s1Segment.compute(t, S1);

         vrpTrajectory.compute(t);
         vrpTrajectory.getPosition().get(referenceVRP);

         computeNB(B, NTranspose, S1);
         computeQ2(C, Q, referenceVRP);
         computeR2(D, Q, referenceVRP);
         computeRs(B, previousS2);

         computeS2Dot(q2, NB, R1Inverse, previousS2, A);

         CommonOps_DDRM.add(previousS2, -dt, s2Dot, newS2);
      }

      Lists.reverse(s2Trajectory);
   }

   public void compute(double timeInState, DMatrixRMaj s2ToPack)
   {
      int startIndex = getStartIndex(timeInState);
      DMatrixRMaj start = s2Trajectory.get(startIndex);
      if (startIndex == s2Trajectory.size() - 1)
      {
         s2ToPack.set(s2Trajectory.getLast());
         return;
      }
      DMatrixRMaj end = s2Trajectory.get(startIndex + 1);
      interpolate(start, end, getAlphaBetweenSegments(timeInState), s2ToPack);
   }

   private int getStartIndex(double timeInState)
   {
      return (int) Math.floor(timeInState / dt);
   }

   private double getAlphaBetweenSegments(double timeInState)
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
      CommonOps_DDRM.multTransA(B, S1, NB);
      CommonOps_DDRM.addEquals(NB, NTranspose);
   }

   private final DMatrixRMaj CTransposeQ = new DMatrixRMaj(3, 6);

   private void computeQ2(DMatrixRMaj C, DMatrixRMaj Q, DMatrixRMaj referenceVRP)
   {
      CommonOps_DDRM.multTransA(-2.0, C, Q, CTransposeQ);
      CommonOps_DDRM.mult(CTransposeQ, referenceVRP, q2);
   }

   private final DMatrixRMaj DTransposeQ = new DMatrixRMaj(3, 6);

   private void computeR2(DMatrixRMaj D, DMatrixRMaj Q, DMatrixRMaj referenceVRP)
   {
      CommonOps_DDRM.multTransA(D, Q, DTransposeQ);
      CommonOps_DDRM.mult(-2.0, DTransposeQ, referenceVRP, r2);
   }

   private void computeRs(DMatrixRMaj B, DMatrixRMaj s2)
   {
      CommonOps_DDRM.multTransA(0.5, B, s2, rs);
      CommonOps_DDRM.addEquals(rs, 0.5, r2);
   }

   private final DMatrixRMaj NBTransposeRInverse = new DMatrixRMaj(3, 3);

   private void computeS2Dot(DMatrixRMaj q2, DMatrixRMaj NB, DMatrixRMaj R1Inverse, DMatrixRMaj s2, DMatrixRMaj A)
   {
      CommonOps_DDRM.multTransA(NB, R1Inverse, NBTransposeRInverse);
      CommonOps_DDRM.mult(2.0, NBTransposeRInverse, rs, s2Dot);

      CommonOps_DDRM.addEquals(s2Dot, -1.0, q2);
      CommonOps_DDRM.multAddTransA(-1.0, A, s2, s2Dot);
   }
}
