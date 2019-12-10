package us.ihmc.commonWalkingControlModules.capturePoint.lqrControl;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.ops.CommonOps;
import us.ihmc.commonWalkingControlModules.messageHandlers.EuclideanTrajectoryHandler;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.matrixlib.NativeCommonOps;
import us.ihmc.robotics.linearAlgebra.MatrixExponentialCalculator;
import us.ihmc.robotics.math.trajectories.Trajectory;
import us.ihmc.robotics.math.trajectories.Trajectory3D;
import us.ihmc.robotics.math.trajectories.YoPolynomial3D;

import java.util.List;

public class LQRMomentumController
{
   private static final double omega = 3.0;
   private static final double defaultVrpTrackingWeight = 10.0;
   private static final double defaultMomentumRateWeight = 1.0;

   private final DenseMatrix64F Q = new DenseMatrix64F(3, 3);
   private final DenseMatrix64F R = new DenseMatrix64F(3, 3);

   private final DenseMatrix64F A = new DenseMatrix64F(6, 6);
   private final DenseMatrix64F AInverse = new DenseMatrix64F(6, 6);
   private final DenseMatrix64F B = new DenseMatrix64F(6, 3);
   private final DenseMatrix64F C = new DenseMatrix64F(3, 6);
   private final DenseMatrix64F D = new DenseMatrix64F(3, 3);

   private final DenseMatrix64F A2 = new DenseMatrix64F(3, 3);
   private final DenseMatrix64F A2Inverse = new DenseMatrix64F(3, 3);
   private final DenseMatrix64F B2 = new DenseMatrix64F(3, 3);

   private final DenseMatrix64F Q1 = new DenseMatrix64F(3, 3);
   private final DenseMatrix64F q2 = new DenseMatrix64F(3, 1);
   private double q3;

   private final DenseMatrix64F R1 = new DenseMatrix64F(3, 3);
   private final DenseMatrix64F R1Inverse = new DenseMatrix64F(3, 3);
   private final DenseMatrix64F r2 = new DenseMatrix64F(3, 1);

   private final DenseMatrix64F N = new DenseMatrix64F(3, 3);

   private final DenseMatrix64F NB = new DenseMatrix64F(3, 3);
   private final DenseMatrix64F rs = new DenseMatrix64F(3, 1);

   private final DenseMatrix64F S1 = new DenseMatrix64F(3, 3);
   private final DenseMatrix64F s2 = new DenseMatrix64F(3, 1);

   private final DenseMatrix64F tempMatrix = new DenseMatrix64F(3, 3);

   private final DenseMatrix64F K1 = new DenseMatrix64F(3, 3);
   private final DenseMatrix64F k2 = new DenseMatrix64F(3, 1);
   private final DenseMatrix64F u = new DenseMatrix64F(3, 1);

   private final RecyclingArrayList<DenseMatrix64F> alpha = new RecyclingArrayList<>(() -> new DenseMatrix64F(3, 1));
   private final RecyclingArrayList<RecyclingArrayList<DenseMatrix64F>> beta = new RecyclingArrayList<>(
         () -> new RecyclingArrayList<>(() -> new DenseMatrix64F(3, 1)));
   private final RecyclingArrayList<RecyclingArrayList<DenseMatrix64F>> gamma = new RecyclingArrayList<>(
         () -> new RecyclingArrayList<>(() -> new DenseMatrix64F(3, 1)));

   private List<Trajectory3D> vrpTrajectory;

   private final LinearSolver<DenseMatrix64F> solver = LinearSolverFactory.linear(3);

   private final MatrixExponentialCalculator matrixExponentialCalculator = new MatrixExponentialCalculator(6);

   public LQRMomentumController()
   {
      CommonOps.setIdentity(Q);
      CommonOps.setIdentity(R);
      CommonOps.scale(defaultVrpTrackingWeight, Q);
      CommonOps.scale(defaultMomentumRateWeight, R);

      MatrixTools.setMatrixBlock(A, 0, 3, CommonOps.identity(3, 3), 0, 0, 3, 3, 1.0);
      MatrixTools.setMatrixBlock(B, 3, 0, CommonOps.identity(3, 3), 0, 0, 3, 3, 1.0);
      MatrixTools.setMatrixBlock(C, 0, 0, CommonOps.identity(3, 3), 0, 0, 3, 3, 1.0);
      MatrixTools.setMatrixBlock(D, 0, 0, CommonOps.identity(3, 3), 0, 0, 3, 3, -MathTools.square(omega));

      CommonOps.invert(A, AInverse);
      NativeCommonOps.multQuad(C, Q, Q1);
      NativeCommonOps.multQuad(D, Q, R1);
      CommonOps.addEquals(R1, R);

      CommonOps.invert(R1, R1Inverse);

      tempMatrix.reshape(3, 3);
      CommonOps.mult(Q, D, tempMatrix);
      CommonOps.multTransA(C, tempMatrix, N);
   }

   public void setVrpTrajectory(List<Trajectory3D> vrpTrajectory)
   {
      this.vrpTrajectory = vrpTrajectory;
   }

   public void reset()
   {
      alpha.clear();
      beta.clear();
      gamma.clear();
   }

   private void computeS1()
   {
      // TODO
   }

   private final DenseMatrix64F A2InverseB2 = new DenseMatrix64F(3, 3);
   private final DenseMatrix64F DQ = new DenseMatrix64F(3, 3);
   private final DenseMatrix64F R1InverseDQ = new DenseMatrix64F(3, 3);
   private final DenseMatrix64F R1InverseBTranspose = new DenseMatrix64F(3, 3);
   private final DenseMatrix64F c = new DenseMatrix64F(3, 1);

   private void computeS2Terms()
   {
      CommonOps.transpose(N, NB);
      CommonOps.multAddTransA(B, S1, NB);

      tempMatrix.reshape(3, 6);
      CommonOps.transpose(A, A2);
      CommonOps.scale(-1.0, A2);
      CommonOps.multTransB(R1Inverse, B, tempMatrix);
      CommonOps.multAddTransA(NB, tempMatrix, A2);

      CommonOps.invert(A2, A2Inverse);
      CommonOps.mult(-1.0, A2Inverse, B2, A2InverseB2);

      CommonOps.mult(D, Q, DQ);
      CommonOps.mult(R1Inverse, DQ, R1InverseDQ);
      CommonOps.multTransB(-0.5, R1Inverse, B, R1InverseBTranspose);

      for (int j = 0; j < vrpTrajectory.size(); j++)
      {
         beta.add();
         gamma.add();
         alpha.add().zero();
      }

      int numberOfSegments = vrpTrajectory.size() - 1;
      for (int j = numberOfSegments; j >= 0; j--)
      {
         Trajectory3D trajectorySegment = vrpTrajectory.get(j);
         int order = trajectorySegment.getNumberOfCoefficients();
         for (int k = 0; k < order; k++)
         {
            beta.get(j).add().zero();
            gamma.get(j).add().zero();
            for (int ordinal = 0; ordinal < 3; ordinal++)
            {
               c.set(ordinal, 0, trajectorySegment.getTrajectory(ordinal).getCoefficient(k));
            }
         }

         // solve for betas and gammas
         for (int ordinal = 0; ordinal < 3; ordinal++)
            c.set(ordinal, 0, trajectorySegment.getTrajectory(ordinal).getCoefficient(order - 1));
         DenseMatrix64F betaLocal = beta.get(j).get(order - 1);
         DenseMatrix64F gammaLocal = gamma.get(j).get(order - 1);

         CommonOps.mult(A2InverseB2, c, betaLocal);
         CommonOps.mult(R1InverseDQ, c, gammaLocal);
         CommonOps.multAdd(R1InverseBTranspose, betaLocal, gammaLocal);

         DenseMatrix64F betaLocalPrevious = betaLocal;

         for (int i = order - 2; i >= 0; i++)
         {
            betaLocal = beta.get(j).get(i);
            gammaLocal = gamma.get(j).get(i);
            for (int ordinal = 0; ordinal < 3; ordinal++)
               c.set(ordinal, 0, trajectorySegment.getTrajectory(ordinal).getCoefficient(i));

            CommonOps.mult(i + 1.0, A2Inverse, betaLocalPrevious, betaLocal);
            CommonOps.multAdd(-1.0, B2, c, betaLocal);

            CommonOps.mult(R1InverseDQ, c, gammaLocal);
            CommonOps.multAdd(R1InverseBTranspose, betaLocal, gammaLocal);

            betaLocalPrevious = betaLocal;
         }

         // TODO this should be checked because it kind of seems wrong.
         double duration = vrpTrajectory.get(j).getDuration();
         if (j == numberOfSegments)
         {
            DenseMatrix64F exponential = new DenseMatrix64F(A2.numRows, A2.numCols);
            DenseMatrix64F timeScaledDynamics = new DenseMatrix64F(A2);
            CommonOps.scale(duration, timeScaledDynamics);
            LQRTools.matrixExponential(timeScaledDynamics, exponential, 10);
            matrixExponentialCalculator.compute(exponential, A);
            DenseMatrix64F summedBetas = new DenseMatrix64F(3, 1);
            for (int i = 0; i < order; i++)
            {
               CommonOps.addEquals(summedBetas, MathTools.pow(duration, i), beta.get(j).get(i));
            }
            CommonOps.scale(-1.0, summedBetas);
            solver.setA(exponential);
            solver.solve(summedBetas, alpha.get(j));
         }
         else
         {
            DenseMatrix64F exponential = new DenseMatrix64F(A2.numRows, A2.numCols);
            DenseMatrix64F timeScaledDynamics = new DenseMatrix64F(A2);
            CommonOps.scale(duration, timeScaledDynamics);
            LQRTools.matrixExponential(timeScaledDynamics, exponential, 10);
            DenseMatrix64F summedBetas = new DenseMatrix64F(3, 1);
            for (int i = 0; i < order - 1; i++)
            {
               CommonOps.addEquals(summedBetas, MathTools.pow(duration, i), beta.get(j).get(i));
            }
            CommonOps.scale(-1.0, summedBetas);
            CommonOps.addEquals(alpha.get(j + 1), summedBetas);
            CommonOps.addEquals(beta.get(j+1).get(1), summedBetas);
            solver.setA(exponential);
            solver.solve(summedBetas, alpha.get(j));
         }
      }
   }

   private void computeS2(double time)
   {
      computeS2Terms();

      int activeSegment = getSegmentNumber(time);
      double timeInSegment = computeTimeInSegment(time, activeSegment);

      DenseMatrix64F exponential = new DenseMatrix64F(A2.numRows, A2.numCols);
      DenseMatrix64F timeScaledDynamics = new DenseMatrix64F(A2);
      CommonOps.scale(timeInSegment, timeScaledDynamics);
      LQRTools.matrixExponential(timeScaledDynamics, exponential, 10);

      CommonOps.mult(exponential, alpha.get(activeSegment), s2);
      for (int i = 0; i < vrpTrajectory.get(activeSegment).getNumberOfCoefficients(); i++)
      {
         CommonOps.addEquals(s2, MathTools.pow(timeInSegment, i), beta.get(activeSegment).get(i));
      }

      tempMatrix.reshape(3, 3);
      CommonOps.mult(-0.5, R1InverseBTranspose, exponential, tempMatrix);
      CommonOps.mult(tempMatrix, alpha.get(activeSegment), k2);
      for (int i = 0; i < vrpTrajectory.get(activeSegment).getNumberOfCoefficients(); i++)
      {
         CommonOps.addEquals(k2, MathTools.pow(timeInSegment, i), gamma.get(activeSegment).get(i));
      }
   }

   private final DenseMatrix64F relativeState = new DenseMatrix64F(6, 1);
   private final DenseMatrix64F relativeVRPPosition = new DenseMatrix64F(3, 1);

   public void computeControlInput(DenseMatrix64F currentState, double time)
   {
      computeS1();
      computeS2(time);

      relativeState.set(currentState);
      Trajectory3D lastTrajectory = vrpTrajectory.get(vrpTrajectory.size() - 1);
      lastTrajectory.compute(lastTrajectory.getFinalTime());
      Point3DReadOnly finalPosition = lastTrajectory.getPosition();
      int activeSegment = getSegmentNumber(time);
      Trajectory3D currentTrajectory = vrpTrajectory.get(activeSegment);
      currentTrajectory.compute(time);
      Point3DReadOnly currentVRPPosition = currentTrajectory.getPosition();

      for (int i = 0; i < 3; i++)
      {
         relativeState.add(i, 0, -finalPosition.getElement(i));
         relativeVRPPosition.set(i, 0, currentVRPPosition.getElement(i) - finalPosition.getElement(i));
      }

      tempMatrix.reshape(3, 1);
      CommonOps.mult(Q, relativeVRPPosition, tempMatrix);
      CommonOps.mult(-2.0, D, tempMatrix, r2);

      CommonOps.scale(0.5, r2, rs);
      CommonOps.multAddTransA(B, s2, rs);

      CommonOps.mult(-1.0, R1Inverse, NB, K1);
      CommonOps.mult(K1, relativeState, u);
      CommonOps.addEquals(u, k2);
      CommonOps.multAdd(-1.0, R1Inverse, rs, u);
   }

   private int getSegmentNumber(double time)
   {
      for (int i = 0; i < vrpTrajectory.size(); i++)
      {
         if (time < vrpTrajectory.get(i).getFinalTime())
            return i;
      }

      return -1;
   }

   private double computeTimeInSegment(double time, int segment)
   {
      return time - vrpTrajectory.get(segment).getInitialTime();
   }

}
