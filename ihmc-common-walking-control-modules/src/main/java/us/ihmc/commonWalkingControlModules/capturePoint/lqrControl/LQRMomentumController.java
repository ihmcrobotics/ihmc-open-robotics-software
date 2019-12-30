package us.ihmc.commonWalkingControlModules.capturePoint.lqrControl;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.ops.CommonOps;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.matrixlib.NativeCommonOps;
import us.ihmc.robotics.linearAlgebra.MatrixExponentialCalculator;
import us.ihmc.robotics.linearAlgebra.careSolvers.*;
import us.ihmc.robotics.math.trajectories.Trajectory3D;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

import java.util.List;

/**
 * This LQR controller tracks the CoM dynamics of the robot, using a VRP output.
 * This is just a 3D extension of http://groups.csail.mit.edu/robotics-center/public_papers/Tedrake15.pdf
 *
 * The equations of motion are as follows:
 *
 * <p> x = [x<sub>com</sub>; xDot<sub>com</sub>]</p>
 * <p> u = [xDdot<sub>com</sub>] </p>
 * <p> y = [x<sub>vrp</sub>] </p>
 *
 * <p> A = [0 I; 0 0]</p>
 * <p> B = [0; I]</p>
 * <p> C = </p>
 */
public class LQRMomentumController
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoFrameVector3D yoK2 = new YoFrameVector3D("k2", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D feedbackForce = new YoFrameVector3D("feedbackForce", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint3D relativeCoMPosition = new YoFramePoint3D("relativeCoMPosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D relativeCoMVelocity = new YoFrameVector3D("relativeCoMVelocity", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint3D finalVRPPosition = new YoFramePoint3D("finalVRPPosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint3D referenceVRPPosition = new YoFramePoint3D("referenceVRPPosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint3D feedbackVRPPosition = new YoFramePoint3D("feedbackVRPPosition", ReferenceFrame.getWorldFrame(), registry);

   static final double defaultVrpTrackingWeight = 1e2;
   static final double defaultMomentumRateWeight = 1e-4;

   private double vrpTrackingWeight = defaultVrpTrackingWeight;
   private double momentumRateWeight = defaultMomentumRateWeight;

   final DenseMatrix64F Q = new DenseMatrix64F(3, 3);
   final DenseMatrix64F R = new DenseMatrix64F(3, 3);

   final DenseMatrix64F A = new DenseMatrix64F(6, 6);
   final DenseMatrix64F B = new DenseMatrix64F(6, 3);
   private final DenseMatrix64F BTranspose = new DenseMatrix64F(3, 6);
   final DenseMatrix64F C = new DenseMatrix64F(3, 6);
   final DenseMatrix64F D = new DenseMatrix64F(3, 3);

   final DenseMatrix64F A2 = new DenseMatrix64F(6, 6);
   final DenseMatrix64F A2Inverse = new DenseMatrix64F(6, 6);
   final DenseMatrix64F B2 = new DenseMatrix64F(6, 3);

   final DenseMatrix64F Q1 = new DenseMatrix64F(3, 3);

   final DenseMatrix64F R1 = new DenseMatrix64F(3, 3);
   final DenseMatrix64F R1Inverse = new DenseMatrix64F(3, 3);

   final DenseMatrix64F N = new DenseMatrix64F(6, 3);
   final DenseMatrix64F NTranspose = new DenseMatrix64F(3, 6);

   final DenseMatrix64F NB = new DenseMatrix64F(3, 6);

   final DenseMatrix64F S1 = new DenseMatrix64F(6, 6);
   private final DenseMatrix64F s2 = new DenseMatrix64F(6, 1);

   private final DenseMatrix64F tempMatrix = new DenseMatrix64F(3, 3);

   final DenseMatrix64F K1 = new DenseMatrix64F(3, 6);
   final DenseMatrix64F k2 = new DenseMatrix64F(3, 1);
   private final DenseMatrix64F u = new DenseMatrix64F(3, 1);

   final DenseMatrix64F QRiccati = new DenseMatrix64F(3, 3);
   final DenseMatrix64F ARiccati = new DenseMatrix64F(6, 6);

   private final DenseMatrix64F A2InverseB2 = new DenseMatrix64F(6, 3);
   private final DenseMatrix64F DQ = new DenseMatrix64F(3, 3);
   private final DenseMatrix64F R1InverseDQ = new DenseMatrix64F(3, 3);
   private final DenseMatrix64F R1InverseBTranspose = new DenseMatrix64F(3, 6);
   private final DenseMatrix64F coefficients = new DenseMatrix64F(3, 1);

   final DenseMatrix64F A2Exponential = new DenseMatrix64F(6, 6);
   final DenseMatrix64F timeScaledA2 = new DenseMatrix64F(6, 6);
   final DenseMatrix64F summedBetas = new DenseMatrix64F(6, 1);

   private final DenseMatrix64F finalVRPState = new DenseMatrix64F(6, 1);

   private final DenseMatrix64F relativeState = new DenseMatrix64F(6, 1);
   private final DenseMatrix64F relativeDesiredVRP = new DenseMatrix64F(3, 1);

   final RecyclingArrayList<DenseMatrix64F> alphas = new RecyclingArrayList<>(() -> new DenseMatrix64F(6, 1));
   final RecyclingArrayList<RecyclingArrayList<DenseMatrix64F>> betas = new RecyclingArrayList<>(
         () -> new RecyclingArrayList<>(() -> new DenseMatrix64F(6, 1)));
   final RecyclingArrayList<RecyclingArrayList<DenseMatrix64F>> gammas = new RecyclingArrayList<>(
         () -> new RecyclingArrayList<>(() -> new DenseMatrix64F(3, 1)));

   final RecyclingArrayList<Trajectory3D> relativeVRPTrajectories = new RecyclingArrayList<>(() -> new Trajectory3D(4));

   private final LinearSolver<DenseMatrix64F> solver = LinearSolverFactory.linear(3);

   private final MatrixExponentialCalculator a2ExponentialCalculator = new MatrixExponentialCalculator(6);
   private final CARESolver careSolver = new DefectCorrectionCARESolver(new SignFunctionCARESolver());

   private boolean shouldUpdateS1 = true;

   public LQRMomentumController(DoubleProvider omega)
   {
      this(omega, null);
   }

   public LQRMomentumController(DoubleProvider omega, YoVariableRegistry parentRegistry)
   {
      computeDynamicsMatrix(omega.getValue());

      computeS1();

      if (parentRegistry != null)
         parentRegistry.addChild(registry);
   }

   public void setVRPTrackingWeight(double vrpTrackingWeight)
   {
      this.vrpTrackingWeight = vrpTrackingWeight;

      shouldUpdateS1 = true;
   }

   public void setMomentumRateWeight(double momentumRateWeight)
   {
      this.momentumRateWeight = momentumRateWeight;

      shouldUpdateS1 = true;
   }

   private void computeDynamicsMatrix(double omega)
   {
      MatrixTools.setMatrixBlock(A, 0, 3, CommonOps.identity(3, 3), 0, 0, 3, 3, 1.0);
      MatrixTools.setMatrixBlock(B, 3, 0, CommonOps.identity(3, 3), 0, 0, 3, 3, 1.0);
      MatrixTools.setMatrixBlock(C, 0, 0, CommonOps.identity(3, 3), 0, 0, 3, 3, 1.0);
      MatrixTools.setMatrixBlock(D, 0, 0, CommonOps.identity(3, 3), 0, 0, 3, 3, -1.0 / MathTools.square(omega));

      CommonOps.transpose(B, BTranspose);

      shouldUpdateS1 = true;
   }

   public void setVRPTrajectory(List<Trajectory3D> vrpTrajectory)
   {
      relativeVRPTrajectories.clear();

      Trajectory3D lastTrajectory = vrpTrajectory.get(vrpTrajectory.size() - 1);
      lastTrajectory.compute(lastTrajectory.getFinalTime());
      finalVRPPosition.set(lastTrajectory.getPosition());
      finalVRPPosition.get(finalVRPState);

      for (int i = 0; i < vrpTrajectory.size(); i++)
      {
         Trajectory3D trajectory = vrpTrajectory.get(i);
         Trajectory3D relativeTrajectory = relativeVRPTrajectories.add();

         relativeTrajectory.set(trajectory);
         relativeTrajectory.offsetTrajectoryPosition(-1.0, finalVRPState);
      }
   }

   void computeS1()
   {
      MatrixTools.setDiagonal(Q, vrpTrackingWeight);
      MatrixTools.setDiagonal(R, momentumRateWeight);

      NativeCommonOps.multQuad(C, Q, Q1);
      NativeCommonOps.multQuad(D, Q, R1);
      CommonOps.addEquals(R1, R);

      NativeCommonOps.invert(R1, R1Inverse);

      CommonOps.mult(D, Q, DQ);

      tempMatrix.reshape(3, 3);
      CommonOps.mult(Q, D, tempMatrix);
      CommonOps.multTransA(C, tempMatrix, N);
      CommonOps.transpose(N, NTranspose);

      /*
        A' S1 + S1 A - Nb' R1inv Nb + Q1 = S1dot = 0
        or
        A1 S1 + S1 A - S1' B R1inv B' S1 - N R1inv N' + Q1
        If the standard CARE is formed as
        A' P + P A - P B R^-1 B' P + Q = 0
        then we can rewrite this as
                     S1 = P
         A - B R1inv N' = A
                      B = B
        Q1 - N R1inv N' = Q
      */
      QRiccati.set(Q1);
      tempMatrix.reshape(6, 6);
      NativeCommonOps.multQuad(NTranspose, R1Inverse, tempMatrix);
      CommonOps.addEquals(QRiccati, -1.0, tempMatrix);

      ARiccati.set(A);
      tempMatrix.reshape(3, 6);
      CommonOps.mult(R1Inverse, NTranspose, tempMatrix);
      CommonOps.multAdd(-1.0, B, tempMatrix, ARiccati);

      careSolver.setMatrices(ARiccati, B, null, QRiccati, R1);
      S1.set(careSolver.computeP());

      // all the below stuff is constant
      // Nb = N' + B' S1
      CommonOps.transpose(N, NB);
      CommonOps.multAddTransA(B, S1, NB);

      // K1 = -R1inv NB
      CommonOps.mult(-1.0, R1Inverse, NB, K1);

      // A2 = Nb' R1inv B' - A'
      tempMatrix.reshape(3, 6);
      MatrixTools.scaleTranspose(-1.0, A, A2);
      CommonOps.multTransB(R1Inverse, B, tempMatrix);
      CommonOps.multAddTransA(NB, tempMatrix, A2);

      // B2 = 2 (C' - Nb' R1inv D) Q
      CommonOps.mult(R1Inverse, DQ, R1InverseDQ);
      CommonOps.multTransA(-2.0, NB, R1InverseDQ, B2);
      CommonOps.multAddTransA(2.0, C, Q, B2);

      NativeCommonOps.invert(A2, A2Inverse);
      CommonOps.mult(-1.0, A2Inverse, B2, A2InverseB2);

      CommonOps.multTransB(-0.5, R1Inverse, B, R1InverseBTranspose);

      shouldUpdateS1 = false;
   }

   private void resetParameters()
   {
      for (int i = 0; i < betas.size(); i++)
         betas.get(i).clear();
      for (int i = 0; i < gammas.size(); i++)
         gammas.get(i).clear();

      betas.clear();
      gammas.clear();
      alphas.clear();

      for (int j = 0; j < relativeVRPTrajectories.size(); j++)
      {
         betas.add();
         gammas.add();
         alphas.add().zero();
      }
   }


   void computeS2Parameters()
   {
      resetParameters();

      int numberOfSegments = relativeVRPTrajectories.size() - 1;
      for (int j = numberOfSegments; j >= 0; j--)
      {
         Trajectory3D trajectorySegment = relativeVRPTrajectories.get(j);
         int k = trajectorySegment.getNumberOfCoefficients() - 1;
         for (int i = 0; i <= k; i++)
         {
            betas.get(j).add().zero();
            gammas.get(j).add().zero();
         }

         // solve for betas and gammas
         trajectorySegment.getCoefficients(k, coefficients);
         DenseMatrix64F betaLocal = betas.get(j).get(k);
         DenseMatrix64F gammaLocal = gammas.get(j).get(k);

         // betaJK = -A2inv B2 cJK
         CommonOps.mult(A2InverseB2, coefficients, betaLocal);
         // gammaJK = R1inv D Q cJK - 0.5 R1inv B' betaJK
         CommonOps.mult(R1InverseDQ, coefficients, gammaLocal);
         CommonOps.multAdd(R1InverseBTranspose, betaLocal, gammaLocal);

         DenseMatrix64F betaLocalPrevious = betaLocal;

         for (int i = k - 1; i >= 0; i--)
         {
            betaLocal = betas.get(j).get(i);
            gammaLocal = gammas.get(j).get(i);
            trajectorySegment.getCoefficients(i, coefficients);

            // betaJI = A2inv ((i + 1) betaJI+1 - B2 cJI)
            CommonOps.mult(i + 1.0, A2Inverse, betaLocalPrevious, betaLocal);
            CommonOps.multAdd(A2InverseB2, coefficients, betaLocal);

            // gammaJI = R1inv D Q cJI - 0.5 R1Inv B' betaJI
            CommonOps.mult(R1InverseDQ, coefficients, gammaLocal);
            CommonOps.multAdd(R1InverseBTranspose, betaLocal, gammaLocal);

            betaLocalPrevious = betaLocal;
         }

         double duration = relativeVRPTrajectories.get(j).getDuration();
         summedBetas.zero();
         for (int i = 0; i <= k; i++)
            CommonOps.addEquals(summedBetas, -MathTools.pow(duration, i), betas.get(j).get(i));

         CommonOps.scale(duration, A2, timeScaledA2);
         a2ExponentialCalculator.compute(A2Exponential, timeScaledA2);

         if (j != numberOfSegments)
         {
            CommonOps.addEquals(summedBetas, alphas.get(j + 1));
            CommonOps.addEquals(summedBetas, betas.get(j + 1).get(0));
         }
         solver.setA(A2Exponential);
         solver.solve(summedBetas, alphas.get(j));
      }
   }

   void computeS2(double time)
   {
      int j = getSegmentNumber(time);
      double timeInSegment = computeTimeInSegment(time, j);

      relativeVRPTrajectories.get(j).compute(timeInSegment);
      referenceVRPPosition.set(relativeVRPTrajectories.get(j).getPosition());
      referenceVRPPosition.add(finalVRPPosition);

      // s2 = exp(A2 (t - tJ) alphaJ + sum_i=0^k betaJI (t - tj)^i
      CommonOps.scale(timeInSegment, A2, timeScaledA2);
      a2ExponentialCalculator.compute(A2Exponential, timeScaledA2);

      CommonOps.mult(A2Exponential, alphas.get(j), s2);
      int k = relativeVRPTrajectories.get(j).getNumberOfCoefficients() - 1;
      for (int i = 0; i <= k; i++)
         CommonOps.addEquals(s2, MathTools.pow(timeInSegment, i), betas.get(j).get(i));

      // defined this way, because the R1Inverse BT already has a -0.5 appended.
      tempMatrix.reshape(3, 6);
      CommonOps.mult(R1InverseBTranspose, A2Exponential, tempMatrix);
      CommonOps.mult(tempMatrix, alphas.get(j), k2);
      for (int i = 0; i <= k; i++)
         CommonOps.addEquals(k2, MathTools.pow(timeInSegment, i), gammas.get(j).get(i));

      yoK2.set(k2);
   }

   public void computeControlInput(DenseMatrix64F currentState, double time)
   {
      if (shouldUpdateS1)
         computeS1();

      computeS2Parameters();
      computeS2(time);

      relativeState.set(currentState);
      for (int i = 0; i < 3; i++)
         relativeState.add(i, 0, -finalVRPState.get(i));

      relativeCoMPosition.set(relativeState);
      relativeCoMVelocity.set(3, relativeState);

      // u = K1 relativeX + k2
      CommonOps.mult(K1, relativeState, u);
      feedbackForce.set(u);

      CommonOps.addEquals(u, k2);

      CommonOps.mult(C, relativeState, relativeDesiredVRP);
      CommonOps.multAdd(D, u, relativeDesiredVRP);

      feedbackVRPPosition.set(relativeDesiredVRP);
      feedbackVRPPosition.add(finalVRPPosition);
   }

   public DenseMatrix64F getU()
   {
      return u;
   }

   public DenseMatrix64F getCostHessian()
   {
      return S1;
   }

   public DenseMatrix64F getCostJacobian()
   {
      return s2;
   }

   private int getSegmentNumber(double time)
   {
      double timeToStart = 0.0;
      for (int i = 0; i < relativeVRPTrajectories.size(); i++)
      {
         double segmentDuration = relativeVRPTrajectories.get(i).getDuration();
         if (time - timeToStart <= segmentDuration)
            return i;
         timeToStart += segmentDuration;
      }

      return -1;
   }

   private double computeTimeInSegment(double time, int segment)
   {
      double timeOffset = 0.0;
      for (int i = 0; i < segment; i++)
         timeOffset += relativeVRPTrajectories.get(i).getDuration();
      return time - timeOffset;
   }
}
