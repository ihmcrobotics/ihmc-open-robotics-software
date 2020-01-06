package us.ihmc.commonWalkingControlModules.capturePoint.lqrControl;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.ops.CommonOps;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactStateProvider;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.SettableContactStateProvider;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.matrixlib.NativeCommonOps;
import us.ihmc.robotics.linearAlgebra.MatrixExponentialCalculator;
import us.ihmc.robotics.linearAlgebra.careSolvers.CARESolver;
import us.ihmc.robotics.linearAlgebra.careSolvers.DefectCorrectionCARESolver;
import us.ihmc.robotics.linearAlgebra.careSolvers.SignFunctionCARESolver;
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
public class LQRJumpMomentumController
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

   final DenseMatrix64F Ad1 = new DenseMatrix64F(6, 6);
   final DenseMatrix64F Ad2 = new DenseMatrix64F(6, 6);
   final DenseMatrix64F Ad1TransposeS1 = new DenseMatrix64F(6, 6);
   final DenseMatrix64F Ad2TransposeS1 = new DenseMatrix64F(6, 6);
   final DenseMatrix64F Bd1 = new DenseMatrix64F(6, 3);
   final DenseMatrix64F Bd2 = new DenseMatrix64F(6, 3);
   final DenseMatrix64F Bd1g = new DenseMatrix64F(6, 1);
   final DenseMatrix64F Bd2g = new DenseMatrix64F(6, 1);
   final DenseMatrix64F g = new DenseMatrix64F(3, 1);

   final DenseMatrix64F Q1 = new DenseMatrix64F(3, 3);

   final DenseMatrix64F R1 = new DenseMatrix64F(3, 3);
   final DenseMatrix64F R1Inverse = new DenseMatrix64F(3, 3);

   final DenseMatrix64F N = new DenseMatrix64F(6, 3);
   final DenseMatrix64F NTranspose = new DenseMatrix64F(3, 6);

   final DenseMatrix64F NB = new DenseMatrix64F(3, 6);

   final DenseMatrix64F P = new DenseMatrix64F(6, 6);
   private final DenseMatrix64F S1 = new DenseMatrix64F(6, 6);
   private final DenseMatrix64F s2 = new DenseMatrix64F(6, 1);
   private final DenseMatrix64F S1Hat = new DenseMatrix64F(6, 6);
   private final DenseMatrix64F S1HatInverse = new DenseMatrix64F(6, 6);
   private final DenseMatrix64F s2Hat = new DenseMatrix64F(6, 1);

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
   final DenseMatrix64F summedGammas = new DenseMatrix64F(6, 1);

   private final DenseMatrix64F finalVRPState = new DenseMatrix64F(6, 1);

   private final DenseMatrix64F relativeState = new DenseMatrix64F(6, 1);
   private final DenseMatrix64F relativeDesiredVRP = new DenseMatrix64F(3, 1);

   private final DenseMatrix64F identity = CommonOps.identity(3);

   final RecyclingArrayList<DenseMatrix64F> alphas = new RecyclingArrayList<>(() -> new DenseMatrix64F(6, 1));
   final RecyclingArrayList<RecyclingArrayList<DenseMatrix64F>> betas = new RecyclingArrayList<>(
         () -> new RecyclingArrayList<>(() -> new DenseMatrix64F(6, 1)));
   final RecyclingArrayList<RecyclingArrayList<DenseMatrix64F>> gammas = new RecyclingArrayList<>(
         () -> new RecyclingArrayList<>(() -> new DenseMatrix64F(6, 1)));

   final RecyclingArrayList<DenseMatrix64F> sigmas = new RecyclingArrayList<>(() -> new DenseMatrix64F(6, 6));
   final RecyclingArrayList<DenseMatrix64F> phis = new RecyclingArrayList<>(() -> new DenseMatrix64F(6, 6));

   final RecyclingArrayList<Trajectory3D> relativeVRPTrajectories = new RecyclingArrayList<>(() -> new Trajectory3D(4));
   final RecyclingArrayList<SettableContactStateProvider> contactStateProviders = new RecyclingArrayList<>(SettableContactStateProvider::new);

   private final LinearSolver<DenseMatrix64F> solver = LinearSolverFactory.linear(3);

   private final MatrixExponentialCalculator a2ExponentialCalculator = new MatrixExponentialCalculator(6);
   private final CARESolver careSolver = new DefectCorrectionCARESolver(new SignFunctionCARESolver());

   private boolean shouldUpdateP = true;

   public LQRJumpMomentumController(DoubleProvider omega)
   {
      this(omega, null);
   }

   public LQRJumpMomentumController(DoubleProvider omega, YoVariableRegistry parentRegistry)
   {
      computeDynamicsMatrix(omega.getValue());

      computeP();

      MatrixTools.setMatrixBlock(Ad1, 0, 3, identity, 0, 0, 3, 3, 1.0);
      MatrixTools.setMatrixBlock(Ad2, 0, 0, identity, 0, 0, 3, 3, 1.0);
      MatrixTools.setMatrixBlock(Ad2, 3, 3, identity, 0, 0, 3, 3, 1.0);

      MatrixTools.setMatrixBlock(Bd1, 0, 0, identity, 0, 0, 3, 3, 1.0);
      MatrixTools.setMatrixBlock(Bd2, 3, 0, identity, 0, 0, 3, 3, 1.0);

      g.set(2, 0, -9.81);
      CommonOps.mult(Bd1, g, Bd1g);
      CommonOps.mult(Bd2, g, Bd2g);

      if (parentRegistry != null)
         parentRegistry.addChild(registry);
   }

   public void setVRPTrackingWeight(double vrpTrackingWeight)
   {
      this.vrpTrackingWeight = vrpTrackingWeight;

      shouldUpdateP = true;
   }

   public void setMomentumRateWeight(double momentumRateWeight)
   {
      this.momentumRateWeight = momentumRateWeight;

      shouldUpdateP = true;
   }

   private void computeDynamicsMatrix(double omega)
   {
      identity.reshape(3, 3);
      CommonOps.setIdentity(identity);

      MatrixTools.setMatrixBlock(A, 0, 3, identity, 0, 0, 3, 3, 1.0);
      MatrixTools.setMatrixBlock(B, 3, 0, identity, 0, 0, 3, 3, 1.0);
      MatrixTools.setMatrixBlock(C, 0, 0, identity, 0, 0, 3, 3, 1.0);
      MatrixTools.setMatrixBlock(D, 0, 0, identity, 0, 0, 3, 3, -1.0 / MathTools.square(omega));

      CommonOps.transpose(B, BTranspose);

      shouldUpdateP = true;
   }

   public void setVRPTrajectory(List<Trajectory3D> vrpTrajectory, List<? extends ContactStateProvider> contactStateProviders)
   {
      relativeVRPTrajectories.clear();
      this.contactStateProviders.clear();

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

         this.contactStateProviders.add().set(contactStateProviders.get(i));
      }
   }

   void computeP()
   {
      MatrixTools.setDiagonal(Q, vrpTrackingWeight);
      MatrixTools.setDiagonal(R, momentumRateWeight);

      NativeCommonOps.multQuad(C, Q, Q1);
      NativeCommonOps.multQuad(D, Q, R1);
      CommonOps.addEquals(R1, R);

      NativeCommonOps.invert(R1, R1Inverse);

      CommonOps.mult(D, Q, DQ);

      CommonOps.mult(R1Inverse, DQ, R1InverseDQ);
      CommonOps.multTransB(-0.5, R1Inverse, B, R1InverseBTranspose);

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
      P.set(careSolver.computeP());



      shouldUpdateP = false;
   }

   private void resetS1Parameters()
   {
      sigmas.clear();
      phis.clear();

      for (int j = 0; j < relativeVRPTrajectories.size(); j++)
      {
         sigmas.add().zero();
         phis.add().zero();
      }
   }

   void computeS1Parameters()
   {
      resetS1Parameters();

      int numberOfSegments = relativeVRPTrajectories.size() - 1;

      sigmas.get(numberOfSegments).zero();
      CommonOps.setIdentity(phis.get(numberOfSegments));

      for (int j = numberOfSegments - 1; j >= 0; j--)
      {
         if (contactStateProviders.get(j).getContactState().isLoadBearing())
         {
            sigmas.get(j).zero();
            phis.get(j).set(phis.get(j+1));
         }
         else
         {
            // sigma_j = phi_j+1 Ad,1
            CommonOps.mult(-1.0, phis.get(j+1), Ad1, sigmas.get(j));

            // phi_j = phi_j+1 (Ad,1 (t_j+1 + t_j) + Ad,2)
            double duration = relativeVRPTrajectories.get(j).getDuration();
            tempMatrix.set(Ad2);
            CommonOps.addEquals(tempMatrix, duration, Ad1);

            CommonOps.mult(phis.get(j+1), tempMatrix, phis.get(j));
         }
      }
   }

   void computeS1AndK1(double time)
   {
      int segmentNumber = getSegmentNumber(time);
      double timeInState = computeTimeInSegment(time, segmentNumber);

      tempMatrix.set(phis.get(segmentNumber));
      CommonOps.addEquals(tempMatrix, timeInState, sigmas.get(segmentNumber));
      NativeCommonOps.multQuad(tempMatrix, P, S1);

      // Nb = N' + B' S1
      CommonOps.transpose(N, NB);
      CommonOps.multAddTransA(B, S1, NB);

      // K1 = -R1inv NB
      CommonOps.mult(-1.0, R1Inverse, NB, K1);
   }

   private void resetS2Parameters()
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
         alphas.add().zero();
         betas.add();
         gammas.add();
      }
   }

   void computeS2Parameters()
   {
      resetS2Parameters();



      int numberOfSegments = relativeVRPTrajectories.size() - 1;
      s2Hat.zero();
      for (int j = numberOfSegments; j >= 0; j--)
      {
         Trajectory3D trajectorySegment = relativeVRPTrajectories.get(j);
         int k = trajectorySegment.getNumberOfCoefficients() - 1;
         for (int i = 0; i <= k; i++)
         {
            betas.get(j).add().zero();
         }
         for (int i = 0; i < 3; i++)
         {
            gammas.get(j).add().zero();
         }

         if (contactStateProviders.get(j).getContactState().isLoadBearing())
         {
            // Nb = N' + B' S1
            CommonOps.transpose(N, NB);
            NativeCommonOps.multQuad(phis.get(j), P, S1Hat);
            CommonOps.multAddTransA(B, S1Hat, NB);

            // A2 = Nb' R1inv B' - A'
            tempMatrix.reshape(3, 6);
            MatrixTools.scaleTranspose(-1.0, A, A2);
            CommonOps.multTransB(R1Inverse, B, tempMatrix);
            CommonOps.multAddTransA(NB, tempMatrix, A2);

            // B2 = 2 (C' - Nb' R1inv D) Q
            CommonOps.multTransA(-2.0, NB, R1InverseDQ, B2);
            CommonOps.multAddTransA(2.0, C, Q, B2);

            NativeCommonOps.invert(A2, A2Inverse);
            CommonOps.mult(-1.0, A2Inverse, B2, A2InverseB2);

            // solve for betas
            trajectorySegment.getCoefficients(k, coefficients);
            DenseMatrix64F betaLocal = betas.get(j).get(k);

            // betaJK = -A2inv B2 cJK
            CommonOps.mult(A2InverseB2, coefficients, betaLocal);

            DenseMatrix64F betaLocalPrevious = betaLocal;

            for (int i = k - 1; i >= 0; i--)
            {
               betaLocal = betas.get(j).get(i);
               trajectorySegment.getCoefficients(i, coefficients);

               // betaJI = A2inv ((i + 1) betaJI+1 - B2 cJI)
               CommonOps.mult(i + 1.0, A2Inverse, betaLocalPrevious, betaLocal);
               CommonOps.multAdd(A2InverseB2, coefficients, betaLocal);

               betaLocalPrevious = betaLocal;
            }

            double duration = relativeVRPTrajectories.get(j).getDuration();
            summedBetas.set(s2Hat);
            for (int i = 0; i <= k; i++)
               CommonOps.addEquals(summedBetas, -MathTools.pow(duration, i), betas.get(j).get(i));

            CommonOps.scale(duration, A2, timeScaledA2);
            a2ExponentialCalculator.compute(A2Exponential, timeScaledA2);

            DenseMatrix64F alphaLocal = alphas.get(j);

            solver.setA(A2Exponential);
            solver.invert(S1HatInverse);
            solver.solve(summedBetas, alphaLocal);

            CommonOps.add(alphaLocal, betas.get(j).get(0), s2Hat);
         }
         else
         {
            double duration = contactStateProviders.get(j).getTimeInterval().getDuration();
            double d2 = duration * duration;
            double d3 = duration * d2;

            NativeCommonOps.multQuad(phis.get(j+1), P, S1Hat);

            CommonOps.multTransA(Ad1, S1Hat, Ad1TransposeS1);
            CommonOps.multTransA(Ad2, S1Hat, Ad2TransposeS1);

            // gamma 0 = 2 Ad2Transpose S1 Bd2 g
            CommonOps.mult(2, Ad2TransposeS1, Bd2g, gammas.get(j).get(0));

            // gamma 1 = 2 Ad1Transpose S1 Bd2 g + 2 Ad2Transpose S1 Bd1 g
            CommonOps.mult(2, Ad1TransposeS1, Bd2g, gammas.get(j).get(1));
            CommonOps.multAdd(2, Ad2TransposeS1, Bd1g, gammas.get(j).get(1));

            // gamma 3 = 2 Ad1Transpose S1 Bd1 g
            CommonOps.mult(2, Ad1TransposeS1, Bd1g, gammas.get(j).get(2));

            CommonOps.scale(duration, gammas.get(j).get(0), s2Hat);
            CommonOps.addEquals(s2Hat, d2, gammas.get(j).get(1));
            CommonOps.addEquals(s2Hat, d3, gammas.get(j).get(2));
         }
      }
   }

   void computeS2AndK2(double time)
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
      tempMatrix.reshape(3, 1);
      referenceVRPPosition.get(tempMatrix);
      CommonOps.mult(R1InverseDQ, tempMatrix, k2);
      CommonOps.multAdd(R1InverseBTranspose, s2, k2);

      yoK2.set(k2);
   }

   public void computeControlInput(DenseMatrix64F currentState, double time)
   {
      if (shouldUpdateP)
         computeP();

      computeS1Parameters();
      computeS1AndK1(time);

      computeS2Parameters();
      computeS2AndK2(time);

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
