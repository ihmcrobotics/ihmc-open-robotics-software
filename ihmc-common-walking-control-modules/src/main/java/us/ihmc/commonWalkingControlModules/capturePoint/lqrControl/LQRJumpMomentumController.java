package us.ihmc.commonWalkingControlModules.capturePoint.lqrControl;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.factory.LinearSolverFactory_DDRM;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.interfaces.linsol.LinearSolverDense;
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
import us.ihmc.robotics.linearAlgebra.cdreSolvers.CDRESolver;
import us.ihmc.robotics.linearAlgebra.cdreSolvers.NumericCDRESolver;
import us.ihmc.robotics.math.trajectories.Trajectory3D;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

import java.util.HashMap;
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

   final DMatrixRMaj Q = new DMatrixRMaj(3, 3);
   final DMatrixRMaj R = new DMatrixRMaj(3, 3);

   final DMatrixRMaj A = new DMatrixRMaj(6, 6);
   final DMatrixRMaj B = new DMatrixRMaj(6, 3);
   private final DMatrixRMaj BTranspose = new DMatrixRMaj(3, 6);
   final DMatrixRMaj C = new DMatrixRMaj(3, 6);
   final DMatrixRMaj D = new DMatrixRMaj(3, 3);

   final DMatrixRMaj A2 = new DMatrixRMaj(6, 6);
   final DMatrixRMaj A2Inverse = new DMatrixRMaj(6, 6);
   final DMatrixRMaj B2 = new DMatrixRMaj(6, 3);

   final DMatrixRMaj Ad1 = new DMatrixRMaj(6, 6);
   final DMatrixRMaj Ad2 = new DMatrixRMaj(6, 6);
   final DMatrixRMaj Ad1TransposeS1 = new DMatrixRMaj(6, 6);
   final DMatrixRMaj Ad2TransposeS1 = new DMatrixRMaj(6, 6);
   final DMatrixRMaj Bd1 = new DMatrixRMaj(6, 3);
   final DMatrixRMaj Bd2 = new DMatrixRMaj(6, 3);
   final DMatrixRMaj Bd1g = new DMatrixRMaj(6, 1);
   final DMatrixRMaj Bd2g = new DMatrixRMaj(6, 1);
   final DMatrixRMaj g = new DMatrixRMaj(3, 1);

   final DMatrixRMaj Q1 = new DMatrixRMaj(3, 3);

   final DMatrixRMaj R1 = new DMatrixRMaj(3, 3);
   final DMatrixRMaj R1Inverse = new DMatrixRMaj(3, 3);

   final DMatrixRMaj N = new DMatrixRMaj(6, 3);
   final DMatrixRMaj NTranspose = new DMatrixRMaj(3, 6);

   final DMatrixRMaj NB = new DMatrixRMaj(3, 6);

   final DMatrixRMaj P = new DMatrixRMaj(6, 6);
   private final DMatrixRMaj S1 = new DMatrixRMaj(6, 6);
   private final DMatrixRMaj s2 = new DMatrixRMaj(6, 1);
   private final DMatrixRMaj S1Hat = new DMatrixRMaj(6, 6);
   private final DMatrixRMaj S1HatInverse = new DMatrixRMaj(6, 6);
   private final DMatrixRMaj s2Hat = new DMatrixRMaj(6, 1);

   private final DMatrixRMaj tempMatrix = new DMatrixRMaj(3, 3);

   final DMatrixRMaj K1 = new DMatrixRMaj(3, 6);
   final DMatrixRMaj k2 = new DMatrixRMaj(3, 1);
   private final DMatrixRMaj u = new DMatrixRMaj(3, 1);

   final DMatrixRMaj QRiccati = new DMatrixRMaj(3, 3);
   final DMatrixRMaj ARiccati = new DMatrixRMaj(6, 6);

   private final DMatrixRMaj A2InverseB2 = new DMatrixRMaj(6, 3);
   private final DMatrixRMaj DQ = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj R1InverseDQ = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj R1InverseBTranspose = new DMatrixRMaj(3, 6);
   private final DMatrixRMaj coefficients = new DMatrixRMaj(3, 1);

   final DMatrixRMaj A2Exponential = new DMatrixRMaj(6, 6);
   final DMatrixRMaj timeScaledA2 = new DMatrixRMaj(6, 6);
   final DMatrixRMaj summedBetas = new DMatrixRMaj(6, 1);
   final DMatrixRMaj summedGammas = new DMatrixRMaj(6, 1);

   private final DMatrixRMaj finalVRPState = new DMatrixRMaj(6, 1);

   private final DMatrixRMaj relativeState = new DMatrixRMaj(6, 1);
   private final DMatrixRMaj relativeDesiredVRP = new DMatrixRMaj(3, 1);

   private final DMatrixRMaj identity = CommonOps_DDRM.identity(3);

   final RecyclingArrayList<DMatrixRMaj> alphas = new RecyclingArrayList<>(() -> new DMatrixRMaj(6, 1));
   final RecyclingArrayList<RecyclingArrayList<DMatrixRMaj>> betas = new RecyclingArrayList<>(
         () -> new RecyclingArrayList<>(() -> new DMatrixRMaj(6, 1)));
   final RecyclingArrayList<RecyclingArrayList<DMatrixRMaj>> gammas = new RecyclingArrayList<>(
         () -> new RecyclingArrayList<>(() -> new DMatrixRMaj(6, 1)));

   final RecyclingArrayList<Trajectory3D> relativeVRPTrajectories = new RecyclingArrayList<>(() -> new Trajectory3D(4));
   final RecyclingArrayList<SettableContactStateProvider> contactStateProviders = new RecyclingArrayList<>(SettableContactStateProvider::new);

   private final LinearSolverDense<DMatrixRMaj> solver = LinearSolverFactory_DDRM.linear(3);

   private final MatrixExponentialCalculator a2ExponentialCalculator = new MatrixExponentialCalculator(6);
   private final CARESolver careSolver = new DefectCorrectionCARESolver(new SignFunctionCARESolver());

   private boolean shouldUpdateP = true;

   private final HashMap<Trajectory3D, CDRESolver> s1Contact = new HashMap<>();
   private final HashMap<Trajectory3D, QuadraticS1Function> s1Flight = new HashMap<>();

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
      CommonOps_DDRM.mult(Bd1, g, Bd1g);
      CommonOps_DDRM.mult(Bd2, g, Bd2g);

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
      CommonOps_DDRM.setIdentity(identity);

      MatrixTools.setMatrixBlock(A, 0, 3, identity, 0, 0, 3, 3, 1.0);
      MatrixTools.setMatrixBlock(B, 3, 0, identity, 0, 0, 3, 3, 1.0);
      MatrixTools.setMatrixBlock(C, 0, 0, identity, 0, 0, 3, 3, 1.0);
      MatrixTools.setMatrixBlock(D, 0, 0, identity, 0, 0, 3, 3, -1.0 / MathTools.square(omega));

      CommonOps_DDRM.transpose(B, BTranspose);

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
         relativeTrajectory.offsetTrajectoryPosition(-finalVRPPosition.getX(), -finalVRPPosition.getY(), -finalVRPPosition.getZ());

         this.contactStateProviders.add().set(contactStateProviders.get(i));
      }
   }

   void computeP()
   {
      MatrixTools.setDiagonal(Q, vrpTrackingWeight);
      MatrixTools.setDiagonal(R, momentumRateWeight);

      NativeCommonOps.multQuad(C, Q, Q1);
      NativeCommonOps.multQuad(D, Q, R1);
      CommonOps_DDRM.addEquals(R1, R);

      NativeCommonOps.invert(R1, R1Inverse);

      CommonOps_DDRM.mult(D, Q, DQ);

      CommonOps_DDRM.mult(R1Inverse, DQ, R1InverseDQ);
      CommonOps_DDRM.multTransB(-0.5, R1Inverse, B, R1InverseBTranspose);

      tempMatrix.reshape(3, 3);
      CommonOps_DDRM.mult(Q, D, tempMatrix);
      CommonOps_DDRM.multTransA(C, tempMatrix, N);
      CommonOps_DDRM.transpose(N, NTranspose);

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
      CommonOps_DDRM.addEquals(QRiccati, -1.0, tempMatrix);

      ARiccati.set(A);
      tempMatrix.reshape(3, 6);
      CommonOps_DDRM.mult(R1Inverse, NTranspose, tempMatrix);
      CommonOps_DDRM.multAdd(-1.0, B, tempMatrix, ARiccati);

      careSolver.setMatrices(ARiccati, B, null, QRiccati, R1);
      P.set(careSolver.computeP());



      shouldUpdateP = false;
   }

   private void resetS1Parameters()
   {
      s1Contact.clear();
      s1Flight.clear();
   }

   void computeS1Parameters()
   {
      resetS1Parameters();

      int numberOfSegments = relativeVRPTrajectories.size() - 1;

      CDRESolver s1Trajectory = new NumericCDRESolver();
      Trajectory3D nextVRPTrajectory = relativeVRPTrajectories.get(numberOfSegments);
      s1Trajectory.setFinalBoundaryCondition(nextVRPTrajectory.getFinalTime(), P);
      s1Trajectory.computePFunction(nextVRPTrajectory.getInitialTime());
      s1Contact.put(nextVRPTrajectory, s1Trajectory);

      for (int j = numberOfSegments - 1; j >= 0; j--)
      {
         DMatrixRMaj nextInitialS1 = new DMatrixRMaj(6, 6);
         if (contactStateProviders.get(j + 1).getContactState().isLoadBearing())
            nextInitialS1.set(s1Contact.get(nextVRPTrajectory).getP(nextVRPTrajectory.getInitialTime()));
         else
            s1Flight.get(nextVRPTrajectory).compute(0.0, nextInitialS1);

         Trajectory3D thisVRPTrajectory = relativeVRPTrajectories.get(j);

         if (contactStateProviders.get(j).getContactState().isLoadBearing())
         {
            CDRESolver thisS1Trajectory = new NumericCDRESolver();

            thisS1Trajectory.setFinalBoundaryCondition(thisVRPTrajectory.getFinalTime(), nextInitialS1);
            thisS1Trajectory.computePFunction(thisVRPTrajectory.getInitialTime());
            s1Contact.put(thisVRPTrajectory, thisS1Trajectory);
         }
         else
         {
            // sigma_j = phi_j+1 Ad,1
            // phi_j = phi_j+1 (Ad,1 (t_j+1 + t_j) + Ad,2)
            double duration = thisVRPTrajectory.getDuration();
            tempMatrix.set(Ad2);
            CommonOps_DDRM.addEquals(tempMatrix, duration, Ad1);

            QuadraticS1Function s1Function = new QuadraticS1Function();
            s1Function.set(Ad1, tempMatrix, nextInitialS1);

            s1Flight.put(thisVRPTrajectory, s1Function);
         }
      }
   }

   void computeS1AndK1(double time)
   {
      int segmentNumber = getSegmentNumber(time);
      double timeInState = computeTimeInSegment(time, segmentNumber);
      Trajectory3D relativeVRPTrajectory = relativeVRPTrajectories.get(segmentNumber);
      if (contactStateProviders.get(segmentNumber).getContactState().isLoadBearing())
         S1.set(s1Contact.get(relativeVRPTrajectory).getP(timeInState));
      else
         s1Flight.get(relativeVRPTrajectory).compute(timeInState, S1);

      // Nb = N' + B' S1
      CommonOps_DDRM.transpose(N, NB);
      CommonOps_DDRM.multAddTransA(B, S1, NB);

      // K1 = -R1inv NB
      CommonOps_DDRM.mult(-1.0, R1Inverse, NB, K1);
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
            S1Hat.set(s1Contact.get(trajectorySegment).getP(0.0));

            // Nb = N' + B' S1
            CommonOps_DDRM.multAddTransA(B, S1Hat, NB);

            // A2 = Nb' R1inv B' - A'
            tempMatrix.reshape(3, 6);
            MatrixTools.scaleTranspose(-1.0, A, A2);
            CommonOps_DDRM.multTransB(R1Inverse, B, tempMatrix);
            CommonOps_DDRM.multAddTransA(NB, tempMatrix, A2);

            // B2 = 2 (C' - Nb' R1inv D) Q
            CommonOps_DDRM.multTransA(-2.0, NB, R1InverseDQ, B2);
            CommonOps_DDRM.multAddTransA(2.0, C, Q, B2);

            NativeCommonOps.invert(A2, A2Inverse);
            CommonOps_DDRM.mult(-1.0, A2Inverse, B2, A2InverseB2);

            // solve for betas
            trajectorySegment.getCoefficients(k, coefficients);
            DMatrixRMaj betaLocal = betas.get(j).get(k);

            // betaJK = -A2inv B2 cJK
            CommonOps_DDRM.mult(A2InverseB2, coefficients, betaLocal);

            DMatrixRMaj betaLocalPrevious = betaLocal;

            for (int i = k - 1; i >= 0; i--)
            {
               betaLocal = betas.get(j).get(i);
               trajectorySegment.getCoefficients(i, coefficients);

               // betaJI = A2inv ((i + 1) betaJI+1 - B2 cJI)
               CommonOps_DDRM.mult(i + 1.0, A2Inverse, betaLocalPrevious, betaLocal);
               CommonOps_DDRM.multAdd(A2InverseB2, coefficients, betaLocal);

               betaLocalPrevious = betaLocal;
            }

            double duration = relativeVRPTrajectories.get(j).getDuration();
            summedBetas.set(s2Hat);
            for (int i = 0; i <= k; i++)
               CommonOps_DDRM.addEquals(summedBetas, -MathTools.pow(duration, i), betas.get(j).get(i));

            CommonOps_DDRM.scale(duration, A2, timeScaledA2);
            a2ExponentialCalculator.compute(A2Exponential, timeScaledA2);

            DMatrixRMaj alphaLocal = alphas.get(j);

            solver.setA(A2Exponential);
            solver.invert(S1HatInverse);
            solver.solve(summedBetas, alphaLocal);

            CommonOps_DDRM.add(alphaLocal, betas.get(j).get(0), s2Hat);
         }
         else
         {
            double duration = contactStateProviders.get(j).getTimeInterval().getDuration();
            double d2 = duration * duration;
            double d3 = duration * d2;

            s1Flight.get(trajectorySegment).compute(0.0, S1Hat);

            CommonOps_DDRM.multTransA(Ad1, S1Hat, Ad1TransposeS1);
            CommonOps_DDRM.multTransA(Ad2, S1Hat, Ad2TransposeS1);

            // gamma 0 = 2 Ad2Transpose S1 Bd2 g
            CommonOps_DDRM.mult(2, Ad2TransposeS1, Bd2g, gammas.get(j).get(0));

            // gamma 1 = 2 Ad1Transpose S1 Bd2 g + 2 Ad2Transpose S1 Bd1 g
            CommonOps_DDRM.mult(2, Ad1TransposeS1, Bd2g, gammas.get(j).get(1));
            CommonOps_DDRM.multAdd(2, Ad2TransposeS1, Bd1g, gammas.get(j).get(1));

            // gamma 3 = 2 Ad1Transpose S1 Bd1 g
            CommonOps_DDRM.mult(2, Ad1TransposeS1, Bd1g, gammas.get(j).get(2));

            CommonOps_DDRM.scale(duration, gammas.get(j).get(0), s2Hat);
            CommonOps_DDRM.addEquals(s2Hat, d2, gammas.get(j).get(1));
            CommonOps_DDRM.addEquals(s2Hat, d3, gammas.get(j).get(2));
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
      CommonOps_DDRM.scale(timeInSegment, A2, timeScaledA2);
      a2ExponentialCalculator.compute(A2Exponential, timeScaledA2);

      CommonOps_DDRM.mult(A2Exponential, alphas.get(j), s2);
      int k = relativeVRPTrajectories.get(j).getNumberOfCoefficients() - 1;
      for (int i = 0; i <= k; i++)
         CommonOps_DDRM.addEquals(s2, MathTools.pow(timeInSegment, i), betas.get(j).get(i));

      // defined this way, because the R1Inverse BT already has a -0.5 appended.
      tempMatrix.reshape(3, 1);
      relativeVRPTrajectories.get(j).getPosition().get(tempMatrix);
      CommonOps_DDRM.mult(R1InverseDQ, tempMatrix, k2);
      CommonOps_DDRM.multAdd(R1InverseBTranspose, s2, k2);

      yoK2.set(k2);
   }

   public void computeControlInput(DMatrixRMaj currentState, double time)
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
      CommonOps_DDRM.mult(K1, relativeState, u);
      feedbackForce.set(u);

      CommonOps_DDRM.addEquals(u, k2);

      CommonOps_DDRM.mult(C, relativeState, relativeDesiredVRP);
      CommonOps_DDRM.multAdd(D, u, relativeDesiredVRP);

      feedbackVRPPosition.set(relativeDesiredVRP);
      feedbackVRPPosition.add(finalVRPPosition);
   }

   public DMatrixRMaj getU()
   {
      return u;
   }

   public DMatrixRMaj getCostHessian()
   {
      return S1;
   }

   public DMatrixRMaj getCostJacobian()
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
