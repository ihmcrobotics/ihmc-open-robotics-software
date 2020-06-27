package us.ihmc.trajectoryOptimization;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.factory.LinearSolverFactory_DDRM;
import org.ejml.interfaces.linsol.LinearSolverDense;

import us.ihmc.commons.lists.RecyclingArrayList;

public class DiscreteTimeVaryingTrackingLQRSolver<E extends Enum> implements LQRSolverInterface<E>
{
   private final DiscreteOptimizationData optimalSequence;
   private final DiscreteOptimizationData desiredSequence;

   private final DiscreteSequence feedbackGainSequence;
   private final DiscreteSequence feedforwardSequence;

   private final DiscreteSequence constantsSequence;

   private final RecyclingArrayList<DMatrixRMaj> s1Sequence;
   private final RecyclingArrayList<DMatrixRMaj> s2Sequence;

   private final LinearSolverDense<DMatrixRMaj> linearSolver = LinearSolverFactory_DDRM.linear(0);

   private final DMatrixRMaj Q;
   private final DMatrixRMaj R;
   private final DMatrixRMaj Qf;


   private final DMatrixRMaj A;
   private final DMatrixRMaj B;

   private final DMatrixRMaj G;
   private final DMatrixRMaj G_inv;
   private final DMatrixRMaj H;

   private final DiscreteHybridDynamics<E> dynamics;
   private final LQTrackingCostFunction<E> costFunction;
   private final LQTrackingCostFunction<E> terminalCostFunction;

   private final DMatrixRMaj tempMatrix = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj tempMatrix2 = new DMatrixRMaj(0, 0);

   private final boolean debug;

   public DiscreteTimeVaryingTrackingLQRSolver(DiscreteHybridDynamics<E> dynamics, LQTrackingCostFunction<E> costFunction,
                                               LQTrackingCostFunction<E> terminalCostFunction)
   {
      this(dynamics, costFunction, terminalCostFunction, false);
   }

   public DiscreteTimeVaryingTrackingLQRSolver(DiscreteHybridDynamics<E> dynamics, LQTrackingCostFunction<E> costFunction,
                                               LQTrackingCostFunction<E> terminalCostFunction, boolean debug)
   {
      this.dynamics = dynamics;
      this.costFunction = costFunction;
      this.terminalCostFunction = terminalCostFunction;
      this.debug = debug;

      int stateSize = dynamics.getStateVectorSize();
      int controlSize = dynamics.getControlVectorSize();
      int constantSize = dynamics.getConstantVectorSize();

      Q = new DMatrixRMaj(stateSize, stateSize);
      Qf = new DMatrixRMaj(stateSize, stateSize);
      R = new DMatrixRMaj(controlSize, controlSize);
      G_inv = new DMatrixRMaj(controlSize, controlSize);
      G = new DMatrixRMaj(controlSize, controlSize);

      A = new DMatrixRMaj(stateSize, stateSize);
      B = new DMatrixRMaj(stateSize, controlSize);
      H = new DMatrixRMaj(stateSize, stateSize);

      optimalSequence = new DiscreteOptimizationSequence(stateSize, controlSize);
      desiredSequence = new DiscreteOptimizationSequence(stateSize, controlSize);

      feedbackGainSequence = new DiscreteSequence(controlSize, stateSize);
      feedforwardSequence = new DiscreteSequence(controlSize);

      constantsSequence = new DiscreteSequence(constantSize);

      s1Sequence = new RecyclingArrayList<>(1000, new VariableVectorBuilder(stateSize, stateSize));
      s2Sequence = new RecyclingArrayList<>(1000, new VariableVectorBuilder(1, stateSize));

      feedbackGainSequence.clear();
      feedforwardSequence.clear();

      s1Sequence.clear();
      s2Sequence.clear();
   }

   @Override
   public void setDesiredSequence(DiscreteOptimizationData desiredSequence, DiscreteSequence constantsSequence, DMatrixRMaj initialState)
   {
      this.desiredSequence.set(desiredSequence);
      this.optimalSequence.setZero(desiredSequence);

      this.s1Sequence.clear();
      this.s2Sequence.clear();

      this.constantsSequence.set(constantsSequence);

      feedbackGainSequence.setLength(desiredSequence.size());
      feedforwardSequence.setLength(desiredSequence.size());

      for (int i = 0; i < desiredSequence.size(); i++)
      {
         s1Sequence.add();
         s2Sequence.add();
      }

      optimalSequence.setState(0, initialState);
   }

   @Override
   public void solveRiccatiEquation(E dynamicState, int startIndex, int endIndex) // backwards pass
   {
      int stateSize = dynamics.getStateVectorSize();
      int controlSize = dynamics.getControlVectorSize();

      int i = endIndex;

      DMatrixRMaj currentDesiredControl = desiredSequence.getControl(i);
      DMatrixRMaj currentDesiredState = desiredSequence.getState(i);
      DMatrixRMaj currentConstants = constantsSequence.get(i);

      terminalCostFunction.getCostStateHessian(dynamicState, currentDesiredState, currentDesiredControl, currentConstants, Qf);

      DMatrixRMaj initialS1 = s1Sequence.get(i);
      DMatrixRMaj initialS2 = s2Sequence.get(i);
      initialS1.set(Qf);
      CommonOps_DDRM.multTransA(-2.0, desiredSequence.getState(i), Qf, initialS2);

      i--;

      for (; i >= startIndex; i--)
      {
         currentDesiredControl = desiredSequence.getControl(i);
         currentDesiredState = desiredSequence.getState(i);
         currentConstants = constantsSequence.get(i);
         dynamics.getDynamicsStateGradient(dynamicState, currentDesiredState, currentDesiredControl, currentConstants, A);
         dynamics.getDynamicsControlGradient(dynamicState, currentDesiredState, currentDesiredControl, currentConstants, B);
         costFunction.getCostStateHessian(dynamicState, currentDesiredState, currentDesiredControl, currentConstants, Q);
         costFunction.getCostControlHessian(dynamicState, currentDesiredState, currentDesiredControl, currentConstants, R);

         if (debug)
         {
            if (isAnyInvalid(A))
               throw new RuntimeException("The A matrix is invalid.");
            if (isAnyInvalid(B))
               throw new RuntimeException("The B matrix is invalid.");
            if (isAnyInvalid(Q))
               throw new RuntimeException("The state Hessian is invalid.");
            if (isAnyInvalid(R))
               throw new RuntimeException("The control Hessian is invalid.");
            if (isAnyInvalid(Qf))
               throw new RuntimeException("The final state Hessian is invalid.");
         }

         DMatrixRMaj currentGainMatrix = feedbackGainSequence.get(i);
         DMatrixRMaj currentFeedForwardMatrix = feedforwardSequence.get(i);

         DMatrixRMaj currentS1Matrix = s1Sequence.get(i);
         DMatrixRMaj currentS2Matrix = s2Sequence.get(i);
         DMatrixRMaj nextS1Matrix = s1Sequence.get(i + 1);
         DMatrixRMaj nextS2Matrix = s2Sequence.get(i + 1);

         dynamics.getDynamicsStateGradient(dynamicState, currentDesiredState, currentDesiredControl, currentConstants, A);
         dynamics.getDynamicsControlGradient(dynamicState, currentDesiredState, currentDesiredControl, currentConstants, B);

         // G = R + B^T S1 B
         G.set(R);
         addMultQuad(B, nextS1Matrix, B, G);

         linearSolver.setA(G);
         linearSolver.invert(G_inv);

         // K = -G^-1 B^T S1 A
         tempMatrix.reshape(controlSize, stateSize);
         CommonOps_DDRM.multTransB(G_inv, B, tempMatrix);

         tempMatrix2.reshape(stateSize, stateSize);
         CommonOps_DDRM.mult(nextS1Matrix, A, tempMatrix2);

         CommonOps_DDRM.mult(-1.0, tempMatrix, tempMatrix2, currentGainMatrix);

         // F = G^-1 (R u_d - 0.5 B^T S2^T)
         tempMatrix.reshape(controlSize, 1);
         CommonOps_DDRM.multTransAB(-0.5, B, nextS2Matrix, tempMatrix);
         CommonOps_DDRM.multAdd(R, currentDesiredControl, tempMatrix);
         CommonOps_DDRM.mult(G_inv, tempMatrix, currentFeedForwardMatrix);

         // S1_k = Q + K^T R K + (A + B K)^T S1_k+1 (A + B K)
         currentS1Matrix.set(Q);
         addMultQuad(currentGainMatrix, R, currentGainMatrix, currentS1Matrix);

         H.set(A);
         CommonOps_DDRM.multAdd(B, currentGainMatrix, H);
         addMultQuad(H, nextS1Matrix, H, currentS1Matrix);

         // S2_k = 2 (B F)^T S1_k+1 (A + B K) + S2_k+1 (A + B K) + 2 (F - u_d)^T R K - 2 x_d^T Q
         tempMatrix.reshape(stateSize, 1);

         // (S2_k+1 + 2 (B F)^T S1_k+1) (A + B K)
         tempMatrix2.set(nextS2Matrix);
         CommonOps_DDRM.mult(B, currentFeedForwardMatrix, tempMatrix);
         CommonOps_DDRM.multAddTransA(2.0, tempMatrix, nextS1Matrix, tempMatrix2);
         CommonOps_DDRM.mult(tempMatrix2, H, currentS2Matrix);

         CommonOps_DDRM.multAddTransA(-2.0, currentDesiredState, Q, currentS2Matrix);

         // 2 (F - u_d)^T R K
         tempMatrix.reshape(controlSize, stateSize);
         CommonOps_DDRM.mult(R, currentGainMatrix, tempMatrix);
         CommonOps_DDRM.multAddTransA(-2.0, currentDesiredControl, tempMatrix, currentS2Matrix);
         CommonOps_DDRM.multAddTransA(2.0, currentFeedForwardMatrix, tempMatrix, currentS2Matrix);

         if (debug && (isAnyInvalid(currentS2Matrix) || isAnyInvalid(currentS1Matrix)))
            throw new RuntimeException("The computed Riccati equation solutions are ill-conditioned.");
      }
   }

   @Override
   public void computeOptimalSequences(E dynamicState, int startIndex, int endIndex) // forward pass
   {
      // the first index is the initial state
      for (int i = startIndex; i < endIndex; i++)
      {
         DMatrixRMaj currentDesiredState = desiredSequence.getState(i);
         DMatrixRMaj currentDesiredControl = desiredSequence.getControl(i);
         DMatrixRMaj currentConstants = constantsSequence.get(i);

         dynamics.getDynamicsStateGradient(dynamicState, currentDesiredState, currentDesiredControl, currentConstants, A);
         dynamics.getDynamicsControlGradient(dynamicState, currentDesiredState, currentDesiredControl, currentConstants, B);

         DMatrixRMaj optimalControl = optimalSequence.getControl(i);

         DMatrixRMaj currentState = optimalSequence.getState(i);
         DMatrixRMaj nextState = optimalSequence.getState(i + 1);

         DMatrixRMaj feedbackGainMatrix = feedbackGainSequence.get(i);
         DMatrixRMaj feedforwardMatrix = feedforwardSequence.get(i);

         // u_k = K x_k + F
         optimalControl.set(feedforwardMatrix);
         CommonOps_DDRM.multAdd(feedbackGainMatrix, currentState, optimalControl);

         // x_k+1 = A x_k + B u_k
         CommonOps_DDRM.mult(A, currentState, nextState);
         CommonOps_DDRM.multAdd(B, optimalControl, nextState);

         if (debug)
         {
            if (isAnyInvalid(nextState))
               throw new RuntimeException("The computed optimal state is ill-conditioned.");
            if (isAnyInvalid(optimalControl))
               throw new RuntimeException("The computed optimal control is ill-conditioned.");
         }
      }
   }

   @Override
   public void getOptimalSequence(DiscreteOptimizationData optimalSequenceToPack)
   {
      optimalSequenceToPack.set(optimalSequence);
   }

   @Override
   public DiscreteOptimizationData getOptimalSequence()
   {
      return optimalSequence;
   }

   @Override
   public DiscreteData getOptimalStateSequence()
   {
      return optimalSequence.getStateSequence();
   }

   @Override
   public DiscreteData getOptimalControlSequence()
   {
      return optimalSequence.getControlSequence();
   }

   @Override
   public DiscreteSequence getOptimalFeedbackGainSequence()
   {
      return feedbackGainSequence;
   }

   @Override
   public DiscreteSequence getOptimalFeedForwardControlSequence()
   {
      return feedforwardSequence;
   }

   @Override
   public DMatrixRMaj getValueHessian()
   {
      return s1Sequence.get(0);
   }

   private final DMatrixRMaj tempMatrix3 = new DMatrixRMaj(0, 0);
   /**
    * D = D + A^T *  B * C
    */
   private void addMultQuad(DMatrixRMaj A, DMatrixRMaj B, DMatrixRMaj C, DMatrixRMaj DToPack)
   {
      tempMatrix3.reshape(A.numCols, B.numCols);
      CommonOps_DDRM.multTransA(A, B, tempMatrix3);
      CommonOps_DDRM.multAdd(tempMatrix3, C, DToPack);
   }

   private boolean isAnyInvalid(DMatrixRMaj matrix)
   {
      for (int i = 0; i < matrix.getNumElements(); i++)
      {
         if (!Double.isFinite(matrix.get(i)))
            return true;
      }
      return false;
   }
}
