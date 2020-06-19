package us.ihmc.trajectoryOptimization;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.matrixlib.DiagonalMatrixTools;

public class ContinuousTrackingLQRSolver<E extends Enum> implements LQRSolverInterface<E>
{
   private final DiscreteOptimizationData optimalSequence;
   private final DiscreteOptimizationData desiredSequence;

   private final DiscreteSequence feedbackGainSequence;
   private final DiscreteSequence constantsSequence;

   private final RecyclingArrayList<DMatrixRMaj> S2Sequence;
   private final RecyclingArrayList<DMatrixRMaj> S1Sequence;

   private final DMatrixRMaj Q;
   private final DMatrixRMaj R;
   private final DMatrixRMaj R_inv;
   private final DMatrixRMaj Qf;
   private final DMatrixRMaj S2Dot;
   private final DMatrixRMaj S1Dot;

   private final DMatrixRMaj A;
   private final DMatrixRMaj B;
   private final DMatrixRMaj XDot;

   private final DMatrixRMaj S2BR_invBT;
   private final DMatrixRMaj R_invBT;

   private final DiscreteHybridDynamics<E> dynamics;
   private final LQTrackingCostFunction<E> costFunction;
   private final LQTrackingCostFunction<E> terminalCostFunction;

   private final double deltaT;

   private final boolean debug;

   public ContinuousTrackingLQRSolver(DiscreteHybridDynamics<E> dynamics, LQTrackingCostFunction costFunction, LQTrackingCostFunction terminalCostFunction,
                                      double deltaT)
   {
      this(dynamics, costFunction, terminalCostFunction, deltaT, false);
   }
   public ContinuousTrackingLQRSolver(DiscreteHybridDynamics<E> dynamics, LQTrackingCostFunction<E> costFunction, LQTrackingCostFunction<E> terminalCostFunction,
                                      double deltaT, boolean debug)
   {
      this.dynamics = dynamics;
      this.costFunction = costFunction;
      this.terminalCostFunction = terminalCostFunction;
      this.deltaT = deltaT;
      this.debug = debug;

      int stateSize = dynamics.getStateVectorSize();
      int controlSize = dynamics.getControlVectorSize();
      int constantSize = dynamics.getConstantVectorSize();

      Q = new DMatrixRMaj(stateSize, stateSize);
      Qf = new DMatrixRMaj(stateSize, stateSize);
      R = new DMatrixRMaj(controlSize, controlSize);
      R_inv = new DMatrixRMaj(controlSize, controlSize);
      S2Dot = new DMatrixRMaj(stateSize, stateSize);
      S1Dot = new DMatrixRMaj(stateSize, 1);

      A = new DMatrixRMaj(stateSize, stateSize);
      B = new DMatrixRMaj(stateSize, controlSize);
      XDot = new DMatrixRMaj(stateSize, 1);

      S2BR_invBT = new DMatrixRMaj(stateSize, stateSize);
      R_invBT = new DMatrixRMaj(controlSize, stateSize);

      VariableVectorBuilder stateBuilder = new VariableVectorBuilder(stateSize, 1);
      VariableVectorBuilder hessianBuilder = new VariableVectorBuilder(stateSize, stateSize);

      optimalSequence = new DiscreteOptimizationSequence(stateSize, controlSize);
      desiredSequence = new DiscreteOptimizationSequence(stateSize, controlSize);

      constantsSequence = new DiscreteSequence(constantSize, 1);
      feedbackGainSequence = new DiscreteSequence(controlSize, stateSize);

      S2Sequence = new RecyclingArrayList<>(1000, hessianBuilder);
      S1Sequence = new RecyclingArrayList<>(1000, stateBuilder);

      feedbackGainSequence.clear();

      S2Sequence.clear();
      S1Sequence.clear();
   }

   @Override
   public void setDesiredSequence(DiscreteOptimizationData desiredSequence, DiscreteSequence constantsSequence, DMatrixRMaj initialState)
   {
      this.desiredSequence.set(desiredSequence);
      this.optimalSequence.setZero(desiredSequence);

      this.S2Sequence.clear();
      this.S1Sequence.clear();

      this.feedbackGainSequence.setLength(desiredSequence.size());
      this.constantsSequence.set(constantsSequence);

      for (int i = 0; i < desiredSequence.size(); i++)
      {
         S2Sequence.add();
         S1Sequence.add();
      }

      optimalSequence.setState(0, initialState);
   }

   private final DMatrixRMaj tempMatrix = new DMatrixRMaj(0, 0);
   public void solveRiccatiEquation(E dynamicState, int startIndex, int endIndex) // backwards pass
   {
      int stateSize = dynamics.getStateVectorSize();
      int controlSize = dynamics.getControlVectorSize();

      int i = endIndex;
      DMatrixRMaj state = optimalSequence.getState(i);
      DMatrixRMaj control = optimalSequence.getControl(i);
      DMatrixRMaj desiredState = desiredSequence.getState(i);
      DMatrixRMaj constants = constantsSequence.get(i);
      DMatrixRMaj desiredControl;

      costFunction.getCostStateHessian(dynamicState, state, control, constants, Q);
      costFunction.getCostControlHessian(dynamicState, state, control, constants, R);
      terminalCostFunction.getCostStateHessian(dynamicState, state, control, constants, Qf);

      dynamics.getContinuousAMatrix(A);
      dynamics.getContinuousBMatrix(B);

      DiagonalMatrixTools.invertDiagonalMatrix(R, R_inv);
      CommonOps_DDRM.multTransB(R_inv, B, R_invBT);


      DMatrixRMaj initialS2 = this.S2Sequence.get(i);
      DMatrixRMaj initialS1 = this.S1Sequence.get(i);
      initialS2.set(Qf);
      CommonOps_DDRM.mult(-2.0, Qf, desiredState, initialS1);

      // backward pass
      for (; i > startIndex; i--)
      {
         desiredState = desiredSequence.getState(i);
         desiredControl = desiredSequence.getControl(i);

         DMatrixRMaj currentS2 = S2Sequence.get(i);
         DMatrixRMaj currentS1 = S1Sequence.get(i);

         // compute currentS2 dot
         S2Dot.set(Q);
         CommonOps_DDRM.multAdd(currentS2, A, S2Dot);
         CommonOps_DDRM.multAddTransA(A, currentS2, S2Dot);

         tempMatrix.reshape(stateSize, controlSize);
         CommonOps_DDRM.multTransA(currentS2, B, tempMatrix);
         CommonOps_DDRM.mult(tempMatrix, R_invBT, S2BR_invBT);

         CommonOps_DDRM.multAdd(-1.0, S2BR_invBT, currentS2, S2Dot);

         // compute currentS1 dot
         CommonOps_DDRM.mult(-2.0, Q, desiredState, S1Dot);
         CommonOps_DDRM.multAddTransA(A, currentS1, S1Dot);
         CommonOps_DDRM.multAdd(-1.0, S2BR_invBT, currentS1, S1Dot);

         tempMatrix.reshape(stateSize, 1);
         CommonOps_DDRM.mult(B, desiredControl, tempMatrix);
         CommonOps_DDRM.multAdd(2.0, currentS2, tempMatrix, S1Dot);

         // compute previous currentS2 and currentS1
         DMatrixRMaj previousS2 = S2Sequence.get(i - 1);
         DMatrixRMaj previousS1 = S1Sequence.get(i - 1);

         previousS2.set(currentS2);
         previousS1.set(currentS1);

         CommonOps_DDRM.addEquals(previousS2, deltaT, S2Dot);
         CommonOps_DDRM.addEquals(previousS1, deltaT, S1Dot);

         if (debug && (isAnyInvalid(currentS2) || isAnyInvalid(currentS1)))
            throw new RuntimeException("The computed Riccati equation solutions are ill-conditioned.");
      }

   }

   // forward pass
   public void computeOptimalSequences(E dynamicsState, int startIndex, int endIndex) // forward pass
   {
      int stateSize = dynamics.getStateVectorSize();
      for (int i = startIndex; i < endIndex; i++)
      {
         DMatrixRMaj desiredControl = desiredSequence.getControl(i);
         DMatrixRMaj state = optimalSequence.getState(i);
         DMatrixRMaj control = optimalSequence.getControl(i);
         DMatrixRMaj gain = feedbackGainSequence.get(i);

         DMatrixRMaj nextState = optimalSequence.getState(i + 1);
         DMatrixRMaj S2 = S2Sequence.get(i);
         DMatrixRMaj S1 = S1Sequence.get(i);

         control.set(desiredControl);

         tempMatrix.reshape(stateSize, 1);
         CommonOps_DDRM.mult(S2, state, tempMatrix);
         CommonOps_DDRM.addEquals(tempMatrix, 0.5, S1);
         CommonOps_DDRM.mult(-1.0, R_invBT, tempMatrix, gain);
         CommonOps_DDRM.multAdd(-1.0, R_invBT, tempMatrix, control);

         CommonOps_DDRM.mult(A, state, XDot);
         CommonOps_DDRM.multAdd(B, control, XDot);

         nextState.set(state);
         CommonOps_DDRM.addEquals(nextState, deltaT, XDot);

         if (debug)
         {
            if (isAnyInvalid(state))
               throw new RuntimeException("The computed optimal state is ill-conditioned.");
            if (isAnyInvalid(control))
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
      throw new RuntimeException("this isn't implemented correctly.");
   }

   @Override
   public DMatrixRMaj getValueHessian()
   {
      return S1Sequence.get(0);
   }

   @Override
   public DiscreteSequence getOptimalFeedForwardControlSequence()
   {
      throw new RuntimeException("this isn't implemented correctly.");
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
