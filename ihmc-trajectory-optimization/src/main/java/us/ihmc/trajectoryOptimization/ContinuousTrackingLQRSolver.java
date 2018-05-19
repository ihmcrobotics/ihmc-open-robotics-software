package us.ihmc.trajectoryOptimization;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.robotics.linearAlgebra.DiagonalMatrixTools;
import us.ihmc.robotics.lists.RecyclingArrayList;

public class ContinuousTrackingLQRSolver<E extends Enum> implements LQRSolverInterface<E>
{
   private final DiscreteOptimizationData optimalSequence;
   private final DiscreteOptimizationData desiredSequence;

   private final DiscreteSequence feedbackGainSequence;
   private final DiscreteSequence constantsSequence;

   private final RecyclingArrayList<DenseMatrix64F> S2Sequence;
   private final RecyclingArrayList<DenseMatrix64F> S1Sequence;

   private final DenseMatrix64F Q;
   private final DenseMatrix64F R;
   private final DenseMatrix64F R_inv;
   private final DenseMatrix64F Qf;
   private final DenseMatrix64F S2Dot;
   private final DenseMatrix64F S1Dot;

   private final DenseMatrix64F A;
   private final DenseMatrix64F B;
   private final DenseMatrix64F XDot;

   private final DenseMatrix64F S2BR_invBT;
   private final DenseMatrix64F R_invBT;

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

      Q = new DenseMatrix64F(stateSize, stateSize);
      Qf = new DenseMatrix64F(stateSize, stateSize);
      R = new DenseMatrix64F(controlSize, controlSize);
      R_inv = new DenseMatrix64F(controlSize, controlSize);
      S2Dot = new DenseMatrix64F(stateSize, stateSize);
      S1Dot = new DenseMatrix64F(stateSize, 1);

      A = new DenseMatrix64F(stateSize, stateSize);
      B = new DenseMatrix64F(stateSize, controlSize);
      XDot = new DenseMatrix64F(stateSize, 1);

      S2BR_invBT = new DenseMatrix64F(stateSize, stateSize);
      R_invBT = new DenseMatrix64F(controlSize, stateSize);

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
   public void setDesiredSequence(DiscreteOptimizationData desiredSequence, DiscreteSequence constantsSequence, DenseMatrix64F initialState)
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

   private final DenseMatrix64F tempMatrix = new DenseMatrix64F(0, 0);
   public void solveRiccatiEquation(E dynamicState, int startIndex, int endIndex) // backwards pass
   {
      int stateSize = dynamics.getStateVectorSize();
      int controlSize = dynamics.getControlVectorSize();

      int i = endIndex;
      DenseMatrix64F state = optimalSequence.getState(i);
      DenseMatrix64F control = optimalSequence.getControl(i);
      DenseMatrix64F desiredState = desiredSequence.getState(i);
      DenseMatrix64F constants = constantsSequence.get(i);
      DenseMatrix64F desiredControl;

      costFunction.getCostStateHessian(dynamicState, state, control, constants, Q);
      costFunction.getCostControlHessian(dynamicState, state, control, constants, R);
      terminalCostFunction.getCostStateHessian(dynamicState, state, control, constants, Qf);

      dynamics.getContinuousAMatrix(A);
      dynamics.getContinuousBMatrix(B);

      DiagonalMatrixTools.invertDiagonalMatrix(R, R_inv);
      CommonOps.multTransB(R_inv, B, R_invBT);


      DenseMatrix64F initialS2 = this.S2Sequence.get(i);
      DenseMatrix64F initialS1 = this.S1Sequence.get(i);
      initialS2.set(Qf);
      CommonOps.mult(-2.0, Qf, desiredState, initialS1);

      // backward pass
      for (; i > startIndex; i--)
      {
         desiredState = desiredSequence.getState(i);
         desiredControl = desiredSequence.getControl(i);

         DenseMatrix64F currentS2 = S2Sequence.get(i);
         DenseMatrix64F currentS1 = S1Sequence.get(i);

         // compute currentS2 dot
         S2Dot.set(Q);
         CommonOps.multAdd(currentS2, A, S2Dot);
         CommonOps.multAddTransA(A, currentS2, S2Dot);

         tempMatrix.reshape(stateSize, controlSize);
         CommonOps.multTransA(currentS2, B, tempMatrix);
         CommonOps.mult(tempMatrix, R_invBT, S2BR_invBT);

         CommonOps.multAdd(-1.0, S2BR_invBT, currentS2, S2Dot);

         // compute currentS1 dot
         CommonOps.mult(-2.0, Q, desiredState, S1Dot);
         CommonOps.multAddTransA(A, currentS1, S1Dot);
         CommonOps.multAdd(-1.0, S2BR_invBT, currentS1, S1Dot);

         tempMatrix.reshape(stateSize, 1);
         CommonOps.mult(B, desiredControl, tempMatrix);
         CommonOps.multAdd(2.0, currentS2, tempMatrix, S1Dot);

         // compute previous currentS2 and currentS1
         DenseMatrix64F previousS2 = S2Sequence.get(i - 1);
         DenseMatrix64F previousS1 = S1Sequence.get(i - 1);

         previousS2.set(currentS2);
         previousS1.set(currentS1);

         CommonOps.addEquals(previousS2, deltaT, S2Dot);
         CommonOps.addEquals(previousS1, deltaT, S1Dot);

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
         DenseMatrix64F desiredControl = desiredSequence.getControl(i);
         DenseMatrix64F state = optimalSequence.getState(i);
         DenseMatrix64F control = optimalSequence.getControl(i);
         DenseMatrix64F gain = feedbackGainSequence.get(i);

         DenseMatrix64F nextState = optimalSequence.getState(i + 1);
         DenseMatrix64F S2 = S2Sequence.get(i);
         DenseMatrix64F S1 = S1Sequence.get(i);

         control.set(desiredControl);

         tempMatrix.reshape(stateSize, 1);
         CommonOps.mult(S2, state, tempMatrix);
         CommonOps.addEquals(tempMatrix, 0.5, S1);
         CommonOps.mult(-1.0, R_invBT, tempMatrix, gain);
         CommonOps.multAdd(-1.0, R_invBT, tempMatrix, control);

         CommonOps.mult(A, state, XDot);
         CommonOps.multAdd(B, control, XDot);

         nextState.set(state);
         CommonOps.addEquals(nextState, deltaT, XDot);

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
   public DenseMatrix64F getValueHessian()
   {
      return S1Sequence.get(0);
   }

   @Override
   public DiscreteSequence getOptimalFeedForwardControlSequence()
   {
      throw new RuntimeException("this isn't implemented correctly.");
   }

   private boolean isAnyInvalid(DenseMatrix64F matrix)
   {
      for (int i = 0; i < matrix.getNumElements(); i++)
      {
         if (!Double.isFinite(matrix.get(i)))
            return true;
      }
      return false;
   }
}
