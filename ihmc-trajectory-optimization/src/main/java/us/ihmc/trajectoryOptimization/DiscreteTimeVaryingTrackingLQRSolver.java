package us.ihmc.trajectoryOptimization;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.ops.CommonOps;
import us.ihmc.robotics.lists.RecyclingArrayList;

public class DiscreteTimeVaryingTrackingLQRSolver<E extends Enum> implements LQRSolverInterface<E>
{
   private final DiscreteOptimizationData optimalSequence;
   private final DiscreteOptimizationData desiredSequence;

   private final DiscreteSequence feedbackGainSequence;
   private final DiscreteSequence feedforwardSequence;

   private final DiscreteSequence constantsSequence;

   private final RecyclingArrayList<DenseMatrix64F> s1Sequence;
   private final RecyclingArrayList<DenseMatrix64F> s2Sequence;

   private final LinearSolver<DenseMatrix64F> linearSolver = LinearSolverFactory.linear(0);

   private final DenseMatrix64F Q;
   private final DenseMatrix64F R;
   private final DenseMatrix64F Qf;


   private final DenseMatrix64F A;
   private final DenseMatrix64F B;

   private final DenseMatrix64F G;
   private final DenseMatrix64F G_inv;
   private final DenseMatrix64F H;

   private final DiscreteHybridDynamics<E> dynamics;
   private final LQTrackingCostFunction<E> costFunction;
   private final LQTrackingCostFunction<E> terminalCostFunction;

   private final DenseMatrix64F tempMatrix = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F tempMatrix2 = new DenseMatrix64F(0, 0);

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

      Q = new DenseMatrix64F(stateSize, stateSize);
      Qf = new DenseMatrix64F(stateSize, stateSize);
      R = new DenseMatrix64F(controlSize, controlSize);
      G_inv = new DenseMatrix64F(controlSize, controlSize);
      G = new DenseMatrix64F(controlSize, controlSize);

      A = new DenseMatrix64F(stateSize, stateSize);
      B = new DenseMatrix64F(stateSize, controlSize);
      H = new DenseMatrix64F(stateSize, stateSize);

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
   public void setDesiredSequence(DiscreteOptimizationData desiredSequence, DiscreteSequence constantsSequence, DenseMatrix64F initialState)
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

      DenseMatrix64F currentDesiredControl = desiredSequence.getControl(i);
      DenseMatrix64F currentDesiredState = desiredSequence.getState(i);
      DenseMatrix64F currentConstants = constantsSequence.get(i);

      terminalCostFunction.getCostStateHessian(dynamicState, currentDesiredState, currentDesiredControl, currentConstants, Qf);

      DenseMatrix64F initialS1 = s1Sequence.get(i);
      DenseMatrix64F initialS2 = s2Sequence.get(i);
      initialS1.set(Qf);
      CommonOps.multTransA(-2.0, desiredSequence.getState(i), Qf, initialS2);

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

         DenseMatrix64F currentGainMatrix = feedbackGainSequence.get(i);
         DenseMatrix64F currentFeedForwardMatrix = feedforwardSequence.get(i);

         DenseMatrix64F currentS1Matrix = s1Sequence.get(i);
         DenseMatrix64F currentS2Matrix = s2Sequence.get(i);
         DenseMatrix64F nextS1Matrix = s1Sequence.get(i + 1);
         DenseMatrix64F nextS2Matrix = s2Sequence.get(i + 1);

         dynamics.getDynamicsStateGradient(dynamicState, currentDesiredState, currentDesiredControl, currentConstants, A);
         dynamics.getDynamicsControlGradient(dynamicState, currentDesiredState, currentDesiredControl, currentConstants, B);

         // G = R + B^T S1 B
         G.set(R);
         addMultQuad(B, nextS1Matrix, B, G);

         linearSolver.setA(G);
         linearSolver.invert(G_inv);

         // K = -G^-1 B^T S1 A
         tempMatrix.reshape(controlSize, stateSize);
         CommonOps.multTransB(G_inv, B, tempMatrix);

         tempMatrix2.reshape(stateSize, stateSize);
         CommonOps.mult(nextS1Matrix, A, tempMatrix2);

         CommonOps.mult(-1.0, tempMatrix, tempMatrix2, currentGainMatrix);

         // F = G^-1 (R u_d - 0.5 B^T S2^T)
         tempMatrix.reshape(controlSize, 1);
         CommonOps.multTransAB(-0.5, B, nextS2Matrix, tempMatrix);
         CommonOps.multAdd(R, currentDesiredControl, tempMatrix);
         CommonOps.mult(G_inv, tempMatrix, currentFeedForwardMatrix);

         // S1_k = Q + K^T R K + (A + B K)^T S1_k+1 (A + B K)
         currentS1Matrix.set(Q);
         addMultQuad(currentGainMatrix, R, currentGainMatrix, currentS1Matrix);

         H.set(A);
         CommonOps.multAdd(B, currentGainMatrix, H);
         addMultQuad(H, nextS1Matrix, H, currentS1Matrix);

         // S2_k = 2 (B F)^T S1_k+1 (A + B K) + S2_k+1 (A + B K) + 2 (F - u_d)^T R K - 2 x_d^T Q
         tempMatrix.reshape(stateSize, 1);

         // (S2_k+1 + 2 (B F)^T S1_k+1) (A + B K)
         tempMatrix2.set(nextS2Matrix);
         CommonOps.mult(B, currentFeedForwardMatrix, tempMatrix);
         CommonOps.multAddTransA(2.0, tempMatrix, nextS1Matrix, tempMatrix2);
         CommonOps.mult(tempMatrix2, H, currentS2Matrix);

         CommonOps.multAddTransA(-2.0, currentDesiredState, Q, currentS2Matrix);

         // 2 (F - u_d)^T R K
         tempMatrix.reshape(controlSize, stateSize);
         CommonOps.mult(R, currentGainMatrix, tempMatrix);
         CommonOps.multAddTransA(-2.0, currentDesiredControl, tempMatrix, currentS2Matrix);
         CommonOps.multAddTransA(2.0, currentFeedForwardMatrix, tempMatrix, currentS2Matrix);

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
         DenseMatrix64F currentDesiredState = desiredSequence.getState(i);
         DenseMatrix64F currentDesiredControl = desiredSequence.getControl(i);
         DenseMatrix64F currentConstants = constantsSequence.get(i);

         dynamics.getDynamicsStateGradient(dynamicState, currentDesiredState, currentDesiredControl, currentConstants, A);
         dynamics.getDynamicsControlGradient(dynamicState, currentDesiredState, currentDesiredControl, currentConstants, B);

         DenseMatrix64F optimalControl = optimalSequence.getControl(i);

         DenseMatrix64F currentState = optimalSequence.getState(i);
         DenseMatrix64F nextState = optimalSequence.getState(i + 1);

         DenseMatrix64F feedbackGainMatrix = feedbackGainSequence.get(i);
         DenseMatrix64F feedforwardMatrix = feedforwardSequence.get(i);

         // u_k = K x_k + F
         optimalControl.set(feedforwardMatrix);
         CommonOps.multAdd(feedbackGainMatrix, currentState, optimalControl);

         // x_k+1 = A x_k + B u_k
         CommonOps.mult(A, currentState, nextState);
         CommonOps.multAdd(B, optimalControl, nextState);

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
   public DenseMatrix64F getValueHessian()
   {
      return s1Sequence.get(0);
   }

   private final DenseMatrix64F tempMatrix3 = new DenseMatrix64F(0, 0);
   /**
    * D = D + A^T *  B * C
    */
   private void addMultQuad(DenseMatrix64F A, DenseMatrix64F B, DenseMatrix64F C, DenseMatrix64F DToPack)
   {
      tempMatrix3.reshape(A.numCols, B.numCols);
      CommonOps.multTransA(A, B, tempMatrix3);
      CommonOps.multAdd(tempMatrix3, C, DToPack);
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
