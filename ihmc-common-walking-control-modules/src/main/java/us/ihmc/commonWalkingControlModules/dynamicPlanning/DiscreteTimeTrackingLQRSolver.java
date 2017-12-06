package us.ihmc.commonWalkingControlModules.dynamicPlanning;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.ops.CommonOps;
import us.ihmc.commons.MathTools;
import us.ihmc.robotics.linearAlgebra.DiagonalMatrixTools;
import us.ihmc.robotics.lists.RecyclingArrayList;

public class DiscreteTimeTrackingLQRSolver<E extends Enum> implements LQRSolverInterface<E>
{
   private final RecyclingArrayList<DenseMatrix64F> optimalStateTrajectory;
   private final RecyclingArrayList<DenseMatrix64F> optimalControlTrajectory;

   private final RecyclingArrayList<DenseMatrix64F> desiredStateTrajectory;
   private final RecyclingArrayList<DenseMatrix64F> desiredControlTrajectory;

   private final RecyclingArrayList<DenseMatrix64F> feedbackGainTrajectory;
   private final RecyclingArrayList<DenseMatrix64F> feedforwardTrajectory;

   private final RecyclingArrayList<DenseMatrix64F> s1Trajectory;
   private final RecyclingArrayList<DenseMatrix64F> s2Trajectory;

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
   private final LQCostFunction costFunction;
   private final LQCostFunction terminalCostFunction;

   private final DenseMatrix64F tempMatrix = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F tempMatrix2 = new DenseMatrix64F(0, 0);

   public DiscreteTimeTrackingLQRSolver(DiscreteHybridDynamics<E> dynamics, LQCostFunction costFunction, LQCostFunction terminalCostFunction)
   {
      this.dynamics = dynamics;
      this.costFunction = costFunction;
      this.terminalCostFunction = terminalCostFunction;

      int stateSize = dynamics.getStateVectorSize();
      int controlSize = dynamics.getControlVectorSize();

      Q = new DenseMatrix64F(stateSize, stateSize);
      Qf = new DenseMatrix64F(stateSize, stateSize);
      R = new DenseMatrix64F(controlSize, controlSize);
      G_inv = new DenseMatrix64F(controlSize, controlSize);
      G = new DenseMatrix64F(controlSize, controlSize);

      A = new DenseMatrix64F(stateSize, stateSize);
      B = new DenseMatrix64F(stateSize, controlSize);
      H = new DenseMatrix64F(stateSize, stateSize);

      VariableVectorBuilder controlBuilder = new VariableVectorBuilder(controlSize, 1);
      VariableVectorBuilder stateBuilder = new VariableVectorBuilder(stateSize, 1);
      VariableVectorBuilder gainBuilder = new VariableVectorBuilder(controlSize, stateSize);

      optimalStateTrajectory = new RecyclingArrayList<DenseMatrix64F>(1000, stateBuilder);
      optimalControlTrajectory = new RecyclingArrayList<DenseMatrix64F>(1000, controlBuilder);
      desiredStateTrajectory = new RecyclingArrayList<DenseMatrix64F>(1000, stateBuilder);
      desiredControlTrajectory = new RecyclingArrayList<DenseMatrix64F>(1000, controlBuilder);

      feedbackGainTrajectory = new RecyclingArrayList<DenseMatrix64F>(1000, gainBuilder);
      feedforwardTrajectory = new RecyclingArrayList<DenseMatrix64F>(1000, controlBuilder);

      s1Trajectory = new RecyclingArrayList<DenseMatrix64F>(1000, new VariableVectorBuilder(stateSize, stateSize));
      s2Trajectory = new RecyclingArrayList<DenseMatrix64F>(1000, new VariableVectorBuilder(1, stateSize));

      optimalStateTrajectory.clear();
      optimalControlTrajectory.clear();
      desiredStateTrajectory.clear();
      desiredControlTrajectory.clear();

      feedbackGainTrajectory.clear();
      feedforwardTrajectory.clear();

      s1Trajectory.clear();
      s2Trajectory.clear();
   }

   public void setDesiredTrajectories(RecyclingArrayList<DenseMatrix64F> desiredStateTrajectory, RecyclingArrayList<DenseMatrix64F> desiredControlTrajectory,
                                      DenseMatrix64F initialState)
   {
      this.optimalStateTrajectory.clear();
      this.optimalControlTrajectory.clear();
      this.desiredStateTrajectory.clear();
      this.desiredControlTrajectory.clear();

      this.feedbackGainTrajectory.clear();
      this.feedforwardTrajectory.clear();

      this.s1Trajectory.clear();
      this.s2Trajectory.clear();

      for (int i = 0; i < desiredStateTrajectory.size(); i++)
      {
         this.desiredStateTrajectory.add().set(desiredStateTrajectory.get(i));
         this.desiredControlTrajectory.add().set(desiredControlTrajectory.get(i));

         optimalStateTrajectory.add();
         optimalControlTrajectory.add();

         feedbackGainTrajectory.add();
         feedforwardTrajectory.add();

         s1Trajectory.add();
         s2Trajectory.add();
      }

      optimalStateTrajectory.getFirst().set(initialState);
   }

   public void solveRiccatiEquation(E dynamicState, int startIndex, int endIndex) // backwards pass
   {
      int stateSize = dynamics.getStateVectorSize();
      int controlSize = dynamics.getControlVectorSize();

      int i = endIndex;
      // assumes a constant A and B matrix throughout the trajectory
      DenseMatrix64F initialDesiredState = desiredStateTrajectory.get(startIndex);
      DenseMatrix64F initialDesiredControl = desiredControlTrajectory.get(startIndex);

      costFunction.getCostStateHessian(initialDesiredState, initialDesiredControl, Q);
      costFunction.getCostControlHessian(initialDesiredState, initialDesiredControl, R);
      terminalCostFunction.getCostStateHessian(initialDesiredState, initialDesiredControl, Qf);

      dynamics.getDynamicsStateGradient(dynamicState, initialDesiredState, initialDesiredControl, A);
      dynamics.getDynamicsControlGradient(dynamicState, initialDesiredState, initialDesiredControl, B);

      DenseMatrix64F initialS1 = s1Trajectory.get(i);
      DenseMatrix64F initialS2 = s2Trajectory.get(i);
      initialS1.set(Qf);
      CommonOps.multTransA(-2.0, desiredStateTrajectory.get(i), Qf, initialS2);

      i--;

      for (; i >= startIndex; i--)
      {
         DenseMatrix64F currentGainMatrix = feedbackGainTrajectory.get(i);
         DenseMatrix64F currentFeedForwardMatrix = feedforwardTrajectory.get(i);

         DenseMatrix64F currentS1Matrix = s1Trajectory.get(i);
         DenseMatrix64F currentS2Matrix = s2Trajectory.get(i);
         DenseMatrix64F nextS1Matrix = s1Trajectory.get(i + 1);
         DenseMatrix64F nextS2Matrix = s2Trajectory.get(i + 1);

         DenseMatrix64F currentDesiredControl = desiredControlTrajectory.get(i);
         DenseMatrix64F currentDesiredState = desiredStateTrajectory.get(i);

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

         // S2_k = (S2_k+1 + 2 (B F)^T S1_k+1) (A + B K) - 2(x_d^T Q + u_d^T R K)
         tempMatrix.reshape(stateSize, 1);
         CommonOps.mult(B, currentFeedForwardMatrix, tempMatrix);
         tempMatrix2.set(nextS2Matrix);
         CommonOps.multAddTransA(2.0, tempMatrix, nextS1Matrix, tempMatrix2);

         CommonOps.mult(tempMatrix2, H, currentS2Matrix);

         CommonOps.multAddTransA(-2.0, currentDesiredState, Q, currentS2Matrix);

         tempMatrix.reshape(controlSize, stateSize);
         CommonOps.mult(R, currentGainMatrix, tempMatrix);
         CommonOps.multAddTransA(-2.0, currentDesiredControl, tempMatrix, currentS2Matrix);
      }
   }

   // forward pass
   public void computeOptimalTrajectories(E dynamicState, int startIndex, int endIndex) // forward pass
   {
      // assumes a constant A and B matrix throughout the trajectory
      DenseMatrix64F initialDesiredState = desiredStateTrajectory.get(startIndex);
      DenseMatrix64F initialDesiredControl = desiredControlTrajectory.get(startIndex);

      dynamics.getDynamicsStateGradient(dynamicState, initialDesiredState, initialDesiredControl, A);
      dynamics.getDynamicsControlGradient(dynamicState, initialDesiredState, initialDesiredControl, B);

      // the first index is the initial state
      for (int i = startIndex; i < endIndex - 1; i++)
      {
         DenseMatrix64F optimalControl = optimalControlTrajectory.get(i);

         DenseMatrix64F currentState = optimalStateTrajectory.get(i);
         DenseMatrix64F nextState = optimalStateTrajectory.get(i + 1);

         DenseMatrix64F feedbackGainMatrix = feedbackGainTrajectory.get(i);
         DenseMatrix64F feedforwardMatrix = feedforwardTrajectory.get(i);

         // u_k = K x_k + F
         optimalControl.set(feedforwardMatrix);
         CommonOps.multAdd(feedbackGainMatrix, currentState, optimalControl);

         // x_k+1 = A x_k + B u_k
         CommonOps.mult(A, currentState, nextState);
         CommonOps.multAdd(B, optimalControl, nextState);

         if (isAnyInvalid(optimalControl) || isAnyInvalid(nextState))
            throw new RuntimeException("bad");
      }
   }

   @Override
   public void getOptimalTrajectories(RecyclingArrayList<DenseMatrix64F> optimalStateTrajectoryToPack,
                                      RecyclingArrayList<DenseMatrix64F> optimalControlTrajectoryToPack)
   {
      for (int i = 0; i < optimalStateTrajectory.size(); i++)
      {
         optimalStateTrajectoryToPack.getAndGrowIfNeeded(i).set(optimalStateTrajectory.get(i));
         optimalControlTrajectoryToPack.getAndGrowIfNeeded(i).set(optimalControlTrajectory.get(i));
      }
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
         if (!MathTools.intervalContains(matrix.get(i), 1e8))
            return true;
      }
      return false;
   }
}
