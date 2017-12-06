package us.ihmc.commonWalkingControlModules.dynamicPlanning;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.commons.MathTools;
import us.ihmc.robotics.linearAlgebra.DiagonalMatrixTools;
import us.ihmc.robotics.lists.RecyclingArrayList;

public class ContinuousTimeTrackingLQRSolver<E extends Enum> implements LQRSolverInterface<E>
{
   private final RecyclingArrayList<DenseMatrix64F> stateTrajectory;
   private final RecyclingArrayList<DenseMatrix64F> controlTrajectory;
   private final RecyclingArrayList<DenseMatrix64F> desiredStateTrajectory;
   private final RecyclingArrayList<DenseMatrix64F> desiredControlTrajectory;

   private final RecyclingArrayList<DenseMatrix64F> feedbackGainTrajectory;

   private final RecyclingArrayList<DenseMatrix64F> S2Trajectory;
   private final RecyclingArrayList<DenseMatrix64F> S1Trajectory;

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
   private final LQCostFunction costFunction;
   private final LQCostFunction terminalCostFunction;

   private final double deltaT;

   public ContinuousTimeTrackingLQRSolver(DiscreteHybridDynamics<E> dynamics, LQCostFunction costFunction, LQCostFunction terminalCostFunction,
                                          double deltaT)
   {
      this.dynamics = dynamics;
      this.costFunction = costFunction;
      this.terminalCostFunction = terminalCostFunction;
      this.deltaT = deltaT;

      int stateSize = dynamics.getStateVectorSize();
      int controlSize = dynamics.getControlVectorSize();

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

      VariableVectorBuilder controlBuilder = new VariableVectorBuilder(controlSize, 1);
      VariableVectorBuilder stateBuilder = new VariableVectorBuilder(stateSize, 1);
      VariableVectorBuilder gainBuilder = new VariableVectorBuilder(controlSize, stateSize);
      VariableVectorBuilder hessianBuilder = new VariableVectorBuilder(stateSize, stateSize);

      stateTrajectory = new RecyclingArrayList<DenseMatrix64F>(1000, stateBuilder);
      controlTrajectory = new RecyclingArrayList<DenseMatrix64F>(1000, controlBuilder);
      desiredStateTrajectory = new RecyclingArrayList<DenseMatrix64F>(1000, stateBuilder);
      desiredControlTrajectory = new RecyclingArrayList<DenseMatrix64F>(1000, controlBuilder);

      feedbackGainTrajectory = new RecyclingArrayList<DenseMatrix64F>(1000, gainBuilder);

      S2Trajectory = new RecyclingArrayList<DenseMatrix64F>(1000, hessianBuilder);
      S1Trajectory = new RecyclingArrayList<DenseMatrix64F>(1000, stateBuilder);

      stateTrajectory.clear();
      controlTrajectory.clear();
      desiredStateTrajectory.clear();
      desiredControlTrajectory.clear();

      feedbackGainTrajectory.clear();

      S2Trajectory.clear();
      S1Trajectory.clear();
   }

   public void setDesiredTrajectories(RecyclingArrayList<DenseMatrix64F> desiredStateTrajectory, RecyclingArrayList<DenseMatrix64F> desiredControlTrajectory,
                                      DenseMatrix64F initialState)
   {
      for (int i = 0; i < desiredStateTrajectory.size(); i++)
      {
         this.desiredStateTrajectory.add().set(desiredStateTrajectory.get(i));
         this.desiredControlTrajectory.add().set(desiredControlTrajectory.get(i));

         stateTrajectory.add();
         controlTrajectory.add();

         feedbackGainTrajectory.add();

         S2Trajectory.add();
         S1Trajectory.add();
      }

      stateTrajectory.getFirst().set(initialState);
   }

   private final DenseMatrix64F tempMatrix = new DenseMatrix64F(0, 0);
   public void solveRicattiEquation(E dynamicState, int startIndex, int endIndex) // backwards pass
   {
      int stateSize = dynamics.getStateVectorSize();
      int controlSize = dynamics.getControlVectorSize();

      int i = endIndex;
      DenseMatrix64F state = stateTrajectory.get(i);
      DenseMatrix64F control = controlTrajectory.get(i);
      DenseMatrix64F desiredState = desiredStateTrajectory.get(i);
      DenseMatrix64F desiredControl = desiredControlTrajectory.get(i);

      costFunction.getCostStateHessian(state, control, Q);
      costFunction.getCostControlHessian(state, control, R);
      terminalCostFunction.getCostStateHessian(state, control, Qf);

      DenseMatrix64F S2 = this.S2Trajectory.get(i);
      DenseMatrix64F S1 = this.S1Trajectory.get(i);
      S2.set(Qf);
      CommonOps.mult(-2.0, Qf, desiredState, S1);

      dynamics.getContinuousAMatrix(A);
      dynamics.getContinuousBMatrix(B);

      DiagonalMatrixTools.invertDiagonalMatrix(R, R_inv);
      CommonOps.multTransB(R_inv, B, R_invBT);

      // compute S2 dot
      S2Dot.set(Q);
      CommonOps.multAdd(S2, A, S2Dot);
      CommonOps.multAddTransA(A, S2, S2Dot);

      tempMatrix.reshape(stateSize, controlSize);
      CommonOps.multTransA(S2, B, tempMatrix);
      CommonOps.mult(tempMatrix, R_invBT, S2BR_invBT);

      CommonOps.multAdd(-1.0, S2BR_invBT, S2, S2Dot);


      // compute S1 dot
      CommonOps.mult(-2.0, Q, desiredState, S1Dot);
      CommonOps.multAddTransA(A, S1, S1Dot);
      CommonOps.multAdd(-1.0, S2BR_invBT, S1, S1Dot);

      tempMatrix.reshape(stateSize, 1);
      CommonOps.mult(B, desiredControl, tempMatrix);
      CommonOps.multAdd(2.0, S2, tempMatrix, S1Dot);


      // compute previous S2 and S1
      DenseMatrix64F previousS2 = S2Trajectory.get(i - 1);
      DenseMatrix64F previousS1 = S1Trajectory.get(i - 1);

      previousS2.set(S2);
      previousS1.set(S1);

      CommonOps.addEquals(previousS2, deltaT, S2Dot);
      CommonOps.addEquals(previousS1, deltaT, S1Dot);

      i--;

      // backward pass
      for (; i > startIndex; i--)
      {
         desiredState = desiredStateTrajectory.get(i);
         desiredControl = desiredControlTrajectory.get(i);

         S2 = this.S2Trajectory.get(i);
         S1 = this.S1Trajectory.get(i);

         // compute S2 dot
         S2Dot.set(Q);
         CommonOps.multAdd(S2, A, S2Dot);
         CommonOps.multAddTransA(A, S2, S2Dot);

         tempMatrix.reshape(stateSize, controlSize);
         CommonOps.multTransA(S2, B, tempMatrix);
         CommonOps.mult(tempMatrix, R_invBT, S2BR_invBT);

         CommonOps.multAdd(-1.0, S2BR_invBT, S2, S2Dot);

         // compute S1 dot
         CommonOps.mult(-2.0, Q, desiredState, S1Dot);
         CommonOps.multAddTransA(A, S1, S1Dot);
         CommonOps.multAdd(-1.0, S2BR_invBT, S1, S1Dot);

         tempMatrix.reshape(stateSize, 1);
         CommonOps.mult(B, desiredControl, tempMatrix);
         CommonOps.multAdd(2.0, S2, tempMatrix, S1Dot);

         // compute previous S2 and S1
         previousS2 = S2Trajectory.get(i - 1);
         previousS1 = S1Trajectory.get(i - 1);

         previousS2.set(S2);
         previousS1.set(S1);

         CommonOps.addEquals(previousS2, deltaT, S2Dot);
         CommonOps.addEquals(previousS1, deltaT, S1Dot);

         if (isAnyInvalid(S2) || isAnyInvalid(S1))
            throw new RuntimeException("bad");
      }

   }

   // forward pass
   public void computeOptimalTrajectories(E dynamicsState, int startIndex, int endIndex) // forward pass
   {
      int stateSize = dynamics.getStateVectorSize();
      for (int i = startIndex; i < endIndex; i++)
      {
         DenseMatrix64F desiredControl = desiredControlTrajectory.get(i);
         DenseMatrix64F state = stateTrajectory.get(i);
         DenseMatrix64F control = controlTrajectory.get(i);

         DenseMatrix64F nextState = stateTrajectory.get(i + 1);
         DenseMatrix64F S2 = S2Trajectory.get(i);
         DenseMatrix64F S1 = S1Trajectory.get(i);

         control.set(desiredControl);

         tempMatrix.reshape(stateSize, 1);
         CommonOps.mult(S2, state, tempMatrix);
         CommonOps.addEquals(tempMatrix, 0.5, S1);
         CommonOps.multAdd(-1.0, R_invBT, tempMatrix, control);

         CommonOps.mult(A, state, XDot);
         CommonOps.multAdd(B, control, XDot);

         nextState.set(state);
         CommonOps.addEquals(nextState, deltaT, XDot);

         if (isAnyInvalid(state) || isAnyInvalid(control))
            throw new RuntimeException("bad");
      }
   }

   @Override
   public void getOptimalTrajectories(RecyclingArrayList<DenseMatrix64F> optimalStateTrajectoryToPack,
                                      RecyclingArrayList<DenseMatrix64F> optimalControlTrajectoryToPack)
   {
      for (int i = 0; i < stateTrajectory.size(); i++)
      {
         optimalStateTrajectoryToPack.getAndGrowIfNeeded(i).set(stateTrajectory.get(i));
         optimalControlTrajectoryToPack.getAndGrowIfNeeded(i).set(controlTrajectory.get(i));
      }
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
