package us.ihmc.trajectoryOptimization;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.robotics.linearAlgebra.DiagonalMatrixTools;
import us.ihmc.robotics.lists.RecyclingArrayList;

public class ContinuousTrackingLQRSolver<E extends Enum> implements LQRSolverInterface<E>
{
   private final DiscreteOptimizationTrajectory optimalTrajectory;
   private final DiscreteOptimizationTrajectory desiredTrajectory;

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

   private final boolean debug;

   public ContinuousTrackingLQRSolver(DiscreteHybridDynamics<E> dynamics, LQCostFunction costFunction, LQCostFunction terminalCostFunction,
                                      double deltaT)
   {
      this(dynamics, costFunction, terminalCostFunction, deltaT, false);
   }
   public ContinuousTrackingLQRSolver(DiscreteHybridDynamics<E> dynamics, LQCostFunction costFunction, LQCostFunction terminalCostFunction,
                                      double deltaT, boolean debug)
   {
      this.dynamics = dynamics;
      this.costFunction = costFunction;
      this.terminalCostFunction = terminalCostFunction;
      this.deltaT = deltaT;
      this.debug = debug;

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

      VariableVectorBuilder stateBuilder = new VariableVectorBuilder(stateSize, 1);
      VariableVectorBuilder hessianBuilder = new VariableVectorBuilder(stateSize, stateSize);

      optimalTrajectory = new DiscreteOptimizationTrajectory(stateSize, controlSize);
      desiredTrajectory = new DiscreteOptimizationTrajectory(stateSize, controlSize);

      feedbackGainTrajectory = new RecyclingArrayList<>(1000, new VariableVectorBuilder(controlSize, stateSize));

      S2Trajectory = new RecyclingArrayList<>(1000, hessianBuilder);
      S1Trajectory = new RecyclingArrayList<>(1000, stateBuilder);

      feedbackGainTrajectory.clear();

      S2Trajectory.clear();
      S1Trajectory.clear();
   }

   @Override
   public void setDesiredTrajectory(DiscreteOptimizationTrajectory desiredTrajectory, DenseMatrix64F initialState)
   {
      this.desiredTrajectory.set(desiredTrajectory);
      this.optimalTrajectory.setZeroTrajectory(desiredTrajectory);

      this.S2Trajectory.clear();
      this.S1Trajectory.clear();
      this.feedbackGainTrajectory.clear();

      for (int i = 0; i < desiredTrajectory.size(); i++)
      {
         S2Trajectory.add();
         S1Trajectory.add();

         feedbackGainTrajectory.add();
      }

      optimalTrajectory.setState(0, initialState);
   }

   private final DenseMatrix64F tempMatrix = new DenseMatrix64F(0, 0);
   public void solveRiccatiEquation(E dynamicState, int startIndex, int endIndex) // backwards pass
   {
      int stateSize = dynamics.getStateVectorSize();
      int controlSize = dynamics.getControlVectorSize();

      int i = endIndex;
      DenseMatrix64F state = optimalTrajectory.getState(i);
      DenseMatrix64F control = optimalTrajectory.getControl(i);
      DenseMatrix64F desiredState = desiredTrajectory.getState(i);
      DenseMatrix64F desiredControl;

      costFunction.getCostStateHessian(state, control, Q);
      costFunction.getCostControlHessian(state, control, R);
      terminalCostFunction.getCostStateHessian(state, control, Qf);

      dynamics.getContinuousAMatrix(A);
      dynamics.getContinuousBMatrix(B);

      DiagonalMatrixTools.invertDiagonalMatrix(R, R_inv);
      CommonOps.multTransB(R_inv, B, R_invBT);


      DenseMatrix64F initialS2 = this.S2Trajectory.get(i);
      DenseMatrix64F initialS1 = this.S1Trajectory.get(i);
      initialS2.set(Qf);
      CommonOps.mult(-2.0, Qf, desiredState, initialS1);

      // backward pass
      for (; i > startIndex; i--)
      {
         desiredState = desiredTrajectory.getState(i);
         desiredControl = desiredTrajectory.getControl(i);

         DenseMatrix64F currentS2 = S2Trajectory.get(i);
         DenseMatrix64F currentS1 = S1Trajectory.get(i);

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
         DenseMatrix64F previousS2 = S2Trajectory.get(i - 1);
         DenseMatrix64F previousS1 = S1Trajectory.get(i - 1);

         previousS2.set(currentS2);
         previousS1.set(currentS1);

         CommonOps.addEquals(previousS2, deltaT, S2Dot);
         CommonOps.addEquals(previousS1, deltaT, S1Dot);

         if (debug && (isAnyInvalid(currentS2) || isAnyInvalid(currentS1)))
            throw new RuntimeException("The computed Riccati equation solutions are ill-conditioned.");
      }

   }

   // forward pass
   public void computeOptimalTrajectories(E dynamicsState, int startIndex, int endIndex) // forward pass
   {
      int stateSize = dynamics.getStateVectorSize();
      for (int i = startIndex; i < endIndex; i++)
      {
         DenseMatrix64F desiredControl = desiredTrajectory.getControl(i);
         DenseMatrix64F state = optimalTrajectory.getState(i);
         DenseMatrix64F control = optimalTrajectory.getControl(i);
         DenseMatrix64F gain = feedbackGainTrajectory.get(i);

         DenseMatrix64F nextState = optimalTrajectory.getState(i + 1);
         DenseMatrix64F S2 = S2Trajectory.get(i);
         DenseMatrix64F S1 = S1Trajectory.get(i);

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
   public void getOptimalTrajectory(DiscreteOptimizationTrajectory optimalTrajectoryToPack)
   {
      optimalTrajectoryToPack.set(optimalTrajectory);
   }

   @Override
   public DiscreteOptimizationTrajectory getOptimalTrajectory()
   {
      return optimalTrajectory;
   }

   @Override
   public DiscreteTrajectory getOptimalStateTrajectory()
   {
      return optimalTrajectory.getStateTrajectory();
   }

   @Override
   public DiscreteTrajectory getOptimalControlTrajectory()
   {
      return optimalTrajectory.getControlTrajectory();
   }

   @Override
   public RecyclingArrayList<DenseMatrix64F> getOptimalFeedbackGainTrajectory()
   {
      throw new RuntimeException("this isn't implemented correctly.");
   }

   @Override
   public DenseMatrix64F getValueHessian()
   {
      return S1Trajectory.get(0);
   }

   @Override
   public RecyclingArrayList<DenseMatrix64F> getOptimalFeedForwardControlTrajectory()
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
