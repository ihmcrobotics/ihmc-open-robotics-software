package us.ihmc.commonWalkingControlModules.dynamicPlanning;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.ops.CommonOps;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.lists.RecyclingArrayList;

public class DDPSolver<E extends Enum>
{
   private double lineSearchGain = 0.15;
   private final DiscreteHybridDynamics<E> dynamics;
   private final LQCostFunction costFunction;
   private final LQCostFunction terminalCostFunction;

   private final RecyclingArrayList<DenseMatrix64F> stateTrajectory;
   private final RecyclingArrayList<DenseMatrix64F> controlTrajectory;
   private final RecyclingArrayList<DenseMatrix64F> desiredStateTrajectory;
   private final RecyclingArrayList<DenseMatrix64F> desiredControlTrajectory;

   private final RecyclingArrayList<DenseMatrix64F> feedBackGainTrajectory;
   private final RecyclingArrayList<DenseMatrix64F> feedForwardTrajectory;

   private final RecyclingArrayList<DenseMatrix64F> valueHessianTrajectory;
   private final RecyclingArrayList<DenseMatrix64F> valueGradientTrajectory;

   private final DenseMatrix64F L_X;
   private final DenseMatrix64F L_U;
   private final DenseMatrix64F L_XX;
   private final DenseMatrix64F L_UU;
   private final DenseMatrix64F L_UX;

   private final DenseMatrix64F f_X;
   private final DenseMatrix64F f_U;
   private final DenseMatrix64F f_XX;
   private final DenseMatrix64F f_UU;
   private final DenseMatrix64F f_UX;

   private final DenseMatrix64F Q_X;
   private final DenseMatrix64F Q_U;
   private final DenseMatrix64F Q_XX;
   private final DenseMatrix64F Q_UX;
   private final DenseMatrix64F Q_UU;
   private final DenseMatrix64F Q_UU_inv;
   private final DenseMatrix64F Q_XX_col;
   private final DenseMatrix64F Q_UX_col;
   private final DenseMatrix64F Q_UU_col;

   private final LinearSolver<DenseMatrix64F> solver = LinearSolverFactory.linear(0);

   private final DenseMatrix64F temp_X;
   private final DenseMatrix64F temp_XX;
   private final DenseMatrix64F temp_U;
   private final DenseMatrix64F temp_UU;
   private final DenseMatrix64F temp_UX;

   private final LQRSolverInterface<E> lqrSolver;

   private boolean useDynamicsHessian = false;
   private final boolean debug;

   public DDPSolver(DiscreteHybridDynamics<E> dynamics, LQCostFunction costFunction, LQCostFunction terminalCostFunction)
   {
      this(dynamics, costFunction, terminalCostFunction, false);
   }

   public DDPSolver(DiscreteHybridDynamics<E> dynamics, LQCostFunction costFunction, LQCostFunction terminalCostFunction, boolean debug)
   {
      this.dynamics = dynamics;
      this.costFunction = costFunction;
      this.terminalCostFunction = terminalCostFunction;
      this.debug = debug;

      this.lqrSolver = new DiscreteTimeVaryingTrackingLQRSolver<>(dynamics, costFunction, terminalCostFunction, debug);

      int stateSize = dynamics.getStateVectorSize();
      int controlSize = dynamics.getControlVectorSize();

      VariableVectorBuilder controlBuilder = new VariableVectorBuilder(controlSize, 1);
      VariableVectorBuilder stateBuilder = new VariableVectorBuilder(stateSize, 1);
      VariableVectorBuilder gainBuilder = new VariableVectorBuilder(controlSize, stateSize);
      VariableVectorBuilder hessianBuilder = new VariableVectorBuilder(stateSize, stateSize);

      stateTrajectory = new RecyclingArrayList<DenseMatrix64F>(1000, stateBuilder);
      controlTrajectory = new RecyclingArrayList<DenseMatrix64F>(1000, controlBuilder);
      desiredStateTrajectory = new RecyclingArrayList<DenseMatrix64F>(1000, stateBuilder);
      desiredControlTrajectory = new RecyclingArrayList<DenseMatrix64F>(1000, controlBuilder);
      feedBackGainTrajectory = new RecyclingArrayList<DenseMatrix64F>(1000, gainBuilder);
      feedForwardTrajectory = new RecyclingArrayList<DenseMatrix64F>(1000, controlBuilder);

      valueGradientTrajectory = new RecyclingArrayList<DenseMatrix64F>(1000, stateBuilder);
      valueHessianTrajectory = new RecyclingArrayList<DenseMatrix64F>(1000, hessianBuilder);

      stateTrajectory.clear();
      controlTrajectory.clear();
      feedBackGainTrajectory.clear();
      feedForwardTrajectory.clear();
      desiredStateTrajectory.clear();
      desiredControlTrajectory.clear();

      valueGradientTrajectory.clear();
      valueHessianTrajectory.clear();

      L_X = new DenseMatrix64F(stateSize, 1);
      L_U = new DenseMatrix64F(controlSize, 1);
      L_XX = new DenseMatrix64F(stateSize, stateSize);
      L_UU = new DenseMatrix64F(controlSize, controlSize);
      L_UX = new DenseMatrix64F(controlSize, stateSize);

      f_X = new DenseMatrix64F(stateSize, stateSize);
      f_U = new DenseMatrix64F(stateSize, controlSize);
      f_XX = new DenseMatrix64F(stateSize, stateSize);
      f_UU = new DenseMatrix64F(stateSize, controlSize);
      f_UX = new DenseMatrix64F(stateSize, controlSize);

      temp_X = new DenseMatrix64F(stateSize, 1);
      temp_U = new DenseMatrix64F(controlSize, 1);
      temp_XX = new DenseMatrix64F(stateSize, stateSize);
      temp_UU = new DenseMatrix64F(controlSize, controlSize);
      temp_UX = new DenseMatrix64F(controlSize, stateSize);

      Q_X = new DenseMatrix64F(stateSize, 1);
      Q_U = new DenseMatrix64F(controlSize, 1);
      Q_XX = new DenseMatrix64F(stateSize, stateSize);
      Q_UU = new DenseMatrix64F(controlSize, controlSize);
      Q_UU_inv = new DenseMatrix64F(controlSize, controlSize);
      Q_UX = new DenseMatrix64F(controlSize, stateSize);
      Q_XX_col = new DenseMatrix64F(stateSize, 1);
      Q_UX_col = new DenseMatrix64F(controlSize, 1);
      Q_UU_col = new DenseMatrix64F(controlSize, 1);
   }

   public void setUseDynamicsHessian(boolean useDynamicsHessian)
   {
      this.useDynamicsHessian = useDynamicsHessian;
   }

   public void setLineSearchGain(double lineSearchGain)
   {
      this.lineSearchGain = lineSearchGain;
   }

   public void solveBackwardLQRPass(E dynamicsState, int startIndex, int endIndex)
   {
      lqrSolver.solveRiccatiEquation(dynamicsState, startIndex, endIndex);
   }

   public void solveForwardLQRPass(E dynamicsState, int startIndex, int endIndex)
   {
      lqrSolver.computeOptimalTrajectories(dynamicsState, startIndex, endIndex);
   }

   public void initializeDDPWithLQRSolution()
   {
      lqrSolver.getOptimalTrajectories(stateTrajectory, controlTrajectory);
   }

   public void setDesiredTrajectories(RecyclingArrayList<DenseMatrix64F> desiredStateTrajectory, RecyclingArrayList<DenseMatrix64F> desiredControlTrajectory,
                                      DenseMatrix64F initialState, DenseMatrix64F initialControl, double totalDuration)
   {
      int numberOfTimeSteps = desiredStateTrajectory.size();

      stateTrajectory.clear();
      controlTrajectory.clear();
      this.desiredControlTrajectory.clear();
      this.desiredStateTrajectory.clear();
      feedBackGainTrajectory.clear();
      feedForwardTrajectory.clear();
      valueGradientTrajectory.clear();
      valueHessianTrajectory.clear();

      for (int i = 0; i < numberOfTimeSteps; i++)
      {
         this.desiredStateTrajectory.add().set(desiredStateTrajectory.get(i));
         this.desiredControlTrajectory.add().set(desiredControlTrajectory.get(i));

         stateTrajectory.add().set(desiredStateTrajectory.get(i));
         controlTrajectory.add().set(desiredControlTrajectory.get(i));

         feedBackGainTrajectory.add().zero();
         feedForwardTrajectory.add().zero();
         valueGradientTrajectory.add().zero();
         valueHessianTrajectory.add().zero();
      }

      stateTrajectory.getFirst().set(initialState);
      controlTrajectory.getFirst().set(initialControl);

      lqrSolver.setDesiredTrajectories(this.desiredStateTrajectory, this.desiredControlTrajectory, initialState);
   }

   public void solveBackwardDDPPass(E dynamicsState, int startIndex, int endIndex)
   {
      L_X.zero();
      L_U.zero();
      L_XX.zero();
      L_UU.zero();
      L_UX.zero();

      Q_U.zero();
      Q_X.zero();
      Q_UU.zero();
      Q_XX.zero();
      Q_UX.zero();

      // TODO get initial V_XX and V_X

      int i = endIndex;
      DenseMatrix64F state = stateTrajectory.get(i);
      DenseMatrix64F desiredState = desiredStateTrajectory.get(i);
      DenseMatrix64F control = controlTrajectory.get(i);
      DenseMatrix64F desiredControl = desiredControlTrajectory.get(i);

      DenseMatrix64F gainMatrix = feedBackGainTrajectory.get(i);
      DenseMatrix64F deltaUMatrix = feedForwardTrajectory.get(i);

      costFunction.getCostStateGradient(control, state, desiredControl, desiredState, L_X);
      costFunction.getCostControlGradient(control, state, desiredControl, desiredState, L_U);
      costFunction.getCostStateHessian(control, state, L_XX);
      costFunction.getCostControlHessian(control, state, L_UU);
      costFunction.getCostStateGradientOfControlGradient(control, state, L_UX);

      if (debug)
      {
         if (isAnyInvalid(L_X))
            throw new RuntimeException("The cost state gradient is invalid.");
         if (isAnyInvalid(L_U))
            throw new RuntimeException("The cost cost gradient is invalid.");
         if (isAnyInvalid(L_XX))
            throw new RuntimeException("The cost state hessian is invalid.");
         if (isAnyInvalid(L_UU))
            throw new RuntimeException("The cost cost hessian is invalid.");
      }

      terminalCostFunction.getCostStateGradient(control, state, desiredControl, desiredState, temp_X);
      terminalCostFunction.getCostControlGradient(control, state, desiredControl, desiredState, temp_U);
      terminalCostFunction.getCostStateHessian(control, state, temp_XX);
      terminalCostFunction.getCostControlHessian(control, state, temp_UU);
      terminalCostFunction.getCostStateGradientOfControlGradient(control, state, temp_UX);


      CommonOps.addEquals(L_X, temp_X);
      CommonOps.addEquals(L_U, temp_U);
      CommonOps.addEquals(L_XX, temp_XX);
      CommonOps.addEquals(L_UU, temp_UU);
      CommonOps.addEquals(L_UX, temp_UX);

      dynamics.getDynamicsStateGradient(dynamicsState, state, control, f_X);
      dynamics.getDynamicsControlGradient(dynamicsState, state, control, f_U);

      if (isAnyInvalid(f_X))
         throw new RuntimeException("The dynamics state gradient is invalid.");
      if (isAnyInvalid(f_U))
         throw new RuntimeException("The dynamics cost gradient is invalid.");

      DenseMatrix64F currentValueGradient = valueGradientTrajectory.getLast();
      DenseMatrix64F currentValueHessian = valueHessianTrajectory.getLast();

      Q_X.set(L_X);
      CommonOps.multAddTransA(f_X, currentValueGradient, Q_X);
      Q_U.set(L_U);
      CommonOps.multAddTransA(f_U, currentValueGradient, Q_U);

      Q_XX.set(L_XX);
      Q_UX.set(L_UX);
      Q_UU.set(L_UU);

      if (useDynamicsHessian)
      {
         for (int stateIndex = 0; stateIndex < dynamics.getStateVectorSize(); stateIndex++)
         {
            dynamics.getDynamicsStateHessian(dynamicsState, stateIndex, state, control, f_XX);
            dynamics.getDynamicsStateGradientOfControlGradient(dynamicsState, stateIndex, state, control, f_UX);

            CommonOps.multTransA(f_XX, currentValueGradient, Q_XX_col);
            MatrixTools.addMatrixBlock(Q_XX, 0, stateIndex, Q_XX_col, 0, 0, dynamics.getStateVectorSize(), 1, 1.0);

            CommonOps.multTransA(f_UX, currentValueGradient, Q_UX_col);
            MatrixTools.addMatrixBlock(Q_UX, 0, stateIndex, Q_UX_col, 0, 0, dynamics.getControlVectorSize(), 1, 1.0);
         }

         for (int controlIndex = 0; controlIndex < dynamics.getControlVectorSize(); controlIndex++)
         {
            dynamics.getDynamicsControlHessian(dynamicsState, controlIndex, state, control, f_UU);

            CommonOps.multTransA(f_UU, currentValueGradient, Q_UU_col);
            MatrixTools.addMatrixBlock(Q_UU, 0, controlIndex, Q_UU_col, 0, 0, dynamics.getControlVectorSize(), 1, 1.0);
         }
      }

      addSquareVector(f_X, currentValueHessian, f_X, Q_XX);
      addSquareVector(f_U, currentValueHessian, f_X, Q_UX);
      addSquareVector(f_U, currentValueHessian, f_U, Q_UU);

      computeFeedbackGainAndFeedForwardTerms(gainMatrix, deltaUMatrix, Q_U, Q_UU, Q_UX);

      DenseMatrix64F nextValueGradient = valueGradientTrajectory.get(i - 1);
      DenseMatrix64F nextValueHessian = valueHessianTrajectory.get(i - 1);

      computeValueApproximationForNextStep(nextValueGradient, nextValueHessian, gainMatrix, Q_X, Q_U, Q_XX, Q_UX);

      i--;

      if (debug)
      {
         if (isAnyInvalid(gainMatrix))
            throw new RuntimeException("The gain matrix is invalid.");
         if (isAnyInvalid(deltaUMatrix))
            throw new RuntimeException("The feedforward matrix is invalid.");
      }

      for (; i >= startIndex; i--)
      {
         currentValueGradient = valueGradientTrajectory.get(i);
         currentValueHessian = valueHessianTrajectory.get(i);

         state = stateTrajectory.get(i);
         desiredState = desiredStateTrajectory.get(i);
         control = controlTrajectory.get(i);
         desiredControl = desiredControlTrajectory.get(i);

         gainMatrix = feedBackGainTrajectory.get(i);
         deltaUMatrix = feedForwardTrajectory.get(i);

         updateLocalCostFunction(control, desiredControl, state, desiredState);

         dynamics.getDynamicsStateGradient(dynamicsState, state, control, f_X);
         dynamics.getDynamicsControlGradient(dynamicsState, state, control, f_U);

         if (debug)
         {
            if (isAnyInvalid(f_X))
               throw new RuntimeException("The dynamics state gradient is invalid.");
            if (isAnyInvalid(f_U))
               throw new RuntimeException("The dynamics cost gradient is invalid.");
         }

         Q_X.set(L_X);
         CommonOps.multAddTransA(f_X, currentValueGradient, Q_X);

         Q_U.set(L_U);
         CommonOps.multAddTransA(f_U, currentValueGradient, Q_U);

         Q_XX.set(L_XX);
         Q_UX.set(L_UX);
         Q_UU.set(L_UU);


         if (useDynamicsHessian)
         {
            for (int stateIndex = 0; stateIndex < dynamics.getStateVectorSize(); stateIndex++)
            {
               dynamics.getDynamicsStateHessian(dynamicsState, stateIndex, state, control, f_XX);
               dynamics.getDynamicsStateGradientOfControlGradient(dynamicsState, stateIndex, state, control, f_UX);

               CommonOps.multTransA(f_XX, currentValueGradient, Q_XX_col);
               MatrixTools.addMatrixBlock(Q_XX, 0, stateIndex, Q_XX_col, 0, 0, dynamics.getStateVectorSize(), 1, 1.0);

               CommonOps.multTransA(f_UX, currentValueGradient, Q_UX_col);
               MatrixTools.addMatrixBlock(Q_UX, 0, stateIndex, Q_UX_col, 0, 0, dynamics.getControlVectorSize(), 1, 1.0);
            }

            for (int controlIndex = 0; controlIndex < dynamics.getControlVectorSize(); controlIndex++)
            {
               dynamics.getDynamicsControlHessian(dynamicsState, controlIndex, state, control, f_UU);

               CommonOps.multTransA(f_UU, currentValueGradient, Q_UU_col);
               MatrixTools.addMatrixBlock(Q_UU, 0, controlIndex, Q_UU_col, 0, 0, dynamics.getControlVectorSize(), 1, 1.0);
            }
         }

         addSquareVector(f_X, currentValueHessian, f_X, Q_XX);
         addSquareVector(f_U, currentValueHessian, f_X, Q_UX);
         addSquareVector(f_U, currentValueHessian, f_U, Q_UU);

         computeFeedbackGainAndFeedForwardTerms(gainMatrix, deltaUMatrix, Q_U, Q_UU, Q_UX);

         if (debug)
         {
            if (isAnyInvalid(gainMatrix))
               throw new RuntimeException("The gain matrix is invalid.");
            if (isAnyInvalid(deltaUMatrix))
               throw new RuntimeException("The feedforward matrix is invalid.");
         }

         if (i > 0)
         {
            nextValueGradient = valueGradientTrajectory.get(i - 1);
            nextValueHessian = valueHessianTrajectory.get(i - 1);

            computeValueApproximationForNextStep(nextValueGradient, nextValueHessian, gainMatrix, Q_X, Q_U, Q_XX, Q_UX);

            if (debug)
            {
               if (isAnyInvalid(nextValueGradient))
                  throw new RuntimeException("The next value gradient is invalid.");
               if (isAnyInvalid(nextValueHessian))
                  throw new RuntimeException("The next value hessian is invalid.");
            }
         }
      }
   }

   private final DenseMatrix64F nextState = new DenseMatrix64F(0, 0);

   private final DenseMatrix64F updatedState = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F updatedControl = new DenseMatrix64F(0, 0);

   public void solveForwardDDPPass(E dynamicsState, int startIndex, int endIndex)
   {
      nextState.set(stateTrajectory.getFirst());
      updatedControl.reshape(dynamics.getControlVectorSize(), 1);

      for (int i = startIndex; i < endIndex; i++)
      {
         DenseMatrix64F state = stateTrajectory.get(i);
         DenseMatrix64F control = controlTrajectory.get(i);
         DenseMatrix64F feedForwardControl = feedForwardTrajectory.get(i);
         DenseMatrix64F gain = feedBackGainTrajectory.get(i);

         updatedState.set(nextState);

         computeUpdatedControl(state, updatedState, gain, feedForwardControl, control, updatedControl);
         dynamics.getNextState(dynamicsState, updatedState, updatedControl, nextState);

         state.set(updatedState);
         control.set(updatedControl);

         if (debug)
         {
            if (isAnyInvalid(state))
               throw new RuntimeException("The updated state is invalid.");
            if (isAnyInvalid(nextState))
               throw new RuntimeException("The next state is invalid.");
         }
      }

      stateTrajectory.getLast().set(nextState);
   }

   void computeFeedbackGainAndFeedForwardTerms(DenseMatrix64F feedbackGainToPack, DenseMatrix64F feedforwardControlToPack,
                                               DenseMatrix64F qControlGradient, DenseMatrix64F qControlHessian,
                                               DenseMatrix64F qStateGradientOfControlGradient)
   {
      solver.setA(qControlHessian);
      solver.invert(Q_UU_inv);

      CommonOps.mult(Q_UU_inv, qStateGradientOfControlGradient, feedbackGainToPack);
      CommonOps.mult(Q_UU_inv, qControlGradient, feedforwardControlToPack);
   }

   void computeValueApproximationForNextStep(DenseMatrix64F nextValueGradientToPack, DenseMatrix64F nextValueHessianToPack,
                                             DenseMatrix64F currentGainMatrix, DenseMatrix64F qStateGradient, DenseMatrix64F qControlGradient,
                                             DenseMatrix64F qStateHessian, DenseMatrix64F qStateGradientOfControlGradient)
   {
      nextValueGradientToPack.set(qStateGradient);
      nextValueHessianToPack.set(qStateHessian);

      CommonOps.multAddTransA(-1.0, currentGainMatrix, qControlGradient, nextValueGradientToPack);
      CommonOps.multAddTransA(-1.0, qStateGradientOfControlGradient, currentGainMatrix, nextValueHessianToPack);
   }

   private void updateLocalCostFunction(DenseMatrix64F currentControl, DenseMatrix64F desiredControl,
                                        DenseMatrix64F currentState, DenseMatrix64F desiredState)
   {
      costFunction.getCostStateGradient(currentControl, currentState, desiredControl, desiredState, L_X);
      costFunction.getCostControlGradient(currentControl, currentState, desiredControl, desiredState, L_U);
      costFunction.getCostStateHessian(currentControl, currentState, L_XX);
      costFunction.getCostControlHessian(currentControl, currentState, L_UU);
      costFunction.getCostStateGradientOfControlGradient(currentControl, currentState, L_UX);

      if (debug)
      {
         if (isAnyInvalid(L_X))
            throw new RuntimeException("The cost state gradient is invalid.");
         if (isAnyInvalid(L_U))
            throw new RuntimeException("The cost cost gradient is invalid.");
         if (isAnyInvalid(L_XX))
            throw new RuntimeException("The cost state hessian is invalid.");
         if (isAnyInvalid(L_UU))
            throw new RuntimeException("The cost cost hessian is invalid.");
         if (isAnyInvalid(L_UX))
            throw new RuntimeException("The cost second gradient is invalid.");
      }
   }


   private final DenseMatrix64F tempStateVector = new DenseMatrix64F(0, 0);

   void computeUpdatedControl(DenseMatrix64F currentState, DenseMatrix64F updatedState, DenseMatrix64F feedbackGainMatrix, DenseMatrix64F feedforwardControl,
                              DenseMatrix64F currentControl, DenseMatrix64F updatedControlToPack)
   {
      tempStateVector.reshape(currentState.getNumRows(), 1);

      CommonOps.add(currentControl, -lineSearchGain, feedforwardControl, updatedControlToPack);

      CommonOps.subtract(currentState, updatedState, tempStateVector);
      CommonOps.multAdd(feedbackGainMatrix, tempStateVector, updatedControlToPack);

      if (isAnyInvalid(updatedControlToPack))
         throw new RuntimeException("The updated control is invalid.");
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

   public RecyclingArrayList<DenseMatrix64F> getControlVector()
   {
      return controlTrajectory;
   }

   public RecyclingArrayList<DenseMatrix64F> getStateVector()
   {
      return stateTrajectory;
   }

   private final DenseMatrix64F tempMatrix = new DenseMatrix64F(0, 0);

   /**
    * D = D + A^T *  B * C
    */
   private void addSquareVector(DenseMatrix64F A, DenseMatrix64F B, DenseMatrix64F C, DenseMatrix64F DToPack)
   {
      tempMatrix.reshape(A.numCols, B.numCols);
      CommonOps.multTransA(A, B, tempMatrix);
      CommonOps.multAdd(tempMatrix, C, DToPack);
   }
}
