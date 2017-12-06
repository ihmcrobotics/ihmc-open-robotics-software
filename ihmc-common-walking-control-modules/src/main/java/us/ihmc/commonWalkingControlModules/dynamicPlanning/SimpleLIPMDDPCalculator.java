package us.ihmc.commonWalkingControlModules.dynamicPlanning;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.ops.CommonOps;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.robotics.linearAlgebra.DiagonalMatrixTools;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.lists.GenericTypeBuilder;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.math.trajectories.SegmentedFrameTrajectory3D;

public class SimpleLIPMDDPCalculator
{
   private double lineSearchGain = 0.0;
   private final DiscreteHybridDynamics<LIPMState> dynamics;
   private final LQCostFunction costFunction;
   private final LQCostFunction terminalCostFunction;

   private double deltaT;
   private double modifiedDeltaT;

   private final RecyclingArrayList<DenseMatrix64F> stateVector;
   private final RecyclingArrayList<DenseMatrix64F> controlVector;
   private final RecyclingArrayList<DenseMatrix64F> desiredStateVector;
   private final RecyclingArrayList<DenseMatrix64F> desiredControlVector;

   private final RecyclingArrayList<DenseMatrix64F> gainVector;
   private final RecyclingArrayList<DenseMatrix64F> feedforwardVector;

   private final RecyclingArrayList<DenseMatrix64F> valueHessian;
   private final RecyclingArrayList<DenseMatrix64F> valueGradient;

   private final RecyclingArrayList<DenseMatrix64F> S2Vector;
   private final RecyclingArrayList<DenseMatrix64F> S1Vector;

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

   private final LQRSolverInterface<LIPMState> lqrSolver;

   private final DenseMatrix64F temp_X;
   private final DenseMatrix64F temp_XX;
   private final DenseMatrix64F temp_U;
   private final DenseMatrix64F temp_UU;
   private final DenseMatrix64F temp_UX;

   private int numberOfTimeSteps;
   private final double mass;
   private final double gravityZ;

   public SimpleLIPMDDPCalculator(double deltaT, double mass, double gravityZ)
   {
      this.dynamics = new SimpleLIPMDynamics(deltaT, 1.0, gravityZ);
      this.costFunction = new SimpleLIPMSimpleCostFunction();
      this.terminalCostFunction = new SimpleLIPMTerminalCostFunction();
      this.deltaT = deltaT;
      this.mass = mass;
      this.gravityZ = gravityZ;

      lqrSolver = new ContinuousTimeTrackingLQRSolver<>(dynamics, costFunction, terminalCostFunction, deltaT);

      int stateSize = dynamics.getStateVectorSize();
      int controlSize = dynamics.getControlVectorSize();

      VariableVectorBuilder controlBuilder = new VariableVectorBuilder(controlSize, 1);
      VariableVectorBuilder stateBuilder = new VariableVectorBuilder(stateSize, 1);
      VariableVectorBuilder gainBuilder = new VariableVectorBuilder(controlSize, stateSize);
      VariableVectorBuilder hessianBuilder = new VariableVectorBuilder(stateSize, stateSize);

      stateVector = new RecyclingArrayList<DenseMatrix64F>(1000, stateBuilder);
      controlVector = new RecyclingArrayList<DenseMatrix64F>(1000, controlBuilder);
      desiredStateVector = new RecyclingArrayList<DenseMatrix64F>(1000, stateBuilder);
      desiredControlVector = new RecyclingArrayList<DenseMatrix64F>(1000, controlBuilder);
      gainVector = new RecyclingArrayList<DenseMatrix64F>(1000, gainBuilder);
      feedforwardVector = new RecyclingArrayList<DenseMatrix64F>(1000, controlBuilder);

      valueGradient = new RecyclingArrayList<DenseMatrix64F>(1000, stateBuilder);
      valueHessian = new RecyclingArrayList<DenseMatrix64F>(1000, hessianBuilder);

      S2Vector = new RecyclingArrayList<DenseMatrix64F>(1000, hessianBuilder);
      S1Vector = new RecyclingArrayList<DenseMatrix64F>(1000, stateBuilder);

      stateVector.clear();
      controlVector.clear();
      gainVector.clear();
      feedforwardVector.clear();
      desiredStateVector.clear();
      desiredControlVector.clear();

      valueGradient.clear();
      valueHessian.clear();
      S2Vector.clear();
      S1Vector.clear();

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

   public void setLineSearchGain(double lineSearchGain)
   {
      this.lineSearchGain = lineSearchGain;
   }

   public void setDeltaT(double deltaT)
   {
      dynamics.setTimeStepSize(deltaT);
      this.deltaT = deltaT;
      this.modifiedDeltaT = deltaT;
   }

   private final FramePoint3D tempPoint = new FramePoint3D();
   private final FrameVector3D tempVector = new FrameVector3D();

   public void initialize(DenseMatrix64F currentState, DenseMatrix64F currentControl, SegmentedFrameTrajectory3D copDesiredPlan)
   {
      modifiedDeltaT = computeDeltaT(copDesiredPlan.getFinalTime());
      dynamics.setTimeStepSize(modifiedDeltaT);

      stateVector.clear();
      controlVector.clear();
      desiredControlVector.clear();
      desiredStateVector.clear();
      gainVector.clear();
      feedforwardVector.clear();
      valueGradient.clear();
      valueHessian.clear();
      S2Vector.clear();
      S1Vector.clear();

      double time = 0.0;
      stateVector.add().set(currentState);
      controlVector.add().set(currentControl);

      copDesiredPlan.update(time, tempPoint, tempVector);
      DenseMatrix64F desiredState = desiredStateVector.add();

      desiredState.set(0, tempPoint.getX());
      desiredState.set(1, tempPoint.getY());
      desiredState.set(2, tempVector.getX());
      desiredState.set(3, tempVector.getY());

      DenseMatrix64F desiredControl = desiredControlVector.add();
      desiredControl.set(0, tempPoint.getX());
      desiredControl.set(1, tempPoint.getY());

      valueGradient.add().zero();
      valueHessian.add().zero();
      gainVector.add().zero();
      feedforwardVector.add().zero();
      S2Vector.add().zero();
      S1Vector.add().zero();

      time += modifiedDeltaT;

      for (int i = 1; i < numberOfTimeSteps; i++)
      {
         copDesiredPlan.update(time, tempPoint, tempVector);
         desiredState = desiredStateVector.add();

         desiredState.set(0, tempPoint.getX());
         desiredState.set(1, tempPoint.getY());
         desiredState.set(2, tempVector.getX());
         desiredState.set(3, tempVector.getY());

         desiredControl = desiredControlVector.add();
         desiredControl.set(0, tempPoint.getX());
         desiredControl.set(1, tempPoint.getY());

         DenseMatrix64F state = stateVector.add();
         DenseMatrix64F control = controlVector.add();

         // TODO change?
         /*
         state.set(currentState);
         control.set(currentControl);
         */
         state.set(desiredState);
         control.set(desiredControl);

         gainVector.add().zero();
         feedforwardVector.add().zero();
         valueGradient.add().zero();
         valueHessian.add().zero();
         S2Vector.add().zero();
         S1Vector.add().zero();

         time += modifiedDeltaT;
      }

      lqrSolver.setDesiredTrajectories(desiredStateVector, desiredControlVector, currentState);
      lqrSolver.solveRicattiEquation(LIPMState.NORMAL, 0, desiredStateVector.size() - 1);
      lqrSolver.computeOptimalTrajectories(LIPMState.NORMAL, 0, desiredStateVector.size() - 1);
      lqrSolver.getOptimalTrajectories(stateVector, controlVector);
   }

   public void backwardDDPPass()
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

      int i = numberOfTimeSteps - 1;
      DenseMatrix64F state = stateVector.get(i);
      DenseMatrix64F desiredState = desiredStateVector.get(i);
      DenseMatrix64F control = controlVector.get(i);
      DenseMatrix64F desiredControl = desiredControlVector.get(i);

      DenseMatrix64F gainMatrix = gainVector.get(i);
      DenseMatrix64F deltaUMatrix = feedforwardVector.get(i);

      costFunction.getCostStateGradient(control, state, desiredControl, desiredState, L_X);
      costFunction.getCostControlGradient(control, state, desiredControl, desiredState, L_U);
      costFunction.getCostStateHessian(control, state, L_XX);
      costFunction.getCostControlHessian(control, state, L_UU);
      costFunction.getCostStateGradientOfControlGradient(control, state, L_UX);

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

      dynamics.getDynamicsStateGradient(LIPMState.NORMAL, state, control, f_X);
      dynamics.getDynamicsControlGradient(LIPMState.NORMAL, state, control, f_U);

      DenseMatrix64F currentValueGradient = valueGradient.getLast();
      DenseMatrix64F currentValueHessian = valueHessian.getLast();

      Q_X.set(L_X);
      CommonOps.multAddTransA(f_X, currentValueGradient, Q_X);
      Q_U.set(L_U);
      CommonOps.multAddTransA(f_U, currentValueGradient, Q_U);

      Q_XX.set(L_XX);
      Q_UX.set(L_UX);
      Q_UU.set(L_UU);

      /*
      for (int stateIndex = 0; stateIndex < dynamics.getStateVectorSize(); stateIndex++)
      {
         dynamics.getDynamicsStateHessian(LIPMState.NORMAL, stateIndex, state, control, f_XX);
         dynamics.getDynamicsStateGradientOfControlGradient(LIPMState.NORMAL, stateIndex, state, control, f_UX);

         CommonOps.multTransA(f_XX, currentValueGradient, Q_XX_col);
         MatrixTools.addMatrixBlock(Q_XX, 0, stateIndex, Q_XX_col, 0, 0, dynamics.getStateVectorSize(), 1, 1.0);

         CommonOps.multTransA(f_UX, currentValueGradient, Q_UX_col);
         MatrixTools.addMatrixBlock(Q_UX, 0, stateIndex, Q_UX_col, 0, 0, dynamics.getControlVectorSize(), 1, 1.0);
      }
      */
      addSquareVector(f_X, currentValueHessian, f_X, Q_XX);
      addSquareVector(f_U, currentValueHessian, f_X, Q_UX);

      Q_UU.set(L_UU);
      /*
      for (int controlIndex = 0; controlIndex < dynamics.getControlVectorSize(); controlIndex++)
      {
         dynamics.getDynamicsControlHessian(LIPMState.NORMAL, controlIndex, state, control, f_UU);

         CommonOps.multTransA(f_UU, currentValueGradient, Q_UU_col);
         MatrixTools.addMatrixBlock(Q_UU, 0, controlIndex, Q_UU_col, 0, 0, dynamics.getControlVectorSize(), 1, 1.0);
      }
      */
      addSquareVector(f_U, currentValueHessian, f_U, Q_UU);

      computeFeedbackGainAndFeedForwardTerms(gainMatrix, deltaUMatrix, Q_U, Q_UU, Q_UX);

      DenseMatrix64F nextValueGradient = valueGradient.get(i - 1);
      DenseMatrix64F nextValueHessian = valueHessian.get(i - 1);

      computeValueApproximationForNextStep(nextValueGradient, nextValueHessian, gainMatrix, Q_X, Q_U, Q_XX, Q_UX);

      sanityCheck(gainMatrix, deltaUMatrix);

      for (i = numberOfTimeSteps - 2; i >= 0; i--)
      {
         currentValueGradient = valueGradient.get(i);
         currentValueHessian = valueHessian.get(i);

         state = stateVector.get(i);
         desiredState = desiredStateVector.get(i);
         control = controlVector.get(i);
         desiredControl = desiredControlVector.get(i);

         gainMatrix = gainVector.get(i);
         deltaUMatrix = feedforwardVector.get(i);

         updateLocalCostFunction(control, desiredControl, state, desiredState);

         dynamics.getDynamicsStateGradient(LIPMState.NORMAL, state, control, f_X);
         dynamics.getDynamicsControlGradient(LIPMState.NORMAL, state, control, f_U);

         Q_X.set(L_X);
         CommonOps.multAddTransA(f_X, currentValueGradient, Q_X);

         Q_U.set(L_U);
         CommonOps.multAddTransA(f_U, currentValueGradient, Q_U);

         Q_XX.set(L_XX);
         Q_UX.set(L_UX);

         /*
         for (int stateIndex = 0; stateIndex < dynamics.getStateVectorSize(); stateIndex++)
         {
            dynamics.getDynamicsStateHessian(LIPMState.NORMAL, stateIndex, state, control, f_XX);
            dynamics.getDynamicsStateGradientOfControlGradient(LIPMState.NORMAL, stateIndex, state, control, f_UX);

            CommonOps.multTransA(f_XX, currentValueGradient, Q_XX_col);
            MatrixTools.addMatrixBlock(Q_XX, 0, stateIndex, Q_XX_col, 0, 0, dynamics.getStateVectorSize(), 1, 1.0);

            CommonOps.multTransA(f_UX, currentValueGradient, Q_UX_col);
            MatrixTools.addMatrixBlock(Q_UX, 0, stateIndex, Q_UX_col, 0, 0, dynamics.getControlVectorSize(), 1, 1.0);
         }
         */
         addSquareVector(f_X, currentValueHessian, f_X, Q_XX);
         addSquareVector(f_U, currentValueHessian, f_X, Q_UX);

         Q_UU.set(L_UU);
         /*
         for (int controlIndex = 0; controlIndex < dynamics.getControlVectorSize(); controlIndex++)
         {
            dynamics.getDynamicsControlHessian(LIPMState.NORMAL, controlIndex, state, control, f_UU);

            CommonOps.multTransA(f_UU, currentValueGradient, Q_UU_col);
            MatrixTools.addMatrixBlock(Q_UU, 0, controlIndex, Q_UU_col, 0, 0, dynamics.getControlVectorSize(), 1, 1.0);
         }
         */
         addSquareVector(f_U, currentValueHessian, f_U, Q_UU);

         computeFeedbackGainAndFeedForwardTerms(gainMatrix, deltaUMatrix, Q_U, Q_UU, Q_UX);

         if (i > 0)
         {
            nextValueGradient = valueGradient.get(i - 1);
            nextValueHessian = valueHessian.get(i - 1);

            computeValueApproximationForNextStep(nextValueGradient, nextValueHessian, gainMatrix, Q_X, Q_U, Q_XX, Q_UX);
         }

         sanityCheck(gainMatrix, deltaUMatrix);
      }
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
   }

   private void sanityCheck(DenseMatrix64F gainMatrix, DenseMatrix64F feedForwardMatrix)
   {
      if (MatrixTools.containsNaN(gainMatrix))
         throw new RuntimeException("Gain matrix contains NaN.");
      if (MatrixTools.containsNaN(feedForwardMatrix))
         throw new RuntimeException("Feed forward dynamics contains NaN.");
   }

   private final DenseMatrix64F tempStateVector = new DenseMatrix64F(4, 1);
   private final DenseMatrix64F nextState = new DenseMatrix64F(4, 1);

   private final DenseMatrix64F updatedState = new DenseMatrix64F(4, 1);
   private final DenseMatrix64F updatedControl = new DenseMatrix64F(2, 1);

   public void forwardDDPPass()
   {
      nextState.set(stateVector.getFirst());

      for (int i = 0; i < stateVector.size() - 1; i++)
      {
         DenseMatrix64F state = stateVector.get(i);
         DenseMatrix64F control = controlVector.get(i);
         DenseMatrix64F feedForwardControl = feedforwardVector.get(i);
         DenseMatrix64F gain = gainVector.get(i);

         updatedState.set(nextState);

         computeUpdatedControl(state, updatedState, gain, feedForwardControl, control, updatedControl);
         dynamics.getNextState(LIPMState.NORMAL, updatedState, updatedControl, nextState);

         state.set(updatedState);
         control.set(updatedControl);

         if (isAnyInvalid(state) || isAnyInvalid(control) || isAnyInvalid(nextState))
            throw new RuntimeException("Dynamics error on " + i + " of " + stateVector.size());
      }

      stateVector.getLast().set(nextState);
   }

   void computeUpdatedControl(DenseMatrix64F currentState, DenseMatrix64F updatedState, DenseMatrix64F feedbackGainMatrix, DenseMatrix64F feedforwardControl,
                              DenseMatrix64F currentControl, DenseMatrix64F updatedControlToPack)
   {
      CommonOps.add(currentControl, -lineSearchGain, feedforwardControl, updatedControlToPack);

      CommonOps.subtract(currentState, updatedState, tempStateVector);
      CommonOps.multAdd(feedbackGainMatrix, tempStateVector, updatedControlToPack);
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

   private double computeDeltaT(double trajectoryLength)
   {
      numberOfTimeSteps = (int) Math.floor(trajectoryLength / deltaT);
      return trajectoryLength / numberOfTimeSteps;
   }

   public double getDT()
   {
      return modifiedDeltaT;
   }

   public RecyclingArrayList<DenseMatrix64F> getControlVector()
   {
      return controlVector;
   }

   public RecyclingArrayList<DenseMatrix64F> getStateVector()
   {
      return stateVector;
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
