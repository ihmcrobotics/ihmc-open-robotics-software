package us.ihmc.commonWalkingControlModules.dynamicPlanning;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.ops.CommonOps;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.robotics.lists.GenericTypeBuilder;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.math.trajectories.SegmentedFrameTrajectory3D;

public class LIPMDDPCalculator
{
   private final DiscreteHybridDynamics<LIPMState> dynamics;
   private final DDPCostFunction costFunction;
   private final DDPCostFunction terminalCostFunction;

   private double deltaT;

   private final RecyclingArrayList<DenseMatrix64F> stateVector;
   private final RecyclingArrayList<DenseMatrix64F> controlVector;
   private final RecyclingArrayList<DenseMatrix64F> desiredStateVector;
   private final RecyclingArrayList<DenseMatrix64F> desiredControlVector;

   private final RecyclingArrayList<DenseMatrix64F> gainVector;
   private final RecyclingArrayList<DenseMatrix64F> deltaUVector;

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

   private final DenseMatrix64F V_X;
   private final DenseMatrix64F V_XX;

   private final LinearSolver<DenseMatrix64F> solver = LinearSolverFactory.linear(0);

   private final DenseMatrix64F temp_X;
   private final DenseMatrix64F temp_XX;
   private final DenseMatrix64F temp_U;
   private final DenseMatrix64F temp_UU;
   private final DenseMatrix64F temp_UX;

   private int numberOfTimeSteps;
   private final double mass;
   private final double gravityZ;

   public LIPMDDPCalculator(double deltaT, double mass, double gravityZ)
   {
      this.dynamics = new LIPMDynamics(deltaT, mass, gravityZ);
      this.costFunction = new LIPMSimpleCostFunction();
      this.terminalCostFunction = new LIPMTerminalCostFunction();
      this.deltaT = deltaT;
      this.mass = mass;
      this.gravityZ = gravityZ;

      int stateSize = dynamics.getStateVectorSize();
      int controlSize = dynamics.getControlVectorSize();

      VariableVectorBuilder controlBuilder = new VariableVectorBuilder(controlSize);
      VariableVectorBuilder stateBuilder = new VariableVectorBuilder(stateSize);

      stateVector = new RecyclingArrayList<DenseMatrix64F>(1000, stateBuilder);
      controlVector = new RecyclingArrayList<DenseMatrix64F>(1000, controlBuilder);
      desiredStateVector = new RecyclingArrayList<DenseMatrix64F>(1000, stateBuilder);
      desiredControlVector = new RecyclingArrayList<DenseMatrix64F>(1000, controlBuilder);
      gainVector = new RecyclingArrayList<DenseMatrix64F>(1000, stateBuilder);
      deltaUVector = new RecyclingArrayList<DenseMatrix64F>(1000, controlBuilder);

      stateVector.clear();
      controlVector.clear();
      gainVector.clear();

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

      V_X = new DenseMatrix64F(1, stateSize);
      V_XX = new DenseMatrix64F(stateSize, stateSize);
   }

   public void setDeltaT(double deltaT)
   {
      dynamics.setTimeStepSize(deltaT);
      this.deltaT = deltaT;
   }

   private final FramePoint3D tempState = new FramePoint3D();
   private final FrameVector3D tempStateVelocity = new FrameVector3D();
   private final FramePoint3D tempControl = new FramePoint3D();

   public void initialize(DenseMatrix64F currentState, DenseMatrix64F currentControl, SegmentedFrameTrajectory3D desiredStates,
                          SegmentedFrameTrajectory3D desiredControls, double trajectoryLength)
   {
      double modifiedDeltaT = computeDeltaT(trajectoryLength);
      dynamics.setTimeStepSize(modifiedDeltaT);

      double time = 0.0;
      for (int i = 0; i < numberOfTimeSteps; i++)
      {
         stateVector.add().set(currentState);
         controlVector.add().set(currentControl);

         desiredStates.update(time, tempState, tempStateVelocity);
         DenseMatrix64F desiredState = desiredStateVector.add();
         desiredState.set(0, tempState.getX());
         desiredState.set(1, tempState.getY());
         desiredState.set(2, tempState.getZ());
         desiredState.set(3, tempStateVelocity.getX());
         desiredState.set(4, tempStateVelocity.getY());
         desiredState.set(5, tempStateVelocity.getZ());

         desiredControls.update(time, tempControl);
         DenseMatrix64F desiredControl = desiredControlVector.add();
         desiredControl.set(0, tempControl.getX());
         desiredControl.set(1, tempControl.getY());
         desiredControl.set(2, mass * gravityZ);

         gainVector.add().zero();
         deltaUVector.add().zero();
      }
   }

   public void backwardPass()
   {
      int i = numberOfTimeSteps - 1;
      DenseMatrix64F state = stateVector.get(i);
      DenseMatrix64F desiredState = desiredStateVector.get(i);
      DenseMatrix64F control = controlVector.get(i);
      DenseMatrix64F desiredControl = desiredControlVector.get(i);

      DenseMatrix64F gainMatrix = gainVector.get(i);
      DenseMatrix64F deltaUMatrix = deltaUVector.get(i);

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

      Q_X.set(L_X);
      Q_U.set(L_U);
      Q_XX.set(L_XX);
      Q_UX.set(L_UX);
      Q_UU.set(L_UU);

      solver.setA(Q_UU);
      solver.invert(Q_UU_inv);

      CommonOps.mult(Q_UU_inv, Q_UX, gainMatrix);
      CommonOps.mult(Q_UU_inv, Q_U, deltaUMatrix);

      V_X.set(Q_X);
      V_XX.set(Q_XX);

      CommonOps.multAdd(Q_U, gainMatrix, V_X);
      CommonOps.multAdd(Q_UX, gainMatrix, V_XX);

      for (i = numberOfTimeSteps - 2; i >= 0; i--)
      {
         state = stateVector.get(i);
         desiredState = desiredStateVector.get(i);
         control = controlVector.get(i);
         desiredControl = desiredControlVector.get(i);

         gainMatrix = gainVector.get(i);
         deltaUMatrix = deltaUVector.get(i);

         costFunction.getCostStateGradient(control, state, desiredControl, desiredState, L_X);
         costFunction.getCostControlGradient(control, state, desiredControl, desiredState, L_U);
         costFunction.getCostStateHessian(control, state, L_XX);
         costFunction.getCostControlHessian(control, state, L_UU);
         costFunction.getCostStateGradientOfControlGradient(control, state, L_UX);

         dynamics.getDynamicsStateGradient(LIPMState.NORMAL, state, control, f_X);
         dynamics.getDynamicsControlGradient(LIPMState.NORMAL, state, control, f_U);

         Q_X.set(L_X);
         CommonOps.multAdd(V_X, f_X, Q_X);

         Q_U.set(L_U);
         CommonOps.multAdd(V_X, f_U, Q_U);

         Q_XX.set(L_XX);
         Q_UX.set(L_UX);

         for (int stateIndex = 0; stateIndex < dynamics.getStateVectorSize(); stateIndex++)
         {
            dynamics.getDynamicsStateHessian(LIPMState.NORMAL, stateIndex, state, control, f_XX);
            dynamics.getDynamicsStateGradientOfControlGradient(LIPMState.NORMAL, stateIndex, state, control, f_UX);

            CommonOps.multAdd(V_X, f_XX, Q_XX);
            CommonOps.multAdd(V_X, f_UX, Q_UX);
         }
         addSquareVector(f_X, V_XX, f_X, Q_XX);
         addSquareVector(f_U, V_XX, f_X, Q_UX);

         Q_UU.set(L_UU);
         for (int controlIndex = 0; controlIndex < dynamics.getStateVectorSize(); controlIndex++)
         {
            dynamics.getDynamicsControlHessian(LIPMState.NORMAL, controlIndex, state, control, f_UU);
            CommonOps.multAdd(V_X, f_UU, Q_UU);
         }
         addSquareVector(f_U, V_XX, f_U, Q_UU);

         solver.setA(Q_UU);
         solver.invert(Q_UU_inv);

         CommonOps.mult(Q_UU_inv, Q_UX, gainMatrix);
         CommonOps.mult(Q_UU_inv, Q_U, deltaUMatrix);

         V_X.set(Q_X);
         V_XX.set(Q_XX);

         CommonOps.multAdd(Q_U, gainMatrix, V_X);
         CommonOps.multAdd(Q_UX, gainMatrix, V_XX);
      }
   }

   private final DenseMatrix64F tempStateVector = new DenseMatrix64F(6, 1);
   private final DenseMatrix64F x_hat = new DenseMatrix64F(6, 1);

   public void forwardPass()
   {
      DenseMatrix64F state = stateVector.get(0);
      DenseMatrix64F control = stateVector.get(0);
      DenseMatrix64F deltaU = deltaUVector.get(0);

      CommonOps.addEquals(control, deltaU);

      dynamics.getDynamics(LIPMState.NORMAL, state, control, x_hat);

      for (int i = 1; i < stateVector.size() - 1; i++)
      {
         state = stateVector.get(i);
         control = stateVector.get(i);
         deltaU = deltaUVector.get(i);
         DenseMatrix64F gain = gainVector.get(i);

         CommonOps.addEquals(control, deltaU);
         CommonOps.subtract(x_hat, state, tempStateVector);
         CommonOps.multAdd(gain, tempStateVector, control);

         state.set(x_hat);

         dynamics.getDynamics(LIPMState.NORMAL, state, control, x_hat);
      }
   }

   private double computeDeltaT(double trajectoryLength)
   {
      numberOfTimeSteps = (int) Math.floorDiv((long) trajectoryLength, (long) deltaT);
      return trajectoryLength / numberOfTimeSteps;
   }

   private final DenseMatrix64F tempMatrix = new DenseMatrix64F();

   /**
    * D = D + A^T *  B * C
    */
   private void addSquareVector(DenseMatrix64F A, DenseMatrix64F B, DenseMatrix64F C, DenseMatrix64F DToPack)
   {
      tempMatrix.reshape(A.numCols, B.numCols);
      CommonOps.multTransA(A, B, tempMatrix);
      CommonOps.multAdd(tempMatrix, C, DToPack);
   }

   private class VariableVectorBuilder extends GenericTypeBuilder
   {
      private final int size;

      public VariableVectorBuilder(int size)
      {
         this.size = size;
      }

      @Override
      public Object newInstance()
      {
         return new DenseMatrix64F(size, 1);
      }
   }
}
