package us.ihmc.trajectoryOptimization;

import java.util.ArrayList;
import java.util.List;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

public class CompositeLQCostFunction<E extends Enum> implements LQTrackingCostFunction<E>
{
   private final List<LQCostFunction<E>> lqCostFunctions = new ArrayList<>();
   private final List<LQTrackingCostFunction<E>> lqTrackingCostFunctions = new ArrayList<>();

   private final DMatrixRMaj tempMatrix = new DMatrixRMaj(0, 0);

   public void addLQCostFunction(LQCostFunction<E> costFunction)
   {
      lqCostFunctions.add(costFunction);
   }

   public void addLQTrackingCostFunction(LQTrackingCostFunction<E> costFunction)
   {
      lqTrackingCostFunctions.add(costFunction);
   }

   @Override
   public double getCost(E state, DMatrixRMaj controlVector, DMatrixRMaj stateVector, DMatrixRMaj desiredControlVector,
                         DMatrixRMaj desiredStateVector, DMatrixRMaj constants)
   {
      double cost = 0.0;
      for (int i = 0; i < lqCostFunctions.size(); i++)
         cost += lqCostFunctions.get(i).getCost(state, controlVector, stateVector, constants);
      for (int i = 0; i < lqTrackingCostFunctions.size(); i++)
         cost += lqTrackingCostFunctions.get(i).getCost(state, controlVector, stateVector, desiredControlVector, desiredStateVector, constants);

      return cost;
   }

   @Override
   public void getCostStateGradient(E state, DMatrixRMaj controlVector, DMatrixRMaj stateVector, DMatrixRMaj desiredControlVector,
                                    DMatrixRMaj desiredStateVector, DMatrixRMaj constants, DMatrixRMaj matrixToPack)
   {
      matrixToPack.zero();
      tempMatrix.reshape(matrixToPack.getNumRows(), matrixToPack.getNumCols());

      for (int i = 0; i < lqCostFunctions.size(); i++)
      {
         lqCostFunctions.get(i).getCostStateGradient(state, controlVector, stateVector, constants, tempMatrix);
         CommonOps_DDRM.addEquals(matrixToPack, tempMatrix);
      }

      for (int i = 0; i < lqTrackingCostFunctions.size(); i++)
      {
         lqTrackingCostFunctions.get(i).getCostStateGradient(state, controlVector, stateVector, desiredControlVector, desiredStateVector, constants, tempMatrix);
         CommonOps_DDRM.addEquals(matrixToPack, tempMatrix);
      }
   }

   @Override
   public void getCostControlGradient(E state, DMatrixRMaj controlVector, DMatrixRMaj stateVector, DMatrixRMaj desiredControlVector,
                                      DMatrixRMaj desiredStateVector, DMatrixRMaj constants, DMatrixRMaj matrixToPack)
   {
      matrixToPack.zero();
      tempMatrix.reshape(matrixToPack.getNumRows(), matrixToPack.getNumCols());

      for (int i = 0; i < lqCostFunctions.size(); i++)
      {
         lqCostFunctions.get(i).getCostControlGradient(state, controlVector, stateVector, constants, tempMatrix);
         CommonOps_DDRM.addEquals(matrixToPack, tempMatrix);
      }

      for (int i = 0; i < lqTrackingCostFunctions.size(); i++)
      {
         lqTrackingCostFunctions.get(i).getCostControlGradient(state, controlVector, stateVector, desiredControlVector, desiredStateVector, constants, tempMatrix);
         CommonOps_DDRM.addEquals(matrixToPack, tempMatrix);
      }
   }

   @Override
   public void getCostStateHessian(E state, DMatrixRMaj controlVector, DMatrixRMaj stateVector, DMatrixRMaj constants, DMatrixRMaj matrixToPack)
   {
      matrixToPack.zero();
      tempMatrix.reshape(matrixToPack.getNumRows(), matrixToPack.getNumCols());

      for (int i = 0; i < lqCostFunctions.size(); i++)
      {
         lqCostFunctions.get(i).getCostStateHessian(state, controlVector, stateVector, constants, tempMatrix);
         CommonOps_DDRM.addEquals(matrixToPack, tempMatrix);
      }

      for (int i = 0; i < lqTrackingCostFunctions.size(); i++)
      {
         lqTrackingCostFunctions.get(i).getCostStateHessian(state, controlVector, stateVector, constants, tempMatrix);
         CommonOps_DDRM.addEquals(matrixToPack, tempMatrix);
      }
   }

   @Override
   public void getCostControlHessian(E state, DMatrixRMaj controlVector, DMatrixRMaj stateVector, DMatrixRMaj constants, DMatrixRMaj matrixToPack)
   {
      matrixToPack.zero();
      tempMatrix.reshape(matrixToPack.getNumRows(), matrixToPack.getNumCols());

      for (int i = 0; i < lqCostFunctions.size(); i++)
      {
         lqCostFunctions.get(i).getCostControlHessian(state, controlVector, stateVector, constants, tempMatrix);
         CommonOps_DDRM.addEquals(matrixToPack, tempMatrix);
      }

      for (int i = 0; i < lqTrackingCostFunctions.size(); i++)
      {
         lqTrackingCostFunctions.get(i).getCostControlHessian(state, controlVector, stateVector, constants, tempMatrix);
         CommonOps_DDRM.addEquals(matrixToPack, tempMatrix);
      }
   }

   @Override
   public void getCostStateGradientOfControlGradient(E state, DMatrixRMaj controlVector, DMatrixRMaj stateVector, DMatrixRMaj constants,
                                                     DMatrixRMaj matrixToPack)
   {
      matrixToPack.zero();
      tempMatrix.reshape(matrixToPack.getNumRows(), matrixToPack.getNumCols());

      for (int i = 0; i < lqCostFunctions.size(); i++)
      {
         lqCostFunctions.get(i).getCostStateGradientOfControlGradient(state, controlVector, stateVector, constants, tempMatrix);
         CommonOps_DDRM.addEquals(matrixToPack, tempMatrix);
      }

      for (int i = 0; i < lqTrackingCostFunctions.size(); i++)
      {
         lqTrackingCostFunctions.get(i).getCostStateGradientOfControlGradient(state, controlVector, stateVector, constants, tempMatrix);
         CommonOps_DDRM.addEquals(matrixToPack, tempMatrix);
      }
   }

   @Override
   public void getCostControlGradientOfStateGradient(E state, DMatrixRMaj controlVector, DMatrixRMaj stateVector, DMatrixRMaj constants, DMatrixRMaj matrixToPack)
   {
      matrixToPack.zero();
      tempMatrix.reshape(matrixToPack.getNumRows(), matrixToPack.getNumCols());

      for (int i = 0; i < lqCostFunctions.size(); i++)
      {
         lqCostFunctions.get(i).getCostControlGradientOfStateGradient(state, controlVector, stateVector, constants, tempMatrix);
         CommonOps_DDRM.addEquals(matrixToPack, tempMatrix);
      }

      for (int i = 0; i < lqTrackingCostFunctions.size(); i++)
      {
         lqTrackingCostFunctions.get(i).getCostControlGradientOfStateGradient(state, controlVector, stateVector, constants, tempMatrix);
         CommonOps_DDRM.addEquals(matrixToPack, tempMatrix);
      }
   }
}
