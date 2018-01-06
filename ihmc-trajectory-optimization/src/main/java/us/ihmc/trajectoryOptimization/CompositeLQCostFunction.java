package us.ihmc.trajectoryOptimization;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import java.util.ArrayList;
import java.util.List;

public class CompositeLQCostFunction<E extends Enum> implements LQTrackingCostFunction<E>
{
   private final List<LQCostFunction<E>> lqCostFunctions = new ArrayList<>();
   private final List<LQTrackingCostFunction<E>> lqTrackingCostFunctions = new ArrayList<>();

   private final DenseMatrix64F tempMatrix = new DenseMatrix64F(0, 0);

   public void addLQCostFunction(LQCostFunction<E> costFunction)
   {
      lqCostFunctions.add(costFunction);
   }

   public void addLQTrackingCostFunction(LQTrackingCostFunction<E> costFunction)
   {
      lqTrackingCostFunctions.add(costFunction);
   }

   @Override
   public double getCost(E state, DenseMatrix64F controlVector, DenseMatrix64F stateVector, DenseMatrix64F desiredControlVector, DenseMatrix64F desiredStateVector)
   {
      double cost = 0.0;
      for (int i = 0; i < lqCostFunctions.size(); i++)
         cost += lqCostFunctions.get(i).getCost(state, controlVector, stateVector);
      for (int i = 0; i < lqTrackingCostFunctions.size(); i++)
         cost += lqTrackingCostFunctions.get(i).getCost(state, controlVector, stateVector, desiredControlVector, desiredStateVector);

      return cost;
   }

   @Override
   public void getCostStateGradient(E state, DenseMatrix64F controlVector, DenseMatrix64F stateVector, DenseMatrix64F desiredControlVector,
                             DenseMatrix64F desiredStateVector, DenseMatrix64F matrixToPack)
   {
      matrixToPack.zero();
      tempMatrix.reshape(matrixToPack.getNumRows(), matrixToPack.getNumCols());

      for (int i = 0; i < lqCostFunctions.size(); i++)
      {
         lqCostFunctions.get(i).getCostStateGradient(state, controlVector, stateVector, tempMatrix);
         CommonOps.addEquals(matrixToPack, tempMatrix);
      }

      for (int i = 0; i < lqTrackingCostFunctions.size(); i++)
      {
         lqTrackingCostFunctions.get(i).getCostStateGradient(state, controlVector, stateVector, desiredControlVector, desiredStateVector, tempMatrix);
         CommonOps.addEquals(matrixToPack, tempMatrix);
      }
   }

   @Override
   public void getCostControlGradient(E state, DenseMatrix64F controlVector, DenseMatrix64F stateVector, DenseMatrix64F desiredControlVector,
                               DenseMatrix64F desiredStateVector, DenseMatrix64F matrixToPack)
   {
      matrixToPack.zero();
      tempMatrix.reshape(matrixToPack.getNumRows(), matrixToPack.getNumCols());

      for (int i = 0; i < lqCostFunctions.size(); i++)
      {
         lqCostFunctions.get(i).getCostControlGradient(state, controlVector, stateVector, tempMatrix);
         CommonOps.addEquals(matrixToPack, tempMatrix);
      }

      for (int i = 0; i < lqTrackingCostFunctions.size(); i++)
      {
         lqTrackingCostFunctions.get(i).getCostControlGradient(state, controlVector, stateVector, desiredControlVector, desiredStateVector, tempMatrix);
         CommonOps.addEquals(matrixToPack, tempMatrix);
      }
   }

   @Override
   public void getCostStateHessian(E state, DenseMatrix64F controlVector, DenseMatrix64F stateVector, DenseMatrix64F matrixToPack)
   {
      matrixToPack.zero();
      tempMatrix.reshape(matrixToPack.getNumRows(), matrixToPack.getNumCols());

      for (int i = 0; i < lqCostFunctions.size(); i++)
      {
         lqCostFunctions.get(i).getCostStateHessian(state, controlVector, stateVector, tempMatrix);
         CommonOps.addEquals(matrixToPack, tempMatrix);
      }

      for (int i = 0; i < lqTrackingCostFunctions.size(); i++)
      {
         lqTrackingCostFunctions.get(i).getCostStateHessian(state, controlVector, stateVector, tempMatrix);
         CommonOps.addEquals(matrixToPack, tempMatrix);
      }
   }

   @Override
   public void getCostControlHessian(E state, DenseMatrix64F controlVector, DenseMatrix64F stateVector, DenseMatrix64F matrixToPack)
   {
      matrixToPack.zero();
      tempMatrix.reshape(matrixToPack.getNumRows(), matrixToPack.getNumCols());

      for (int i = 0; i < lqCostFunctions.size(); i++)
      {
         lqCostFunctions.get(i).getCostControlHessian(state, controlVector, stateVector, tempMatrix);
         CommonOps.addEquals(matrixToPack, tempMatrix);
      }

      for (int i = 0; i < lqTrackingCostFunctions.size(); i++)
      {
         lqTrackingCostFunctions.get(i).getCostControlHessian(state, controlVector, stateVector, tempMatrix);
         CommonOps.addEquals(matrixToPack, tempMatrix);
      }
   }

   @Override
   public void getCostStateGradientOfControlGradient(E state, DenseMatrix64F controlVector, DenseMatrix64F stateVector, DenseMatrix64F matrixToPack)
   {
      matrixToPack.zero();
      tempMatrix.reshape(matrixToPack.getNumRows(), matrixToPack.getNumCols());

      for (int i = 0; i < lqCostFunctions.size(); i++)
      {
         lqCostFunctions.get(i).getCostStateGradientOfControlGradient(state, controlVector, stateVector, tempMatrix);
         CommonOps.addEquals(matrixToPack, tempMatrix);
      }

      for (int i = 0; i < lqTrackingCostFunctions.size(); i++)
      {
         lqTrackingCostFunctions.get(i).getCostStateGradientOfControlGradient(state, controlVector, stateVector, tempMatrix);
         CommonOps.addEquals(matrixToPack, tempMatrix);
      }
   }

   @Override
   public void getCostControlGradientOfStateGradient(E state, DenseMatrix64F controlVector, DenseMatrix64F stateVector, DenseMatrix64F matrixToPack)
   {
      matrixToPack.zero();
      tempMatrix.reshape(matrixToPack.getNumRows(), matrixToPack.getNumCols());

      for (int i = 0; i < lqCostFunctions.size(); i++)
      {
         lqCostFunctions.get(i).getCostControlGradientOfStateGradient(state, controlVector, stateVector, tempMatrix);
         CommonOps.addEquals(matrixToPack, tempMatrix);
      }

      for (int i = 0; i < lqTrackingCostFunctions.size(); i++)
      {
         lqTrackingCostFunctions.get(i).getCostControlGradientOfStateGradient(state, controlVector, stateVector, tempMatrix);
         CommonOps.addEquals(matrixToPack, tempMatrix);
      }
   }
}
