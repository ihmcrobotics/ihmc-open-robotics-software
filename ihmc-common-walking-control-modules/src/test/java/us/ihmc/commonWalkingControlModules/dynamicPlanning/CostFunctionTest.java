package us.ihmc.commonWalkingControlModules.dynamicPlanning;

import java.util.Random;

import org.ejml.data.DMatrixRMaj;

import us.ihmc.matrixlib.MatrixTestTools;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.trajectoryOptimization.LQCostFunction;

public abstract class CostFunctionTest<E extends Enum<E>>
{
   public abstract int getNumberOfStates();
   public abstract int getStateVectorSize();
   public abstract int getControlVectorSize();
   public abstract int getConstantVectorSize();
   public abstract E getHybridState(int hybridStateIndex);
   public abstract LQCostFunction<E> getCostFunction();

   public abstract void testCost();

   public void testCostStateGradientNumerically()
   {
      double epsilon = 1e-9;
      LQCostFunction<E> costFunction = getCostFunction();

      for (int hybridStateIndex = 0; hybridStateIndex < getNumberOfStates(); hybridStateIndex++)
      {
         E hybridState = getHybridState(hybridStateIndex);

         Random random = new Random(1738L);
         DMatrixRMaj currentState = RandomGeometry.nextDenseMatrix64F(random, getStateVectorSize(), 1);
         DMatrixRMaj currentControl = RandomGeometry.nextDenseMatrix64F(random, getControlVectorSize(), 1);
         DMatrixRMaj constants = RandomGeometry.nextDenseMatrix64F(random, getConstantVectorSize(), 1);

         DMatrixRMaj costGradient = new DMatrixRMaj(getStateVectorSize(), 1);
         DMatrixRMaj expectedCostGradient = new DMatrixRMaj(getStateVectorSize(), 1);

         double cost = costFunction.getCost(hybridState, currentControl, currentState, constants);

         costFunction.getCostStateGradient(hybridState, currentControl, currentState, constants, costGradient);

         for (int modifiedStateIndex = 0; modifiedStateIndex < getStateVectorSize(); modifiedStateIndex++)
         {
            DMatrixRMaj currentStateModified = new DMatrixRMaj(currentState);
            currentStateModified.add(modifiedStateIndex, 0, epsilon);
            double modifiedCost = costFunction.getCost(hybridState, currentControl, currentStateModified, constants);
            expectedCostGradient.set(modifiedStateIndex, 0, (modifiedCost - cost) / epsilon);
         }

         MatrixTestTools.assertMatrixEquals(expectedCostGradient, costGradient, 1e-2);
      }
   }

   public void testCostControlGradientNumerically()
   {
      double epsilon = 1e-9;
      LQCostFunction<E> costFunction = getCostFunction();

      for (int hybridStateIndex = 0; hybridStateIndex < getNumberOfStates(); hybridStateIndex++)
      {
         E hybridState = getHybridState(hybridStateIndex);

         Random random = new Random(1738L);
         DMatrixRMaj currentState = RandomGeometry.nextDenseMatrix64F(random, getStateVectorSize(), 1);
         DMatrixRMaj currentControl = RandomGeometry.nextDenseMatrix64F(random, getControlVectorSize(), 1);
         DMatrixRMaj constants = RandomGeometry.nextDenseMatrix64F(random, getConstantVectorSize(), 1);

         DMatrixRMaj costGradient = new DMatrixRMaj(getControlVectorSize(), 1);
         DMatrixRMaj expectedCostGradient = new DMatrixRMaj(getControlVectorSize(), 1);

         double cost = costFunction.getCost(hybridState, currentControl, currentState, constants);

         costFunction.getCostControlGradient(hybridState, currentControl, currentState, constants, costGradient);

         for (int controlModifiedIndex = 0; controlModifiedIndex < getControlVectorSize(); controlModifiedIndex++)
         {
            DMatrixRMaj currentControlModified = new DMatrixRMaj(currentControl);
            currentControlModified.add(controlModifiedIndex, 0, epsilon);
            double modifiedCost = costFunction.getCost(hybridState, currentControlModified, currentState, constants);
            expectedCostGradient.set(controlModifiedIndex, 0, (modifiedCost - cost) / epsilon);
         }

         MatrixTestTools.assertMatrixEquals(expectedCostGradient, costGradient, 1e-2);
      }
   }

   public void testCostStateHessianNumerically()
   {
      double epsilon = 1e-9;
      LQCostFunction<E> costFunction = getCostFunction();

      for (int hybridStateIndex = 0; hybridStateIndex < getNumberOfStates(); hybridStateIndex++)
      {
         E hybridState = getHybridState(hybridStateIndex);

         Random random = new Random(1738L);
         DMatrixRMaj currentState = RandomGeometry.nextDenseMatrix64F(random, getStateVectorSize(), 1);
         DMatrixRMaj currentControl = RandomGeometry.nextDenseMatrix64F(random, getControlVectorSize(), 1);
         DMatrixRMaj constants = RandomGeometry.nextDenseMatrix64F(random, getConstantVectorSize(), 1);

         DMatrixRMaj costHessian = new DMatrixRMaj(getStateVectorSize(), getStateVectorSize());
         DMatrixRMaj expectedCostHessian = new DMatrixRMaj(getStateVectorSize(), getStateVectorSize());

         DMatrixRMaj currentGradient = new DMatrixRMaj(getStateVectorSize(), 1);
         DMatrixRMaj modifiedGradient = new DMatrixRMaj(getStateVectorSize(), 1);

         costFunction.getCostStateHessian(hybridState, currentControl, currentState, constants, costHessian);
         costFunction.getCostStateGradient(hybridState, currentControl, currentState, constants, currentGradient);

         for (int partialState = 0; partialState < getStateVectorSize(); partialState++)
         {
            DMatrixRMaj currentStateModified = new DMatrixRMaj(currentState);
            currentStateModified.add(partialState, 0, epsilon);
            costFunction.getCostStateGradient(hybridState, currentControl, currentStateModified, constants, modifiedGradient);

            for (int state = 0; state < getStateVectorSize(); state++)
               expectedCostHessian.set(state, partialState, (modifiedGradient.get(state) - currentGradient.get(state)) / epsilon);
         }

         MatrixTestTools.assertMatrixEquals(expectedCostHessian, costHessian, 1e-2);
      }
   }

   public void testCostControlHessianNumerically()
   {
      double epsilon = 1e-9;
      LQCostFunction<E> costFunction = getCostFunction();

      for (int hybridStateIndex = 0; hybridStateIndex < getNumberOfStates(); hybridStateIndex++)
      {
         E hybridState = getHybridState(hybridStateIndex);

         Random random = new Random(1738L);
         DMatrixRMaj currentState = RandomGeometry.nextDenseMatrix64F(random, getStateVectorSize(), 1);
         DMatrixRMaj currentControl = RandomGeometry.nextDenseMatrix64F(random, getControlVectorSize(), 1);
         DMatrixRMaj constants = RandomGeometry.nextDenseMatrix64F(random, getConstantVectorSize(), 1);

         DMatrixRMaj costHessian = new DMatrixRMaj(getControlVectorSize(), getControlVectorSize());
         DMatrixRMaj expectedCostHessian = new DMatrixRMaj(getControlVectorSize(), getControlVectorSize());

         DMatrixRMaj currentGradient = new DMatrixRMaj(getControlVectorSize(), 1);
         DMatrixRMaj modifiedGradient = new DMatrixRMaj(getControlVectorSize(), 1);

         costFunction.getCostControlHessian(hybridState, currentControl, currentState, constants, costHessian);
         costFunction.getCostControlGradient(hybridState, currentControl, currentState, constants, currentGradient);

         for (int partialControl = 0; partialControl < getControlVectorSize(); partialControl++)
         {
            DMatrixRMaj currentControlModified = new DMatrixRMaj(currentControl);
            currentControlModified.add(partialControl, 0, epsilon);
            costFunction.getCostControlGradient(hybridState, currentControlModified, currentState, constants, modifiedGradient);

            for (int control = 0; control < getControlVectorSize(); control++)
               expectedCostHessian.set(control, partialControl, (modifiedGradient.get(control) - currentGradient.get(control)) / epsilon);
         }

         MatrixTestTools.assertMatrixEquals(expectedCostHessian, costHessian, 1e-2);
      }
   }

   public void testCostStateControlHessianNumerically()
   {
      double epsilon = 1e-9;
      LQCostFunction<E> costFunction = getCostFunction();

      for (int hybridStateIndex = 0; hybridStateIndex < getNumberOfStates(); hybridStateIndex++)
      {
         E hybridState = getHybridState(hybridStateIndex);

         Random random = new Random(1738L);
         DMatrixRMaj currentState = RandomGeometry.nextDenseMatrix64F(random, getStateVectorSize(), 1);
         DMatrixRMaj currentControl = RandomGeometry.nextDenseMatrix64F(random, getControlVectorSize(), 1);
         DMatrixRMaj constants = RandomGeometry.nextDenseMatrix64F(random, getConstantVectorSize(), 1);

         DMatrixRMaj costHessian = new DMatrixRMaj(getStateVectorSize(), getControlVectorSize());
         DMatrixRMaj expectedCostHessian = new DMatrixRMaj(getStateVectorSize(), getControlVectorSize());

         DMatrixRMaj currentGradient = new DMatrixRMaj(getStateVectorSize(), 1);
         DMatrixRMaj modifiedGradient = new DMatrixRMaj(getStateVectorSize(), 1);

         costFunction.getCostControlGradientOfStateGradient(hybridState, currentControl, currentState, constants, costHessian);
         costFunction.getCostStateGradient(hybridState, currentControl, currentState, constants, currentGradient);

         for (int partialControl = 0; partialControl < getControlVectorSize(); partialControl++)
         {
            DMatrixRMaj currentControlModified = new DMatrixRMaj(currentControl);
            currentControlModified.add(partialControl, 0, epsilon);
            costFunction.getCostStateGradient(hybridState, currentControlModified, currentState, constants, modifiedGradient);

            for (int state = 0; state < getStateVectorSize(); state++)
               expectedCostHessian.set(state, partialControl, (modifiedGradient.get(state) - currentGradient.get(state)) / epsilon);
         }

         MatrixTestTools.assertMatrixEquals(expectedCostHessian, costHessian, 1e-2);
      }
   }
}
