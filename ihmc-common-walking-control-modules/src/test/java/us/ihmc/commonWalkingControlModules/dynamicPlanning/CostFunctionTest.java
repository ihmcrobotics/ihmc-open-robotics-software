package us.ihmc.commonWalkingControlModules.dynamicPlanning;

import org.ejml.data.DenseMatrix64F;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.robotics.testing.JUnitTools;
import us.ihmc.trajectoryOptimization.LQCostFunction;

import java.util.Random;

public abstract class CostFunctionTest<E extends Enum>
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
         DenseMatrix64F currentState = RandomGeometry.nextDenseMatrix64F(random, getStateVectorSize(), 1);
         DenseMatrix64F currentControl = RandomGeometry.nextDenseMatrix64F(random, getControlVectorSize(), 1);
         DenseMatrix64F constants = RandomGeometry.nextDenseMatrix64F(random, getConstantVectorSize(), 1);

         DenseMatrix64F costGradient = new DenseMatrix64F(getStateVectorSize(), 1);
         DenseMatrix64F expectedCostGradient = new DenseMatrix64F(getStateVectorSize(), 1);

         double cost = costFunction.getCost(hybridState, currentControl, currentState, constants);

         costFunction.getCostStateGradient(hybridState, currentControl, currentState, constants, costGradient);

         for (int modifiedStateIndex = 0; modifiedStateIndex < getStateVectorSize(); modifiedStateIndex++)
         {
            DenseMatrix64F currentStateModified = new DenseMatrix64F(currentState);
            currentStateModified.add(modifiedStateIndex, 0, epsilon);
            double modifiedCost = costFunction.getCost(hybridState, currentControl, currentStateModified, constants);
            expectedCostGradient.set(modifiedStateIndex, 0, (modifiedCost - cost) / epsilon);
         }

         JUnitTools.assertMatrixEquals(expectedCostGradient, costGradient, 1e-2);
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
         DenseMatrix64F currentState = RandomGeometry.nextDenseMatrix64F(random, getStateVectorSize(), 1);
         DenseMatrix64F currentControl = RandomGeometry.nextDenseMatrix64F(random, getControlVectorSize(), 1);
         DenseMatrix64F constants = RandomGeometry.nextDenseMatrix64F(random, getConstantVectorSize(), 1);

         DenseMatrix64F costGradient = new DenseMatrix64F(getControlVectorSize(), 1);
         DenseMatrix64F expectedCostGradient = new DenseMatrix64F(getControlVectorSize(), 1);

         double cost = costFunction.getCost(hybridState, currentControl, currentState, constants);

         costFunction.getCostControlGradient(hybridState, currentControl, currentState, constants, costGradient);

         for (int controlModifiedIndex = 0; controlModifiedIndex < getControlVectorSize(); controlModifiedIndex++)
         {
            DenseMatrix64F currentControlModified = new DenseMatrix64F(currentControl);
            currentControlModified.add(controlModifiedIndex, 0, epsilon);
            double modifiedCost = costFunction.getCost(hybridState, currentControlModified, currentState, constants);
            expectedCostGradient.set(controlModifiedIndex, 0, (modifiedCost - cost) / epsilon);
         }

         JUnitTools.assertMatrixEquals(expectedCostGradient, costGradient, 1e-2);
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
         DenseMatrix64F currentState = RandomGeometry.nextDenseMatrix64F(random, getStateVectorSize(), 1);
         DenseMatrix64F currentControl = RandomGeometry.nextDenseMatrix64F(random, getControlVectorSize(), 1);
         DenseMatrix64F constants = RandomGeometry.nextDenseMatrix64F(random, getConstantVectorSize(), 1);

         DenseMatrix64F costHessian = new DenseMatrix64F(getStateVectorSize(), getStateVectorSize());
         DenseMatrix64F expectedCostHessian = new DenseMatrix64F(getStateVectorSize(), getStateVectorSize());

         DenseMatrix64F currentGradient = new DenseMatrix64F(getStateVectorSize(), 1);
         DenseMatrix64F modifiedGradient = new DenseMatrix64F(getStateVectorSize(), 1);

         costFunction.getCostStateHessian(hybridState, currentControl, currentState, constants, costHessian);
         costFunction.getCostStateGradient(hybridState, currentControl, currentState, constants, currentGradient);

         for (int partialState = 0; partialState < getStateVectorSize(); partialState++)
         {
            DenseMatrix64F currentStateModified = new DenseMatrix64F(currentState);
            currentStateModified.add(partialState, 0, epsilon);
            costFunction.getCostStateGradient(hybridState, currentControl, currentStateModified, constants, modifiedGradient);

            for (int state = 0; state < getStateVectorSize(); state++)
               expectedCostHessian.set(state, partialState, (modifiedGradient.get(state) - currentGradient.get(state)) / epsilon);
         }

         JUnitTools.assertMatrixEquals(expectedCostHessian, costHessian, 1e-2);
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
         DenseMatrix64F currentState = RandomGeometry.nextDenseMatrix64F(random, getStateVectorSize(), 1);
         DenseMatrix64F currentControl = RandomGeometry.nextDenseMatrix64F(random, getControlVectorSize(), 1);
         DenseMatrix64F constants = RandomGeometry.nextDenseMatrix64F(random, getConstantVectorSize(), 1);

         DenseMatrix64F costHessian = new DenseMatrix64F(getControlVectorSize(), getControlVectorSize());
         DenseMatrix64F expectedCostHessian = new DenseMatrix64F(getControlVectorSize(), getControlVectorSize());

         DenseMatrix64F currentGradient = new DenseMatrix64F(getControlVectorSize(), 1);
         DenseMatrix64F modifiedGradient = new DenseMatrix64F(getControlVectorSize(), 1);

         costFunction.getCostControlHessian(hybridState, currentControl, currentState, constants, costHessian);
         costFunction.getCostControlGradient(hybridState, currentControl, currentState, constants, currentGradient);

         for (int partialControl = 0; partialControl < getControlVectorSize(); partialControl++)
         {
            DenseMatrix64F currentControlModified = new DenseMatrix64F(currentControl);
            currentControlModified.add(partialControl, 0, epsilon);
            costFunction.getCostControlGradient(hybridState, currentControlModified, currentState, constants, modifiedGradient);

            for (int control = 0; control < getControlVectorSize(); control++)
               expectedCostHessian.set(control, partialControl, (modifiedGradient.get(control) - currentGradient.get(control)) / epsilon);
         }

         JUnitTools.assertMatrixEquals(expectedCostHessian, costHessian, 1e-2);
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
         DenseMatrix64F currentState = RandomGeometry.nextDenseMatrix64F(random, getStateVectorSize(), 1);
         DenseMatrix64F currentControl = RandomGeometry.nextDenseMatrix64F(random, getControlVectorSize(), 1);
         DenseMatrix64F constants = RandomGeometry.nextDenseMatrix64F(random, getConstantVectorSize(), 1);

         DenseMatrix64F costHessian = new DenseMatrix64F(getStateVectorSize(), getControlVectorSize());
         DenseMatrix64F expectedCostHessian = new DenseMatrix64F(getStateVectorSize(), getControlVectorSize());

         DenseMatrix64F currentGradient = new DenseMatrix64F(getStateVectorSize(), 1);
         DenseMatrix64F modifiedGradient = new DenseMatrix64F(getStateVectorSize(), 1);

         costFunction.getCostControlGradientOfStateGradient(hybridState, currentControl, currentState, constants, costHessian);
         costFunction.getCostStateGradient(hybridState, currentControl, currentState, constants, currentGradient);

         for (int partialControl = 0; partialControl < getControlVectorSize(); partialControl++)
         {
            DenseMatrix64F currentControlModified = new DenseMatrix64F(currentControl);
            currentControlModified.add(partialControl, 0, epsilon);
            costFunction.getCostStateGradient(hybridState, currentControlModified, currentState, constants, modifiedGradient);

            for (int state = 0; state < getStateVectorSize(); state++)
               expectedCostHessian.set(state, partialControl, (modifiedGradient.get(state) - currentGradient.get(state)) / epsilon);
         }

         JUnitTools.assertMatrixEquals(expectedCostHessian, costHessian, 1e-2);
      }
   }
}
