package us.ihmc.commonWalkingControlModules.dynamicPlanning;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.robotics.testing.JUnitTools;
import us.ihmc.trajectoryOptimization.LQTrackingCostFunction;

import java.util.Random;

public abstract class TrackingCostFunctionTest<E extends Enum>
{
   public abstract int getNumberOfStates();
   public abstract int getStateVectorSize();
   public abstract int getControlVectorSize();
   public abstract int getConstantVectorSize();
   public abstract E getHybridState(int hybridStateIndex);
   public abstract LQTrackingCostFunction<E> getCostFunction();

   public abstract void testCost();

   public void testCostStateGradientNumerically()
   {
      double epsilon = 1e-18;
      LQTrackingCostFunction<E> costFunction = getCostFunction();

      for (int hybridStateIndex = 0; hybridStateIndex < getNumberOfStates(); hybridStateIndex++)
      {
         E hybridState = getHybridState(hybridStateIndex);

         Random random = new Random(1738L);
         DenseMatrix64F currentState = RandomGeometry.nextDenseMatrix64F(random, getStateVectorSize(), 1);
         DenseMatrix64F currentControl = RandomGeometry.nextDenseMatrix64F(random, getControlVectorSize(), 1);
         DenseMatrix64F desiredState = RandomGeometry.nextDenseMatrix64F(random, getStateVectorSize(), 1);
         DenseMatrix64F desiredControl = RandomGeometry.nextDenseMatrix64F(random, getControlVectorSize(), 1);
         DenseMatrix64F constant = RandomGeometry.nextDenseMatrix64F(random, getConstantVectorSize(), 1);

         DenseMatrix64F costGradient = new DenseMatrix64F(getStateVectorSize(), 1);
         DenseMatrix64F expectedCostGradient = new DenseMatrix64F(getStateVectorSize(), 1);

         double cost = costFunction.getCost(hybridState, currentControl, currentState, desiredControl, desiredState, constant);
         int length = (int)(Math.log10(cost) + 1);
         double scale = 1.0 * Math.pow(10, length);
         double modifiedEpsilon = Math.max(scale * epsilon, 1e-7);

         costFunction.getCostStateGradient(hybridState, currentControl, currentState, desiredControl, desiredState, constant, costGradient);

         for (int modifiedStateIndex = 0; modifiedStateIndex < getStateVectorSize(); modifiedStateIndex++)
         {
            DenseMatrix64F currentStateModified = new DenseMatrix64F(currentState);
            currentStateModified.add(modifiedStateIndex, 0, modifiedEpsilon);
            double modifiedCost = costFunction.getCost(hybridState, currentControl, currentStateModified, desiredControl, desiredState, constant);
            expectedCostGradient.set(modifiedStateIndex, 0, (modifiedCost - cost) / modifiedEpsilon);
         }

         double value = Math.abs(CommonOps.elementSum(expectedCostGradient));
         JUnitTools.assertMatrixEquals(expectedCostGradient, costGradient, 1e-3 * value);
      }
   }

   public void testCostControlGradientNumerically()
   {
      double epsilon = 1e-18;
      LQTrackingCostFunction<E> costFunction = getCostFunction();

      for (int hybridStateIndex = 0; hybridStateIndex < getNumberOfStates(); hybridStateIndex++)
      {
         E hybridState = getHybridState(hybridStateIndex);

         Random random = new Random(1738L);
         DenseMatrix64F currentState = RandomGeometry.nextDenseMatrix64F(random, getStateVectorSize(), 1);
         DenseMatrix64F currentControl = RandomGeometry.nextDenseMatrix64F(random, getControlVectorSize(), 1);
         DenseMatrix64F desiredState = RandomGeometry.nextDenseMatrix64F(random, getStateVectorSize(), 1);
         DenseMatrix64F desiredControl = RandomGeometry.nextDenseMatrix64F(random, getControlVectorSize(), 1);
         DenseMatrix64F constant = RandomGeometry.nextDenseMatrix64F(random, getConstantVectorSize(), 1);

         DenseMatrix64F costGradient = new DenseMatrix64F(getControlVectorSize(), 1);
         DenseMatrix64F expectedCostGradient = new DenseMatrix64F(getControlVectorSize(), 1);

         double cost = costFunction.getCost(hybridState, currentControl, currentState, desiredControl, desiredState, constant);
         int length = (int)(Math.log10(cost) + 1);
         double scale = 1.0 * Math.pow(10, length);
         //double scale = 1.0;
         double modifiedEpsilon = Math.max(scale * epsilon, 1e-9);

         costFunction.getCostControlGradient(hybridState, currentControl, currentState, desiredControl, desiredState, constant, costGradient);

         for (int controlModifiedIndex = 0; controlModifiedIndex < getControlVectorSize(); controlModifiedIndex++)
         {
            DenseMatrix64F currentControlModified = new DenseMatrix64F(currentControl);
            currentControlModified.add(controlModifiedIndex, 0, modifiedEpsilon);
            double modifiedCost = costFunction.getCost(hybridState, currentControlModified, currentState, desiredControl, desiredState, constant);
            expectedCostGradient.set(controlModifiedIndex, 0, (modifiedCost - cost) / (modifiedEpsilon));
         }

         double value = Math.abs(CommonOps.elementSum(expectedCostGradient));
         JUnitTools.assertMatrixEquals(expectedCostGradient, costGradient, 1e-3 * value);
      }
   }

   public void testCostStateHessianNumerically()
   {
      double epsilon = 1e-9;
      LQTrackingCostFunction<E> costFunction = getCostFunction();

      for (int hybridStateIndex = 0; hybridStateIndex < getNumberOfStates(); hybridStateIndex++)
      {
         E hybridState = getHybridState(hybridStateIndex);

         Random random = new Random(1738L);
         DenseMatrix64F currentState = RandomGeometry.nextDenseMatrix64F(random, getStateVectorSize(), 1);
         DenseMatrix64F currentControl = RandomGeometry.nextDenseMatrix64F(random, getControlVectorSize(), 1);
         DenseMatrix64F desiredState = RandomGeometry.nextDenseMatrix64F(random, getStateVectorSize(), 1);
         DenseMatrix64F desiredControl = RandomGeometry.nextDenseMatrix64F(random, getControlVectorSize(), 1);
         DenseMatrix64F constant = RandomGeometry.nextDenseMatrix64F(random, getConstantVectorSize(), 1);

         DenseMatrix64F costHessian = new DenseMatrix64F(getStateVectorSize(), getStateVectorSize());
         DenseMatrix64F expectedCostHessian = new DenseMatrix64F(getStateVectorSize(), getStateVectorSize());

         DenseMatrix64F currentGradient = new DenseMatrix64F(getStateVectorSize(), 1);
         DenseMatrix64F modifiedGradient = new DenseMatrix64F(getStateVectorSize(), 1);

         costFunction.getCostStateHessian(hybridState, currentControl, currentState, constant, costHessian);
         costFunction.getCostStateGradient(hybridState, currentControl, currentState, desiredControl, desiredState, constant, currentGradient);

         for (int partialState = 0; partialState < getStateVectorSize(); partialState++)
         {
            DenseMatrix64F currentStateModified = new DenseMatrix64F(currentState);
            currentStateModified.add(partialState, 0, epsilon);
            costFunction.getCostStateGradient(hybridState, currentControl, currentStateModified, desiredControl, desiredState, constant, modifiedGradient);

            for (int state = 0; state < getStateVectorSize(); state++)
               expectedCostHessian.set(state, partialState, (modifiedGradient.get(state) - currentGradient.get(state)) / epsilon);
         }

         double trace = Math.abs(CommonOps.trace(expectedCostHessian));
         JUnitTools.assertMatrixEquals(expectedCostHessian, costHessian, 1e-5 * trace);
      }
   }

   public void testCostControlHessianNumerically()
   {
      double epsilon = 1e-9;
      LQTrackingCostFunction<E> costFunction = getCostFunction();

      for (int hybridStateIndex = 0; hybridStateIndex < getNumberOfStates(); hybridStateIndex++)
      {
         E hybridState = getHybridState(hybridStateIndex);

         Random random = new Random(1738L);
         DenseMatrix64F currentState = RandomGeometry.nextDenseMatrix64F(random, getStateVectorSize(), 1);
         DenseMatrix64F currentControl = RandomGeometry.nextDenseMatrix64F(random, getControlVectorSize(), 1);
         DenseMatrix64F desiredState = RandomGeometry.nextDenseMatrix64F(random, getStateVectorSize(), 1);
         DenseMatrix64F desiredControl = RandomGeometry.nextDenseMatrix64F(random, getControlVectorSize(), 1);
         DenseMatrix64F constant = RandomGeometry.nextDenseMatrix64F(random, getConstantVectorSize(), 1);

         DenseMatrix64F costHessian = new DenseMatrix64F(getControlVectorSize(), getControlVectorSize());
         DenseMatrix64F expectedCostHessian = new DenseMatrix64F(getControlVectorSize(), getControlVectorSize());

         DenseMatrix64F currentGradient = new DenseMatrix64F(getControlVectorSize(), 1);
         DenseMatrix64F modifiedGradient = new DenseMatrix64F(getControlVectorSize(), 1);


         costFunction.getCostControlHessian(hybridState, currentControl, currentState, constant, costHessian);
         costFunction.getCostControlGradient(hybridState, currentControl, currentState, desiredControl, desiredState, constant, currentGradient);


         for (int partialControl = 0; partialControl < getControlVectorSize(); partialControl++)
         {
            DenseMatrix64F currentControlModified = new DenseMatrix64F(currentControl);
            currentControlModified.add(partialControl, 0, epsilon);
            costFunction.getCostControlGradient(hybridState, currentControlModified, currentState, desiredControl, desiredState, constant, modifiedGradient);

            for (int control = 0; control < getControlVectorSize(); control++)
               expectedCostHessian.set(control, partialControl, (modifiedGradient.get(control) - currentGradient.get(control)) / epsilon);
         }

         double trace = Math.abs(CommonOps.trace(expectedCostHessian));
         JUnitTools.assertMatrixEquals(expectedCostHessian, costHessian, 1e-5 * trace);
      }
   }

   public void testCostStateControlHessianNumerically()
   {
      double epsilon = 1e-9;
      LQTrackingCostFunction<E> costFunction = getCostFunction();

      for (int hybridStateIndex = 0; hybridStateIndex < getNumberOfStates(); hybridStateIndex++)
      {
         E hybridState = getHybridState(hybridStateIndex);

         Random random = new Random(1738L);
         DenseMatrix64F currentState = RandomGeometry.nextDenseMatrix64F(random, getStateVectorSize(), 1);
         DenseMatrix64F currentControl = RandomGeometry.nextDenseMatrix64F(random, getControlVectorSize(), 1);
         DenseMatrix64F desiredState = RandomGeometry.nextDenseMatrix64F(random, getStateVectorSize(), 1);
         DenseMatrix64F desiredControl = RandomGeometry.nextDenseMatrix64F(random, getControlVectorSize(), 1);
         DenseMatrix64F constant = RandomGeometry.nextDenseMatrix64F(random, getConstantVectorSize(), 1);

         DenseMatrix64F costHessian = new DenseMatrix64F(getStateVectorSize(), getControlVectorSize());
         DenseMatrix64F expectedCostHessian = new DenseMatrix64F(getStateVectorSize(), getControlVectorSize());

         DenseMatrix64F currentGradient = new DenseMatrix64F(getStateVectorSize(), 1);
         DenseMatrix64F modifiedGradient = new DenseMatrix64F(getStateVectorSize(), 1);

         costFunction.getCostControlGradientOfStateGradient(hybridState, currentControl, currentState, constant, costHessian);
         costFunction.getCostStateGradient(hybridState, currentControl, currentState, desiredControl, desiredState, constant, currentGradient);

         for (int partialControl = 0; partialControl < getControlVectorSize(); partialControl++)
         {
            DenseMatrix64F currentControlModified = new DenseMatrix64F(currentControl);
            currentControlModified.add(partialControl, 0, epsilon);
            costFunction.getCostStateGradient(hybridState, currentControlModified, currentState, desiredControl, desiredState, constant, modifiedGradient);

            for (int state = 0; state < getStateVectorSize(); state++)
               expectedCostHessian.set(state, partialControl, (modifiedGradient.get(state) - currentGradient.get(state)) / epsilon);
         }

         double trace = Math.abs(CommonOps.trace(expectedCostHessian));
         JUnitTools.assertMatrixEquals(expectedCostHessian, costHessian, 1e-5 * trace);
      }
   }
}
