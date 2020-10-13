package us.ihmc.commonWalkingControlModules.dynamicPlanning;

import java.util.Random;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import us.ihmc.matrixlib.MatrixTestTools;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.trajectoryOptimization.LQTrackingCostFunction;

public abstract class TrackingCostFunctionTest<E extends Enum<E>>
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
         DMatrixRMaj currentState = RandomGeometry.nextDenseMatrix64F(random, getStateVectorSize(), 1);
         DMatrixRMaj currentControl = RandomGeometry.nextDenseMatrix64F(random, getControlVectorSize(), 1);
         DMatrixRMaj desiredState = RandomGeometry.nextDenseMatrix64F(random, getStateVectorSize(), 1);
         DMatrixRMaj desiredControl = RandomGeometry.nextDenseMatrix64F(random, getControlVectorSize(), 1);
         DMatrixRMaj constant = RandomGeometry.nextDenseMatrix64F(random, getConstantVectorSize(), 1);

         DMatrixRMaj costGradient = new DMatrixRMaj(getStateVectorSize(), 1);
         DMatrixRMaj expectedCostGradient = new DMatrixRMaj(getStateVectorSize(), 1);

         double cost = costFunction.getCost(hybridState, currentControl, currentState, desiredControl, desiredState, constant);
         int length = (int)(Math.log10(cost) + 1);
         double scale = 1.0 * Math.pow(10, length);
         double modifiedEpsilon = Math.max(scale * epsilon, 1e-7);

         costFunction.getCostStateGradient(hybridState, currentControl, currentState, desiredControl, desiredState, constant, costGradient);

         for (int modifiedStateIndex = 0; modifiedStateIndex < getStateVectorSize(); modifiedStateIndex++)
         {
            DMatrixRMaj currentStateModified = new DMatrixRMaj(currentState);
            currentStateModified.add(modifiedStateIndex, 0, modifiedEpsilon);
            double modifiedCost = costFunction.getCost(hybridState, currentControl, currentStateModified, desiredControl, desiredState, constant);
            expectedCostGradient.set(modifiedStateIndex, 0, (modifiedCost - cost) / modifiedEpsilon);
         }

         double value = Math.abs(CommonOps_DDRM.elementSum(expectedCostGradient));
         MatrixTestTools.assertMatrixEquals(expectedCostGradient, costGradient, 1e-3 * value);
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
         DMatrixRMaj currentState = RandomGeometry.nextDenseMatrix64F(random, getStateVectorSize(), 1);
         DMatrixRMaj currentControl = RandomGeometry.nextDenseMatrix64F(random, getControlVectorSize(), 1);
         DMatrixRMaj desiredState = RandomGeometry.nextDenseMatrix64F(random, getStateVectorSize(), 1);
         DMatrixRMaj desiredControl = RandomGeometry.nextDenseMatrix64F(random, getControlVectorSize(), 1);
         DMatrixRMaj constant = RandomGeometry.nextDenseMatrix64F(random, getConstantVectorSize(), 1);

         DMatrixRMaj costGradient = new DMatrixRMaj(getControlVectorSize(), 1);
         DMatrixRMaj expectedCostGradient = new DMatrixRMaj(getControlVectorSize(), 1);

         double cost = costFunction.getCost(hybridState, currentControl, currentState, desiredControl, desiredState, constant);
         int length = (int)(Math.log10(cost) + 1);
         double scale = 1.0 * Math.pow(10, length);
         //double scale = 1.0;
         double modifiedEpsilon = Math.max(scale * epsilon, 1e-9);

         costFunction.getCostControlGradient(hybridState, currentControl, currentState, desiredControl, desiredState, constant, costGradient);

         for (int controlModifiedIndex = 0; controlModifiedIndex < getControlVectorSize(); controlModifiedIndex++)
         {
            DMatrixRMaj currentControlModified = new DMatrixRMaj(currentControl);
            currentControlModified.add(controlModifiedIndex, 0, modifiedEpsilon);
            double modifiedCost = costFunction.getCost(hybridState, currentControlModified, currentState, desiredControl, desiredState, constant);
            expectedCostGradient.set(controlModifiedIndex, 0, (modifiedCost - cost) / (modifiedEpsilon));
         }

         double value = Math.max(1.0e-10, 1e-3 * Math.abs(CommonOps_DDRM.elementSum(expectedCostGradient)));
         MatrixTestTools.assertMatrixEquals(expectedCostGradient, costGradient, value);
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
         DMatrixRMaj currentState = RandomGeometry.nextDenseMatrix64F(random, getStateVectorSize(), 1);
         DMatrixRMaj currentControl = RandomGeometry.nextDenseMatrix64F(random, getControlVectorSize(), 1);
         DMatrixRMaj desiredState = RandomGeometry.nextDenseMatrix64F(random, getStateVectorSize(), 1);
         DMatrixRMaj desiredControl = RandomGeometry.nextDenseMatrix64F(random, getControlVectorSize(), 1);
         DMatrixRMaj constant = RandomGeometry.nextDenseMatrix64F(random, getConstantVectorSize(), 1);

         DMatrixRMaj costHessian = new DMatrixRMaj(getStateVectorSize(), getStateVectorSize());
         DMatrixRMaj expectedCostHessian = new DMatrixRMaj(getStateVectorSize(), getStateVectorSize());

         DMatrixRMaj currentGradient = new DMatrixRMaj(getStateVectorSize(), 1);
         DMatrixRMaj modifiedGradient = new DMatrixRMaj(getStateVectorSize(), 1);

         costFunction.getCostStateHessian(hybridState, currentControl, currentState, constant, costHessian);
         costFunction.getCostStateGradient(hybridState, currentControl, currentState, desiredControl, desiredState, constant, currentGradient);

         for (int partialState = 0; partialState < getStateVectorSize(); partialState++)
         {
            DMatrixRMaj currentStateModified = new DMatrixRMaj(currentState);
            currentStateModified.add(partialState, 0, epsilon);
            costFunction.getCostStateGradient(hybridState, currentControl, currentStateModified, desiredControl, desiredState, constant, modifiedGradient);

            for (int state = 0; state < getStateVectorSize(); state++)
               expectedCostHessian.set(state, partialState, (modifiedGradient.get(state) - currentGradient.get(state)) / epsilon);
         }

         double trace = Math.abs(CommonOps_DDRM.trace(expectedCostHessian));
         MatrixTestTools.assertMatrixEquals(expectedCostHessian, costHessian, 1e-5 * trace);
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
         DMatrixRMaj currentState = RandomGeometry.nextDenseMatrix64F(random, getStateVectorSize(), 1);
         DMatrixRMaj currentControl = RandomGeometry.nextDenseMatrix64F(random, getControlVectorSize(), 1);
         DMatrixRMaj desiredState = RandomGeometry.nextDenseMatrix64F(random, getStateVectorSize(), 1);
         DMatrixRMaj desiredControl = RandomGeometry.nextDenseMatrix64F(random, getControlVectorSize(), 1);
         DMatrixRMaj constant = RandomGeometry.nextDenseMatrix64F(random, getConstantVectorSize(), 1);

         DMatrixRMaj costHessian = new DMatrixRMaj(getControlVectorSize(), getControlVectorSize());
         DMatrixRMaj expectedCostHessian = new DMatrixRMaj(getControlVectorSize(), getControlVectorSize());

         DMatrixRMaj currentGradient = new DMatrixRMaj(getControlVectorSize(), 1);
         DMatrixRMaj modifiedGradient = new DMatrixRMaj(getControlVectorSize(), 1);


         costFunction.getCostControlHessian(hybridState, currentControl, currentState, constant, costHessian);
         costFunction.getCostControlGradient(hybridState, currentControl, currentState, desiredControl, desiredState, constant, currentGradient);


         for (int partialControl = 0; partialControl < getControlVectorSize(); partialControl++)
         {
            DMatrixRMaj currentControlModified = new DMatrixRMaj(currentControl);
            currentControlModified.add(partialControl, 0, epsilon);
            costFunction.getCostControlGradient(hybridState, currentControlModified, currentState, desiredControl, desiredState, constant, modifiedGradient);

            for (int control = 0; control < getControlVectorSize(); control++)
               expectedCostHessian.set(control, partialControl, (modifiedGradient.get(control) - currentGradient.get(control)) / epsilon);
         }

         double trace = Math.abs(CommonOps_DDRM.trace(expectedCostHessian));
         MatrixTestTools.assertMatrixEquals(expectedCostHessian, costHessian, 1e-5 * trace);
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
         DMatrixRMaj currentState = RandomGeometry.nextDenseMatrix64F(random, getStateVectorSize(), 1);
         DMatrixRMaj currentControl = RandomGeometry.nextDenseMatrix64F(random, getControlVectorSize(), 1);
         DMatrixRMaj desiredState = RandomGeometry.nextDenseMatrix64F(random, getStateVectorSize(), 1);
         DMatrixRMaj desiredControl = RandomGeometry.nextDenseMatrix64F(random, getControlVectorSize(), 1);
         DMatrixRMaj constant = RandomGeometry.nextDenseMatrix64F(random, getConstantVectorSize(), 1);

         DMatrixRMaj costHessian = new DMatrixRMaj(getStateVectorSize(), getControlVectorSize());
         DMatrixRMaj expectedCostHessian = new DMatrixRMaj(getStateVectorSize(), getControlVectorSize());

         DMatrixRMaj currentGradient = new DMatrixRMaj(getStateVectorSize(), 1);
         DMatrixRMaj modifiedGradient = new DMatrixRMaj(getStateVectorSize(), 1);

         costFunction.getCostControlGradientOfStateGradient(hybridState, currentControl, currentState, constant, costHessian);
         costFunction.getCostStateGradient(hybridState, currentControl, currentState, desiredControl, desiredState, constant, currentGradient);

         for (int partialControl = 0; partialControl < getControlVectorSize(); partialControl++)
         {
            DMatrixRMaj currentControlModified = new DMatrixRMaj(currentControl);
            currentControlModified.add(partialControl, 0, epsilon);
            costFunction.getCostStateGradient(hybridState, currentControlModified, currentState, desiredControl, desiredState, constant, modifiedGradient);

            for (int state = 0; state < getStateVectorSize(); state++)
               expectedCostHessian.set(state, partialControl, (modifiedGradient.get(state) - currentGradient.get(state)) / epsilon);
         }

         double trace = Math.abs(CommonOps_DDRM.trace(expectedCostHessian));
         MatrixTestTools.assertMatrixEquals(expectedCostHessian, costHessian, 1e-5 * trace);
      }
   }
}
