package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import org.ejml.EjmlUnitTests;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.CubicTrackingCommand;

import static us.ihmc.robotics.Assert.assertEquals;

public class CubicTrackingCostCalculatorTest
{
   @Test
   public void testCalculation()
   {
      double startValue = -5.0;
      double startRate = 11.0;
      double endValue = 7.0;
      double endRate = -9.0;

      double duration = 0.7;

      CubicTrackingCommand command = new CubicTrackingCommand();
      command.setStartValue(startValue);
      command.setFinalValue(endValue);
      command.setStartRate(startRate);
      command.setFinalRate(endRate);
      command.setDuration(duration);
      command.setStartIndex(0);
      command.setWeight(100.0);

      DMatrixRMaj hessian = new DMatrixRMaj(4, 4);
      DMatrixRMaj gradient = new DMatrixRMaj(4, 1);

      CubicTrackingCostCalculator.calculateTrackingObjective(hessian, gradient, command);

      DMatrixRMaj hessianExpected = new DMatrixRMaj(4, 4);
      hessianExpected.set(0, 0, Math.pow(duration, 7) / 7.0);
      hessianExpected.set(0, 1, Math.pow(duration, 6) / 6.0);
      hessianExpected.set(1, 0, Math.pow(duration, 6) / 6.0);
      hessianExpected.set(0, 2, Math.pow(duration, 5) / 5.0);
      hessianExpected.set(1, 1, Math.pow(duration, 5) / 5.0);
      hessianExpected.set(2, 0, Math.pow(duration, 5) / 5.0);
      hessianExpected.set(0, 3, Math.pow(duration, 4) / 4.0);
      hessianExpected.set(1, 2, Math.pow(duration, 4) / 4.0);
      hessianExpected.set(2, 1, Math.pow(duration, 4) / 4.0);
      hessianExpected.set(3, 0, Math.pow(duration, 4) / 4.0);
      hessianExpected.set(1, 3, Math.pow(duration, 3) / 3.0);
      hessianExpected.set(2, 2, Math.pow(duration, 3) / 3.0);
      hessianExpected.set(3, 1, Math.pow(duration, 3) / 3.0);
      hessianExpected.set(2, 3, Math.pow(duration, 2) / 2.0);
      hessianExpected.set(3, 2, Math.pow(duration, 2) / 2.0);
      hessianExpected.set(3, 3, duration);

      EjmlUnitTests.assertEquals(hessianExpected, hessian, 1e-5);


      DMatrixRMaj gradientExpected = new DMatrixRMaj(4, 1);
      double a3 = startValue;
      double a2 = startRate;
      double a1 = 3.0 / Math.pow(duration, 2) * (endValue - a3) - (endRate + 2.0 * a2) / duration;
      double a0 = (endRate + a2) / Math.pow(duration, 2) - 2.0 / Math.pow(duration, 3) * (endValue - a3);

      gradientExpected.set(0, 0, -(a0 * Math.pow(duration, 7) / 7.0 + a1 * Math.pow(duration, 6) / 6.0 + a2 * Math.pow(duration, 5) / 5.0 + a3 * Math.pow(duration, 4) / 4.0));
      gradientExpected.set(1, 0, -(a0 * Math.pow(duration, 6) / 6.0 + a1 * Math.pow(duration, 5) / 5.0 + a2 * Math.pow(duration, 4) / 4.0 + a3 * Math.pow(duration, 3) / 3.0));
      gradientExpected.set(2, 0, -(a0 * Math.pow(duration, 5) / 5.0 + a1 * Math.pow(duration, 4) / 4.0 + a2 * Math.pow(duration, 3) / 3.0 + a3 * Math.pow(duration, 2) / 2.0));
      gradientExpected.set(3, 0, -(a0 * Math.pow(duration, 4) / 4.0 + a1 * Math.pow(duration, 3) / 3.0 + a2 * Math.pow(duration, 2) / 2.0 + a3 * duration));

      EjmlUnitTests.assertEquals(gradientExpected, gradient, 1e-5);

      DMatrixRMaj perfectSolution = new DMatrixRMaj(4, 1);
      perfectSolution.set(0, 0, a0);
      perfectSolution.set(1, 0, a1);
      perfectSolution.set(2, 0, a2);
      perfectSolution.set(3, 0, a3);

      DMatrixRMaj Hx = new DMatrixRMaj(4, 1);
      DMatrixRMaj cost = new DMatrixRMaj(1, 1);
      CommonOps_DDRM.mult(hessian, perfectSolution, Hx);
      CommonOps_DDRM.multTransA(perfectSolution, Hx, cost);
      CommonOps_DDRM.multAddTransA(gradient, perfectSolution, cost);

      assertEquals(0.0, cost.get(0, 0), 1e-5);
   }
}
