package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import org.ejml.EjmlUnitTests;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.AngleTrackingCommand;

import static us.ihmc.robotics.Assert.assertEquals;

public class AngleTrackingCostCalculatorTest
{
   @Test
   public void testCalculation()
   {
      double startValue = -5.0;
      double startRate = 11.0;
      double endValue = 7.0;
      double endRate = -9.0;

      double omega = 3.0;
      double duration = 0.7;

      AngleTrackingCommand command = new AngleTrackingCommand();
      command.setStartValue(startValue);
      command.setFinalValue(endValue);
      command.setStartRate(startRate);
      command.setFinalRate(endRate);
      command.setDuration(duration);
      command.setStartIndex(0);
      command.setOmega(omega);
      command.setWeight(100.0);

      DMatrixRMaj hessian = new DMatrixRMaj(6, 6);
      DMatrixRMaj gradient = new DMatrixRMaj(6, 1);

      AngleTrackingCostCalculator.calculateTrackingObjective(hessian, gradient, command);

      DMatrixRMaj hessianExpected = new DMatrixRMaj(6, 6);
      hessianExpected.set(0, 0, 1.0 / (2.0 * omega) * (Math.exp(2.0 * omega * duration) - 1.0));
      hessianExpected.set(1, 0, duration);
      hessianExpected.set(2, 0, Math.exp(omega * duration) / Math.pow(omega, 4.0) * (Math.pow(omega * duration, 3.0) - 3.0 * Math.pow(omega * duration, 2.0) + 6.0 * omega * duration - 6.0) + 6.0 / Math.pow(omega, 4.0));
      hessianExpected.set(3, 0, Math.exp(omega * duration) / Math.pow(omega, 3.0) * (Math.pow(omega * duration, 2.0) - 2.0 * omega * duration + 2.0) - 2.0 / Math.pow(omega, 3.0));
      hessianExpected.set(4, 0, Math.exp(omega * duration) / Math.pow(omega, 2.0) * (omega * duration - 1.0) + 1.0 / Math.pow(omega, 2.0));
      hessianExpected.set(5, 0, (Math.exp(omega * duration) - 1.0) / omega);

      hessianExpected.set(0, 1, hessianExpected.get(1, 0));
      hessianExpected.set(1, 1, -1.0 / (2.0 * omega) * (Math.exp(-2.0 * omega * duration) - 1.0));
      hessianExpected.set(2, 1, Math.exp(-omega * duration) / Math.pow(omega, 4.0) * (Math.pow(omega * duration, 3.0) + 3.0 * Math.pow(omega * duration, 2.0) + 6.0 * omega * duration + 6.0) - 6.0 / Math.pow(omega, 4.0));
      hessianExpected.set(3, 1, Math.exp(-omega * duration) / Math.pow(omega, 3.0) * (Math.pow(omega * duration, 2.0) + 2.0 * omega * duration + 2.0) - 2.0 / Math.pow(omega, 3.0));
      hessianExpected.set(4, 1, Math.exp(-omega * duration) / Math.pow(omega, 2.0) * (omega * duration + 1.0) - 1.0 / Math.pow(omega, 2.0));
      hessianExpected.set(5, 1, -(Math.exp(-omega * duration) - 1.0) / omega);

      hessianExpected.set(0, 2, hessianExpected.get(2, 0));
      hessianExpected.set(1, 2, hessianExpected.get(2, 1));
      hessianExpected.set(2, 2, Math.pow(duration, 7) / 7.0);
      hessianExpected.set(3, 2, Math.pow(duration, 6) / 6.0);
      hessianExpected.set(4, 2, Math.pow(duration, 5) / 5.0);
      hessianExpected.set(5, 2, Math.pow(duration, 4) / 4.0);

      hessianExpected.set(0, 3, hessianExpected.get(3, 0));
      hessianExpected.set(1, 3, hessianExpected.get(3, 1));
      hessianExpected.set(2, 3, hessianExpected.get(3, 2));
      hessianExpected.set(3, 3, Math.pow(duration, 5) / 5.0);
      hessianExpected.set(4, 3, Math.pow(duration, 4) / 4.0);
      hessianExpected.set(5, 3, Math.pow(duration, 3) / 3.0);

      hessianExpected.set(0, 4, hessianExpected.get(4, 0));
      hessianExpected.set(1, 4, hessianExpected.get(4, 1));
      hessianExpected.set(2, 4, hessianExpected.get(4, 2));
      hessianExpected.set(3, 4, hessianExpected.get(4, 3));
      hessianExpected.set(4, 4, Math.pow(duration, 3) / 3.0);
      hessianExpected.set(5, 4, Math.pow(duration, 2) / 2.0);

      hessianExpected.set(0, 5, hessianExpected.get(5, 0));
      hessianExpected.set(1, 5, hessianExpected.get(5, 1));
      hessianExpected.set(2, 5, hessianExpected.get(5, 2));
      hessianExpected.set(3, 5, hessianExpected.get(5, 3));
      hessianExpected.set(4, 5, hessianExpected.get(5, 4));
      hessianExpected.set(5, 5, duration);


      EjmlUnitTests.assertEquals(hessianExpected, hessian, 1e-5);


      DMatrixRMaj gradientExpected = new DMatrixRMaj(6, 1);
      double a5 = startValue;
      double a4 = startRate;
      double a3 = 3.0 / Math.pow(duration, 2) * (endValue - a5) - (endRate + 2.0 * a4) / duration;
      double a2 = (endRate + a4) / Math.pow(duration, 2) - 2.0 / Math.pow(duration, 3) * (endValue - a5);

      gradientExpected.set(0, 0, -(a2 * hessianExpected.get(2, 0) + a3 * hessianExpected.get(3, 0) + a4 * hessianExpected.get(4, 0) + a5 * hessianExpected.get(5, 0)));
      gradientExpected.set(1, 0, -(a2 * hessianExpected.get(2, 1) + a3 * hessianExpected.get(3, 1) + a4 * hessianExpected.get(4, 1) + a5 * hessianExpected.get(5, 1)));
      gradientExpected.set(2, 0, -(a2 * Math.pow(duration, 7) / 7.0 + a3 * Math.pow(duration, 6) / 6.0 + a4 * Math.pow(duration, 5) / 5.0 + a5 * Math.pow(duration, 4) / 4.0));
      gradientExpected.set(3, 0, -(a2 * Math.pow(duration, 6) / 6.0 + a3 * Math.pow(duration, 5) / 5.0 + a4 * Math.pow(duration, 4) / 4.0 + a5 * Math.pow(duration, 3) / 3.0));
      gradientExpected.set(4, 0, -(a2 * Math.pow(duration, 5) / 5.0 + a3 * Math.pow(duration, 4) / 4.0 + a4 * Math.pow(duration, 3) / 3.0 + a5 * Math.pow(duration, 2) / 2.0));
      gradientExpected.set(5, 0, -(a2 * Math.pow(duration, 4) / 4.0 + a3 * Math.pow(duration, 3) / 3.0 + a4 * Math.pow(duration, 2) / 2.0 + a5 * duration));

      EjmlUnitTests.assertEquals(gradientExpected, gradient, 1e-5);

      DMatrixRMaj perfectSolution = new DMatrixRMaj(6, 1);
      perfectSolution.set(2, 0, a2);
      perfectSolution.set(3, 0, a3);
      perfectSolution.set(4, 0, a4);
      perfectSolution.set(5, 0, a5);

      DMatrixRMaj Hx = new DMatrixRMaj(6, 1);
      DMatrixRMaj cost = new DMatrixRMaj(1, 1);
      CommonOps_DDRM.mult(hessian, perfectSolution, Hx);
      CommonOps_DDRM.multTransA(perfectSolution, Hx, cost);
      CommonOps_DDRM.multAddTransA(gradient, perfectSolution, cost);

      assertEquals(0.0, cost.get(0, 0), 1e-5);
   }
}
