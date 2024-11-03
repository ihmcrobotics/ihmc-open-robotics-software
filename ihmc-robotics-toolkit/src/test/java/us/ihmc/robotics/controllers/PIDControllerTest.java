package us.ihmc.robotics.controllers;

import static us.ihmc.robotics.Assert.*;

import java.util.Random;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.robotics.controllers.pidGains.implementations.YoPIDGains;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class PIDControllerTest
{
   private final Random random = new Random();

   @BeforeEach
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @AfterEach
   public void showMemoryUsageAfterTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

	@Test
   public void testPIDControllerConstructor()
   {
      YoRegistry registry = new YoRegistry("mike");
      YoDouble proportional = new YoDouble("proportional", registry);
      proportional.set(2.0);

      YoDouble integral = new YoDouble("integral", registry);
      integral.set(3.0);

      YoDouble derivative = new YoDouble("derivative", registry);
      derivative.set(4.0);

      YoDouble maxError = new YoDouble("maxError", registry);
      maxError.set(10.0);

      new PIDController(proportional, integral, derivative, maxError, "", registry);
      assertEquals(2.0, proportional.getDoubleValue(), 0.001);
      assertEquals(3.0, integral.getDoubleValue(), 0.001);
      assertEquals(4.0, derivative.getDoubleValue(), 0.001);
      assertEquals(10.0, maxError.getDoubleValue(), 0.001);
   }

   @Test
   public void testPIDControllerConstructorFromGains()
   {
      YoRegistry registry = new YoRegistry("robert");

      double proportional = random.nextDouble();
      double integral = random.nextDouble();
      double derivative = random.nextDouble();
      double maxError = random.nextDouble();
      double deadband = random.nextDouble();
      double leakRate = random.nextDouble();
      double maxOutput = 100 * random.nextDouble();

      YoPIDGains pidGains = new YoPIDGains("", registry);
      pidGains.setKp(proportional);
      pidGains.setKi(integral);
      pidGains.setKd(derivative);
      pidGains.setMaximumIntegralError(maxError);
      pidGains.setPositionDeadband(deadband);
      pidGains.setIntegralLeakRatio(leakRate);
      pidGains.setMaximumFeedback(maxOutput);

      PIDController pid = new PIDController(pidGains, "", registry);
      assertEquals(proportional, pid.getProportionalGain(), 0.001);
      assertEquals(integral, pid.getIntegralGain(), 0.001);
      assertEquals(derivative, pid.getDerivativeGain(), 0.001);
      assertEquals(maxError, pid.getMaxIntegralError(), 0.001);
      assertEquals(deadband, pid.getPositionDeadband(), 0.001);
      assertEquals(leakRate, pid.getIntegralLeakRatio(), 0.001);
      assertEquals(maxOutput, pid.getMaximumFeedback(), 1e-5);
   }

   @Test
   public void testPIDControllerConstructorFromGains2()
   {
      YoRegistry registry = new YoRegistry("robert");

      double proportional = random.nextDouble();
      double integral = random.nextDouble();
      double derivative = random.nextDouble();
      double maxError = random.nextDouble();
      double deadband = random.nextDouble();
      double maxOutput = random.nextDouble() * 100;

      YoPIDGains pidGains = new YoPIDGains("", registry);
      pidGains.setKp(proportional);
      pidGains.setKi(integral);
      pidGains.setKd(derivative);
      pidGains.setMaximumIntegralError(maxError);
      pidGains.setPositionDeadband(deadband);
      pidGains.setMaximumFeedback(maxOutput);

      PIDController pid = new PIDController(pidGains, "", registry);
      assertEquals(proportional, pid.getProportionalGain(), 0.001);
      assertEquals(integral, pid.getIntegralGain(), 0.001);
      assertEquals(derivative, pid.getDerivativeGain(), 0.001);
      assertEquals(maxError, pid.getMaxIntegralError(), 0.001);
      assertEquals(deadband, pid.getPositionDeadband(), 0.001);
      assertEquals(maxOutput, pid.getMaximumFeedback(), 0.001);
      assertEquals(1.0, pid.getIntegralLeakRatio(), 0.001);
   }

   @Test
   public void testPIDControllerConstructorFromGains3()
   {
      YoRegistry registry = new YoRegistry("robert");

      double proportional = random.nextDouble();
      double integral = random.nextDouble();
      double derivative = random.nextDouble();
      double maxIntegralError = random.nextDouble();
      double maxOutput = random.nextDouble() * 100;

      YoPIDGains pidGains = new YoPIDGains("", registry);
      pidGains.setKp(proportional);
      pidGains.setKi(integral);
      pidGains.setKd(derivative);
      pidGains.setMaximumIntegralError(maxIntegralError);
      pidGains.setMaximumFeedback(maxOutput);

      PIDController pid = new PIDController(pidGains, "", registry);
      assertEquals(proportional, pid.getProportionalGain(), 0.001);
      assertEquals(integral, pid.getIntegralGain(), 0.001);
      assertEquals(derivative, pid.getDerivativeGain(), 0.001);
      assertEquals(maxIntegralError, pid.getMaxIntegralError(), 0.001);
      assertEquals(maxOutput, pid.getMaximumFeedback(), 1e-5);
      assertEquals(0.0, pid.getPositionDeadband(), 0.001);
      assertEquals(1.0, pid.getIntegralLeakRatio(), 0.001);
   }

   @Test
   public void testPIDControllerConstructorFromGains4()
   {
      YoRegistry registry = new YoRegistry("robert");

      double proportional = random.nextDouble();
      double integral = random.nextDouble();
      double derivative = random.nextDouble();
      double maxIntegralError = random.nextDouble();

      YoPIDGains pidGains = new YoPIDGains("", registry);
      pidGains.setKp(proportional);
      pidGains.setKi(integral);
      pidGains.setKd(derivative);
      pidGains.setMaximumIntegralError(maxIntegralError);

      PIDController pid = new PIDController(pidGains, "", registry);
      assertEquals(proportional, pid.getProportionalGain(), 0.001);
      assertEquals(integral, pid.getIntegralGain(), 0.001);
      assertEquals(derivative, pid.getDerivativeGain(), 0.001);
      assertEquals(maxIntegralError, pid.getMaxIntegralError(), 0.001);
      assertEquals(Double.POSITIVE_INFINITY, pid.getMaximumFeedback(), 0.001);
      assertEquals(0.0, pid.getPositionDeadband(), 0.001);
      assertEquals(1.0, pid.getIntegralLeakRatio(), 0.001);
   }

	@Test
   public void testGetProportionalGain()
   {
      YoRegistry registry = new YoRegistry("mike");
      PIDController pid = new PIDController("", registry);
      assertEquals(0.0, pid.getProportionalGain(), 0.001);
   }

	@Test
   public void testGetIntegralGain()
   {
      YoRegistry registry = new YoRegistry("mike");
      PIDController pid = new PIDController("", registry);
      assertEquals(0.0, pid.getIntegralGain(), 0.001);
   }

	@Test
   public void testGetDerivativeGain()
   {
      YoRegistry registry = new YoRegistry("mike");
      PIDController pid = new PIDController("", registry);
      assertEquals(0.0, pid.getDerivativeGain(), 0.001);
   }

   @Test
   public void testGetDeadband()
   {
      YoRegistry registry = new YoRegistry("robert");
      PIDController pid = new PIDController("", registry);
      assertEquals(0.0, pid.getPositionDeadband(), 0.001);
   }

	@Test
   public void testGetMaxIntegralError()
   {
      YoRegistry registry = new YoRegistry("mike");
      PIDController pid = new PIDController("", registry);
      assertEquals(Double.POSITIVE_INFINITY, pid.getMaxIntegralError(), 0.001);
   }

	@Test
   public void testGetCumulativeError()
   {
      YoRegistry registry = new YoRegistry("mike");
      PIDController pid = new PIDController("", registry);
      assertEquals(0.0, pid.getCumulativeError(), 0.001);
   }

   @Test
   public void testGetLeakRate()
   {
      YoRegistry registry = new YoRegistry("robert");
      PIDController pid = new PIDController("", registry);
      assertEquals(1.0, pid.getIntegralLeakRatio(), 0.001);
   }

	@Test
   public void testSetProportionalGain()
   {
      YoRegistry registry = new YoRegistry("mike");
      PIDController pid = new PIDController("", registry);
      pid.setProportionalGain(5.0);
      assertEquals(5.0, pid.getProportionalGain(), 0.001);
   }

	@Test
   public void testSetIntegralGain()
   {
      YoRegistry registry = new YoRegistry("mike");
      PIDController pid = new PIDController("", registry);
      pid.setIntegralGain(5.0);
      assertEquals(5.0, pid.getIntegralGain(), 0.001);
   }

	@Test
   public void testSetDerivativeGain()
   {
      YoRegistry registry = new YoRegistry("mike");
      PIDController pid = new PIDController("", registry);
      pid.setDerivativeGain(5.0);
      assertEquals(5.0, pid.getDerivativeGain(), 0.001);
   }

   @Test
   public void testSetDeadband()
   {
      double deadband = random.nextDouble() * 10.0;

      YoRegistry registry = new YoRegistry("robert");
      PIDController pid = new PIDController("", registry);
      pid.setPositionDeadband(deadband);
      assertEquals(deadband, pid.getPositionDeadband(), 0.001);
   }

	@Test
   public void testSetMaxIntegralError()
   {
      YoRegistry registry = new YoRegistry("mike");
      PIDController pid = new PIDController("", registry);
      pid.setMaxIntegralError(5.0);
      assertEquals(5.0, pid.getMaxIntegralError(), 0.001);
   }

	@Test
   public void testSetCumulativeError()
   {
      YoRegistry registry = new YoRegistry("mike");
      PIDController pid = new PIDController("", registry);
      pid.setCumulativeError(5.0);
      assertEquals(5.0, pid.getCumulativeError(), 0.001);
   }

   @Test
   public void testSetIntegralLeakRatio()
   {
      double leakRatio = random.nextDouble();
      YoRegistry registry = new YoRegistry("robert");
      PIDController pid = new PIDController("", registry);
      pid.setIntegralLeakRatio(leakRatio);
      assertEquals(leakRatio, pid.getIntegralLeakRatio(), 0.001);
   }

   @Test
   public void testSetIntegralLeakRatio2()
   {
      YoRegistry registry = new YoRegistry("robert");
      PIDController pid = new PIDController("", registry);

      double leakRatio = random.nextDouble() * 100.0;
      pid.setIntegralLeakRatio(leakRatio);

      assertTrue(pid.getIntegralLeakRatio() <= 1.0);

      leakRatio = random.nextDouble() * -100.0;
      pid.setIntegralLeakRatio(leakRatio);

      assertTrue(pid.getIntegralLeakRatio() >= 0.0);
   }

   @Test
   public void testSetIntegralLeakRatio3()
   {
      YoRegistry registry = new YoRegistry("robert");
      PIDController pid = new PIDController("", registry);

      YoDouble yoLeakRatio = (YoDouble)registry.findVariable("leak_");

      double leakRatio = random.nextDouble();
      yoLeakRatio.set(leakRatio);
      assertEquals(leakRatio, yoLeakRatio.getDoubleValue(), 1e-5);
      assertEquals(leakRatio, pid.getIntegralLeakRatio(), 1e-5);

      leakRatio = random.nextDouble() * 100.0;
      yoLeakRatio.set(leakRatio);
      assertTrue(pid.getIntegralLeakRatio() <= 1.0);
      assertTrue(pid.getIntegralLeakRatio() >= 0.0);

      leakRatio = random.nextDouble() * -100.0;
      yoLeakRatio.set(leakRatio);
      assertTrue(pid.getIntegralLeakRatio() <= 1.0);
      assertTrue(pid.getIntegralLeakRatio() >= 0.0);
   }

	@Test
   public void testCompute()
   {
      YoRegistry registry = new YoRegistry("mike");
      YoDouble proportional = new YoDouble("proportional", registry);
      proportional.set(3.0);

      YoDouble integral = new YoDouble("integral", registry);
      integral.set(2.0);

      YoDouble derivative = new YoDouble("derivative", registry);
      derivative.set(1.0);

      YoDouble maxError = new YoDouble("maxError", registry);
      maxError.set(10.0);

      PIDController pid = new PIDController(proportional, integral, derivative, maxError, "", registry);

      double currentPosition = 0.0;
      double desiredPosition = 5.0;
      double currentRate = 0.0;
      double desiredRate = 1.0;

      assertEquals(17.0, pid.compute(currentPosition, desiredPosition, currentRate, desiredRate, 0.1), 0.001);
   }

   @Test
   public void testComputeFromYoPIDGains()
   {
      YoRegistry registry = new YoRegistry("robert");

      double proportional = 3.0;
      double integral = 2.0;
      double derivative = 1.0;
      double maxError = 10.0;

      YoPIDGains pidGains = new YoPIDGains("", registry);
      pidGains.setKp(proportional);
      pidGains.setKi(integral);
      pidGains.setKd(derivative);
      pidGains.setMaximumIntegralError(maxError);

      PIDController pid = new PIDController(pidGains, "", registry);

      double currentPosition = 0.0;
      double desiredPosition = 5.0;
      double currentRate = 0.0;
      double desiredRate = 1.0;

      assertEquals(17.0, pid.compute(currentPosition, desiredPosition, currentRate, desiredRate, 0.1), 0.001);
   }

	@Test
   public void testCompute_proportional()
   {
      PIDController pid = new PIDController("", null);
      pid.setProportionalGain(3.0);

      double currentPosition = 0.0;
      double desiredPosition = 5.0;
      double currentRate = 0.0;
      double desiredRate = 1.0;

      assertEquals(15.0, pid.compute(currentPosition, desiredPosition, currentRate, desiredRate, 0.1), 0.001);

      pid.setProportionalGain(6.0);
      assertEquals(30.0, pid.compute(currentPosition, desiredPosition, currentRate, desiredRate, 0.1), 0.001);
   }

   @Test
   public void testCompute_proportional_withDeadband()
   {
      PIDController pid = new PIDController("", null);
      pid.setProportionalGain(3.0);
      pid.setPositionDeadband(1.0);

      double currentPosition = 0.0;
      double desiredPosition = 5.0;
      double currentRate = 0.0;
      double desiredRate = 1.0;

      assertEquals(12.0, pid.compute(currentPosition, desiredPosition, currentRate, desiredRate, 0.1), 0.001);

      pid.setProportionalGain(6.0);
      pid.setPositionDeadband(4.0);
      assertEquals(6.0, pid.compute(currentPosition, desiredPosition, currentRate, desiredRate, 0.1), 0.001);
   }

	@Test
   public void testCompute_integral()
   {
      PIDController pid = new PIDController("", null);
      pid.setIntegralGain(4.0);

      double currentPosition = 0.0;
      double desiredPosition = 5.0;
      double currentRate = 0.0;
      double desiredRate = 3.0;

      assertEquals(2.0, pid.compute(currentPosition, desiredPosition, currentRate, desiredRate, 0.1), 0.001);

      pid.setIntegralGain(8.0);
      assertEquals(8.0, pid.compute(currentPosition, desiredPosition, currentRate, desiredRate, 0.1), 0.001);
   }

	@Test
   public void testCompute_derivative()
   {
      PIDController pid = new PIDController("", null);
      pid.setDerivativeGain(6.0);

      double currentPosition = 0.0;
      double desiredPosition = 5.0;
      double currentRate = 0.0;
      double desiredRate = 3.0;

      assertEquals(18.0, pid.compute(currentPosition, desiredPosition, currentRate, desiredRate, 0.1), 0.001);

      pid.setDerivativeGain(12.0);
      assertEquals(36.0, pid.compute(currentPosition, desiredPosition, currentRate, desiredRate, 0.1), 0.001);
   }

	@Test
   public void testCompute_all_PID()
   {
      PIDController pid = new PIDController("", null);
      pid.setProportionalGain(2.0);
      pid.setIntegralGain(3.0);
      pid.setDerivativeGain(4.0);
      pid.setMaxIntegralError(10.0);

      double currentPosition = 0.0;
      double desiredPosition = 5.0;
      double currentRate = 0.0;
      double desiredRate = 2.0;

      assertEquals((10.0 + 1.5 + 8.0), pid.compute(currentPosition, desiredPosition, currentRate, desiredRate, 0.1), 0.001);
      assertEquals((10.0 + 3.0 + 8.0), pid.compute(currentPosition, desiredPosition, currentRate, desiredRate, 0.1), 0.001);
      assertEquals((10.0 + 18.0 + 8.0), pid.compute(currentPosition, desiredPosition, currentRate, desiredRate, 1.0), 0.001);

      // tests max integration error
      assertEquals((10.0 + 30.0 + 8.0), pid.compute(currentPosition, desiredPosition, currentRate, desiredRate, 1.01), 0.001);
   }

   @Test
   public void testCompute_all_PID_withDeadband()
   {
      PIDController pid = new PIDController("", null);
      pid.setProportionalGain(2.0);
      pid.setIntegralGain(3.0);
      pid.setDerivativeGain(4.0);
      pid.setMaxIntegralError(10.0);
      pid.setPositionDeadband(1.5);

      double currentPosition = 0.0;
      double desiredPosition = 5.0;
      double currentRate = 0.0;
      double desiredRate = 2.0;

      assertEquals((7.0 + 1.05 + 8.0), pid.compute(currentPosition, desiredPosition, currentRate, desiredRate, 0.1), 0.001);
      assertEquals((7.0 + 2.1 + 8.0), pid.compute(currentPosition, desiredPosition, currentRate, desiredRate, 0.1), 0.001);
      assertEquals((7.0 + 12.6 + 8.0), pid.compute(currentPosition, desiredPosition, currentRate, desiredRate, 1.0), 0.001);

      // tests max integration error
      assertEquals((7.0 + 30.0 + 8.0), pid.compute(currentPosition, desiredPosition, currentRate, desiredRate, 3.0), 0.001);
   }

   @Test
   public void testCompute_all_PID_From_YoPID()
   {
      YoRegistry registry = new YoRegistry("robert");

      YoPIDGains pidGains = new YoPIDGains("", registry);
      double proportional = 2.0;
      double integral = 3.0;
      double derivative = 4.0;
      double maxIntegral = 10.0;

      pidGains.setKp(proportional);
      pidGains.setKi(integral);
      pidGains.setKd(derivative);
      pidGains.setMaximumIntegralError(maxIntegral);

      PIDController pid1 = new PIDController(pidGains, "1", registry);
      PIDController pid2 = new PIDController("2", registry);
      pid2.setProportionalGain(proportional);
      pid2.setIntegralGain(integral);
      pid2.setDerivativeGain(derivative);
      pid2.setMaxIntegralError(maxIntegral);

      assertEquals(proportional, pid1.getProportionalGain(), 0.001);
      assertEquals(integral, pid1.getIntegralGain(), 0.001);
      assertEquals(derivative, pid1.getDerivativeGain(), 0.001);
      assertEquals(maxIntegral, pid1.getMaxIntegralError(), 0.001);

      assertEquals(pid2.getProportionalGain(), pid1.getProportionalGain(), 0.001);
      assertEquals(pid2.getIntegralGain(), pid1.getIntegralGain(), 0.001);
      assertEquals(pid2.getDerivativeGain(), pid1.getDerivativeGain(), 0.001);
      assertEquals(pid2.getMaxIntegralError(), pid1.getMaxIntegralError(), 0.001);

      double currentPosition = 0.0;
      double desiredPosition = 5.0;
      double currentRate = 0.0;
      double desiredRate = 2.0;

      assertEquals((10.0 + 1.5 + 8.0), pid1.compute(currentPosition, desiredPosition, currentRate, desiredRate, 0.1), 0.001);
      assertEquals((10.0 + 3.0 + 8.0), pid1.compute(currentPosition, desiredPosition, currentRate, desiredRate, 0.1), 0.001);
      assertEquals((10.0 + 18.0 + 8.0), pid1.compute(currentPosition, desiredPosition, currentRate, desiredRate, 1.0), 0.001);

      assertEquals((10.0 + 1.5 + 8.0), pid2.compute(currentPosition, desiredPosition, currentRate, desiredRate, 0.1), 0.001);
      assertEquals((10.0 + 3.0 + 8.0), pid2.compute(currentPosition, desiredPosition, currentRate, desiredRate, 0.1), 0.001);
      assertEquals((10.0 + 18.0 + 8.0), pid2.compute(currentPosition, desiredPosition, currentRate, desiredRate, 1.0), 0.001);

      assertEquals(pid2.compute(currentPosition, desiredPosition, currentRate, desiredRate, 0.1), pid1.compute(currentPosition, desiredPosition, currentRate, desiredRate, 0.1), 0.001);
      assertEquals(pid2.compute(currentPosition, desiredPosition, currentRate, desiredRate, 0.1), pid1.compute(currentPosition, desiredPosition, currentRate, desiredRate, 0.1), 0.001);
      assertEquals(pid2.compute(currentPosition, desiredPosition, currentRate, desiredRate, 1.0), pid1.compute(currentPosition, desiredPosition, currentRate, desiredRate, 1.0), 0.001);

      // tests max integration error
      assertEquals((10.0 + 30.0 + 8.0), pid1.compute(currentPosition, desiredPosition, currentRate, desiredRate, 1.01), 0.001);
   }

}
