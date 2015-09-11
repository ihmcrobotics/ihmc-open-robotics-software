package us.ihmc.robotics.controllers;

import static org.junit.Assert.assertEquals;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.robotics.controllers.PIDController;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;


public class PIDControllerTest
{
   @Before
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }
   
   @After
   public void showMemoryUsageAfterTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

	@DeployableTestMethod
	@Test(timeout=300000)
   public void testPIDControllerConstructor()
   {
      YoVariableRegistry registry = new YoVariableRegistry("mike");
      DoubleYoVariable proportional = new DoubleYoVariable("proportional", registry);
      proportional.set(2.0);

      DoubleYoVariable integral = new DoubleYoVariable("integral", registry);
      integral.set(3.0);

      DoubleYoVariable derivative = new DoubleYoVariable("derivative", registry);
      derivative.set(4.0);

      DoubleYoVariable maxError = new DoubleYoVariable("maxError", registry);
      maxError.set(10.0);

      new PIDController(proportional, integral, derivative, maxError, "", registry);
      assertEquals(2.0, proportional.getDoubleValue(), 0.001);
      assertEquals(3.0, integral.getDoubleValue(), 0.001);
      assertEquals(4.0, derivative.getDoubleValue(), 0.001);
      assertEquals(10.0, maxError.getDoubleValue(), 0.001);
   }

	@DeployableTestMethod
	@Test(timeout=300000)
   public void testGetProportionalGain()
   {
      YoVariableRegistry registry = new YoVariableRegistry("mike");
      PIDController pid = new PIDController("", registry);
      assertEquals(0.0, pid.getProportionalGain(), 0.001);
   }

	@DeployableTestMethod
	@Test(timeout=300000)
   public void testGetIntegralGain()
   {
      YoVariableRegistry registry = new YoVariableRegistry("mike");
      PIDController pid = new PIDController("", registry);
      assertEquals(0.0, pid.getIntegralGain(), 0.001);
   }

	@DeployableTestMethod
	@Test(timeout=300000)
   public void testGetDerivativeGain()
   {
      YoVariableRegistry registry = new YoVariableRegistry("mike");
      PIDController pid = new PIDController("", registry);
      assertEquals(0.0, pid.getDerivativeGain(), 0.001);
   }

	@DeployableTestMethod
	@Test(timeout=300000)
   public void testGetMaxIntegralError()
   {
      YoVariableRegistry registry = new YoVariableRegistry("mike");
      PIDController pid = new PIDController("", registry);
      assertEquals(Double.POSITIVE_INFINITY, pid.getMaxIntegralError(), 0.001);
   }

	@DeployableTestMethod
	@Test(timeout=300000)
   public void testGetCumulativeError()
   {
      YoVariableRegistry registry = new YoVariableRegistry("mike");
      PIDController pid = new PIDController("", registry);
      assertEquals(0.0, pid.getCumulativeError(), 0.001);
   }

	@DeployableTestMethod
	@Test(timeout=300000)
   public void testSetProportionalGain()
   {
      YoVariableRegistry registry = new YoVariableRegistry("mike");
      PIDController pid = new PIDController("", registry);
      pid.setProportionalGain(5.0);
      assertEquals(5.0, pid.getProportionalGain(), 0.001);
   }

	@DeployableTestMethod
	@Test(timeout=300000)
   public void testSetIntegralGain()
   {
      YoVariableRegistry registry = new YoVariableRegistry("mike");
      PIDController pid = new PIDController("", registry);
      pid.setIntegralGain(5.0);
      assertEquals(5.0, pid.getIntegralGain(), 0.001);
   }

	@DeployableTestMethod
	@Test(timeout=300000)
   public void testSetDerivativeGain()
   {
      YoVariableRegistry registry = new YoVariableRegistry("mike");
      PIDController pid = new PIDController("", registry);
      pid.setDerivativeGain(5.0);
      assertEquals(5.0, pid.getDerivativeGain(), 0.001);
   }

	@DeployableTestMethod
	@Test(timeout=300000)
   public void testSetMaxIntegralError()
   {
      YoVariableRegistry registry = new YoVariableRegistry("mike");
      PIDController pid = new PIDController("", registry);
      pid.setMaxIntegralError(5.0);
      assertEquals(5.0, pid.getMaxIntegralError(), 0.001);
   }

	@DeployableTestMethod
	@Test(timeout=300000)
   public void testSetCumulativeError()
   {
      YoVariableRegistry registry = new YoVariableRegistry("mike");
      PIDController pid = new PIDController("", registry);
      pid.setCumulativeError(5.0);
      assertEquals(5.0, pid.getCumulativeError(), 0.001);
   }

	@DeployableTestMethod
	@Test(timeout=300000)
   public void testCompute()
   {
      YoVariableRegistry registry = new YoVariableRegistry("mike");
      DoubleYoVariable proportional = new DoubleYoVariable("proportional", registry);
      proportional.set(3.0);

      DoubleYoVariable integral = new DoubleYoVariable("integral", registry);
      integral.set(2.0);

      DoubleYoVariable derivative = new DoubleYoVariable("derivative", registry);
      derivative.set(1.0);

      DoubleYoVariable maxError = new DoubleYoVariable("maxError", registry);
      maxError.set(10.0);

      PIDController pid = new PIDController(proportional, integral, derivative, maxError, "", registry);

      double currentPosition = 0.0;
      double desiredPosition = 5.0;
      double currentRate = 0.0;
      double desiredRate = 1.0;

      assertEquals(17.0, pid.compute(currentPosition, desiredPosition, currentRate, desiredRate, 0.1), 0.001);
   }

	@DeployableTestMethod
	@Test(timeout=300000)
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

	@DeployableTestMethod
	@Test(timeout=300000)
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

	@DeployableTestMethod
	@Test(timeout=300000)
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

	@DeployableTestMethod
	@Test(timeout=300000)
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

}
