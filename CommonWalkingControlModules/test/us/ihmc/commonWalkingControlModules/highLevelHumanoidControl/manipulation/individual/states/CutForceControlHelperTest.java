package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.states;

import static org.junit.Assert.assertEquals;

import java.util.Random;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.agileTesting.BambooAnnotations.EstimatedDuration;
import us.ihmc.tools.random.RandomTools;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class CutForceControlHelperTest {
	/**
	 * Unit test to test the functions of the TaskspaceToJointspaceHandForcefeedbackControlState extracted in CutForceControlHelper.
	 */
	YoVariableRegistry registry = new YoVariableRegistry("testregistry");
	private final long random = System.currentTimeMillis();
	private Random randomNumberGenerator = new Random(random);
	private double v;
	private double F;
	private double C1;
	private double C2;
	private final DoubleYoVariable epsilon = new DoubleYoVariable("epsilon", registry);
	private final DoubleYoVariable w11 = new DoubleYoVariable("w11", registry);
	private final DoubleYoVariable w22 = new DoubleYoVariable("w22", registry);
	private final DoubleYoVariable C1adapted = new DoubleYoVariable("c1", registry);
	private final DoubleYoVariable C2adapted = new DoubleYoVariable("c2", registry);
	private static final double VELOCITYERROR = 10e-2;
	private static final double EPSILON = 10e-6;
	
	
	
	@Before
	public void showMemoryUsageBeforeTest()
	{
		MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getName() + " before test: ");
	}
	
	@After
	public void showMemoryUsageAfterTest()
	{
		MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getName() + " after test: ");
	}
	
	/**
	 * Test the exponential character of the force model.
	 */
	@EstimatedDuration
	@Test(timeout = 30000)
	public void testExponentialForceModel()
	{
		C1 = RandomTools.generateRandomDouble(randomNumberGenerator, 80.0, 120.0);
		C2 = RandomTools.generateRandomDouble(randomNumberGenerator, 0.01, 10.0);
		v = RandomTools.generateRandomDouble(randomNumberGenerator, 0.001, 10.0);
		
		F = CutForceControlHelper.exponentialForceModel(v, C1, C2);
		double C2_hat = 1.0/v *Math.log(F / C1 + 1.0);
		
		assertEquals(CutForceControlHelper.exponentialForceModel(0.0, C1, C2), 0.0, EPSILON);
		assertEquals(C2, C2_hat, EPSILON);
	}
	
	/**
	 * Test that the MPA algorithm settles to a possible set of parameters C1,C2 to represent the desired F(v).
	 * Note that the adapted parameters do not necessarily correspond to the real values.
	 */
	@EstimatedDuration
	@Test(timeout = 30000)
	public void testModelParameterAdaption()
	{
		C1 = RandomTools.generateRandomDouble(randomNumberGenerator, 80.0, 120.0);
		C2 = RandomTools.generateRandomDouble(randomNumberGenerator, 0.01, 2.0);
		
		C1adapted.set(RandomTools.generateRandomDouble(randomNumberGenerator, 80.0, 120.0));
		C2adapted.set(RandomTools.generateRandomDouble(randomNumberGenerator, 0.01, 2.0));
		v = RandomTools.generateRandomDouble(randomNumberGenerator, 0.005, 10.0);
		epsilon.set(0.0);
		
		while(Math.abs(CutForceControlHelper.exponentialForceModel(v, C1adapted.getDoubleValue(), C2adapted.getDoubleValue()) - CutForceControlHelper.exponentialForceModel(v, C1, C2)) > VELOCITYERROR)
		{
			double realForce = CutForceControlHelper.exponentialForceModel(v, C1, C2);
			double modelForce = CutForceControlHelper.exponentialForceModel(v, C1adapted.getDoubleValue(), C2adapted.getDoubleValue());
			CutForceControlHelper.modelParameterAdaption(realForce, modelForce, v, epsilon, C1adapted, C2adapted, 0.001, 0.001);
		}
	}
	
	/**
	 * Test the ramp function that scales the weigthing matrix.
	 */
	@EstimatedDuration
	@Test(timeout = 3000000)
	public void testAdaptW()
	{
		final double w10 = RandomTools.generateRandomDouble(randomNumberGenerator, 0.1, 50.0);
		final double w20 = RandomTools.generateRandomDouble(randomNumberGenerator, 0.1, 50.0);
		final double x0 = RandomTools.generateRandomDouble(randomNumberGenerator, 0.1, 50.0);
		final double x1 = x0 + RandomTools.generateRandomDouble(randomNumberGenerator,0.1, 20.0);
		
		double xHigh = RandomTools.generateRandomDouble(randomNumberGenerator, x1, 70.1);
		double xLow = RandomTools.generateRandomDouble(randomNumberGenerator, 0.0, x0);
		double xMiddle = x0 + (x1-x0)/2.0;
		
		CutForceControlHelper.adaptW(x0, x0, x1, w10, w20, w11, w22);
		assertEquals(0.0, w11.getDoubleValue(), EPSILON);
		assertEquals(0.0, w22.getDoubleValue(), EPSILON);
		
		CutForceControlHelper.adaptW(x1, x0, x1, w10, w20, w11, w22);
		assertEquals(w10, w11.getDoubleValue(), EPSILON);
		assertEquals(w20, w22.getDoubleValue(), EPSILON);
		
		CutForceControlHelper.adaptW(xLow, x0, x1, w10, w20, w11, w22);
		assertEquals(0.0, w11.getDoubleValue(), EPSILON);
		assertEquals(0.0, w22.getDoubleValue(), EPSILON);
		
		CutForceControlHelper.adaptW(xHigh, x0, x1, w10, w20, w11, w22);
		assertEquals(w10, w11.getDoubleValue(), EPSILON);
		assertEquals(w20, w22.getDoubleValue(), EPSILON);
		
		CutForceControlHelper.adaptW(xMiddle, x0, x1, w10, w20, w11, w22);
		assertEquals(w10/2.0, w11.getDoubleValue(), EPSILON);
		assertEquals(w20/2.0, w22.getDoubleValue(), EPSILON);
	}
	
}
