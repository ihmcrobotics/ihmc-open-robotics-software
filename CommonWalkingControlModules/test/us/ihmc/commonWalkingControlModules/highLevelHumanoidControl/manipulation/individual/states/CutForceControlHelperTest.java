package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.states;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import javax.vecmath.Vector3d;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.testing.TestPlanTarget;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestClass;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

@DeployableTestClass(targets = {TestPlanTarget.Flaky})
public class CutForceControlHelperTest {
	/**
	 * Unit test to test the functions of the TaskspaceToJointspaceHandForcefeedbackControlState extracted in CutForceControlHelper.
	 * Random is initialized with a new seed every time, eventually the MPA algorithm does not converge in time.
	 */
	YoVariableRegistry registry = new YoVariableRegistry("testregistry");
	private final long random = System.currentTimeMillis();
	private Random randomNumberGenerator = new Random(random);

	private final DoubleYoVariable epsilon = new DoubleYoVariable("epsilon", registry);
	private final DoubleYoVariable w11 = new DoubleYoVariable("w11", registry);
	private final DoubleYoVariable w22 = new DoubleYoVariable("w22", registry);
	private final DoubleYoVariable C1adapted = new DoubleYoVariable("c1", registry);
	private final DoubleYoVariable C2adapted = new DoubleYoVariable("c2", registry);
	
	private final DoubleYoVariable fxRaw = new DoubleYoVariable("fx", registry);
	private final DoubleYoVariable fyRaw = new DoubleYoVariable("fy", registry);
	private final DoubleYoVariable fzRaw = new DoubleYoVariable("fz", registry);
	
	private final double alpha = 0.3;
	private final AlphaFilteredYoVariable fxFiltered = new AlphaFilteredYoVariable("fxFiltered", registry, alpha, fxRaw);
	private final AlphaFilteredYoVariable fyFiltered = new AlphaFilteredYoVariable("fyFiltered", registry, alpha, fyRaw);
	private final AlphaFilteredYoVariable fzFiltered = new AlphaFilteredYoVariable("fzFiltered", registry, alpha, fzRaw);
	
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
	@DeployableTestMethod(estimatedDuration = 0.5)
	@Test(timeout = 50000)
	public void testExponentialForceModel()
	{
		double C1 = RandomTools.generateRandomDouble(randomNumberGenerator, 80.0, 120.0);
		double C2 = RandomTools.generateRandomDouble(randomNumberGenerator, 0.01, 10.0);
		double v = RandomTools.generateRandomDouble(randomNumberGenerator, 0.001, 10.0);
		
		double F = CutForceControlHelper.exponentialForceModel(v, C1, C2);
		double C2_hat = 1.0/v *Math.log(F / C1 + 1.0);
		
		assertEquals(CutForceControlHelper.exponentialForceModel(0.0, C1, C2), 0.0, EPSILON);
		assertEquals(C2, C2_hat, EPSILON);
	}
	
	/**
	 * Test that the MPA algorithm settles to a possible set of parameters C1,C2 to represent the desired F(v).
	 * Note that the adapted parameters do not necessarily correspond to the real values.
	 * The selection range for c2 and v has to be chosen small since they are in the exponent of the function.
	 */
	@DeployableTestMethod(estimatedDuration = 0.5)
	@Test(timeout = 50000)
	public void testModelParameterAdaption()
	{
		double C1 = RandomTools.generateRandomDouble(randomNumberGenerator, 80.0, 120.0);
		double C2 = RandomTools.generateRandomDouble(randomNumberGenerator, 0.01, 2.0);
		
		C1adapted.set(RandomTools.generateRandomDouble(randomNumberGenerator, 80.0, 120.0));
		C2adapted.set(RandomTools.generateRandomDouble(randomNumberGenerator, 0.01, 2.0));
		double v = RandomTools.generateRandomDouble(randomNumberGenerator, 0.005, 10.0);
		epsilon.set(0.0);
		
		while(Math.abs(CutForceControlHelper.exponentialForceModel(v, C1adapted.getDoubleValue(), C2adapted.getDoubleValue()) - CutForceControlHelper.exponentialForceModel(v, C1, C2)) > VELOCITYERROR)
		{
			double realForce = CutForceControlHelper.exponentialForceModel(v, C1, C2);
			double modelForce = CutForceControlHelper.exponentialForceModel(v, C1adapted.getDoubleValue(), C2adapted.getDoubleValue());
			CutForceControlHelper.modelParameterAdaption(realForce, modelForce, v, epsilon, C1adapted, C2adapted, 0.001, 0.005);
		}
	}
	
	/**
	 * Test the ramp function that scales the weigthing matrix.
	 */
	@DeployableTestMethod(estimatedDuration = 0.5)
	@Test(timeout = 50000)
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
	/**
	 * Test the getTangentForce method using geometric considerations.
	 */
	@DeployableTestMethod(estimatedDuration = 0.5)
	@Test(timeout = 50000)
	public void testGetTangentForce()
	{
		final Vector3d lastTangentVector = RandomTools.generateRandomVector(randomNumberGenerator);
		final Vector3d tangentVector = RandomTools.generateRandomVector(randomNumberGenerator);
		final Vector3d forceVector = new Vector3d();
	
		
		final Vector3d eX = new Vector3d(1.0, 0.0, 0.0);
		final Vector3d eY = new Vector3d(0.0, 1.0, 0.0);
		final Vector3d eZ = new Vector3d(0.0, 0.0, 1.0);
		
		
		DoubleYoVariable currentTangentialForce = new DoubleYoVariable("Ftang", registry);
		
		double fxFiltered = RandomTools.generateRandomDouble(randomNumberGenerator, 10.0);
		
		
		double fyFiltered = RandomTools.generateRandomDouble(randomNumberGenerator, 10.0);
		double fzFiltered = RandomTools.generateRandomDouble(randomNumberGenerator, 10.0);
		
		// Projection on world base vectors
		CutForceControlHelper.getTangentForce(forceVector, currentTangentialForce, eX, lastTangentVector, fxFiltered, fyFiltered, fzFiltered, false, 0.0);
		assertEquals(forceVector.getX(), fxFiltered, EPSILON);
		assertEquals(currentTangentialForce.getDoubleValue(), fxFiltered, EPSILON);
		
		CutForceControlHelper.getTangentForce(forceVector, currentTangentialForce, eY, lastTangentVector, fxFiltered, fyFiltered, fzFiltered, false, 0.0);
		assertEquals(forceVector.getY(), fyFiltered, EPSILON);
		assertEquals(currentTangentialForce.getDoubleValue(), fyFiltered, EPSILON);
		
		CutForceControlHelper.getTangentForce(forceVector, currentTangentialForce, eZ, lastTangentVector, fxFiltered, fyFiltered, fzFiltered, false, 0.0);
		assertEquals(forceVector.getZ(), fzFiltered, EPSILON);
		assertEquals(currentTangentialForce.getDoubleValue(), fzFiltered, EPSILON);
		
		// Projection on random tangent vector
		CutForceControlHelper.getTangentForce(forceVector, currentTangentialForce, tangentVector, lastTangentVector, fxFiltered, fyFiltered, fzFiltered, false, 0.0);
		
		assertEquals(forceVector.dot(tangentVector), currentTangentialForce.getDoubleValue(), EPSILON);
		assertEquals(currentTangentialForce.getDoubleValue(), fxFiltered * tangentVector.getX() + fyFiltered * tangentVector.getY() + fzFiltered * tangentVector.getZ(), EPSILON);
		
		//Make sure old tangent vector is updated
		assertEquals(lastTangentVector.getX(), tangentVector.getX(), EPSILON);
		assertEquals(lastTangentVector.getY(), tangentVector.getY(), EPSILON);
		assertEquals(lastTangentVector.getZ(), tangentVector.getZ(), EPSILON);
		
		//Make sure set to zero when tangent vector is zero
		tangentVector.set(0.0, 0.0, 0.0);
		CutForceControlHelper.getTangentForce(forceVector, currentTangentialForce, tangentVector, lastTangentVector, fxFiltered, fyFiltered, fzFiltered, false, 0.0);
		assertEquals(currentTangentialForce.getDoubleValue(), 0.0, EPSILON);
	}
	@DeployableTestMethod(estimatedDuration = 0.5)
	@Test(timeout = 50000)
	public void testWristSensorUpdate()
	{
		ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
		ReferenceFrame randomFrame = ReferenceFrame.generateRandomReferenceFrame("randomFrame", randomNumberGenerator, worldFrame);
		Wrench testWrench = new Wrench(randomFrame, randomFrame);
		double mass = RandomTools.generateRandomDouble(randomNumberGenerator, 0.1, 10.0);
		double fzEmpty = RandomTools.generateRandomDouble(randomNumberGenerator, 5.0);
		
		CutForceControlHelper.wristSensorUpdate(testWrench, worldFrame, fxRaw, fyRaw, fzRaw, fxFiltered, fyFiltered, fzFiltered, mass, false, randomNumberGenerator, 0.0);
		
		// Check if reference frame is changed
		assertTrue(testWrench.getExpressedInFrame() == worldFrame);
		
		// Check gravity constant
		assertEquals(CutForceControlHelper.GRAVITY, 9.81, EPSILON);
		
		// Check gravity compensation in z direction
		testWrench.changeFrame(worldFrame);
		testWrench.setLinearPartZ(fzEmpty);
		testWrench.changeBodyFrameAttachedToSameBody(worldFrame);
		CutForceControlHelper.wristSensorUpdate(testWrench, worldFrame, fxRaw, fyRaw, fzRaw, fxFiltered, fyFiltered, fzFiltered, mass, false, randomNumberGenerator, 0.0);
		
		assertEquals(testWrench.getLinearPartZ() + mass * CutForceControlHelper.GRAVITY, fzRaw.getDoubleValue(), EPSILON);

	}
}
