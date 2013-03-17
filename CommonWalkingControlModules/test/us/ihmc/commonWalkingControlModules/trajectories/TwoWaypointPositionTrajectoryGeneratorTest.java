package us.ihmc.commonWalkingControlModules.trajectories;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;
import com.yobotics.simulationconstructionset.util.trajectory.ConstantPositionProvider;
import com.yobotics.simulationconstructionset.util.trajectory.PositionProvider;
import com.yobotics.simulationconstructionset.util.trajectory.VectorProvider;
import com.yobotics.simulationconstructionset.util.trajectory.YoPositionProvider;

public class TwoWaypointPositionTrajectoryGeneratorTest {

	private static ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

	@Test
	public void testSimpleTrajectories()
	{
		testSimpleTrajectory(20);
		testSimpleTrajectory(4);
	}
	
	private void testSimpleTrajectory(int numDesiredSplines)
	{
		YoVariableDoubleProvider stepTimeProvider = new YoVariableDoubleProvider("", new YoVariableRegistry(""));
		stepTimeProvider.set(0.8);
		PositionProvider initialPositionProvider = new ConstantPositionProvider(new FramePoint(worldFrame, new double[]{-0.1, 2.3, 0.0}));
		VectorProvider initialVelocityProvider = new ConstantVectorProvider(new FrameVector(worldFrame, new double[]{0.2, 0.0, -0.05}));
		
		FramePoint firstIntermediatePosition = new FramePoint(worldFrame, new double[]{0.12, 2.4, 0.2});
		FramePoint secondIntermediatePosition = new FramePoint(worldFrame, new double[]{0.16, 2.3, 0.15});
		
		YoFramePoint finalPosition = new YoFramePoint("", worldFrame, new YoVariableRegistry(""));
		finalPosition.set(new FramePoint(worldFrame, new double[]{0.2, 2.35, 0.03}));
		YoPositionProvider finalPositionProvider = new YoPositionProvider(finalPosition);
		VectorProvider finalVelocityProvider = new ConstantVectorProvider(new FrameVector(worldFrame, new double[]{0.1, 0.01, -0.02}));
		
		TwoWaypointPositionTrajectoryGenerator trajectory = new TwoWaypointPositionTrajectoryGenerator("", worldFrame, stepTimeProvider, 
				initialPositionProvider, initialVelocityProvider, finalPositionProvider, finalVelocityProvider, new YoVariableRegistry(""), 0.2, numDesiredSplines,
				null);
		
		trajectory.initialize(new FramePoint[]{firstIntermediatePosition, secondIntermediatePosition});

		trajectory.compute(0.0);
		FramePoint actual = new FramePoint(worldFrame);
		FramePoint expected = new FramePoint(worldFrame);
		initialPositionProvider.get(expected);
		trajectory.get(actual);
		assertEquals(actual.getX(), expected.getX(), 1e-7);
		assertEquals(actual.getY(), expected.getY(), 1e-7);
		assertEquals(actual.getZ(), expected.getZ(), 1e-7);
		assertFalse(trajectory.isDone());
		
		FrameVector actualVel = new FrameVector(worldFrame);
		FrameVector expectedVel = new FrameVector(worldFrame);
		trajectory.packVelocity(actualVel);
		initialVelocityProvider.get(expectedVel);
		assertEquals(actualVel.getX(), expectedVel.getX(), 1e-7);
		assertEquals(actualVel.getY(), expectedVel.getY(), 1e-7);
		assertEquals(actualVel.getZ(), expectedVel.getZ(), 1e-7);
		
		trajectory.compute(0.8);
		finalPositionProvider.get(expected);
		trajectory.get(actual);
		assertEquals(actual.getX(), expected.getX(), 1e-7);
		assertEquals(actual.getY(), expected.getY(), 1e-7);
		assertEquals(actual.getZ(), expected.getZ(), 1e-7);
		
		trajectory.packVelocity(actualVel);
		finalVelocityProvider.get(expectedVel);
		assertEquals(actualVel.getX(), expectedVel.getX(), 1e-7);
		assertEquals(actualVel.getY(), expectedVel.getY(), 1e-7);
		assertEquals(actualVel.getZ(), expectedVel.getZ(), 1e-7);
		assertTrue(trajectory.isDone());
	}
	
	// make a test where you can know the arc lengths of the various portions in order to test intermediate points
	
	// take a look at other trajectory generator tests
	
	// incorporate tests of the two public voids
	
}
