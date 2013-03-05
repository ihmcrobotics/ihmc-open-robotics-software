package us.ihmc.commonWalkingControlModules.trajectories;

import org.junit.Test;

import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.trajectory.ConstantDoubleProvider;
import com.yobotics.simulationconstructionset.util.trajectory.ConstantPositionProvider;
import com.yobotics.simulationconstructionset.util.trajectory.DoubleProvider;
import com.yobotics.simulationconstructionset.util.trajectory.PositionProvider;
import com.yobotics.simulationconstructionset.util.trajectory.VectorProvider;

public class TwoWaypointPositionTrajectoryGeneratorTest {

	private static ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

	@Test
	public void testSimpleLinearTrajectory()
	{
		DoubleProvider stepTimeProvider = new ConstantDoubleProvider(1.0);
		PositionProvider initialPositionProvider = new ConstantPositionProvider(new FramePoint(worldFrame, new double[]{0.0, 0.0, 0.0}));
		VectorProvider initialVelocityProvider = new ConstantVectorProvider(new FrameVector(worldFrame, new double[]{0.0, 0.0, 0.0}));
		
		PositionProvider firstIntermediatePositionProvider = new ConstantPositionProvider(new FramePoint(worldFrame, new double[]{0.0, 0.0, 0.0}));
		PositionProvider secondIntermediatePositionProvider = new ConstantPositionProvider(new FramePoint(worldFrame, new double[]{0.0, 0.0, 0.0}));
		
		int numDesiredSplines = 3;
		int arcLengthCalculatorDivisions = 100;
		
		TwoWaypointPositionTrajectoryGenerator trajectory = new TwoWaypointPositionTrajectoryGenerator("", worldFrame, stepTimeProvider, 
				initialPositionProvider, initialVelocityProvider, secondIntermediatePositionProvider, initialVelocityProvider, new YoVariableRegistry(""), firstIntermediatePositionProvider, 
				secondIntermediatePositionProvider, numDesiredSplines, arcLengthCalculatorDivisions);
		
		
	}
	
}
