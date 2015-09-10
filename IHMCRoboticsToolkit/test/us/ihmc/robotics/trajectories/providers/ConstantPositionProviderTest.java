package us.ihmc.robotics.trajectories.providers;

import org.junit.Test;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

import java.util.Random;

import static org.junit.Assert.assertTrue;

public class ConstantPositionProviderTest
{

   private static final double EPS = 1e-12;

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void test()
   {
      Random random = new Random();
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      double[] xyz = RandomTools.generateRandomDoubleArray(random, 3, Double.MAX_VALUE);
      FramePoint positionExpected = new FramePoint(worldFrame, xyz);
      FramePoint positionActual = new FramePoint(positionExpected);
      ConstantPositionProvider constantPositionProvider = new ConstantPositionProvider(positionActual);
      constantPositionProvider.get(positionActual);
      
      assertTrue(positionActual.epsilonEquals(positionExpected, EPS));
   }

}
