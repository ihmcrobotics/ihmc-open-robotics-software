package us.ihmc.robotics.trajectories.providers;

import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.junit.Test;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class ConstantPositionProviderTest
{

   private static final double EPS = 1e-12;

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void test()
   {
      Random random = new Random();
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      double[] xyz = RandomNumbers.nextDoubleArray(random, 3, Double.MAX_VALUE);
      FramePoint positionExpected = new FramePoint(worldFrame, xyz);
      FramePoint positionActual = new FramePoint(positionExpected);
      ConstantPositionProvider constantPositionProvider = new ConstantPositionProvider(positionActual);
      constantPositionProvider.getPosition(positionActual);
      
      assertTrue(positionActual.epsilonEquals(positionExpected, EPS));
   }

}
