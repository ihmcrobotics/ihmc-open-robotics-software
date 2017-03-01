package us.ihmc.robotics.hyperCubeTree;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class SphericalLinearResolutionProviderTest
{
   private static final double eps = 1e-7;

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void test()
   {
      double minRes = 0.05;
      double maxRes = 0.5;
      SphericalLinearResolutionProvider resolutionProvider = new SphericalLinearResolutionProvider(new FramePoint(ReferenceFrame.getWorldFrame(),0.0,0.0,0.0),
                                                                new OneDimensionalBounds(1.0, 10.0), minRes, maxRes);
      double[] ds = {0.0, 0.0, 0.0};
      assertEquals(minRes, resolutionProvider.getResolution(ds), eps);
      double[] ds1 = {1.0, 0.0, 0.0};
      assertEquals(minRes, resolutionProvider.getResolution(ds1), eps);
      double[] ds2 = {10.0, 0.0, 0.0};
      assertEquals(maxRes, resolutionProvider.getResolution(ds2), eps);
      double[] ds3 = {100.0, 0.0, 0.0};
      assertEquals(maxRes, resolutionProvider.getResolution(ds3), eps);
      double[] ds4 = {5.5, 0.0, 0.0};

      assertEquals((maxRes + minRes) * 0.5, resolutionProvider.getResolution(ds4), eps);
   }

}
