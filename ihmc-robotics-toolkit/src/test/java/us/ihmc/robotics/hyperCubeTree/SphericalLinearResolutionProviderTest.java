package us.ihmc.robotics.hyperCubeTree;

import static us.ihmc.robotics.Assert.*;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;

public class SphericalLinearResolutionProviderTest
{
   private static final double eps = 1e-7;

   @AfterEach
   public void tearDown()
   {
      ReferenceFrameTools.clearWorldFrameTree();
   }

	@Test
   public void test()
   {
      double minRes = 0.05;
      double maxRes = 0.5;
      SphericalLinearResolutionProvider resolutionProvider = new SphericalLinearResolutionProvider(new FramePoint3D(ReferenceFrame.getWorldFrame(),0.0,0.0,0.0),
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
