package us.ihmc.robotics.hyperCubeTree;

import static us.ihmc.robotics.Assert.*;

import org.junit.jupiter.api.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
public class OneDimensionalBoundsTest
{
   private static final double eps = 1e-14;

	@Test
   public void testIntersection()
   {
      OneDimensionalBounds zeroToOne = new OneDimensionalBounds(0.0, 1.0);
      OneDimensionalBounds zeroToTwenty = new OneDimensionalBounds(0.0, 20.0);
      OneDimensionalBounds result = zeroToOne.intersectionWith(zeroToTwenty);
      assertEquals(1.0,result.max(),eps);
   }

}
