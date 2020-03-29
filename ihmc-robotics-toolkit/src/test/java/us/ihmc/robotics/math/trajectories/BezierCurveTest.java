package us.ihmc.robotics.math.trajectories;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.InterpolationTools;
import us.ihmc.robotics.Assert;

public class BezierCurveTest
{
   @Test
   public void testStraightLine()
   {
      double minX = 0.25;
      double maxX = 1.79;
      double minZ = 0.68;
      double maxZ = -1.62;

      BezierCurve curve = new BezierCurve();
      curve.addPoint(minX, minZ);
      curve.addPoint(maxX, maxZ);

      for (double x = minX; x <= maxX; x += 0.001)
      {
         double alpha = (x - minX) / (maxX - minX);
         double z = InterpolationTools.linearInterpolate(minZ, maxZ, alpha);
         curve.compute(x);

         Assert.assertEquals(z, curve.getPosition(), 1e-8);
      }


      double slope = (maxZ - minZ) / (maxX - minX);
      double offset = minZ - slope * minX;
      double thirdX = 0.89;
      double thirdZ = slope * thirdX + offset;

      curve.addPoint(thirdX, thirdZ);

      for (double x = minX; x <= maxX; x += 0.001)
      {
         double alpha = (x - minX) / (maxX - minX);
         double z = InterpolationTools.linearInterpolate(minZ, maxZ, alpha);
         curve.compute(x);

         Assert.assertEquals(z, curve.getPosition(), 1e-8);
      }
   }
}
