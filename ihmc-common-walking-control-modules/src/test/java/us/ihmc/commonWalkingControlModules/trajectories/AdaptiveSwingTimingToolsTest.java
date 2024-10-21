package us.ihmc.commonWalkingControlModules.trajectories;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.tools.EuclidCoreMissingRandomTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

import java.util.Random;

import static us.ihmc.robotics.Assert.assertEquals;

public class AdaptiveSwingTimingToolsTest
{
   private static final int iters = 1000;

   @Test
   public void testAPerfectStep()
   {
      Random random = new Random(1738L);

      double idealStepLength = 0.4;
      double maxSwingReach = 2.0;
      double maxStepHeight = 0.5;
      double minTime = 0.5;
      double maxTime = 1.5;

      for (int i = 0; i < iters; i++)
      {
         Point3DReadOnly startPoint = EuclidCoreRandomTools.nextPoint3D(random);
         Point3D perfectEndPoint = new Point3D(startPoint);
         perfectEndPoint.addX(idealStepLength * 2.0);

         Point3D maxEndPoint = new Point3D(startPoint);
         maxEndPoint.addX(maxSwingReach);
         maxEndPoint.addZ(2.0 * maxStepHeight);


         double swingTimeAtIdealLengths = AdaptiveSwingTimingTools.calculateSwingTime(idealStepLength, maxSwingReach, maxStepHeight, minTime, maxTime, startPoint, perfectEndPoint);
         double swingTimeAtMaxDistances = AdaptiveSwingTimingTools.calculateSwingTime(idealStepLength, maxSwingReach, maxStepHeight, minTime, maxTime, startPoint, maxEndPoint);

         assertEquals(minTime, swingTimeAtIdealLengths, 1e-5);
         assertEquals(maxTime, swingTimeAtMaxDistances, 1e-5);
      }
   }

   @Test
   public void testZeroStep()
   {
      Random random = new Random(1738L);

      double idealStepLength = 0.4;
      double maxSwingReach = 2.0;
      double maxStepHeight = 0.5;
      double minTime = 0.5;
      double maxTime = 1.5;

      for (int i = 0; i < iters; i++)
      {
         Point3DReadOnly startPoint = EuclidCoreRandomTools.nextPoint3D(random);

         double swingTime = AdaptiveSwingTimingTools.calculateSwingTime(idealStepLength, maxSwingReach, maxStepHeight, minTime, maxTime, startPoint, startPoint);

         assertEquals(minTime, swingTime, 1e-5);
      }
   }
}
