package us.ihmc.robotics.filters;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class TimeWindowVelocityEstimator3DTest
{
   @Test
   public void testWindow()
   {
      YoRegistry registry = new YoRegistry("test");
      double windowDuration = 0.25;
      double dt = 0.002;

      int windowSize = (int) Math.ceil(0.25 / dt);

      TimeWindowVelocityEstimator3D estimator = new TimeWindowVelocityEstimator3D("test", ReferenceFrame.getWorldFrame(), windowDuration, dt, registry);

      Random random = new Random(1738L);
      for (int i = 0; i < 200; i++)
      {
         FramePoint3D position = EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D velocity = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());

         estimator.reset();
         estimator.update(position);
         for (int index = 0; index < 2 * windowSize; index++)
         {
            position.scaleAdd(dt, velocity, position);
            estimator.update(position);

            EuclidFrameTestTools.assertEquals(velocity, estimator, 1e-5);
         }
      }
   }
}
