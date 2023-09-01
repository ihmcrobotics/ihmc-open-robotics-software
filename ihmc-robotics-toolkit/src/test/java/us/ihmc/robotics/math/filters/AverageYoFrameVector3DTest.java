package us.ihmc.robotics.math.filters;

import static org.junit.jupiter.api.Assertions.*;

import java.util.Random;

import org.apache.commons.math3.stat.descriptive.moment.Mean;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;

public class AverageYoFrameVector3DTest
{
   @Test
   public void testAgainstApacheMean()
   {
      Random random = new Random(1);
      AverageYoFrameVector3D yoAverage = new AverageYoFrameVector3D("", ReferenceFrame.getWorldFrame(), null);
      Mean meanX = new Mean();
      Mean meanY = new Mean();
      Mean meanZ = new Mean();
      double nextX = -10.0;
      double nextY = -10.0;
      double nextZ = -10.0;

      for (int i = 0; i < 1000; i++)
      {
         nextX += random.nextDouble();
         nextY += random.nextDouble();
         nextZ += random.nextDouble();
         yoAverage.update(nextX, nextY, nextZ);
         meanX.increment(nextX);
         meanY.increment(nextY);
         meanZ.increment(nextZ);

         assertEquals(meanX.getResult(), yoAverage.getX());
         assertEquals(meanY.getResult(), yoAverage.getY());
         assertEquals(meanZ.getResult(), yoAverage.getZ());
         assertEquals(meanX.getN(), yoAverage.getSampleSize());
         assertEquals(meanY.getN(), yoAverage.getSampleSize());
         assertEquals(meanZ.getN(), yoAverage.getSampleSize());
      }
   }
}
