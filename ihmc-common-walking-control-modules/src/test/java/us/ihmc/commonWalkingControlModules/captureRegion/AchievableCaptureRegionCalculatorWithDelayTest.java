package us.ihmc.commonWalkingControlModules.captureRegion;

import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.capturePoint.CapturePointTools;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;

import java.util.Random;

public class AchievableCaptureRegionCalculatorWithDelayTest
{
   private static final int iter = 500;

   @Test
   public void testComputeCoPLocationToCapture()
   {
      Random random = new Random(1738L);

      double omega = 3.0;

      for (int i = 0; i < iter; i++)
      {
         FramePoint2D initialCoP = EuclidFrameRandomTools.nextFramePoint2D(random, ReferenceFrame.getWorldFrame());
         FramePoint2D initialICP = EuclidFrameRandomTools.nextFramePoint2D(random, ReferenceFrame.getWorldFrame());

         FramePoint2D capturingCoP = new FramePoint2D();

         double duration = RandomNumbers.nextDouble(random, 0.05, 2.0);

         AchievableCaptureRegionCalculatorWithDelay.computeCoPLocationToCapture(initialICP, initialCoP, omega, duration, capturingCoP);

         FramePoint2D outputICP = new FramePoint2D();
         CapturePointTools.computeDesiredCapturePointPosition(omega, duration, duration, initialICP, initialCoP, capturingCoP, outputICP);

         EuclidFrameTestTools.assertFramePoint2DGeometricallyEquals(outputICP, capturingCoP, 1e-2);
      }
   }

}
