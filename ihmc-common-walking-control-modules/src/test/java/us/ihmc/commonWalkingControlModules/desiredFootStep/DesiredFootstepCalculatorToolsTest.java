package us.ihmc.commonWalkingControlModules.desiredFootStep;

import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.tools.MemoryTools;

public class DesiredFootstepCalculatorToolsTest
{
   @Before
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @After
   public void showMemoryUsageAfterTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.4)
   @Test(timeout = 30000)
   public void testComputeMaximumPoints()
   {
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      List<FramePoint3D> input = new ArrayList<FramePoint3D>();
      Random random = new Random(1245L);

      int nTests = 100;
      for (int i = 0; i < nTests; i++)
      {
         int nPointsIn = 4;
         double maxLength = 5.0;
         for (int j = 0; j < nPointsIn; j++)
         {
            input.add(new FramePoint3D(worldFrame, RandomGeometry.nextVector3D(random, maxLength)));
         }

         FrameVector3D minusYDirection = new FrameVector3D(worldFrame, 0.0, -1.0, 0.0);
         int nPointsOut = random.nextInt(nPointsIn);
         List<FramePoint3D> outputX = DesiredFootstepCalculatorTools.computeMaximumPointsInDirection(input, minusYDirection, nPointsOut);

         List<FramePoint3D> notIncludedX = new ArrayList<FramePoint3D>(input);
         notIncludedX.removeAll(outputX);

         for (FramePoint3D notIncludedPoint : notIncludedX)
         {
            for (FramePoint3D includedPoint : outputX)
            {
               assertTrue(notIncludedPoint.getY() > includedPoint.getY());
            }
         }
      }
   }
}
