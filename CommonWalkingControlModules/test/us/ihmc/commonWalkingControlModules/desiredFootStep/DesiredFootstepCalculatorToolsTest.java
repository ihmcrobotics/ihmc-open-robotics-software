package us.ihmc.commonWalkingControlModules.desiredFootStep;

import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
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
      List<FramePoint> input = new ArrayList<FramePoint>();
      Random random = new Random(1245L);

      int nTests = 100;
      for (int i = 0; i < nTests; i++)
      {
         int nPointsIn = 4;
         double maxLength = 5.0;
         for (int j = 0; j < nPointsIn; j++)
         {
            input.add(new FramePoint(worldFrame, RandomTools.generateRandomVector(random, maxLength)));
         }

         FrameVector minusYDirection = new FrameVector(worldFrame, 0.0, -1.0, 0.0);
         int nPointsOut = random.nextInt(nPointsIn);
         List<FramePoint> outputX = DesiredFootstepCalculatorTools.computeMaximumPointsInDirection(input, minusYDirection, nPointsOut);

         List<FramePoint> notIncludedX = new ArrayList<FramePoint>(input);
         notIncludedX.removeAll(outputX);

         for (FramePoint notIncludedPoint : notIncludedX)
         {
            for (FramePoint includedPoint : outputX)
            {
               assertTrue(notIncludedPoint.getY() > includedPoint.getY());
            }
         }
      }
   }
}
