package us.ihmc.robotics.math.filters;

import java.util.Random;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class AlphaFilteredYoFramePoint2dTest
{
   private static final double EPSILON = 1.0e-15;

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 1000)
   public void testConsistencyWithAlphaFilteredYoVariable()
   {
      Random random = new Random(3453456);

      for (int i = 0; i < 100; i++)
      {
         double alpha = random.nextDouble();
         YoVariableRegistry registry = new YoVariableRegistry("blop");

         AlphaFilteredYoFramePoint2d filteredPoint = new AlphaFilteredYoFramePoint2d("tested", "", registry, alpha, ReferenceFrame.getWorldFrame());
         AlphaFilteredYoVariable xFiltered = new AlphaFilteredYoVariable("xRef", registry, alpha);
         AlphaFilteredYoVariable yFiltered = new AlphaFilteredYoVariable("yRef", registry, alpha);

         Point2D unfilteredPoint = new Point2D();

         for (int j = 0; j < 10; j++)
         {
            unfilteredPoint.add(EuclidCoreRandomTools.nextPoint2D(random, 0.0, 0.5));

            filteredPoint.update(unfilteredPoint);
            xFiltered.update(unfilteredPoint.getX());
            yFiltered.update(unfilteredPoint.getY());

            EuclidCoreTestTools.assertTuple2DEquals(new Point2D(xFiltered.getValue(), yFiltered.getValue()), filteredPoint, EPSILON);
         }
      }
   }
}
