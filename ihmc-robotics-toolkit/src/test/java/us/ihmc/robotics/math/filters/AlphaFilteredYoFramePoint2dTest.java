package us.ihmc.robotics.math.filters;

import java.util.Random;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.yoVariables.registry.YoRegistry;

public class AlphaFilteredYoFramePoint2dTest
{
   private static final double EPSILON = 1.0e-15;

   @AfterEach
   public void tearDown()
   {
      ReferenceFrameTools.clearWorldFrameTree();
   }

   @Test
   public void testConsistencyWithAlphaFilteredYoVariable()
   {
      Random random = new Random(3453456);

      for (int i = 0; i < 100; i++)
      {
         double alpha = random.nextDouble();
         YoRegistry registry = new YoRegistry("blop");

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
