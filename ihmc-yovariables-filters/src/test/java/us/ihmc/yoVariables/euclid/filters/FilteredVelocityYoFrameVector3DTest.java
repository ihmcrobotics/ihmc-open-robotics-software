package us.ihmc.yoVariables.euclid.filters;

import java.util.Random;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.yoVariables.filters.FilteredFiniteDifferenceYoVariable;
import us.ihmc.yoVariables.registry.YoRegistry;

public class FilteredVelocityYoFrameVector3DTest
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
         double dt = EuclidCoreRandomTools.nextDouble(random, 1.0e-7, 1.0e-1);
         double alpha = random.nextDouble();
         YoRegistry registry = new YoRegistry("blop");

         FilteredFiniteDifferenceYoFrameVector3D filteredPoint = new FilteredFiniteDifferenceYoFrameVector3D("tested", "", () -> alpha, dt, registry, ReferenceFrame.getWorldFrame());
         FilteredFiniteDifferenceYoVariable xFiltered = new FilteredFiniteDifferenceYoVariable("xRef", "", alpha, dt, registry);
         FilteredFiniteDifferenceYoVariable yFiltered = new FilteredFiniteDifferenceYoVariable("yRef", "", alpha, dt, registry);
         FilteredFiniteDifferenceYoVariable zFiltered = new FilteredFiniteDifferenceYoVariable("zRef", "", alpha, dt, registry);

         Point3D unfilteredPoint = new Point3D();

         for (int j = 0; j < 10; j++)
         {
            unfilteredPoint.scaleAdd(dt, EuclidCoreRandomTools.nextPoint3D(random), unfilteredPoint);

            filteredPoint.update(unfilteredPoint);
            xFiltered.update(unfilteredPoint.getX());
            yFiltered.update(unfilteredPoint.getY());
            zFiltered.update(unfilteredPoint.getZ());

            EuclidCoreTestTools.assertEquals(new Point3D(xFiltered.getValue(), yFiltered.getValue(), zFiltered.getValue()), filteredPoint, EPSILON);
         }
      }
   }
}
