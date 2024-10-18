package us.ihmc.yoVariables.euclid.filters;

import java.util.Random;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tools.EuclidCoreMissingRandomTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.yoVariables.filters.AlphaFilteredYoVariable;
import us.ihmc.yoVariables.registry.YoRegistry;

public class AlphaFilteredYoFrameVector3DTest
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

         AlphaFilteredYoFrameVector3D filteredVector = new AlphaFilteredYoFrameVector3D("tested", "", registry, alpha, ReferenceFrame.getWorldFrame());
         AlphaFilteredYoVariable xFiltered = new AlphaFilteredYoVariable("xRef", registry, alpha);
         AlphaFilteredYoVariable yFiltered = new AlphaFilteredYoVariable("yRef", registry, alpha);
         AlphaFilteredYoVariable zFiltered = new AlphaFilteredYoVariable("zRef", registry, alpha);

         Vector3D unfilteredVector = new Vector3D();

         for (int j = 0; j < 10; j++)
         {
            unfilteredVector.add(EuclidCoreRandomTools.nextPoint3D(random, 0.0, 0.5));

            filteredVector.update(unfilteredVector);
            xFiltered.update(unfilteredVector.getX());
            yFiltered.update(unfilteredVector.getY());
            zFiltered.update(unfilteredVector.getZ());

            EuclidCoreTestTools.assertEquals(new Point3D(xFiltered.getValue(), yFiltered.getValue(), zFiltered.getValue()), filteredVector, EPSILON);
         }
      }
   }
}
