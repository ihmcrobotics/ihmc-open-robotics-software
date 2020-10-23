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
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.yoVariables.registry.YoRegistry;

public class AlphaFilteredYoFrameVectorTest
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

         AlphaFilteredYoFrameVector filteredVector = new AlphaFilteredYoFrameVector("tested", "", registry, alpha, ReferenceFrame.getWorldFrame());
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

            EuclidCoreTestTools.assertTuple3DEquals(new Point3D(xFiltered.getValue(), yFiltered.getValue(), zFiltered.getValue()), filteredVector, EPSILON);
         }
      }
   }
}
