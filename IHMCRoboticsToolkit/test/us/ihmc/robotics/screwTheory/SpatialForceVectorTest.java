package us.ihmc.robotics.screwTheory;

import static org.junit.Assert.assertEquals;

import us.ihmc.euclid.tools.EuclidCoreTestTools;

/**
 * Created by nathan on 8/7/15.
 */
public class SpatialForceVectorTest
{
   //TODO

   public static void assertSpatialForceVectorEquals(SpatialForceVector vector1, SpatialForceVector vector2, double epsilon)
   {
      assertEquals(vector1.getExpressedInFrame(), vector2.getExpressedInFrame());
      EuclidCoreTestTools.assertTuple3DEquals(vector1.getAngularPartCopy(), vector2.getAngularPartCopy(), epsilon);
      EuclidCoreTestTools.assertTuple3DEquals(vector1.getLinearPartCopy(), vector2.getLinearPartCopy(), epsilon);
   }
}
