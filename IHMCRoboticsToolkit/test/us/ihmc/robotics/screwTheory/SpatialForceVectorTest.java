package us.ihmc.robotics.screwTheory;

import us.ihmc.tools.test.JUnitTools;

import static org.junit.Assert.assertEquals;

/**
 * Created by nathan on 8/7/15.
 */
public class SpatialForceVectorTest
{
   //TODO

   public static void assertSpatialForceVectorEquals(SpatialForceVector vector1, SpatialForceVector vector2, double epsilon)
   {
      assertEquals(vector1.getExpressedInFrame(), vector2.getExpressedInFrame());
      JUnitTools.assertTuple3dEquals(vector1.getAngularPartCopy(), vector2.getAngularPartCopy(), epsilon);
      JUnitTools.assertTuple3dEquals(vector1.getLinearPartCopy(), vector2.getLinearPartCopy(), epsilon);
   }
}
