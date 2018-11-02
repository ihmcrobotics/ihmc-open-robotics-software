package us.ihmc.robotics.screwTheory;

import static org.junit.Assert.assertEquals;

import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.spatial.SpatialForce;

/**
 * Created by nathan on 8/7/15.
 */
public class SpatialForceVectorTest
{
   //TODO

   public static void assertSpatialForceVectorEquals(SpatialForce vector1, SpatialForce vector2, double epsilon)
   {
      assertEquals(vector1.getReferenceFrame(), vector2.getReferenceFrame());
      EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(vector1.getAngularPart()), new Vector3D(vector2.getAngularPart()), epsilon);
      EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(vector1.getLinearPart()), new Vector3D(vector2.getLinearPart()), epsilon);
   }
}
