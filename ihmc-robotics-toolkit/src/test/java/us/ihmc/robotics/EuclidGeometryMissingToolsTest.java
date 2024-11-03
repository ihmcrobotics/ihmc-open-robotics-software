package us.ihmc.robotics;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import static org.junit.jupiter.api.Assertions.*;

public class EuclidGeometryMissingToolsTest
{
   @Test
   public void testGetZOnPlane()
   {
      Point3D point = new Point3D(1.0, 2.0, -3.0);
      Vector3D normal = new Vector3D(0.2, 1.7, 0.4);
      Plane3D plane = new Plane3D(point, normal);

      double x = 2.33;
      double y = 1.97;

      double z = EuclidGeometryMissingTools.getZOnPlane(point, normal, x, y);

      Point3D testPoint = new Point3D(x, y, z);
      assertTrue(plane.distance(testPoint) < 1e-10);

      normal = new Vector3D(0.2, 1.7, 0.0);

      z = EuclidGeometryMissingTools.getZOnPlane(point, normal, x, y);
      assertTrue(Double.isNaN(z));
   }
}
