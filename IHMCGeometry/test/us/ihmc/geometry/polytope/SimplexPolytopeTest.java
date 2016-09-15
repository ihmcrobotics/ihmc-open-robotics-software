package us.ihmc.geometry.polytope;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.tools.testing.JUnitTools;

public class SimplexPolytopeTest
{

   @Test
   public void testSimplex()
   {
      SimplexPolytope simplex = new SimplexPolytope();

      Point3d pointOne = new Point3d(1.0, 2.0, 3.0);
      simplex.addPoint(pointOne);

      Point3d closestPointToOrigin = new Point3d();
      simplex.getClosestPointToOriginOnConvexHull(closestPointToOrigin);

      JUnitTools.assertPoint3dEquals("", new Point3d(1.0, 2.0, 3.0), closestPointToOrigin, 1e-7);

      Point3d pointTwo = new Point3d(1.0, 2.0, 4.0);
      simplex.addPoint(pointTwo);

      simplex.getClosestPointToOriginOnConvexHull(closestPointToOrigin);
      JUnitTools.assertPoint3dEquals("", new Point3d(1.0, 2.0, 3.0), closestPointToOrigin, 1e-7);

      simplex.removePoint(pointTwo);
      pointTwo = new Point3d(1.0, 2.0, 2.5);
      simplex.addPoint(pointTwo);

      simplex.getClosestPointToOriginOnConvexHull(closestPointToOrigin);
      JUnitTools.assertPoint3dEquals("", new Point3d(1.0, 2.0, 2.5), closestPointToOrigin, 1e-7);


      simplex.removePoint(pointTwo);
      pointTwo = new Point3d(1.0, 2.0, -12.9);
      simplex.addPoint(pointTwo);

      simplex.getClosestPointToOriginOnConvexHull(closestPointToOrigin);
      JUnitTools.assertPoint3dEquals("", new Point3d(1.0, 2.0, 0.0), closestPointToOrigin, 1e-7);



   }

}
