package us.ihmc.geometry.polytope;

import javax.vecmath.Point3d;

public class ConvexPolytopeConstructor
{

   public static ConvexPolytope constructUnitCube()
   {
      ConvexPolytope polytope = new ConvexPolytope();
      polytope.addVertex(0.0, 0.0, 0.0);
      polytope.addVertex(1.0, 0.0, 0.0);
      polytope.addVertex(1.0, 1.0, 0.0);
      polytope.addVertex(0.0, 1.0, 0.0);

      polytope.addVertex(new Point3d(0.0, 0.0, 1.0));
      polytope.addVertex(new Point3d(1.0, 0.0, 1.0));
      polytope.addVertex(new Point3d(1.0, 1.0, 1.0));
      polytope.addVertex(new Point3d(0.0, 1.0, 1.0));

      return polytope;
   }
}
