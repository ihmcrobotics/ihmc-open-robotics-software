package us.ihmc.geometry.polytope;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

public class GilbertJohnsonKeerthiCollisionDetector
{

   private final Vector3d negativeSupportDirection = new Vector3d();

   public void computeSupportPointOnMinkowskiDifference(ConvexPolytope cubeOne, ConvexPolytope cubeTwo, Vector3d supportDirection, Point3d supportPoint)
   {
      // Because everything is linear and convex, the support point on the Minkowski difference is s_{a minkowskidiff b}(d) = s_a(d) - s_b(-d)

      PolytopeVertex supportingVertexOne = cubeOne.getSupportingVertex(supportDirection);

      negativeSupportDirection.set(supportDirection);
      negativeSupportDirection.scale(-1.0);

      PolytopeVertex supportingVertexTwo = cubeTwo.getSupportingVertex(negativeSupportDirection);

      supportPoint.set(supportingVertexOne.getPosition());
      supportPoint.sub(supportingVertexTwo.getPosition());
   }

}
