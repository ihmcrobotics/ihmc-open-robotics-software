package us.ihmc.geometry.polytope;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

public interface GilbertJohnsonKeerthiCollisionDetectorListener
{
   public abstract void addedVertexToSimplex(SimplexPolytope simplex, Point3d vertexOnSimplex, Point3d vertexOnA, Point3d vertexOnB);

   public abstract void foundClosestPointOnSimplex(SimplexPolytope simplex, Point3d closestPointToOrigin);

   public abstract void foundCollision(SimplexPolytope simplex, Point3d pointOnAToPack, Point3d pointOnBToPack);

   public abstract void foundSupportPoints(SimplexPolytope simplex, Point3d supportingPointOnA, Point3d supportingPointOnB, Vector3d supportPointOnSimplex);

   public abstract void computeVDotPAndPercentCloser(double vDotP, double percentCloser);

   public abstract void metStoppingConditionForNoIntersection(double vDotP, double percentCloser, Point3d pointOnAToPack, Point3d pointOnBToPack);

   public abstract void tooManyIterationsStopping(SimplexPolytope simplex, Point3d pointOnAToPack, Point3d pointOnBToPack);

   public abstract void metStoppingConditionForNoIntersection(Point3d pointOnAToPack, Point3d pointOnBToPack);

   public abstract void checkingIfPolytopesAreColliding(SupportingVertexHolder polytopeA, SupportingVertexHolder polytopeB);
}
