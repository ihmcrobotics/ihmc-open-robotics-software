package us.ihmc.geometry.polytope;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

public interface ExpandingPolytopeAlgorithmListener
{

   public abstract void setPolytopes(SimplexPolytope simplex, ConvexPolytope polytopeOne, ConvexPolytope polytopeTwo, ExpandingPolytopeEntry triangleEntry);

   public abstract void polledEntryToExpand(ExpandingPolytopeEntry triangleEntryToExpand);

   public abstract void computedSupportingVertices(PolytopeVertex supportingVertexA, PolytopeVertex supportingVertexB, Vector3d w);

   public abstract void computedCloseEnough(double vDotW, double lengthSquared, double mu, boolean closeEnough);

   public abstract void computedSilhouetteFromW(ExpandingPolytopeEdgeList edgeList);

   public abstract void addedNewEntryToQueue(ExpandingPolytopeEntry newEntry);

   public abstract void createdNewEntry(ExpandingPolytopeEntry newEntry);

   public abstract void expandedPolytope(ExpandingPolytopeEntry firstNewEntry);

   public abstract void foundMinimumPenetrationVector(Vector3d minimumPenetrationVector, Point3d closestPointOnA, Point3d closestPointOnB);
}
