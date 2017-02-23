package us.ihmc.geometry.polytope;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;

public interface ExpandingPolytopeAlgorithmListener
{
   public abstract void setPolytopes(SimplexPolytope simplex, SupportingVertexHolder polytopeOne, SupportingVertexHolder polytopeTwo, ExpandingPolytopeEntry triangleEntry);

   public abstract void polledEntryToExpand(ExpandingPolytopeEntry triangleEntryToExpand);

   public abstract void computedSupportingVertices(Point3D supportingVertexA, Point3D supportingVertexB, Vector3D w);

   public abstract void computedCloseEnough(double vDotW, double lengthSquared, double mu, boolean closeEnough);

   public abstract void computedSilhouetteFromW(ExpandingPolytopeEdgeList edgeList);

   public abstract void addedNewEntryToQueue(ExpandingPolytopeEntry newEntry);

   public abstract void createdNewEntry(ExpandingPolytopeEntry newEntry);

   public abstract void expandedPolytope(ExpandingPolytopeEntry firstNewEntry);

   public abstract void foundMinimumPenetrationVector(Vector3D minimumPenetrationVector, Point3D closestPointOnA, Point3D closestPointOnB);
}
