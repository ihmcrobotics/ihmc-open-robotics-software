package us.ihmc.geometry.polytope.DCELPolytope.Basics;

import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public interface PolytopeHalfEdgeReadOnly extends Vector3DReadOnly, EpsilonComparable<PolytopeHalfEdgeReadOnly>
{
   PolytopeVertexReadOnly getOriginVertex();

   PolytopeVertexReadOnly getDestinationVertex();

   PolytopeHalfEdgeReadOnly getTwinHalfEdge();

   PolytopeHalfEdgeReadOnly getNextHalfEdge();

   PolytopeHalfEdgeReadOnly getPreviousHalfEdge();

   ConvexPolytopeFaceReadOnly getFace();

   Vector3DReadOnly getEdgeVector();

   Vector3DReadOnly getNormalizedEdgeVector();

   boolean isTwin(PolytopeHalfEdgeReadOnly twinEdge, double epsilon);

   boolean containsNaN();

   String toString();

   double getShortestDistanceTo(Point3DReadOnly point);

}