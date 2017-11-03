package us.ihmc.geometry.polytope.DCELPolytope.Basics;

import java.util.List;

import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public interface PolytopeVertexReadOnly extends Point3DReadOnly, EpsilonComparable<PolytopeVertexReadOnly>
{
   Point3DReadOnly getPosition();

   List<? extends PolytopeHalfEdgeReadOnly> getAssociatedEdges();

   PolytopeHalfEdgeReadOnly getAssociatedEdge(int index);

   boolean isAssociatedWithEdge(PolytopeHalfEdgeReadOnly edgeToCheck);

   boolean isAssociatedWithEdge(PolytopeHalfEdgeReadOnly edgeToCheck, double epsilon);

   int getNumberOfAssociatedEdges();

   double dot(Vector3DReadOnly vector);

   String toString();

   boolean isAnyFaceMarked();

   boolean containsNaN();

}