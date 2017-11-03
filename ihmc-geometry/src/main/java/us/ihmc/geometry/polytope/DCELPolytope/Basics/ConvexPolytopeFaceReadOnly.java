package us.ihmc.geometry.polytope.DCELPolytope.Basics;

import java.util.List;

import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;

public interface ConvexPolytopeFaceReadOnly extends EpsilonComparable<ConvexPolytopeFaceReadOnly>
{
   List<? extends PolytopeHalfEdgeReadOnly> getEdgeList();

   PolytopeHalfEdgeReadOnly getEdge(int index);

   PolytopeHalfEdgeReadOnly getFirstVisibleEdge(Point3DReadOnly vertex);

   boolean isPointOnInteriorSideOfEdgeInternal(Point3DBasics point, int index);

   double getFaceVisibilityProduct(Point3DReadOnly point);

   boolean isPointInFacePlane(Point3DReadOnly vertexToCheck, double epsilon);

   boolean isInteriorPoint(Point3DReadOnly vertexToCheck);

   Point3D getFaceCentroid();

   Vector3D getFaceNormal();

   int getNumberOfEdges();

   boolean epsilonEquals(ConvexPolytopeFaceReadOnly other, double epsilon);

   boolean containsNaN();

   double dotFaceNormal(Vector3DBasics direction);

   boolean isFaceVisible(Point3DReadOnly point, double epsilon);

   double getMaxElement(int index);

   double getMinElement(int index);

   double getMaxX();

   double getMaxY();

   double getMaxZ();

   double getMinX();

   double getMinY();

   double getMinZ();

   ConvexPolytopeFaceReadOnly getNeighbouringFace(int index);

   boolean isMarked();

   boolean isNotMarked();

   String toString();

   double getShortestDistanceTo(Point3DReadOnly point);

   PolytopeHalfEdgeReadOnly getEdgeClosestTo(Point3DReadOnly point);

}