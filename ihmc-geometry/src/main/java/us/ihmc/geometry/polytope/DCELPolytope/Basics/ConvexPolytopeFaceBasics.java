package us.ihmc.geometry.polytope.DCELPolytope.Basics;

import java.util.ArrayList;
import java.util.List;

import javax.crypto.CipherInputStream;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commons.Epsilons;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.interfaces.Clearable;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.geometry.polytope.SupportingVertexHolder;
import us.ihmc.geometry.polytope.DCELPolytope.Providers.PolytopeHalfEdgeProvider;
import us.ihmc.robotics.MathTools;

public abstract class ConvexPolytopeFaceBasics<A extends PolytopeVertexBasics<A, B, C>, B extends PolytopeHalfEdgeBasics<A, B, C>, C extends ConvexPolytopeFaceBasics<A, B, C>>
      implements SimplexBasics, SupportingVertexHolder, ConvexPolytopeFaceReadOnly, Clearable, Settable<ConvexPolytopeFaceReadOnly>, Transformable
{
   private final double EPSILON = Epsilons.ONE_MILLIONTH;

   /**
    * Ordered list of half edges that bound the face
    */
   private final ArrayList<B> edges = new ArrayList<>();
   /**
    * Do not access directly since this is updated only when the getter is called
    */
   private final Vector3D faceNormal = new Vector3D();
   /**
    * Do not access directly since this is updated only when the getter is called
    */
   private final Point3D faceCentroid = new Point3D();
   /**
    * 
    */

   // Temporary variables for calculations
   private final Vector3D tempVector = new Vector3D();
   private final Point3D tempPoint = new Point3D();
   private final ArrayList<B> visibleEdgeList = new ArrayList<>();
   private boolean marked = false;

   /**
    * Default constructor. Does not initialize anything
    */
   public ConvexPolytopeFaceBasics()
   {

   }

   public ConvexPolytopeFaceBasics(C other)
   {
      this(other.getEdgeList());
   }

   public ConvexPolytopeFaceBasics(List<B> edgeList)
   {
      this.copyEdgeList(edgeList);
   }

   public ConvexPolytopeFaceBasics(PolytopeHalfEdgeReadOnly[] edgeListArray)
   {
      this.copyEdgeList(edgeListArray);
   }

   public void copyEdgeList(List<? extends PolytopeHalfEdgeReadOnly> edgeList)
   {
      this.edges.clear();
      for (int i = 0; i < edgeList.size(); i++)
         this.edges.add(getHalfEdgeProvider().getHalfEdge(edgeList.get(i)));
   }

   public void copyEdgeList(PolytopeHalfEdgeReadOnly[] edgeListArray)
   {
      edges.clear();
      for (int i = 0; i < edgeListArray.length; i++)
         this.edges.add(getHalfEdgeProvider().getHalfEdge(edgeListArray[i]));
   }

   public List<B> getEdgeList()
   {
      return edges;
   }

   public B getEdge(int index)
   {
      return edges.get(index);
   }

   public void addVertex(A vertexToAdd)
   {
      switch (edges.size())
      {
      case 0:
      {
         B newEdge = getHalfEdgeProvider().getHalfEdge(vertexToAdd, vertexToAdd);
         newEdge.setFace((C) this);
         newEdge.setNextHalfEdge(newEdge);
         newEdge.setPreviousHalfEdge(newEdge);
         edges.add(newEdge);
         break;
      }
      case 1:
      {
         // Set the edge for the two points and then create its twin
         edges.get(0).setDestinationVertex(vertexToAdd);
         B newEdge = getHalfEdgeProvider().getHalfEdge(vertexToAdd, edges.get(0).getOriginVertex());
         newEdge.setFace((C) this);
         newEdge.setNextHalfEdge(edges.get(0));
         newEdge.setPreviousHalfEdge(edges.get(0));
         edges.get(0).setNextHalfEdge(newEdge);
         edges.get(0).setPreviousHalfEdge(newEdge);
         edges.add(newEdge);
         break;
      }
      case 2:
      {
         // Create a new edge and assign an arbitrary configuration since there is no way to tell up and down in 3D space
         edges.get(1).setDestinationVertex(vertexToAdd);
         B newEdge = getHalfEdgeProvider().getHalfEdge(vertexToAdd, edges.get(0).getOriginVertex());
         newEdge.setFace((C) this);
         edges.add(newEdge);
         newEdge.setNextHalfEdge(edges.get(0));
         edges.get(0).setPreviousHalfEdge(newEdge);
         newEdge.setPreviousHalfEdge(edges.get(1));
         edges.get(1).setNextHalfEdge(newEdge);
         break;
      }
      default:
      {
         // Now a ordering is available and all new vertices to add must be done accordingly. Also points must lie in the same plane
         if (!isPointInFacePlane(vertexToAdd, EPSILON))
            return;

         getVisibleEdgeList(vertexToAdd, visibleEdgeList);
         switch (visibleEdgeList.size())
         {
         case 0:
            return; // Case where the point is internal
         case 1:
            B additionalEdge = getHalfEdgeProvider().getHalfEdge(vertexToAdd, visibleEdgeList.get(0).getDestinationVertex());
            additionalEdge.setFace((C) this);
            visibleEdgeList.get(0).setDestinationVertex(vertexToAdd);
            additionalEdge.setNextHalfEdge(visibleEdgeList.get(0).getNextHalfEdge());
            visibleEdgeList.get(0).getNextHalfEdge().setPreviousHalfEdge(additionalEdge);
            visibleEdgeList.get(0).setNextHalfEdge(additionalEdge);
            additionalEdge.setPreviousHalfEdge(visibleEdgeList.get(0));
            edges.add(additionalEdge);
            break;
         default:
            visibleEdgeList.get(0).setDestinationVertex(vertexToAdd);
            visibleEdgeList.get(visibleEdgeList.size() - 1).setOriginVertex(vertexToAdd);
            visibleEdgeList.get(0).setNextHalfEdge(visibleEdgeList.get(visibleEdgeList.size() - 1));
            visibleEdgeList.get(visibleEdgeList.size() - 1).setPreviousHalfEdge(visibleEdgeList.get(0));
            for (int i = 1; i < visibleEdgeList.size() - 1; i++)
               edges.remove(visibleEdgeList.get(i));
            break;
         }
         break;
      }
      }
   }

   /**
    * You have been given power. Do not abuse it (just ensure that the additions are all consistent)
    * @param newEdge
    */
   public void addEdge(B newEdge)
   {
      edges.add(newEdge);
   }

   public void getVisibleEdgeList(Point3DReadOnly vertex, List<B> edgeList)
   {
      edgeList.clear();
      B edgeUnderConsideration = getFirstVisibleEdge(vertex);
      for (int i = 0; edgeUnderConsideration != null && i < edges.size(); i++)
      {
         edgeList.add(edgeUnderConsideration);
         edgeUnderConsideration = edgeUnderConsideration.getNextHalfEdge();
         if (isPointOnInteriorSideOfEdgeInternal(vertex, edgeUnderConsideration))
            break;
      }
   }

   public B getFirstVisibleEdge(Point3DReadOnly vertex)
   {
      if (edges.size() == 0)
         return null;
      else if (edges.size() == 1 || edges.size() == 2)
         return edges.get(0);

      B edgeUnderConsideration = edges.get(0);
      double previousDotProduct = Double.NaN;
      for (int i = 0; i < getNumberOfEdges() + 1; i++)
      {
         double dotProduct = getEdgeVisibilityProduct(vertex, edgeUnderConsideration);
         if (dotProduct <= 0)
            edgeUnderConsideration = edgeUnderConsideration.getNextHalfEdge();
         else
            edgeUnderConsideration = edgeUnderConsideration.getPreviousHalfEdge();

         if (previousDotProduct * dotProduct <= 0)
         {
            if (previousDotProduct < 0)
               edgeUnderConsideration = edgeUnderConsideration.getNextHalfEdge();
            return edgeUnderConsideration;
         }
         else
            previousDotProduct = dotProduct;
      }
      PrintTools.debug("Reaching here - this is never good");
      return null;
   }

   public boolean isPointOnInteriorSideOfEdgeInternal(Point3DBasics point, int index)
   {
      updateFaceNormal();
      return isPointOnInteriorSideOfEdgeInternal(point, edges.get(index));
   }

   private boolean isPointOnInteriorSideOfEdgeInternal(Point3DReadOnly point, B halfEdge)
   {
      return getEdgeVisibilityProduct(point, halfEdge) < 0;
   }

   public double getFaceVisibilityProduct(Point3DReadOnly point)
   {
      tempVector.sub(point, getEdge(0).getOriginVertex());
      return dotFaceNormal(tempVector);
   }

   private double getEdgeVisibilityProduct(Point3DReadOnly point, B halfEdge)
   {
      tempVector.sub(point, halfEdge.getOriginVertex());
      tempVector.cross(halfEdge.getEdgeVector());
      return tempVector.dot(getFaceNormal());
   }

   public boolean isPointInFacePlane(Point3DReadOnly vertexToCheck, double epsilon)
   {
      boolean isInFacePlane;
      tempVector.sub(vertexToCheck, edges.get(0).getOriginVertex());
      if (edges.size() < 3)
         isInFacePlane = !MathTools.epsilonEquals(Math.abs(edges.get(0).getEdgeVector().dot(tempVector))
               / Math.sqrt(edges.get(0).getEdgeVector().dot(edges.get(0).getEdgeVector()) * tempVector.dot(tempVector)), 1.0, epsilon);
      else
         isInFacePlane = MathTools.epsilonEquals(tempVector.dot(getFaceNormal()), 0.0, epsilon);
      return isInFacePlane;
   }

   public boolean isInteriorPoint(Point3DReadOnly vertexToCheck)
   {
      return (isPointInFacePlane(vertexToCheck, EPSILON) && isInteriorPointInternal(vertexToCheck));
   }

   private boolean isInteriorPointInternal(Point3DReadOnly vertexToCheck)
   {
      if (edges.size() < 3)
         return false;

      boolean result = true;
      B halfEdge = edges.get(0);
      for (int i = 0; result && i < edges.size(); i++)
      {
         result &= isPointOnInteriorSideOfEdgeInternal(vertexToCheck, halfEdge);
         halfEdge = halfEdge.getNextHalfEdge();
      }
      return result;
   }

   public Point3D getFaceCentroid()
   {
      updateFaceCentroid();
      return faceCentroid;
   }

   private void updateFaceCentroid()
   {
      faceCentroid.setToZero();
      for (int i = 0; i < edges.size(); i++)
         faceCentroid.add(edges.get(i).getOriginVertex());
      faceCentroid.scale(1.0 / edges.size());
   }

   public Vector3D getFaceNormal()
   {
      updateFaceNormal();
      return faceNormal;
   }

   private void updateFaceNormal()
   {
      if (edges.size() < 3)
         faceNormal.setToZero();
      else
      {
         faceNormal.cross(edges.get(0).getEdgeVector(), edges.get(0).getNextHalfEdge().getEdgeVector());
         if (faceNormal.dot(faceNormal) > EPSILON)
            faceNormal.normalize();
      }
   }

   public int getNumberOfEdges()
   {
      return edges.size();
   }

   @Override
   public void applyTransform(Transform transform)
   {
      for (int i = 0; i < getNumberOfEdges(); i++)
         edges.get(i).getOriginVertex().applyTransform(transform);
   }

   @Override
   public void applyInverseTransform(Transform transform)
   {
      for (int i = 0; i < getNumberOfEdges(); i++)
         edges.get(i).getOriginVertex().applyInverseTransform(transform);
   }

   @Override
   public boolean epsilonEquals(ConvexPolytopeFaceReadOnly other, double epsilon)
   {
      if (other.getNumberOfEdges() == this.getNumberOfEdges())
      {
         int index = findMatchingEdgeIndex(other.getEdge(0), epsilon);
         if (index != -1)
         {
            boolean result = true;
            B matchedEdge = edges.get(index);
            PolytopeHalfEdgeReadOnly candidateEdge = other.getEdge(0);
            for (int i = 0; result && i < edges.size() - 1; i++)
            {
               matchedEdge = matchedEdge.getNextHalfEdge();
               candidateEdge = candidateEdge.getNextHalfEdge();
               result &= matchedEdge.epsilonEquals(candidateEdge, epsilon);
            }
            return result;
         }
         else
            return false;
      }
      else
         return false;
   }

   public int findMatchingEdgeIndex(PolytopeHalfEdgeReadOnly edgeToSearch, double epsilon)
   {
      for (int i = 0; i < edges.size(); i++)
      {
         if (edges.get(i).epsilonEquals(edgeToSearch, epsilon))
            return i;
      }
      return -1;
   }

   public PolytopeHalfEdgeReadOnly findMatchingEdge(PolytopeHalfEdgeReadOnly edgeToSearch, double epsilon)
   {
      return edges.get(findMatchingEdgeIndex(edgeToSearch, epsilon));
   }

   public void reverseFaceNormal()
   {
      for (int i = 0; i < edges.size(); i++)
      {
         edges.get(i).reverseEdge();
      }
      updateFaceNormal();
   }

   @Override
   public void set(ConvexPolytopeFaceReadOnly other)
   {
      clearEdgeList();
      copyEdgeList(other.getEdgeList());
   }

   public void clearEdgeList()
   {
      this.edges.clear();
   }

   @Override
   public boolean containsNaN()
   {
      boolean result = (edges.size() > 0 && edges.get(0).containsNaN());
      for (int i = 1; !result && i < edges.size(); i++)
         result |= edges.get(i).getDestinationVertex().containsNaN();
      return result;
   }

   @Override
   public void setToNaN()
   {
      for (int i = 0; i < edges.size(); i++)
         edges.get(i).setToNaN();
   }

   public double dotFaceNormal(Vector3DBasics direction)
   {
      updateFaceNormal();
      return direction.dot(faceNormal);
   }

   public boolean isFaceVisible(Point3DReadOnly point, double epsilon)
   {
      return getFaceVisibilityProduct(point) > epsilon;
   }

   @Override
   public void setToZero()
   {
      for (int i = 0; i < edges.size(); i++)
         edges.get(i).setToZero();
   }

   public double getMaxElement(int index)
   {
      B edgeReference = edges.get(0);
      double maxElement = edgeReference.getOriginVertex().getElement(index);
      for (int i = 0; i < edges.size(); i++)
      {
         if (maxElement < edgeReference.getDestinationVertex().getElement(index))
            maxElement = edgeReference.getDestinationVertex().getElement(index);
         edgeReference = edgeReference.getNextHalfEdge();
      }
      return maxElement;
   }

   public double getMinElement(int index)
   {
      B edgeReference = edges.get(0);
      double minElement = edgeReference.getOriginVertex().getElement(index);
      for (int i = 0; i < edges.size(); i++)
      {
         if (minElement > edgeReference.getDestinationVertex().getElement(index))
            minElement = edgeReference.getDestinationVertex().getElement(index);
         edgeReference = edgeReference.getNextHalfEdge();
      }
      return minElement;
   }

   public double getMaxX()
   {
      return getMaxElement(0);
   }

   public double getMaxY()
   {
      return getMaxElement(1);
   }

   public double getMaxZ()
   {
      return getMaxElement(2);
   }

   public double getMinX()
   {
      return getMinElement(0);
   }

   public double getMinY()
   {
      return getMinElement(1);
   }

   public double getMinZ()
   {
      return getMinElement(2);
   }

   public C getNeighbouringFace(int index)
   {
      if (index > edges.size() || edges.get(index).getTwinHalfEdge() == null)
         return null;
      else
         return edges.get(index).getTwinHalfEdge().getFace();

   }

   @Override
   public Point3D getSupportingVertex(Vector3D supportVector)
   {
      A bestVertex = edges.get(0).getOriginVertex();
      double maxDot = bestVertex.dot(supportVector);
      A bestVertexCandidate = bestVertex;
      while (true)
      {
         bestVertexCandidate = bestVertex;
         for (int i = 0; i < bestVertex.getNumberOfAssociatedEdges(); i++)
         {
            double dotCandidate = bestVertex.getAssociatedEdges().get(i).getDestinationVertex().dot(supportVector);
            if (maxDot < dotCandidate)
            {
               maxDot = dotCandidate;
               bestVertexCandidate = bestVertex.getAssociatedEdge(i).getDestinationVertex();
            }
         }
         if (bestVertexCandidate == bestVertex)
            return (Point3D) bestVertex.getPosition();
         else
            bestVertex = bestVertexCandidate;
      }
   }

   public void mark()
   {
      this.marked = true;
   }

   public void unmark()
   {
      this.marked = false;
   }

   public boolean isMarked()
   {
      return this.marked;
   }

   public boolean isNotMarked()
   {
      return !this.marked;
   }

   @Override
   public String toString()
   {
      String string = "";
      B edge = edges.get(0);
      for (int i = 0; i < edges.size(); i++)
      {
         string += "\n" + edge.toString() + " Twin: " + (edge.getTwinHalfEdge() == null ? "null" : edge.getTwinHalfEdge().toString());
         edge = edge.getNextHalfEdge();
      }
      return string;
   }

   @Override
   public double getShortestDistanceTo(Point3DReadOnly point)
   {
      EuclidGeometryTools.orthogonalProjectionOnPlane3D(point, edges.get(0).getOriginVertex(), getFaceNormal(), tempPoint);
      if (isInteriorPointInternal(tempPoint))
         return point.distance(tempPoint);
      else
         return getEdgeClosestTo(tempPoint).getShortestDistanceTo(point);
   }

   public B getEdgeClosestTo(Point3DReadOnly point)
   {
      B edge = getFirstVisibleEdge(tempPoint);
      double shortestDistance = edge.getShortestDistanceTo(tempPoint);
      double shortestDistanceCandidate = Double.NEGATIVE_INFINITY;
      while (shortestDistanceCandidate < shortestDistance)
      {
         edge = edge.getNextHalfEdge();
         shortestDistanceCandidate = edge.getShortestDistanceTo(tempPoint);
      }
      return edge.getPreviousHalfEdge();
   }

   @Override
   public void getSupportVectorDirectionTo(Point3DReadOnly point, Vector3D supportVectorToPack)
   {
      EuclidGeometryTools.orthogonalProjectionOnPlane3D(point, edges.get(0).getOriginVertex(), getFaceNormal(), tempPoint);
      if (isInteriorPointInternal(tempPoint))
         supportVectorToPack.set(getFaceNormal());
      else
         getEdgeClosestTo(tempPoint).getSupportVectorDirectionTo(point, supportVectorToPack);
   }

   protected abstract PolytopeHalfEdgeProvider<A, B, C> getHalfEdgeProvider();

   @Override
   public SimplexBasics getSmallestSimplexMemberReference(Point3DReadOnly point)
   {
      EuclidGeometryTools.orthogonalProjectionOnPlane3D(point, edges.get(0).getOriginVertex(), getFaceNormal(), tempPoint);
      if (isInteriorPointInternal(tempPoint))
         return this;
      else
         return getEdgeClosestTo(tempPoint).getSmallestSimplexMemberReference(point);
   }
}
