package us.ihmc.geometry.polytope.DCELPolytope.Basics;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.interfaces.Clearable;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.geometry.polytope.DCELPolytope.Providers.PolytopeHalfEdgeProvider;

public abstract class PolytopeHalfEdgeBasics<A extends PolytopeVertexBasics<A, B, C>, B extends PolytopeHalfEdgeBasics<A, B, C>, C extends ConvexPolytopeFaceBasics<A, B, C>>
      implements PolytopeHalfEdgeReadOnly, SimplexBasics, Clearable, Transformable, Settable<B>
{
   private B twinEdge;
   private B nextHalfEdge;
   private B previousHalfEdge;
   private C face;
   private A originVertex;
   private A destinationVertex;
   /**
    * Not recomputed on change of values. Only recomputed when called through its getter
    */
   private Vector3D edgeVector = new Vector3D();
   private Point3D tempPoint = new Point3D();
   
   protected abstract PolytopeHalfEdgeProvider<A, B, C> getHalfEdgeProvider();
   
   public PolytopeHalfEdgeBasics()
   {

   }

   /**
    * Primary constructor for half edge
    * @param originVertex
    * @param destinationVertex
    */
   public PolytopeHalfEdgeBasics(A originVertex, A destinationVertex)
   {
      setOriginVertex(originVertex);
      setDestinationVertex(destinationVertex);
   }
   
   public PolytopeHalfEdgeBasics(B edge)
   {
      set(edge);
   }
   
   public B createTwinHalfEdge()
   {
      B twinEdge = getHalfEdgeProvider().getHalfEdge(getDestinationVertex(), getOriginVertex());
      twinEdge.setTwinHalfEdge((B) this); 
      return twinEdge;
   }
   
   public B setAndCreateTwinHalfEdge()
   {
      B twinEdge = createTwinHalfEdge();
      setTwinHalfEdge(twinEdge);
      return twinEdge;
   }
   
   public void setToTwin(B twinEdge)
   {
      twinEdge.clear();
      twinEdge.setOriginVertex(this.destinationVertex);
      twinEdge.setDestinationVertex(this.originVertex);
      twinEdge.setTwinHalfEdge((B) this);
   }

   public PolytopeHalfEdgeBasics(B twinEdge, C face)
   {
      setTwinHalfEdge(twinEdge);
      setOriginVertex(twinEdge.getDestinationVertex());
      setDestinationVertex(twinEdge.getOriginVertex());
      setFace(face);
   }

   public PolytopeHalfEdgeBasics(A originVertex, A destinationVertex, B twinEdge, B nextHalfEdge, B previousHalfEdge, C face)
   {
      setOriginVertex(originVertex);
      setDestinationVertex(destinationVertex);
      setTwinHalfEdge(twinEdge);
      setNextHalfEdge(nextHalfEdge);
      setPreviousHalfEdge(previousHalfEdge);
      setFace(face);
   }

   public void setOriginVertex(A originVertex)
   {
      if (this.originVertex != null)
         this.originVertex.removeAssociatedEdge((B) this);
      setOriginVertexUnsafe(originVertex);
      if (this.originVertex != null)
         this.originVertex.addAssociatedEdge((B) this);
      updateTwinDestination();
   }

   public void setOriginVertexUnsafe(A originVertex)
   {
      this.originVertex = originVertex;
   }

   private void updateTwinOrigin()
   {
      if (twinEdge != null)
      {
         if (twinEdge.getOriginVertex() != null)
            twinEdge.getOriginVertex().removeAssociatedEdge(twinEdge);
         twinEdge.setOriginVertexUnsafe(this.destinationVertex);
         if (twinEdge.getOriginVertex() != null)
            twinEdge.getOriginVertex().addAssociatedEdge(twinEdge);
      }
   }

   private void updateTwinDestination()
   {
      if (twinEdge != null)
         twinEdge.setDestinationVertexUnsafe(this.originVertex);
   }

   public A getOriginVertex()
   {
      return originVertex;
   }

   public void setDestinationVertex(A destinationVertex)
   {
      this.destinationVertex = destinationVertex;
      updateTwinOrigin();
   }

   public void setDestinationVertexUnsafe(A destinationVertex)
   {
      this.destinationVertex = destinationVertex;
   }

   public A getDestinationVertex()
   {
      return destinationVertex;
   }

   public void setTwinHalfEdge(B twinEdge)
   {
      this.twinEdge = twinEdge;
   }

   public B getTwinHalfEdge()
   {
      return twinEdge;
   }

   public void setNextHalfEdge(B nextHalfEdge)
   {
      if (nextHalfEdge == null || (nextHalfEdge.getOriginVertex() == this.getDestinationVertex() && nextHalfEdge.getFace() == this.getFace()))
         setNextHalfEdgeUnsafe(nextHalfEdge);
      else
         throw new RuntimeException("Mismatch between vertices, destination vertex: " + getDestinationVertex().toString() + " , next edge origin vertex: "
               + nextHalfEdge.getOriginVertex().toString());
   }

   private void setNextHalfEdgeUnsafe(B nextHalfEdge)
   {
      this.nextHalfEdge = nextHalfEdge;
   }

   public B getNextHalfEdge()
   {
      return nextHalfEdge;
   }

   public void setPreviousHalfEdge(B previousHalfEdge)
   {
      if (previousHalfEdge == null || (previousHalfEdge.getDestinationVertex() == this.getOriginVertex() && previousHalfEdge.getFace() == this.getFace()))
         setPreviousHalfEdgeUnsafe(previousHalfEdge);
      else
         throw new RuntimeException("Mismatch between vertices, origin vertex: " + getOriginVertex().toString() + " , previous edge destination vertex: "
               + previousHalfEdge.getDestinationVertex().toString());
   }

   private void setPreviousHalfEdgeUnsafe(B previousHalfEdge)
   {
      this.previousHalfEdge = previousHalfEdge;
   }

   public B getPreviousHalfEdge()
   {
      return previousHalfEdge;
   }

   public void setFace(C face)
   {
      this.face = face;
   }

   public C getFace()
   {
      return face;
   }

   public Vector3DReadOnly getEdgeVector()
   {
      edgeVector.sub(this.destinationVertex.getPosition(), this.originVertex.getPosition());
      return edgeVector;
   }

   public Vector3DReadOnly getNormalizedEdgeVector()
   {
      getEdgeVector();
      edgeVector.normalize();
      return edgeVector;
   }

   @Override
   public void applyTransform(Transform transform)
   {
      originVertex.applyTransform(transform);
      destinationVertex.applyTransform(transform);
   }

   @Override
   public void applyInverseTransform(Transform transform)
   {
      originVertex.applyInverseTransform(transform);
      destinationVertex.applyInverseTransform(transform);
   }

   @Override
   public boolean epsilonEquals(PolytopeHalfEdgeReadOnly other, double epsilon)
   {
      return getOriginVertex().epsilonEquals(other.getOriginVertex(), epsilon) && getDestinationVertex().epsilonEquals(other.getDestinationVertex(), epsilon);
   }

   public boolean isTwin(PolytopeHalfEdgeReadOnly twinEdge, double epsilon)
   {
      return epsilonEquals(twinEdge.getTwinHalfEdge(), epsilon);
   }

   public void set(B other)
   {
      setOriginVertex(other.getOriginVertex());
      setDestinationVertex(other.getDestinationVertex());
      setTwinHalfEdge(other.getTwinHalfEdge());
      setNextHalfEdge(other.getNextHalfEdge());
      setPreviousHalfEdge(other.getPreviousHalfEdge());
      setFace(other.getFace());
   }

   @Override
   public boolean containsNaN()
   {
      return originVertex.containsNaN() || destinationVertex.containsNaN();
   }

   @Override
   public void setToNaN()
   {
      originVertex.setToNaN();
      destinationVertex.setToNaN();
   }

   @Override
   public void setToZero()
   {
      originVertex.setToZero();
      destinationVertex.setToZero();
   }

   public void clear()
   {
      setTwinHalfEdge(null);
      setOriginVertex(null);
      setDestinationVertex(null);
      setNextHalfEdge(null);
      setPreviousHalfEdge(null);
      setFace(null);
   }

   public void reverseEdge()
   {
      A newDestinationVertex = this.originVertex;
      setOriginVertex(destinationVertex);
      setDestinationVertex(newDestinationVertex);
      B newNextHalfEdge = this.previousHalfEdge;
      setPreviousHalfEdgeUnsafe(nextHalfEdge);
      setNextHalfEdgeUnsafe(newNextHalfEdge);
   }

   public String toString()
   {
      return "From: " + ((originVertex == null) ? "null" : originVertex.toString()) + ", To: "
            + ((destinationVertex == null) ? "null" : destinationVertex.toString());
   }

   @Override
   public double getX()
   {
      return getEdgeVector().getX();
   }

   @Override
   public double getY()
   {
      return getEdgeVector().getY();
   }

   @Override
   public double getZ()
   {
      return getEdgeVector().getZ();
   }

   @Override
   public double getShortestDistanceTo(Point3DReadOnly point)
   {
      return EuclidGeometryTools.distanceFromPoint3DToLineSegment3D(point, this.originVertex, this.destinationVertex);
   }

   @Override
   public void getSupportVectorDirectionTo(Point3DReadOnly point, Vector3D supportVectorToPack)
   {
      double percentage = EuclidGeometryTools.percentageAlongLineSegment3D(point, this.originVertex, this.destinationVertex);
      if (percentage <= 0.0)
         this.originVertex.getSupportVectorDirectionTo(point, supportVectorToPack);
      else if (percentage >= 1.0)
         this.destinationVertex.getSupportVectorDirectionTo(point, supportVectorToPack);
      else
      {
         tempPoint.interpolate(this.originVertex, this.destinationVertex, percentage);
         supportVectorToPack.sub(point, tempPoint);
      }
   }

   @Override
   public SimplexBasics getSmallestSimplexMemberReference(Point3DReadOnly point)
   {
      double percentage = EuclidGeometryTools.percentageAlongLineSegment3D(point, this.originVertex, this.destinationVertex);
      if (percentage <= 0.0)
         return this.originVertex;
      else if (percentage >= 1.0)
         return this.destinationVertex;
      else
         return this;
   }
}
