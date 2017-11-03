package us.ihmc.geometry.polytope.DCELPolytope.Basics;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.interfaces.Clearable;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public abstract class PolytopeVertexBasics<A extends PolytopeVertexBasics<A, B, C>, B extends PolytopeHalfEdgeBasics<A, B, C>, C extends ConvexPolytopeFaceBasics<A, B, C>>
      implements SimplexBasics, PolytopeVertexReadOnly, Clearable, Settable<A>, Transformable, Point3DBasics
{
   private final ArrayList<B> associatedEdges = new ArrayList<>();

   protected abstract Point3DBasics getPointObjectReference();

   public PolytopeVertexBasics()
   {
   }

   public void set(A vertex)
   {
      getPointObjectReference().set(vertex.getPosition());
      clearAssociatedEdgeList();
      copyEdges(vertex.getAssociatedEdges());
   }

   public List<B> getAssociatedEdges()
   {
      return associatedEdges;
   }

   public B getAssociatedEdge(int index)
   {
      return associatedEdges.get(index);
   }

   public void removeAssociatedEdge(B edgeToAdd)
   {
      associatedEdges.remove(edgeToAdd);
   }

   public void clearAssociatedEdgeList()
   {
      associatedEdges.clear();
   }

   public void copyEdges(List<B> edgeList)
   {
      for (int i = 0; i < edgeList.size(); i++)
      {
         addAssociatedEdge(edgeList.get(i));
      }
   }

   public void addAssociatedEdge(B edge)
   {
      if (!isAssociatedWithEdge(edge))
         associatedEdges.add(edge);
   }

   public boolean isAssociatedWithEdge(PolytopeHalfEdgeReadOnly edgeToCheck)
   {
      return associatedEdges.contains(edgeToCheck);
   }

   public boolean isAssociatedWithEdge(PolytopeHalfEdgeReadOnly edgeToCheck, double epsilon)
   {
      boolean result = associatedEdges.size() > 0;
      for (int i = 0; result && i < associatedEdges.size(); i++)
      {
         result &= associatedEdges.get(i).epsilonEquals(edgeToCheck, epsilon);
      }
      return result;
   }

   public int getNumberOfAssociatedEdges()
   {
      return associatedEdges.size();
   }

   public double dot(Vector3DReadOnly vector)
   {
      return getX() * vector.getX() + getY() * vector.getY() + getZ() * vector.getZ();
   }

   public String toString()
   {
      return "( " + getX() + ", " + getY() + ", " + getZ() + ")";
   }

   @Override
   public double getX()
   {
      return getPointObjectReference().getX();
   }

   @Override
   public double getY()
   {
      return getPointObjectReference().getY();
   }

   @Override
   public double getZ()
   {
      return getPointObjectReference().getZ();
   }

   public double getElement(int index)
   {
      return getPointObjectReference().getElement(index);
   }

   @Override
   public void applyTransform(Transform transform)
   {
      getPointObjectReference().applyTransform(transform);
   }

   @Override
   public void applyInverseTransform(Transform transform)
   {
      getPointObjectReference().applyInverseTransform(transform);
   }

   public boolean isAnyFaceMarked()
   {
      boolean isMarked = false;
      for (int i = 0; !isMarked && i < associatedEdges.size(); i++)
      {
         isMarked |= associatedEdges.get(i).getFace().isMarked();
      }
      //PrintTools.debug(toString() + " " +isMarked);
      return isMarked;
   }

   @Override
   public boolean containsNaN()
   {
      return getPosition().containsNaN();
   }

   @Override
   public void setToNaN()
   {
      getPointObjectReference().setToNaN();
   }

   @Override
   public void setToZero()
   {
      getPointObjectReference().setToZero();
   }

   public void setX(double x)
   {
      getPointObjectReference().setX(x);
   }

   public void setY(double y)
   {
      getPointObjectReference().setY(y);
   }

   public void setZ(double z)
   {
      getPointObjectReference().setZ(z);
   }

   @Override
   public boolean epsilonEquals(PolytopeVertexReadOnly other, double epsilon)
   {
      return getPointObjectReference().epsilonEquals(other, epsilon);
   }

   @Override
   public double getShortestDistanceTo(Point3DReadOnly point)
   {
      return getPointObjectReference().distance(point);
   }

   @Override
   public void getSupportVectorDirectionTo(Point3DReadOnly point, Vector3D supportVectorToPack)
   {
      supportVectorToPack.sub(point, this);
   }

   @Override
   public SimplexBasics getSmallestSimplexMemberReference(Point3DReadOnly point)
   {
      return this;
   }
}
