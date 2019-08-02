package us.ihmc.robotEnvironmentAwareness.geometry;

import java.util.*;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerTools;

public class ConcaveHull implements Iterable<Point2D>
{
   private final List<Point2D> hullVertices;

   public ConcaveHull()
   {
      hullVertices = new ArrayList<>();
   }

   public ConcaveHull(List<? extends Point2DReadOnly> hullVertices)
   {
      this.hullVertices = hullVertices.stream().map(Point2D::new).collect(Collectors.toList());
   }

   public ConcaveHull(ConcaveHull other)
   {
      this.hullVertices = new ArrayList<>();
      other.forEach(hullVertices::add);
   }

   public boolean isEmpty()
   {
      return hullVertices.isEmpty();
   }

   public void ensureCounterClockwiseOrdering()
   {
      ConcaveHullTools.ensureCounterClockwiseOrdering(hullVertices);
   }

   public void ensureClockwiseOrdering()
   {
      ConcaveHullTools.ensureClockwiseOrdering(hullVertices);
   }

   public void removeSuccessiveDuplicateVertices()
   {
      ConcaveHullTools.removeSuccessiveDuplicateVertices(hullVertices);
   }

   /**
    * Returns true only if removing the vertex would generate a kink in the concave polygon.
    * Meaning, it would cause several edges to cross each other.
    */
   public boolean isVertexPreventingKink(int vertexIndex)
   {
      return ConcaveHullTools.isVertexPreventingKink(vertexIndex, hullVertices);
   }

   public double computePerimeter()
   {
      return ConcaveHullTools.computePerimeter(hullVertices);
   }

   public boolean computeConcaveHullPocket(int concaveVertexIndex, ConcaveHullPocket pocketToPack)
   {
      return ConcaveHullTools.computeConcaveHullPocket(concaveVertexIndex, pocketToPack, hullVertices);
   }

   public ConcaveHullPocket computeConcaveHullPocket(int concaveVertexIndex)
   {
      return ConcaveHullTools.computeConcaveHullPocket(concaveVertexIndex, hullVertices);
   }

   public Set<ConcaveHullPocket> findConcaveHullPockets(double depthThreshold)
   {
      return ConcaveHullTools.findConcaveHullPockets(hullVertices, depthThreshold);
   }

   public ConcaveHullPocket findFirstConcaveHullPocket()
   {
      return findFirstConcaveHullPocket(0);
   }

   public ConcaveHullPocket findFirstConcaveHullPocket(int startIndex)
   {
      return ConcaveHullTools.findFirstConcaveHullPocket(hullVertices, startIndex);
   }

   public boolean isConvexAtVertex(int vertexIndex)
   {
      return ConcaveHullTools.isConvexAtVertex(vertexIndex, hullVertices);
   }

   public boolean isAlmostConvexAtVertex(int vertexIndex, double angleTolerance)
   {
      return ConcaveHullTools.isAlmostConvexAtVertex(vertexIndex, angleTolerance, hullVertices);
   }

   public double getAngleFromPreviousEdgeToNextEdge(int vertexIndex)
   {
      return ConcaveHullTools.getAngleFromPreviousEdgeToNextEdge(vertexIndex, hullVertices);
   }

   public boolean isHullConvex()
   {
      return ConcaveHullTools.isHullConvex(hullVertices);
   }

   public int getNumberOfVertices()
   {
      return hullVertices.size();
   }

   public List<Point2D> getConcaveHullVertices()
   {
      return hullVertices;
   }

   public void addVertex(double x, double y)
   {
      hullVertices.add(new Point2D(x, y));
   }

   public void addVertex(Point2D vertex)
   {
      hullVertices.add(vertex);
   }

   public Point2D getVertex(int vertexIndex)
   {
      return hullVertices.get(vertexIndex);
   }

   public List<Point3D> toVerticesInWorld(Point3DReadOnly hullOrigin, Orientation3DReadOnly hullOrientation)
   {
      return PolygonizerTools.toPointsInWorld(hullVertices, hullOrigin, hullOrientation);
   }

   public List<Point3D> toVerticesInWorld(Point3DReadOnly hullOrigin, Vector3DReadOnly hullNormal)
   {
      return PolygonizerTools.toPointsInWorld(hullVertices, hullOrigin, hullNormal);
   }

   public List<Point3D> toVerticesInWorld(RigidBodyTransform transformToWorld)
   {
      List<Point3D> vertices3d = toVertices3d(0.0);
      vertices3d.forEach(vertex -> transformToWorld.transform(vertex));
      return vertices3d;
   }

   public List<Point3D> toVertices3d(double zOffset)
   {
      return stream().map(vertex -> new Point3D(vertex.getX(), vertex.getY(), zOffset)).collect(Collectors.toList());
   }

   /**
    * Two concave hulls are equal if they contain the same points in the same order,
    * but the start/end doesn't need to line up.
    */
   public boolean epsilonEquals(ConcaveHull other, double epsilon)
   {
      if (getNumberOfVertices() != other.getNumberOfVertices())
      {
         return false;
      }

      if (getNumberOfVertices() == 0)
      {
         return true;
      }

      int alignmentOffset = 0;
      boolean failed = true;
      while (failed && alignmentOffset < getNumberOfVertices())
      {
         failed = false;
         for (int i = 0; i < getNumberOfVertices(); i++)
         {
            int compareIndex = (i + alignmentOffset) % getNumberOfVertices();
            boolean epsilonEquals = hullVertices.get(i).epsilonEquals(other.hullVertices.get(compareIndex), epsilon);
            if (!epsilonEquals)
            {
               failed = true;
               ++alignmentOffset;
               break; // optimization
            }
         }
      }

      return !failed;
   }

   public Stream<Point2D> stream()
   {
      return hullVertices.stream();
   }

   @Override
   public Iterator<Point2D> iterator()
   {
      return hullVertices.iterator();
   }

   @Override
   public int hashCode()
   {
      int hashCode = 1;
      for (Point2D vertex : this)
         hashCode = 31 * hashCode + (vertex == null ? 0 : vertex.hashCode());
      return hashCode;
   }

   @Override
   public boolean equals(Object object)
   {
      if (object instanceof ConcaveHull)
         return equals((ConcaveHull) object);
      else
         return false;
   }

   public boolean equals(ConcaveHull other)
   {
      if (getNumberOfVertices() != other.getNumberOfVertices())
         return false;

      for (int vertexIndex = 0; vertexIndex <= getNumberOfVertices(); vertexIndex++)
      {
         if (!hullVertices.get(vertexIndex).equals(other.hullVertices.get(vertexIndex)))
            return false;
      }
      return true;
   }

   @Override
   public String toString()
   {
      return "Size: " + getNumberOfVertices() + "\n" + ConcaveHullTools.vertexListToString(hullVertices);
   }
}
