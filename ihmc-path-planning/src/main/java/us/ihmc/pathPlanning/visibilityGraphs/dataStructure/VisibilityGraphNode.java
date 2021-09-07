package us.ihmc.pathPlanning.visibilityGraphs.dataStructure;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;

import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

/**
 * Visibility graph node data structure associated with a navigable region,
 * holding cost information, and connected graph edges
 */
public class VisibilityGraphNode implements EpsilonComparable<VisibilityGraphNode>
{
   private static final double hashGridSize = 1e-3;

   private final VisibilityGraphNavigableRegion visibilityGraphNavigableRegion;
   private final ConnectionPoint3D pointInWorld;
   private final Point2D point2DInLocal;

   private boolean edgesHaveBeenDetermined = false;

   private double costFromStart = Double.NaN;

   private boolean hasBeenExpanded = false;
   private VisibilityGraphNode bestParentNode = null;

   private final int hashCode;

   private final HashSet<VisibilityGraphEdge> edges = new HashSet<>();

   public VisibilityGraphNode(Point3DReadOnly pointInWorld, Point2DReadOnly pointInLocal, VisibilityGraphNavigableRegion visibilityGraphNavigableRegion)
   {
      this(pointInWorld, pointInLocal, visibilityGraphNavigableRegion, visibilityGraphNavigableRegion.getMapId());
   }

   public VisibilityGraphNode(Point3DReadOnly pointInWorld, Point2DReadOnly pointInLocal, VisibilityGraphNavigableRegion visibilityGraphNavigableRegion,
                              int mapId)
   {
      this.visibilityGraphNavigableRegion = visibilityGraphNavigableRegion;
      this.pointInWorld = new ConnectionPoint3D(pointInWorld, mapId);
      this.point2DInLocal = new Point2D(pointInLocal);

      hashCode = computeHashCode(this);
   }

   public int getRegionId()
   {
      return pointInWorld.getRegionId();
   }

   public VisibilityGraphNavigableRegion getVisibilityGraphNavigableRegion()
   {
      return visibilityGraphNavigableRegion;
   }

   public ConnectionPoint3D getPointInWorld()
   {
      return pointInWorld;
   }

   public Point2DReadOnly getPoint2DInLocal()
   {
      return point2DInLocal;
   }

   public synchronized void addEdge(VisibilityGraphEdge edge)
   {
      if (edge != null)
      {
         edges.add(edge);
      }
   }

   public HashSet<VisibilityGraphEdge> getEdges()
   {
      return edges;
   }

   public double distanceXYSquared(VisibilityGraphNode target)
   {
      return pointInWorld.distanceXYSquared(target.pointInWorld);
   }

   public double getCostFromStart()
   {
      return costFromStart;
   }

   public void setCostFromStart(double costFromStart, VisibilityGraphNode bestParentNode)
   {
      this.costFromStart = costFromStart;
      this.bestParentNode = bestParentNode;
   }

   public VisibilityGraphNode getBestParentNode()
   {
      return bestParentNode;
   }

   public boolean getHasBeenExpanded()
   {
      return hasBeenExpanded;
   }

   public void setHasBeenExpanded(boolean hasBeenExpanded)
   {
      this.hasBeenExpanded = hasBeenExpanded;
   }

   public boolean getEdgesHaveBeenDetermined()
   {
      return edgesHaveBeenDetermined;
   }

   public void setEdgesHaveBeenDetermined(boolean edgesHaveBeenDetermined)
   {
      this.edgesHaveBeenDetermined = edgesHaveBeenDetermined;
   }

   @Override
   public int hashCode()
   {
      return hashCode;
   }

   @Override
   public boolean equals(Object obj)
   {
      if (this == obj)
         return true;
      if (obj == null)
         return false;
      if (getClass() != obj.getClass())
         return false;
      VisibilityGraphNode other = (VisibilityGraphNode) obj;
      return epsilonEquals(other, 1e-8);
   }

   @Override
   public boolean epsilonEquals(VisibilityGraphNode other, double epsilon)
   {
      return pointInWorld.epsilonEquals(other.pointInWorld, epsilon);
   }

   @Override
   public String toString()
   {
      return pointInWorld.toString();
   }

   private static int computeHashCode(VisibilityGraphNode node)
   {
      final int prime = 31;
      int result = 1;
      result = prime * result + (int) Math.round(node.getPointInWorld().getX() / hashGridSize);
      result = prime * result + (int) Math.round(node.getPointInWorld().getY() / hashGridSize);
      return result;
   }
}
