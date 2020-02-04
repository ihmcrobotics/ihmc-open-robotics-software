package us.ihmc.pathPlanning.visibilityGraphs.dataStructure;

import us.ihmc.euclid.geometry.interfaces.LineSegment3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

/**
 * Directed visibility graph edge data structure containing weight and cost information,
 * and source and target nodes that this edge connects.
 */
public class VisibilityGraphEdge implements EpsilonComparable<VisibilityGraphEdge>, LineSegment3DReadOnly
{
   private static final double defaultEdgeWeight = 1.0;

   private final VisibilityGraphNode sourceNode;
   private final VisibilityGraphNode targetNode;

   private double edgeWeight;
   private final int hashCode;

   public VisibilityGraphEdge(VisibilityGraphNode source, VisibilityGraphNode target)
   {
      this.sourceNode = source;
      this.targetNode = target;
      this.edgeWeight = defaultEdgeWeight;
      this.hashCode = computeHashCode(this);
   }

   public VisibilityGraphNode getSourceNode()
   {
      return sourceNode;
   }

   public VisibilityGraphNode getTargetNode()
   {
      return targetNode;
   }

   public ConnectionPoint3D getSourcePointInWorld()
   {
      return sourceNode.getPointInWorld();
   }

   public ConnectionPoint3D getTargetPointInWorld()
   {
      return targetNode.getPointInWorld();
   }

   public void setEdgeWeight(double edgeWeight)
   {
      this.edgeWeight = edgeWeight;
   }

   public double getEdgeWeight()
   {
      return edgeWeight;
   }

   public void registerEnds()
   {
      sourceNode.addEdge(this);
      targetNode.addEdge(this);
   }

   @Override
   public boolean epsilonEquals(VisibilityGraphEdge other, double epsilon)
   {
      return sourceNode.epsilonEquals(other.sourceNode, epsilon) && targetNode.epsilonEquals(other.targetNode, epsilon)
            || sourceNode.epsilonEquals(other.targetNode, epsilon) && targetNode.epsilonEquals(other.sourceNode, epsilon);
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
      VisibilityGraphEdge other = (VisibilityGraphEdge) obj;
      return epsilonEquals(other, 1e-8);
   }

   @Override
   public int hashCode()
   {
      return hashCode;
   }

   public boolean equals(VisibilityGraphEdge other)
   {
      if (other == null)
         return false;
      else
         return (sourceNode.equals(other.sourceNode) && targetNode.equals(other.targetNode))
               || (sourceNode.equals(other.targetNode) && targetNode.equals(other.sourceNode));
   }

   @Override
   public String toString()
   {
      return "Connection: source = " + EuclidCoreIOTools.getTuple3DString(sourceNode.getPointInWorld()) + ", target = "
            + EuclidCoreIOTools.getTuple3DString(targetNode.getPointInWorld());
   }

   @Override
   public Point3DReadOnly getFirstEndpoint()
   {
      return sourceNode.getPointInWorld();
   }

   @Override
   public Point3DReadOnly getSecondEndpoint()
   {
      return targetNode.getPointInWorld();
   }

   private static int computeHashCode(VisibilityGraphEdge edge)
   {
      return edge.getSourceNode().hashCode() + edge.getTargetNode().hashCode();
   }
}
