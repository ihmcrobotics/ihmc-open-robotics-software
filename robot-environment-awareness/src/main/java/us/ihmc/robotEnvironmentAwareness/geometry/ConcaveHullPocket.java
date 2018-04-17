package us.ihmc.robotEnvironmentAwareness.geometry;

import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;

/**
 * A pocket is a concave region.
 * It has different properties:
 *  - It has a unique bridge
 *  - It has a deepest vertex (straight-line criterion)
 *  - It has a maximum depth. 
 * @author Sylvain
 *
 */
public class ConcaveHullPocket
{
   private int startBridgeIndex;
   private int endBridgeIndex;

   private Point2DReadOnly startBridgeVertex;
   private Point2DReadOnly endBridgeVertex;

   private int deepestVertexIndex;
   private Point2DReadOnly deepestVertex;

   private double maxDepth;

   public ConcaveHullPocket()
   {
      clear();
   }

   public void clear()
   {
      startBridgeIndex = -1;
      endBridgeIndex = -1;

      startBridgeVertex = null;
      endBridgeVertex = null;

      clearDepthParameters();
   }

   public void clearDepthParameters()
   {
      deepestVertexIndex = -1;
      deepestVertex = null;

      maxDepth = Double.NEGATIVE_INFINITY;
   }

   public void setBridgeIndices(int startBridgeIndex, int endBridgeIndex)
   {
      this.startBridgeIndex = startBridgeIndex;
      this.endBridgeIndex = endBridgeIndex;
   }

   public void setBridgeVertices(Point2DReadOnly startBridgeVertex, Point2DReadOnly endBridgeVertex)
   {
      this.startBridgeVertex = startBridgeVertex;
      this.endBridgeVertex = endBridgeVertex;
   }

   public void setDeepestVertexIndex(int deepestVertexIndex)
   {
      this.deepestVertexIndex = deepestVertexIndex;
   }

   public void setDeepestVertex(Point2DReadOnly deepestVertex)
   {
      this.deepestVertex = deepestVertex;
   }

   public void setMaxDepth(double maxDepth)
   {
      this.maxDepth = maxDepth;
   }

   public int getStartBridgeIndex()
   {
      return startBridgeIndex;
   }

   public int getEndBridgeIndex()
   {
      return endBridgeIndex;
   }

   public Point2DReadOnly getStartBridgeVertex()
   {
      return startBridgeVertex;
   }

   public Point2DReadOnly getEndBridgeVertex()
   {
      return endBridgeVertex;
   }

   public int getDeepestVertexIndex()
   {
      return deepestVertexIndex;
   }

   public Point2DReadOnly getDeepestVertex()
   {
      return deepestVertex;
   }

   public double getMaxDepth()
   {
      return maxDepth;
   }
}
