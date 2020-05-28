package us.ihmc.robotics.geometry.concavePolygon2D.weilerAtherton;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;

class IntersectionInfo
{
   private final IntersectionType type;
   private final Point2DReadOnly intersection;
   private final Point2DReadOnly startVertexOfIntersectingEdge;
   private final Point2DReadOnly endVertexOfIntersectingEdge;

   public IntersectionInfo(IntersectionType type,
                           Point2DReadOnly intersection,
                           Point2DReadOnly startVertexOfIntersectingEdge,
                           Point2DReadOnly endVertexOfIntersectingEdge)
   {
      this.type = type;
      if (intersection != null)
         this.intersection = new Point2D(intersection);
      else
         this.intersection = null;
      this.startVertexOfIntersectingEdge = startVertexOfIntersectingEdge;
      this.endVertexOfIntersectingEdge = endVertexOfIntersectingEdge;
   }

   public IntersectionType getIntersectionType()
   {
      return type;
   }

   public Point2DReadOnly getIntersection()
   {
      return intersection;
   }

   public Point2DReadOnly getStartVertexOfIntersectingEdge()
   {
      return startVertexOfIntersectingEdge;
   }

   public Point2DReadOnly getEndVertexOfIntersectingEdge()
   {
      return endVertexOfIntersectingEdge;
   }
}
