package us.ihmc.robotics.geometry.concavePolygon2D.clippingAndMerging;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;

class IntersectionInfo
{
   private final IntersectionType type;
   private final Point2DReadOnly intersection;

   public IntersectionInfo(IntersectionType type, Point2DReadOnly intersection)
   {
      this.type = type;
      if (intersection != null)
         this.intersection = new Point2D(intersection);
      else
         this.intersection = null;
   }

   public IntersectionType getIntersectionType()
   {
      return type;
   }

   public Point2DReadOnly getIntersection()
   {
      return intersection;
   }

   /**
    * Defines the type of intersection that this represents. If it's at the end of an edge, this means that it occurs at a pre-existing vertex.
    * END: Intersection occurs at a pre-existing vertex, meaning it's at one end of a line-segment.
    * NEW: Intersection occurs somewhere along the line-segment.
    * NONE: There is no intersection.
    */
   enum IntersectionType
   {END, NEW, NONE}
}
