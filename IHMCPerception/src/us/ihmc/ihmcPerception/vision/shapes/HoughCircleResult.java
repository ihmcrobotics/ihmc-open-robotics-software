package us.ihmc.ihmcPerception.vision.shapes;

import us.ihmc.euclid.tuple2D.Point2D;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class HoughCircleResult
{
   private final Point2D center;
   private final double radius;

   public HoughCircleResult(Point2D center, double radius)
   {
      this.center = center;
      this.radius = radius;
   }

   public Point2D getCenter()
   {
      return center;
   }

   public double getRadius()
   {
      return radius;
   }
}
