package us.ihmc.ihmcPerception.vision.shapes;

import javax.vecmath.Point2d;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class HoughCircleResult
{
   private final Point2d center;
   private final double radius;

   public HoughCircleResult(Point2d center, double radius)
   {
      this.center = center;
      this.radius = radius;
   }

   public Point2d getCenter()
   {
      return center;
   }

   public double getRadius()
   {
      return radius;
   }
}
