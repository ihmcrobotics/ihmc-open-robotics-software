package us.ihmc.ihmcPerception.lineSegmentDetector;

import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;

public class LineTools
{
   public static Point2D getProjection(LineSegment2D segment, Point2D point)
   {
      Vector2D a = new Vector2D(segment.getFirstEndpointX(), segment.getFirstEndpointY());
      Vector2D b = new Vector2D(segment.getSecondEndpointX(), segment.getSecondEndpointY());
      Vector2D ba = new Vector2D();
      ba.sub(b, a);
      Vector2D pa = new Vector2D();
      pa.sub(point, a);
      double d = ba.dot(pa);
      Vector2D proj = new Vector2D();
      ba.scale(d / ba.lengthSquared());
      proj.add(a, ba);
      return new Point2D(proj);
   }
}
