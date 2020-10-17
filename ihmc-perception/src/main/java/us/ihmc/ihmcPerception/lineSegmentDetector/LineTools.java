package us.ihmc.ihmcPerception.lineSegmentDetector;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
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

   public static void drawLines(Mat img, Mat currentLines){
      for (int i = 0; i < currentLines.rows(); i++)
      {
         double[] val = currentLines.get(i, 0);
         Imgproc.line(img, new Point(val[0], val[1]), new Point(val[2], val[3]), new Scalar(0, 255, 255), 2);
      }
   }

}
