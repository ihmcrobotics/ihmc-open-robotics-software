package us.ihmc.ihmcPerception.lineSegmentDetector;

import org.opencv.core.*;
import org.opencv.highgui.HighGui;
import org.opencv.imgproc.Imgproc;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.tuple2D.Point2D;

import java.util.ArrayList;
import java.util.List;

public class DisplayTools
{
   private static List<Mat> images = new ArrayList<>();
   private static Mat dispImage = new Mat();
   private static int height = 1000;
   private static int width = 2400;

   public static void addImage(Mat image)
   {
      images.add(image);
   }

   public static void lineSegment(Point2D a, Point2D b, Scalar color, int thickness, int index)
   {
      Imgproc.line(images.get(index), new Point(a.getX(), a.getY()), new Point(b.getX(), b.getY()), color, thickness);
   }

   public static void drawLines(Mat currentLines, int index)
   {
      for (int i = 0; i < currentLines.rows(); i++)
      {
         double[] val = currentLines.get(i, 0);
         Imgproc.line(images.get(index), new Point(val[0], val[1]), new Point(val[2], val[3]), new Scalar(0, 255, 255), 2);
      }
   }

   public static void drawPolygon(PlanarSegment segment, int index)
   {
      if (segment.getVertices().size() > 2)
      {
         MatOfPoint points = new MatOfPoint();
         ArrayList<Point> pointList = getOpenCVPoints(segment.getVertices());
         points.fromList(pointList);

         Scalar color = new Scalar(segment.getRegionId() * 123 % 255, segment.getRegionId() * 321 % 255, segment.getRegionId() * 135 % 255);
         Imgproc.fillConvexPoly(images.get(index), points, color, 4);
         Imgproc.circle(images.get(index), new Point(segment.getCentroid().getX(), segment.getCentroid().getY()), 3, new Scalar(255, 140, 255), -1);
      }
   }

   public static ArrayList<Point> getOpenCVPoints(ArrayList<Point2D> euclidPoints)
   {
      ArrayList<Point> opencvPoints = new ArrayList<>();
      for (Point2D point : euclidPoints)
      {
         opencvPoints.add(new Point(point.getX(), point.getY()));
      }
      return opencvPoints;
   }

   public static void display(int delay)
   {
      Core.hconcat(images, dispImage);
      Imgproc.resize(dispImage, dispImage, new Size(width, height));
      HighGui.namedWindow("LineEstimator", HighGui.WINDOW_AUTOSIZE);
      HighGui.imshow("LineEstimator", dispImage);
      HighGui.waitKey(delay);
      images.clear();
   }
}
