package us.ihmc.ihmcPerception.lineSegmentDetector;

import org.opencv.core.Mat;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;

import java.util.ArrayList;

import static java.lang.StrictMath.acos;
import static java.lang.StrictMath.pow;

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

   public static ArrayList<LineMatch> matchFLDLines(Mat prevFLDLines, Mat FLDlines)
   {
      ArrayList<LineMatch> corresp = new ArrayList<>();

      if(prevFLDLines == null || FLDlines == null) return corresp;

      double angleCost, midPointCost, lengthCost = 0;
      double angleThreshold = 50, midPointThreshold = 100, lengthThreshold = 1200;
      for (int i = 0; i < prevFLDLines.rows(); i++)
      {
         double[] pl = prevFLDLines.get(i, 0);
         for (int j = 0; j < FLDlines.rows(); j++)
         {
            double[] cl = FLDlines.get(j, 0);
            angleCost = calcAngleCost( new Vector2D(pl[0] - pl[2], pl[1] - pl[3]), new Vector2D(cl[0] - cl[2], cl[1] - cl[3]));
            midPointCost = calcMidPointCost(pl, cl);
            lengthCost = calcLengthCost(pl, cl);
            if (angleCost < angleThreshold && midPointCost < midPointThreshold
               // &&  lengthCost < lengthThreshold
            )
            {
               LineMatch lm = new LineMatch(pl, cl, angleCost, midPointCost, lengthCost);
               corresp.add(lm);
            }
         }
      }
      return corresp;
   }

   public static double calcAngleCost(Vector2D ab, Vector2D cd)
   {
      ab.normalize();
      cd.normalize();
      return pow(acos(ab.dot(cd)) * 360 / EuclidCoreTools.TwoPI, 2);
   }

   public static double calcMidPointCost(double[] l1, double[] l2)
   {
      Vector2D a = new Vector2D(l1[0], l1[1]);
      Vector2D b = new Vector2D(l1[2], l1[3]);
      Vector2D c = new Vector2D(l2[0], l2[1]);
      Vector2D d = new Vector2D(l2[2], l2[3]);
      Vector2D m1 = new Vector2D();
      m1.add(a, b);
      m1.scale(0.5);
      Vector2D m2 = new Vector2D();
      m2.add(c, d);
      m2.scale(0.5);
      Vector2D p = new Vector2D();
      p.sub(m1, m2);
      return pow(p.length(), 2);
   }

   public static double calcLengthCost(double[] l1, double[] l2)
   {
      Vector2D a = new Vector2D(l1[0], l1[1]);
      Vector2D b = new Vector2D(l1[2], l1[3]);
      Vector2D c = new Vector2D(l2[0], l2[1]);
      Vector2D d = new Vector2D(l2[2], l2[3]);
      Vector2D ab = new Vector2D();
      Vector2D cd = new Vector2D();
      ab.sub(a, b);
      cd.sub(c, d);
      return pow(ab.length() - cd.length(), 2);
   }
}
