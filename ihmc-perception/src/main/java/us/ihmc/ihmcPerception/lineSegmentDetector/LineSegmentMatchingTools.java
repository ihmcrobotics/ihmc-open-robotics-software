package us.ihmc.ihmcPerception.lineSegmentDetector;

import org.bytedeco.opencv.opencv_core.Scalar4i;
import org.bytedeco.opencv.opencv_imgproc.Vec4iVector;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.Vector2D;

import java.util.ArrayList;

import static java.lang.StrictMath.acos;
import static java.lang.StrictMath.pow;

public class LineSegmentMatchingTools
{
   public static ArrayList<LineSegmentMatch> matchLineSegments(Vec4iVector previousLineSegments, Vec4iVector currentLineSegments)
   {
      ArrayList<LineSegmentMatch> matches = new ArrayList<>();
      double angleCost, midPointCost, lengthCost;
      double angleThreshold = 50, midPointThreshold = 100, lengthThreshold = 1200;

      for (int i = 0; i < previousLineSegments.size(); i++)
      {
         Scalar4i previousLineSegment = previousLineSegments.get(i);

         for (int j = 0; j < currentLineSegments.size(); j++)
         {
            Scalar4i currentLineSegment = currentLineSegments.get(j);

            angleCost = calculateAngleCost(previousLineSegment, currentLineSegment);
            midPointCost = calculateMidPointCost(previousLineSegment, currentLineSegment);
            lengthCost = calculateLengthCost(previousLineSegment, currentLineSegment);
            if (angleCost < angleThreshold && midPointCost < midPointThreshold
               // &&  lengthCost < lengthThreshold
            )
            {
               LineSegmentMatch lm = new LineSegmentMatch(previousLineSegment, currentLineSegment, angleCost, midPointCost, lengthCost);
               matches.add(lm);
            }
         }
      }
      return matches;
   }

   public static double calculateAngleCost(Scalar4i l1, Scalar4i l2)
   {
      Vector2D a = new Vector2D(l1.get(0), l1.get(1));
      Vector2D b = new Vector2D(l1.get(2), l1.get(3));
      Vector2D c = new Vector2D(l2.get(0), l2.get(1));
      Vector2D d = new Vector2D(l2.get(2), l2.get(3));
      Vector2D l1_hat = new Vector2D();
      l1_hat.sub(a, b);
      Vector2D l2_hat = new Vector2D();
      l2_hat.sub(c, d);
      l1_hat.normalize();
      l2_hat.normalize();
      return pow(acos(l1_hat.dot(l2_hat)) * 360 / EuclidCoreTools.TwoPI, 2);
   }

   public static double calculateMidPointCost(Scalar4i l1, Scalar4i l2)
   {
      Vector2D a = new Vector2D(l1.get(0), l1.get(1));
      Vector2D b = new Vector2D(l1.get(2), l1.get(3));
      Vector2D c = new Vector2D(l2.get(0), l2.get(1));
      Vector2D d = new Vector2D(l2.get(2), l2.get(3));
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

   public static double calculateLengthCost(Scalar4i l1, Scalar4i l2)
   {
      Vector2D a = new Vector2D(l1.get(0), l1.get(1));
      Vector2D b = new Vector2D(l1.get(2), l1.get(3));
      Vector2D c = new Vector2D(l2.get(0), l2.get(1));
      Vector2D d = new Vector2D(l2.get(2), l2.get(3));
      Vector2D ab = new Vector2D();
      Vector2D cd = new Vector2D();
      ab.sub(a, b);
      cd.sub(c, d);
      return pow(ab.length() - cd.length(), 2);
   }
}
