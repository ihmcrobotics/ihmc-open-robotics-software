package us.ihmc.ihmcPerception.lineSegmentDetector;

import org.opencv.core.Mat;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.Vector2D;

import java.util.ArrayList;

import static java.lang.StrictMath.acos;
import static java.lang.StrictMath.pow;

public class LineMatchingTools
{
   public static ArrayList<LineMatch> matchFLDLines(Mat prevFLDLines, Mat FLDlines)
   {
      ArrayList<LineMatch> corresp = new ArrayList<>();
      double angleCost, midPointCost, lengthCost = 0;
      double angleThreshold = 50, midPointThreshold = 100, lengthThreshold = 1200;
      for (int i = 0; i < prevFLDLines.rows(); i++)
      {
         double[] pl = prevFLDLines.get(i, 0);
         for (int j = 0; j < FLDlines.rows(); j++)
         {
            double[] cl = FLDlines.get(j, 0);
            angleCost = calcAngleCost(pl, cl);
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

   public static double calcAngleCost(double[] l1, double[] l2)
   {
      Vector2D a = new Vector2D(l1[0], l1[1]);
      Vector2D b = new Vector2D(l1[2], l1[3]);
      Vector2D c = new Vector2D(l2[0], l2[1]);
      Vector2D d = new Vector2D(l2[2], l2[3]);
      Vector2D l1_hat = new Vector2D();
      l1_hat.sub(a, b);
      Vector2D l2_hat = new Vector2D();
      l2_hat.sub(c, d);
      l1_hat.normalize();
      l2_hat.normalize();
      return pow(acos(l1_hat.dot(l2_hat)) * 360 / EuclidCoreTools.TwoPI, 2);
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
