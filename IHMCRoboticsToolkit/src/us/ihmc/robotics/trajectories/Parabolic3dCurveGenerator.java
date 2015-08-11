package us.ihmc.robotics.trajectories;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

import javax.vecmath.Point3d;
import java.util.ArrayList;

/**
 * <p>Title: </p>
 *
 * <p>Description: </p>
 *
 * <p>Copyright: Copyright (c) 2007</p>
 *
 * <p>Company: </p>
 *
 * @author not attributable
 * @version 1.0
 */
public class Parabolic3dCurveGenerator
{
   @SuppressWarnings("unused")
   private static double EPSILON = 1e-6;

// private static int numberOfPointsToGenerate = 10;

   /**
    * Parabolic3dGenerator
    * gets start & end frame points in cartesian coordinate, and creates a
    * z-splined trajectory as the output.
    *
    * @param startPoint FramePoint = (xs,ys,zs)
    * @param endPoint FramePoint = (xe,ye,ze)
    * @param apexOfParabola double = h
    *   is equal to the difference in Z between start point and a
    *   point in half the way from start point and endpoint. 'apexOfParabola' should be
    *   greater than the difference in Z- components of start and end points.
    */



   public static ListOfPointsTrajectory generateParabolicListOfPointsTrajectory(FramePoint startPoint, FramePoint endPoint, double apexOfParabola,
           int numberOfPointsToGenerate)
   {
      ArrayList<FramePoint> listOfPoints = generate3dParabola(startPoint, endPoint, apexOfParabola, numberOfPointsToGenerate);

      ListOfPointsTrajectory ret = new ListOfPointsTrajectory(listOfPoints);

      return ret;
   }

   public static ArrayList<FramePoint> generate3dParabola(FramePoint startFramePoint, FramePoint endFramePoint, double apexOfParabola, int numberOfPointsToGenerate)
   {
      startFramePoint.checkReferenceFrameMatch(endFramePoint);
      ReferenceFrame referenceFrame = startFramePoint.getReferenceFrame();

      Point3d startPoint = new Point3d();
      startFramePoint.get(startPoint);
      Point3d endPoint = new Point3d();
      endFramePoint.get(endPoint);
      ArrayList<Point3d> listOf3dPoints = generate3dParabola(startPoint, endPoint, apexOfParabola, numberOfPointsToGenerate);

      ArrayList<FramePoint> ret = new ArrayList<FramePoint>();

      for (Point3d point3d : listOf3dPoints)
      {
         FramePoint pointToAdd = new FramePoint(referenceFrame, point3d);
         ret.add(pointToAdd);
      }

      return ret;
   }


   public static ArrayList<Point3d> generate3dParabola(Point3d startPoint, Point3d endPoint, double apexOfParabola, int numberOfPointsToGenerate)
   {
      if (apexOfParabola <= Math.max(startPoint.z, endPoint.z))
      {
         System.err.println("Apex should be higher than the maxmium z- value of Start/Target points");
      }

//    System.out.println(apexOfParabola);
//    if (apexOfParabola <= (endPoint.z - startPoint.z))
//       throw new RuntimeException(
//           "Apex of parabola should be greater than the difference in Z- components of start and end points!");


      Point3d pointToAdd = new Point3d();
      ArrayList<Point3d> pointsOnPath = new ArrayList<Point3d>(numberOfPointsToGenerate);
      double[] coefficientsOfParabola = new double[3];

      double lengthOfParabola = Math.sqrt(MathTools.square(endPoint.x - startPoint.x) + MathTools.square(endPoint.y - startPoint.y));
      double delta = lengthOfParabola / (numberOfPointsToGenerate - 1);

      double angle = Math.atan2((endPoint.y - startPoint.y), (endPoint.x - startPoint.x));

      coefficientsOfParabola = findCoefficientOfParabola(startPoint.z, endPoint.z, lengthOfParabola, apexOfParabola);

      double cosineAngle = Math.cos(angle);
      double sinAngle = Math.sin(angle);

      for (int i = 0; i < numberOfPointsToGenerate; i++)
      {
         double deltaTimesI = delta * (i);

         pointToAdd.x = startPoint.x + deltaTimesI * cosineAngle;
         pointToAdd.y = startPoint.y + deltaTimesI * sinAngle;
         pointToAdd.z = coefficientsOfParabola[2] * deltaTimesI * deltaTimesI + coefficientsOfParabola[1] * deltaTimesI + coefficientsOfParabola[0];
         pointsOnPath.add(new Point3d(pointToAdd));
      }

      return pointsOnPath;
   }

   private static double[] findCoefficientOfParabola(double z0, double zf, double lengthOfParabola, double apexOfParabola)
   {
      double[] coefficientList = new double[3];
      coefficientList[0] = z0;

      double a, b, c, root, xs;
      double lengthOfParabolaSquared = MathTools.square(lengthOfParabola);
      c = MathTools.square(zf - z0) / MathTools.square(lengthOfParabolaSquared);
      b = (4.0 * apexOfParabola - 2.0 * (zf + z0)) / lengthOfParabolaSquared;
      a = 1.0;

      root = (-b + Math.sqrt(b * b - 4.0 * a * c)) / (2.0 * a);
      xs = -(zf - z0) / (2.0 * root * lengthOfParabola) + lengthOfParabola / 2.0;

      if ((xs >= 0.0) && (xs <= lengthOfParabola))
      {
         coefficientList[2] = root;
         coefficientList[1] = (zf - z0) / lengthOfParabola - root * lengthOfParabola;
      }
      else
      {
         root = (-b - Math.sqrt(b * b - 4.0 * a * c)) / (2.0 * a);
         xs = -(zf - z0) / (2.0 * root * lengthOfParabola) + lengthOfParabola / 2.0;

         if ((xs >= 0.0) && (xs <= lengthOfParabola))
         {
            coefficientList[2] = root;
            coefficientList[1] = (zf - z0) / lengthOfParabola - root * lengthOfParabola;
         }
         else
         {
            System.err.println("ParabolicTrajectory.findCoefficientOfParabola(): could not find satifactory root");
            coefficientList[2] = 0.0;
            coefficientList[1] = 0.0;
         }
      }

      if (coefficientList[2] > 0.0)
      {
         System.err.println("Warning: encountered an upside-down parabola!");
      }

      return coefficientList;
   }




}
