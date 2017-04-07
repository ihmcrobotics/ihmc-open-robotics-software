package us.ihmc.robotics.trajectories;

import java.util.ArrayList;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

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

      Point3D startPoint = new Point3D();
      startFramePoint.get(startPoint);
      Point3D endPoint = new Point3D();
      endFramePoint.get(endPoint);
      ArrayList<Point3D> listOf3dPoints = generate3dParabola(startPoint, endPoint, apexOfParabola, numberOfPointsToGenerate);

      ArrayList<FramePoint> ret = new ArrayList<FramePoint>();

      for (Point3D point3d : listOf3dPoints)
      {
         FramePoint pointToAdd = new FramePoint(referenceFrame, point3d);
         ret.add(pointToAdd);
      }

      return ret;
   }


   public static ArrayList<Point3D> generate3dParabola(Point3D startPoint, Point3D endPoint, double apexOfParabola, int numberOfPointsToGenerate)
   {
      if (apexOfParabola <= Math.max(startPoint.getZ(), endPoint.getZ()))
      {
         System.err.println("Apex should be higher than the maxmium z- value of Start/Target points");
      }

//    System.out.println(apexOfParabola);
//    if (apexOfParabola <= (endPoint.z - startPoint.z))
//       throw new RuntimeException(
//           "Apex of parabola should be greater than the difference in Z- components of start and end points!");


      Point3D pointToAdd = new Point3D();
      ArrayList<Point3D> pointsOnPath = new ArrayList<Point3D>(numberOfPointsToGenerate);
      double[] coefficientsOfParabola = new double[3];

      double lengthOfParabola = Math.sqrt(MathTools.square(endPoint.getX() - startPoint.getX()) + MathTools.square(endPoint.getY() - startPoint.getY()));
      double delta = lengthOfParabola / (numberOfPointsToGenerate - 1);

      double angle = Math.atan2((endPoint.getY() - startPoint.getY()), (endPoint.getX() - startPoint.getX()));

      coefficientsOfParabola = findCoefficientOfParabola(startPoint.getZ(), endPoint.getZ(), lengthOfParabola, apexOfParabola);

      double cosineAngle = Math.cos(angle);
      double sinAngle = Math.sin(angle);

      for (int i = 0; i < numberOfPointsToGenerate; i++)
      {
         double deltaTimesI = delta * (i);

         pointToAdd.setX(startPoint.getX() + deltaTimesI * cosineAngle);
         pointToAdd.setY(startPoint.getY() + deltaTimesI * sinAngle);
         pointToAdd.setZ(coefficientsOfParabola[2] * deltaTimesI * deltaTimesI + coefficientsOfParabola[1] * deltaTimesI + coefficientsOfParabola[0]);
         pointsOnPath.add(new Point3D(pointToAdd));
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
