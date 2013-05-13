package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator;

import java.util.ArrayList;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

public class NewDoubleSupportICPComputer
{
   public static Point3d[] computeICPCornerPoints(int numberOfCornerPoints, ArrayList<Point3d> constantEquivalentCoPs, double steppingTime, double omega0)
   {
      Point3d[] icpCornerPoints = new Point3d[numberOfCornerPoints];

      Point3d icpCornerPoint = JojosICPutilities.extrapolateDCMpos(constantEquivalentCoPs.get(numberOfCornerPoints - 1), -steppingTime, omega0,
                                  constantEquivalentCoPs.get(numberOfCornerPoints));

      icpCornerPoints[numberOfCornerPoints - 1] = icpCornerPoint;

      for (int i = numberOfCornerPoints - 1; i > 0; i--)
      {
         icpCornerPoint = JojosICPutilities.extrapolateDCMpos(constantEquivalentCoPs.get(i - 1), -steppingTime, omega0, icpCornerPoints[i]);

         icpCornerPoints[i - 1] = icpCornerPoint;
      }

      return icpCornerPoints;
   }

   
   public static void computeSingleSupportStartICPAndVelocity(Point3d singleSupportICPToPack,  Vector3d singleSupportICPVelocityToPack, 
         Point3d constantCenterOfPressure, Point3d cornerPoint0, double doubleSupportDuration, double doubleSupportFirstStepFraction,
         double omega0)
   {
      double initialDoubleSupportDuration = doubleSupportDuration * (1.0 - doubleSupportFirstStepFraction);

      JojosICPutilities.extrapolateDCMposAndVel(singleSupportICPToPack, singleSupportICPVelocityToPack, 
            constantCenterOfPressure, initialDoubleSupportDuration, omega0, cornerPoint0);
   }
    
   
   public static void computeSingleSupportEndICPAndVelocity(Point3d singleSupportICPToPack,  Vector3d singleSupportICPVelocityToPack, 
         Point3d constantCenterOfPressure, Point3d cornerPoint0, double doubleSupportDuration, double doubleSupportFirstStepFraction,
         double singleSupportDuration, double omega0)
   {
      double initialDoubleSupportDuration = doubleSupportDuration * (1.0 - doubleSupportFirstStepFraction);

      JojosICPutilities.extrapolateDCMposAndVel(singleSupportICPToPack, singleSupportICPVelocityToPack, 
            constantCenterOfPressure, initialDoubleSupportDuration + singleSupportDuration, omega0, cornerPoint0);
   }
   
   
   public static Point3d computeSingleSupportStartICP(Point3d constantCenterOfPressure, Point3d cornerPoint0, double doubleSupportDuration, double doubleSupportFirstStepFraction,
         double omega0)
   {
      Point3d singleSupportStateICP = new Point3d();
      Vector3d singleSupportStateICPVelocity = new Vector3d();

      double initialDoubleSupportDuration = doubleSupportDuration * (1.0 - doubleSupportFirstStepFraction);

      JojosICPutilities.extrapolateDCMposAndVel(singleSupportStateICP, singleSupportStateICPVelocity, 
            constantCenterOfPressure, initialDoubleSupportDuration, omega0, cornerPoint0);

      return singleSupportStateICP;
   }

   public static Point3d computeSingleSupportEndICP(Point3d constantCenterOfPressure, Point3d cornerPoint0, double doubleSupportDuration, double doubleSupportFirstStepFraction,
         double singleSupportDuration, double omega0)
   {
      Point3d singleSupportStateICP = new Point3d();
      Vector3d singleSupportStateICPVelocity = new Vector3d();

      double initialDoubleSupportDuration = doubleSupportDuration * (1.0 - doubleSupportFirstStepFraction);

      JojosICPutilities.extrapolateDCMposAndVel(singleSupportStateICP, singleSupportStateICPVelocity, 
            constantCenterOfPressure, initialDoubleSupportDuration + singleSupportDuration, omega0, cornerPoint0);

      return singleSupportStateICP;
   }

   public static void computeSingleSupportICPPositionAndVelocity(Point3d icpPositionToPack, Vector3d icpVelocityToPack, Point3d constantCenterOfPressure, Point3d singleSupportStartICP, double omega0,
         double time)
   {
      JojosICPutilities.extrapolateDCMposAndVel(icpPositionToPack, icpVelocityToPack, constantCenterOfPressure, time, omega0, singleSupportStartICP);
      
   }
}
