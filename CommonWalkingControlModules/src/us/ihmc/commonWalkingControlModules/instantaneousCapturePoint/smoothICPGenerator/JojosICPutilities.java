package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator;


import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

public class JojosICPutilities
{
   public JojosICPutilities()
   {
   }
   
   public static Point3d extrapolateDCMpos(Point3d constantCenterOfPressure, double time, double omega0, Point3d initialICP)
   {
      double exponentialTerm = Math.exp(time * omega0);
      Vector3d tempVect = new Vector3d(initialICP);
      tempVect.sub(constantCenterOfPressure);
      tempVect.scale(exponentialTerm);

      Point3d finalICP = new Point3d(constantCenterOfPressure);
      finalICP.add(tempVect);
      
      return finalICP;
   }
   
   public static void extrapolateDCMposAndVel(Point3d icpPositionToPack, Vector3d icpVelocityToPack, 
         Point3d constantCenterOfPressure, double time, double omega0, Point3d initialICP)
   {
      double exponentialTerm = Math.exp(time * omega0);
      Vector3d tempVect = new Vector3d(initialICP);
      tempVect.sub(constantCenterOfPressure);
      tempVect.scale(exponentialTerm);

      icpPositionToPack.set(constantCenterOfPressure);
      icpPositionToPack.add(tempVect);

      icpVelocityToPack.set(tempVect);
      icpVelocityToPack.scale(omega0);
   }


   public static void discreteIntegrateCoMAndGetCoMVelocity(double sampleTime, double omega0, Point3d icp, Point3d comPositionToPack,
           Vector3d comVelocityToPack)
   {
      double exponentialFactor = Math.exp(-sampleTime * omega0);
      Vector3d tempVector = new Vector3d(comPositionToPack);
      tempVector.sub(icp);
      tempVector.scale(exponentialFactor);
      
      comPositionToPack.set(icp);
      comPositionToPack.add(tempVector);
      
      comVelocityToPack.set(comPositionToPack);
      comVelocityToPack.sub(icp);
      comVelocityToPack.scale(-1.0 * omega0);
   }  

}
