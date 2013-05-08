package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator;


import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

public class JojosICPutilities
{
   public JojosICPutilities()
   {
   }

   public static void extrapolateDCMpos(DenseMatrix64F constCoPcurrentStep, double time, double dcmConst, DenseMatrix64F ficalICPcurrentFootStep,
           DenseMatrix64F finalDoubleSupportICP)
   {
      double exponentialTerm = Math.exp(time / dcmConst);
      DenseMatrix64F tempVect = new DenseMatrix64F(3, 1);
      CommonOps.sub(ficalICPcurrentFootStep, constCoPcurrentStep, tempVect);
      CommonOps.scale(exponentialTerm, tempVect);
      CommonOps.add(constCoPcurrentStep, tempVect, finalDoubleSupportICP);
   }
   
   public static Point3d extrapolateDCMpos(Point3d constCoPcurrentStep, double time, double omega0, Point3d initialICP)
   {
      double exponentialTerm = Math.exp(time * omega0);
      Vector3d tempVect = new Vector3d(initialICP);
      tempVect.sub(constCoPcurrentStep);
      tempVect.scale(exponentialTerm);

      Point3d finalDoubleSupportICP = new Point3d(constCoPcurrentStep);
      finalDoubleSupportICP.add(tempVect);
      
      return finalDoubleSupportICP;
   }

   public static void extrapolateDCMposAndVel(DenseMatrix64F constCoPcurrentStep, double time, double dcmConst, 
         DenseMatrix64F finalICPCurrentFootStep,
         DenseMatrix64F finalDoubleSupportICPpos, DenseMatrix64F finalDoubleSupportICPvel)
   {
      double exponentialTerm = Math.exp(time / dcmConst);
      DenseMatrix64F tempVect = new DenseMatrix64F(3, 1);
      CommonOps.sub(finalICPCurrentFootStep, constCoPcurrentStep, tempVect);
      CommonOps.scale(exponentialTerm, tempVect);
      CommonOps.add(constCoPcurrentStep, tempVect, finalDoubleSupportICPpos);
      CommonOps.scale(1.0 / dcmConst, tempVect, finalDoubleSupportICPvel);
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


   public static void discreteIntegrateCoMAndGetCoMVelocity(double sampleTime, double dcmConst, DenseMatrix64F icp, DenseMatrix64F comPosition,
           DenseMatrix64F comVelocity)
   {
      double exponentialFactor = Math.exp(-sampleTime / dcmConst);
      DenseMatrix64F tempMatrix = new DenseMatrix64F(3, 1);

      CommonOps.sub(comPosition, icp, tempMatrix);
      CommonOps.scale(exponentialFactor, tempMatrix);
      CommonOps.add(icp, tempMatrix, comPosition);

      CommonOps.sub(comPosition, icp, comVelocity);
      CommonOps.scale(-1 / dcmConst, comVelocity);
   }

}
