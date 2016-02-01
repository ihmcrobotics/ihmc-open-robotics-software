package us.ihmc.commonWalkingControlModules.calibration.virtualChain;

import java.util.ArrayList;

import javax.vecmath.Matrix3d;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.robotics.geometry.FramePoint2d;

public class VirtualChainDataMatrix
{
   private DenseMatrix64F yMatrix;
   private DenseMatrix64F AMatrix;
   
   public VirtualChainDataMatrix(VirtualChainData virtualChainData)
   {
      int numberOfDataPoints = virtualChainData.getNumberOfDataPoints();
      int numberOfDegreesOfFreedom = virtualChainData.getNumberOfDegreesOfFreedom();
      
      yMatrix = new DenseMatrix64F(2 * numberOfDataPoints, 1);
      AMatrix = new DenseMatrix64F(2 * numberOfDataPoints, 3 * numberOfDegreesOfFreedom);
      
      
      ArrayList<VirtualChainDataPoint> virtualChainDataPoints = virtualChainData.getVirtualChainDataPoints();
      
      for (int i=0; i<virtualChainDataPoints.size(); i++)
      {
         VirtualChainDataPoint virtualChainDataPoint = virtualChainDataPoints.get(i);
         
         int xRowOffset = 2*i;
         int yRowOffset = 2*i+1;
//         int zRowOffset = 3*i+2;
         
         FramePoint2d centerOfMassProjection = virtualChainDataPoint.getCenterOfMassProjection();
         yMatrix.set(xRowOffset, 0, centerOfMassProjection.getX());
         yMatrix.set(yRowOffset, 0, centerOfMassProjection.getY());
//         yMatrix.set(zRowOffset, 0, centerOfMassProjection.getZ());
         
         ArrayList<Matrix3d> rotationMatrices = virtualChainDataPoint.getRotationMatrices();

         for (int j=0; j<numberOfDegreesOfFreedom; j++)
         {
            Matrix3d rotationMatrix = rotationMatrices.get(j);
            int columnOffset = 3*j;
            
            for (int m=0; m<2; m++)
            {
               for (int n=0; n<3; n++)
               {
                  AMatrix.set(xRowOffset + m, columnOffset + n, rotationMatrix.getElement(m,n));
               }
            }
         }  
      }
   }
   
   public DenseMatrix64F getYMatrix()
   {
      return yMatrix;
   }
   
   public DenseMatrix64F getAMatrix()
   {
      return AMatrix;
   }
   
}
