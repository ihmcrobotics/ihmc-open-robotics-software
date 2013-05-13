package us.ihmc.commonWalkingControlModules.wrenchDistribution;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.utilities.math.geometry.PoseReferenceFrame;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.SpatialForceVector;

public class OptimizerCylinderContactModel implements OptimizerContactModel
{

   private static final int PHI_SIZE = 5;
   private static final int RHO_SIZE = 8;
   private final double[] phiMin = new double[PHI_SIZE];
   private final double[] phiMax = new double[PHI_SIZE];
   private final DenseMatrix64F[] qPhi = new DenseMatrix64F[PHI_SIZE];
   private final DenseMatrix64F[] qRho = new DenseMatrix64F[RHO_SIZE];
   private static final int[] phiVectorDirections = new int[]{3,4,5,0,1};

   public int getSizeInRho()
   {
      return RHO_SIZE;
   }

   public int getSizeInPhi()
   {
      return PHI_SIZE;
   }

   public double getRhoMin(int i)
   {
      return 0;
   }

   public double getPhiMin(int i)
   {
      return phiMin[i];
   }

   public double getPhiMax(int i)
   {
      return phiMax[i];
   }

   public void packQRhoBodyFrame(int i, SpatialForceVector spatialForceVector, ReferenceFrame referenceFrame)
   {
      spatialForceVector.set(referenceFrame, qRho[i]);
   }

   public void packQPhiBodyFrame(int i, SpatialForceVector spatialForceVector, ReferenceFrame referenceFrame)
   {
      spatialForceVector.set(referenceFrame, qPhi[i]);

   }

   public void setup(double mu, double cylinderRadius, double cylinderHalfHandWidth,
         double cylinderTensileGripStrength)
   {
      initializePhiLimitsToHandStrength(cylinderTensileGripStrength);
      accountForFrictionalContact(mu);
      accountForMomentArms(cylinderRadius, cylinderHalfHandWidth);
      setQPhiToFirstFiveColumnsOfIdentity();
      setupQRho(mu, cylinderRadius, cylinderHalfHandWidth);
   }

   private void setupQRho(double mu, double cylinderRadius, double cylinderHalfHandWidth)
   {
      int qRhoLocation=0;
      for (int x=-1;x<=1;x+=2)
      {
         for (int xx=-1;xx<=1;xx+=2)
         {
            for (int zz=-1;zz<=1;zz+=2)
            {
               qRho[qRhoLocation] = new DenseMatrix64F(6,1,false,xx*mu*cylinderRadius,0.0,zz*cylinderHalfHandWidth,x*mu,1.0,0.0);
               qRhoLocation++;
            }
         }
      }
   }

   private void setQPhiToFirstFiveColumnsOfIdentity()
   {
      for (int i=0;i<PHI_SIZE;i++)
      {
         qPhi[i] = new DenseMatrix64F(SpatialForceVector.SIZE,1);
         qPhi[i].set(phiVectorDirections[i], 0, 1);
      }
   }

   private void initializePhiLimitsToHandStrength(double cylinderTensileGripStrength)
   {
      for (int i=0;i<PHI_SIZE;i++)
      {
         phiMin[i]=-cylinderTensileGripStrength;
         phiMax[i]=cylinderTensileGripStrength;
      }
   }

   private void accountForFrictionalContact(double mu)
   {
      for (int i=0;i<2;i++)
      {
         phiMin[i*3]*=mu;
         phiMax[i*3]*=mu;
      }
   }
      
   private void accountForMomentArms(double cylinderRadius, double cylinderHalfHandWidth)
   {
      phiMin[0]*=cylinderRadius;
      phiMax[0]*=cylinderRadius;
      phiMin[1]*=cylinderHalfHandWidth;
      phiMax[1]*=cylinderHalfHandWidth;
   }

}
