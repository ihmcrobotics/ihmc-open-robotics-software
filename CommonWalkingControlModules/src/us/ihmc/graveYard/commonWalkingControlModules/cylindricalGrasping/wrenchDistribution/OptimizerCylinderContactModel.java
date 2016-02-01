package us.ihmc.graveYard.commonWalkingControlModules.cylindricalGrasping.wrenchDistribution;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.SpatialForceVector;

// TODO the problem could be defined in a simpler way making the problem faster to solve and the problem easier to understand
// Probably should get rid of either the phi or the rho part.
public class OptimizerCylinderContactModel implements OptimizerContactModel
{
   private static final int PHI_SIZE = 5;
   private static final int RHO_SIZE = 8;
   private final double[] phiMin = new double[PHI_SIZE];
   private final double[] phiMax = new double[PHI_SIZE];
   private final DenseMatrix64F[] qPhi = new DenseMatrix64F[PHI_SIZE];
   private final DenseMatrix64F[] qRho = new DenseMatrix64F[RHO_SIZE];
   private ReferenceFrame grippedCylinderFrame;
   private final static int PHI_X = 0;
   private final static int PHI_Y = 1; 
   private final static int PHI_Z = 2;
   private final static int PHI_XX = 3;
   private final static int PHI_YY = 4;
   private double wPhi;
   private double wRho;

   // Expects hand to be strongest pushing in positive y, and basically open in positive z, so pulling(positive z) is extremely weak. 
   // This weakness is described by the gripWeaknessFactor which goes from 0, a fully useless grip to 1, for a grip which is just as 
   // strong as the tensile max load)

   public OptimizerCylinderContactModel()
   {
      for (int i = 0; i < PHI_SIZE; i++)
         qPhi[i] = new DenseMatrix64F(SpatialForceVector.SIZE, 1);
      

      for (int i = 0; i < RHO_SIZE; i++)
         qRho[i] = new DenseMatrix64F(SpatialForceVector.SIZE, 1);
   }
   
   public int getRhoSize()
   {
      return RHO_SIZE;
   }

   public int getPhiSize()
   {
      return PHI_SIZE;
   }
   public ReferenceFrame getCylinderFrame()
   {
      return this.grippedCylinderFrame;
   }

   public double getRhoMin(int i)
   {
      return 0.0;
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
      spatialForceVector.set(grippedCylinderFrame, qRho[i]);
      spatialForceVector.changeFrame(referenceFrame);
   }

   public void packQPhiBodyFrame(int i, SpatialForceVector spatialForceVector, ReferenceFrame referenceFrame)
   {
      spatialForceVector.set(grippedCylinderFrame, qPhi[i]);
      spatialForceVector.changeFrame(referenceFrame);

   }

   public void setup(double mu, double cylinderRadius, double cylinderHalfHandWidth, double cylinderTensileGripStrength, double gripWeaknessFactor,
                     ReferenceFrame grippedCylinderFrame, double wRho, double wPhi)
   {
      this.grippedCylinderFrame = grippedCylinderFrame;
      setupPhiLimits(mu, cylinderRadius, cylinderHalfHandWidth, cylinderTensileGripStrength, gripWeaknessFactor);
      setQPhiToFirstFiveColumnsOfIdentity();
      setupQRho(mu, cylinderRadius, cylinderHalfHandWidth);
      this.wRho = wRho;
      this.wPhi = wPhi;
   }

   private void setupQRho(double mu, double cylinderRadius, double cylinderHalfHandWidth)
   {
      int qRhoLocation = 0;
      for (int x = -1; x <= 1; x += 2)
      {
         for (int xx = -1; xx <= 1; xx += 2)
         {
            for (int zz = -1; zz <= 1; zz += 2)
            {
               qRho[qRhoLocation].set(0, 0, xx * mu * cylinderRadius);
               qRho[qRhoLocation].set(1, 0, 0.0);
               qRho[qRhoLocation].set(2, 0, zz * cylinderHalfHandWidth);
               qRho[qRhoLocation].set(3, 0, x * mu);
               qRho[qRhoLocation].set(4, 0, -1.0);
               qRho[qRhoLocation].set(5, 0, 0.0);
               qRhoLocation++;
            }
         }
      }
   }

   private void setQPhiToFirstFiveColumnsOfIdentity()
   {
      qPhi[PHI_X].set(3, 0, 1);
      qPhi[PHI_Y].set(4, 0, 1);
      qPhi[PHI_Z].set(5, 0, 1);
      qPhi[PHI_XX].set(0, 0, 1);
      qPhi[PHI_YY].set(1, 0, 1);
   }

   private void setupPhiLimits(double mu, double cylinderRadius, double cylinderHalfHandWidth, double cylinderTensileGripStrength, double gripWeaknessFactor)
   {
      phiMin[PHI_X] = -cylinderTensileGripStrength * mu;
      phiMin[PHI_Y] = -cylinderTensileGripStrength;
      phiMin[PHI_Z] = -cylinderTensileGripStrength;
      phiMin[PHI_XX] = -cylinderTensileGripStrength * mu * cylinderRadius;
      phiMin[PHI_YY] = -cylinderTensileGripStrength * gripWeaknessFactor * cylinderHalfHandWidth;

      for (int i = 0; i < PHI_SIZE; i++)
         phiMax[i] = -phiMin[i];
   
      phiMax[PHI_Z] *= gripWeaknessFactor;
   }
   
   public double getWPhi()
   {
      return wPhi;
   }

   public double getWRho()
   {
      return wRho;
   }
}
