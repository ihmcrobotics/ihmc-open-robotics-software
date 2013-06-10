package us.ihmc.commonWalkingControlModules.wrenchDistribution;

import java.util.List;

import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.SpatialForceVector;

public class OptimizerPlaneContactModel implements OptimizerContactModel
{
   private double rhoMin;
   private static final int VECTORS = 3;
   private static final int MAXPOINTS = 4;
   private int points=MAXPOINTS;
   private static final int MAX_RHO_SIZE = MAXPOINTS * VECTORS;
   private static final double ANGLE_INCRMENT = 2 * Math.PI / ((double) VECTORS);
   private double mu = 0.3;
   private double wRho;
   private final DenseMatrix64F[] rhoQ = new DenseMatrix64F[MAX_RHO_SIZE];
   private SpatialForceVector tempForceVector= new SpatialForceVector();
   private Vector3d tempLinearPart = new Vector3d();
   private Vector3d tempArm = new Vector3d();


   public OptimizerPlaneContactModel()
   {
      for (int i = 0; i < MAXPOINTS; i++)
      {
         for (int j = 0; j < VECTORS; j++)
         {
            int rhoPosition = i * VECTORS + j;
            rhoQ[rhoPosition]= new DenseMatrix64F(6,1);
         }
      }
   }
   
   public void setup(double coefficientOfFriction, List<FramePoint> contactPointsInPlaneFrame, ReferenceFrame endEffectorFrame, double wRho, double rhoMin)
   {
      this.mu = coefficientOfFriction;
      points = contactPointsInPlaneFrame.size();
      if ((points >MAXPOINTS)||(points<0))
      {
         throw new RuntimeException("Unhandled number of contact points: "+points);
      }
      for (int i = 0; i < points; i++)
      {
         for (int j = 0; j < VECTORS; j++)
         {
            int rhoPosition = i * VECTORS + j;
            
            double angle = j*ANGLE_INCRMENT;
            tempLinearPart.set(Math.cos(angle)*mu, Math.sin(angle)*mu, 1);
            tempLinearPart.normalize();
            
            FramePoint framePoint = contactPointsInPlaneFrame.get(i);
            tempArm.set(framePoint.getX(), framePoint.getY(), 0.0);
            tempForceVector.setUsingArm(framePoint.getReferenceFrame(), tempLinearPart,
                    tempArm);            
            tempForceVector.changeFrame(endEffectorFrame);
            
            tempForceVector.packMatrix(rhoQ[rhoPosition]);
         }
      }
      this.wRho = wRho;
      this.rhoMin = rhoMin;
   }
   
   public int getSizeInRho()
   {
      return VECTORS*points;
   }

   public int getSizeInPhi()
   {
      return 0;
   }

   public double getRhoMin(int i)
   {
      return rhoMin;
   }

   public double getPhiMin(int i)
   {
      return 0;
   }

   public double getPhiMax(int i)
   {
      return 0;
   }

   public void packQRhoBodyFrame(int i, SpatialForceVector spatialForceVector, ReferenceFrame referenceFrame)
   {
      spatialForceVector.set(referenceFrame, rhoQ[i]);
   }

   public void packQPhiBodyFrame(int i, SpatialForceVector spatialForceVector, ReferenceFrame referenceFrame)
   {
   }

   public double getWPhi()
   {
      return Double.NaN;
   }

   public double getWRho()
   {
      return wRho;
   }

}
