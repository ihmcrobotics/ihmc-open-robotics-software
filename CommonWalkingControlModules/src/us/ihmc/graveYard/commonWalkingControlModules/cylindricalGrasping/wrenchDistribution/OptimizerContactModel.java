package us.ihmc.graveYard.commonWalkingControlModules.cylindricalGrasping.wrenchDistribution;

import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.SpatialForceVector;

public interface OptimizerContactModel
{

   public abstract int getRhoSize();
   public abstract int getPhiSize();
   public abstract double getRhoMin(int i);
   public abstract double getPhiMin(int i);
   public abstract double getPhiMax(int i);
   public abstract void getQRhoBodyFrame(int i, SpatialForceVector spatialForceVector, ReferenceFrame referenceFrame);
   public abstract void getQPhiBodyFrame(int i, SpatialForceVector spatialForceVector, ReferenceFrame referenceFrame);
   public abstract double getWPhi();
   public abstract double getWRho();
}
