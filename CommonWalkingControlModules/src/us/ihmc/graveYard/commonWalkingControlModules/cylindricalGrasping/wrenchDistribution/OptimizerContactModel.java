package us.ihmc.graveYard.commonWalkingControlModules.cylindricalGrasping.wrenchDistribution;

import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.SpatialForceVector;

public interface OptimizerContactModel
{

   public int getRhoSize();
   public int getPhiSize();
   public double getRhoMin(int i);
   public double getPhiMin(int i);
   public double getPhiMax(int i);
   public void packQRhoBodyFrame(int i, SpatialForceVector spatialForceVector, ReferenceFrame referenceFrame);
   public void packQPhiBodyFrame(int i, SpatialForceVector spatialForceVector, ReferenceFrame referenceFrame);
   public double getWPhi();
   public double getWRho();
}
