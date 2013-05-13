package us.ihmc.commonWalkingControlModules.wrenchDistribution;

import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.SpatialForceVector;

public interface OptimizerContactModel
{

   public int getSizeInRho();
   public int getSizeInPhi();
   public double getRhoMin(int i);
   public double getPhiMin(int i);
   public double getPhiMax(int i);
   public void packQRhoBodyFrame(int i, SpatialForceVector spatialForceVector, ReferenceFrame referenceFrame);
   public void packQPhiBodyFrame(int i, SpatialForceVector spatialForceVector, ReferenceFrame referenceFrame);

}
