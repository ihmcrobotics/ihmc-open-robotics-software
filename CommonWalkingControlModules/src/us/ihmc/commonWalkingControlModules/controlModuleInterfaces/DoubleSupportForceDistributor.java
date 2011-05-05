package us.ihmc.commonWalkingControlModules.controlModuleInterfaces;

import us.ihmc.commonWalkingControlModules.SideDependentList;
import us.ihmc.utilities.math.geometry.FrameVector;

public interface DoubleSupportForceDistributor
{
   void packForcesAndTorques(SideDependentList<Double> zForcesInPelvisFrameToPack, SideDependentList<FrameVector> torquesInPelvisFrameToPack,
                             double zForceInPelvisFrameTotal, FrameVector torqueInPelvisFrameTotal, 
                             SideDependentList<Double> legStrengths);
}

