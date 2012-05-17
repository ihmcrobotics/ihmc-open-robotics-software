package us.ihmc.commonWalkingControlModules.controlModuleInterfaces;

import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector;

public interface DoubleSupportForceDistributor
{
   void packForcesAndTorques(SideDependentList<Double> zForcesInPelvisFrameToPack, SideDependentList<FrameVector> torquesInPelvisFrameToPack,
                             double zForceInPelvisFrameTotal, FrameVector torqueInPelvisFrameTotal, 
                             SideDependentList<Double> legStrengths, SideDependentList<FramePoint2d> virtualToePoints);
}

