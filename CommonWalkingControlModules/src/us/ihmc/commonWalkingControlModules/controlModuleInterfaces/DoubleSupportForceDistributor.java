package us.ihmc.commonWalkingControlModules.controlModuleInterfaces;

import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.robotSide.SideDependentList;

public interface DoubleSupportForceDistributor
{
   void packForcesAndTorques(SideDependentList<Double> zForcesInPelvisFrameToPack, SideDependentList<FrameVector> torquesInPelvisFrameToPack,
                             double zForceInPelvisFrameTotal, FrameVector torqueInPelvisFrameTotal, 
                             SideDependentList<Double> legStrengths, SideDependentList<FramePoint2d> virtualToePoints);
}

