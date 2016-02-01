package us.ihmc.commonWalkingControlModules.controlModuleInterfaces;

import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.robotSide.SideDependentList;

public interface DoubleSupportForceDistributor
{
   void packForcesAndTorques(SideDependentList<Double> zForcesInPelvisFrameToPack, SideDependentList<FrameVector> torquesInPelvisFrameToPack,
                             double zForceInPelvisFrameTotal, FrameVector torqueInPelvisFrameTotal, 
                             SideDependentList<Double> legStrengths, SideDependentList<FramePoint2d> virtualToePoints);
}

