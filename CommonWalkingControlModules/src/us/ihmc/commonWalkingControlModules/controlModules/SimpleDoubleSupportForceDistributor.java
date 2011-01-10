package us.ihmc.commonWalkingControlModules.controlModules;

import us.ihmc.commonWalkingControlModules.RobotSide;
import us.ihmc.commonWalkingControlModules.SideDependentList;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedLegStrengthAndVirtualToePoint;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.DoubleSupportForceDistributor;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

public class SimpleDoubleSupportForceDistributor implements DoubleSupportForceDistributor
{
   private final ReferenceFrame spineRollFrame;

   public SimpleDoubleSupportForceDistributor(CommonWalkingReferenceFrames referenceFrames)
   {
      this.spineRollFrame = referenceFrames.getPelvisFrame();
   }

   public void packForcesAndTorques(SideDependentList<Double> zForcesInPelvisFrameToPack, SideDependentList<FrameVector> torquesOnPelvis,
                                    double zForceInPelvisFrameTotal, FrameVector torqueOnPelvisTotal,
                                    SideDependentList<BipedLegStrengthAndVirtualToePoint> legStrengthsAndVirtualToePoints)
   {
      for (RobotSide robotSide : RobotSide.values())
      {
         double legStrength = legStrengthsAndVirtualToePoints.get(robotSide).getFilteredLegStrength();

         zForcesInPelvisFrameToPack.put(robotSide, zForceInPelvisFrameTotal * legStrength);

         FrameVector torque = torqueOnPelvisTotal.changeFrameCopy(spineRollFrame);
         torque.scale(legStrength);

         torquesOnPelvis.set(robotSide, torque);
      }
   }

}
