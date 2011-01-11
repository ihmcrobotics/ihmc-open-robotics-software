package us.ihmc.commonWalkingControlModules.controlModules.virtualSupportActuator;

import us.ihmc.commonWalkingControlModules.SideDependentList;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedLegStrengthAndVirtualToePoint;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.VirtualSupportActuatorControlModule;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegTorques;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LowerBodyTorques;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.screwTheory.Wrench;

public class DoNothingVirtualSupportActuatorControlModule implements VirtualSupportActuatorControlModule
{
   public void controlSingleSupport(LegTorques supportLegTorquesToPack, FramePoint2d vtpInAnklePitchFrame, double fZOnPelvisInPelvisFrame,
                                    FrameVector torqueOnPelvisInPelvisFrame, Wrench upperBodyWrench)
   {
      supportLegTorquesToPack.setTorquesToZero();
   }

   public void controlDoubleSupport(LowerBodyTorques lowerBodyTorquesToPack, SideDependentList<BipedLegStrengthAndVirtualToePoint> legStrengthsAndVirtualToePoints,
                                    double fZOnPelvisInPelvisFrame, FrameVector torqueOnPelvisInPelvisFrame)
   {
      lowerBodyTorquesToPack.setLowerBodyTorquesToZero();
   }

}
