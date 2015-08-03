package us.ihmc.commonWalkingControlModules.controlModules.virtualSupportActuator;

import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.VirtualSupportActuatorControlModule;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegTorques;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LowerBodyTorques;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.Wrench;

public class DoNothingVirtualSupportActuatorControlModule implements VirtualSupportActuatorControlModule
{
   public void controlSingleSupport(LegTorques supportLegTorquesToPack, FramePoint2d vtpInAnklePitchFrame, double fZOnPelvisInPelvisFrame,
                                    FrameVector torqueOnPelvisInPelvisFrame, Wrench upperBodyWrench)
   {
      supportLegTorquesToPack.setTorquesToZero();
   }

   public void controlDoubleSupport(LowerBodyTorques lowerBodyTorquesToPack, SideDependentList<FramePoint2d> virtualToePoints,
         SideDependentList<Double> legStrengths, double fZOnPelvisInPelvisFrame, FrameVector torqueOnPelvisInPelvisFrame, double deltaNx)
   {
      lowerBodyTorquesToPack.setLowerBodyTorquesToZero();      
   }
}
