package us.ihmc.commonWalkingControlModules.controlModuleInterfaces;

import us.ihmc.commonWalkingControlModules.SideDependentList;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegTorques;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LowerBodyTorques;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector;

public interface VirtualSupportActuatorControlModule
{
   /**
    * Packs a LegTorques object for the support leg side, given the vtp and the desired z-component of the force and the torques on the pelvis and the.
    */
   public abstract void controlSingleSupport(LegTorques supportLegTorquesToPack, FramePoint2d vtpInAnklePitchFrame, double fZOnPelvisInPelvisFrame,
                             FrameVector torqueOnPelvisInPelvisFrame);

   /**
    * Packs a LowerBodyTorques object, given the leg strengths, vtps and desired total z-component of force and torque on the pelvis.
    */
   public abstract void controlDoubleSupport(LowerBodyTorques lowerBodyTorquesToPack, SideDependentList<FramePoint2d> vtpsInAnklePitchFrame,
                             SideDependentList<Double> legStrengths, double fZOnPelvisInPelvisFrame, FrameVector torqueOnPelvisInPelvisFrame);
}
