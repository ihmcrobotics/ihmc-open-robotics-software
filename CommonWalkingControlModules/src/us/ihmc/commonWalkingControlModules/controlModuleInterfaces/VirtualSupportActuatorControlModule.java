package us.ihmc.commonWalkingControlModules.controlModuleInterfaces;

import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegTorques;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LowerBodyTorques;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.utilities.screwTheory.Wrench;

public interface VirtualSupportActuatorControlModule
{
   /**
    * Packs a LegTorques object for the support leg side, given the vtp and the desired z-component of the force and the torques on the pelvis and the.
    * @param upperBodyWrench TODO
    */
   public abstract void controlSingleSupport(LegTorques supportLegTorquesToPack, FramePoint2d vtpInAnklePitchFrame, double fZOnPelvisInPelvisFrame,
           FrameVector torqueOnPelvisInPelvisFrame, Wrench upperBodyWrench);

   /**
    * Packs a LowerBodyTorques object, given the leg strengths, vtps and desired total z-component of force and torque on the pelvis.
    */
   public abstract void controlDoubleSupport(LowerBodyTorques lowerBodyTorquesToPack,
           SideDependentList<FramePoint2d> virtualToePoints, SideDependentList<Double> legStrengths, double fZOnPelvisInPelvisFrame,
           FrameVector torqueOnPelvis, double deltaNx);
}
