package us.ihmc.quadrupedRobotics.planning.chooser.footstepChooser;

import us.ihmc.quadrupedRobotics.geometry.supportPolygon.QuadrupedSupportPolygon;
import us.ihmc.robotics.geometry.FramePoint3D;
import us.ihmc.robotics.geometry.FrameVector3D;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public interface SwingTargetGenerator
{
   public abstract void getSwingTarget(RobotQuadrant swingLeg, FrameVector3D desiredBodyVelocity, FramePoint3D swingTargetToPack, double desiredYawRate);
   
   public abstract void getSwingTarget(QuadrupedSupportPolygon footPostions, RobotQuadrant swingLeg, FrameVector3D desiredBodyVelocity, FramePoint3D swingTargetToPack, double desiredYawRate);

   public abstract void getSwingTarget(RobotQuadrant swingLeg, ReferenceFrame swingLegAttachmentFrame, FrameVector3D desiredBodyVelocity, double swingDuration, FramePoint3D swingTargetToPack,
         double desiredYawRate);
}
