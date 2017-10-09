package us.ihmc.quadrupedRobotics.planning.chooser.footstepChooser;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.quadrupedRobotics.geometry.supportPolygon.QuadrupedSupportPolygon;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public interface SwingTargetGenerator
{
   public abstract void getSwingTarget(RobotQuadrant swingLeg, FrameVector3D desiredBodyVelocity, FramePoint3D swingTargetToPack, double desiredYawRate);
   
   public abstract void getSwingTarget(QuadrupedSupportPolygon footPostions, RobotQuadrant swingLeg, FrameVector3D desiredBodyVelocity, FramePoint3D swingTargetToPack, double desiredYawRate);

   public abstract void getSwingTarget(RobotQuadrant swingLeg, ReferenceFrame swingLegAttachmentFrame, FrameVector3D desiredBodyVelocity, double swingDuration, FramePoint3D swingTargetToPack,
         double desiredYawRate);
}
