package us.ihmc.commonWalkingControlModules.controlModuleInterfaces;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.OldBipedSupportPolygons;
import us.ihmc.robotics.geometry.FrameLineSegment2d;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.robotSide.RobotSide;

public interface GuideLineToDesiredCoPControlModule
{
   public abstract FramePoint2d computeDesiredCoPSingleSupport(RobotSide supportLeg, OldBipedSupportPolygons bipedSupportPolygons, FramePoint2d capturePoint, FrameVector2d desiredVelocity,
           FrameLineSegment2d guideLine);
}
