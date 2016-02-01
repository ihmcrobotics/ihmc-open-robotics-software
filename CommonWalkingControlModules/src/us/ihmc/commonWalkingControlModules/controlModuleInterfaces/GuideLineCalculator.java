package us.ihmc.commonWalkingControlModules.controlModuleInterfaces;


import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.OldBipedSupportPolygons;
import us.ihmc.robotics.geometry.FrameLineSegment2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.robotSide.RobotSide;

public interface GuideLineCalculator
{
   public abstract void update(RobotSide supportLeg, OldBipedSupportPolygons bipedSupportPolygons, FramePoint2d capturePoint,
                      FramePoint finalDesiredSwingTarget, FrameVector2d desiredVelocity);

   public abstract void reset();

   public abstract FrameLineSegment2d getGuideLine(RobotSide supportLeg);
}
