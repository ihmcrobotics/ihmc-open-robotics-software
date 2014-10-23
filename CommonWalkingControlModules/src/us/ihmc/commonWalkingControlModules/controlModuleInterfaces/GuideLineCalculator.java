package us.ihmc.commonWalkingControlModules.controlModuleInterfaces;


import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.utilities.math.geometry.FrameLineSegment2d;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.robotSide.RobotSide;

public interface GuideLineCalculator
{
   public abstract void update(RobotSide supportLeg, BipedSupportPolygons bipedSupportPolygons, FramePoint2d capturePoint,
                      FramePoint finalDesiredSwingTarget, FrameVector2d desiredVelocity);

   public abstract void reset();

   public abstract FrameLineSegment2d getGuideLine(RobotSide supportLeg);
}
