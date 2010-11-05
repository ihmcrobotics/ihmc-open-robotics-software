package us.ihmc.commonWalkingControlModules.controlModuleInterfaces;


import us.ihmc.commonWalkingControlModules.RobotSide;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.utilities.math.geometry.FrameLineSegment2d;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector2d;

public interface GuideLineCalculator
{
   public abstract void update(RobotSide supportLeg, BipedSupportPolygons bipedSupportPolygons, FramePoint2d capturePointInSupportFootZUp,
                      FramePoint finalDesiredSwingTarget, FrameVector2d desiredVelocityInSupportFootFrame,
                      FrameVector2d actualCenterOfMassVelocityInSupportFootFrame);

   public abstract void reset();

   public abstract FrameLineSegment2d getGuideLine(RobotSide supportLeg);
}
