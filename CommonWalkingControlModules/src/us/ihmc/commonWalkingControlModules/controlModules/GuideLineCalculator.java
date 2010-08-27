package us.ihmc.commonWalkingControlModules.controlModules;


import us.ihmc.commonWalkingControlModules.RobotSide;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.utilities.math.geometry.*;

public interface GuideLineCalculator
{
   public void update(RobotSide supportLeg, BipedSupportPolygons bipedSupportPolygons, FramePoint2d capturePointInSupportFootZUp,
                      FramePoint finalDesiredSwingTarget, FrameVector2d desiredVelocityInSupportFootFrame,
                      FrameVector2d actualCenterOfMassVelocityInSupportFootFrame);

   public void reset();

   public FrameLineSegment2d getGuideLine(RobotSide supportLeg);
}
