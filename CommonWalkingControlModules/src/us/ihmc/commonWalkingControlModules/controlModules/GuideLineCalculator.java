package us.ihmc.commonWalkingControlModules.controlModules;


import us.ihmc.commonWalkingControlModules.RobotSide;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.utilities.math.geometry.FrameLineSegment2d;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;

public interface GuideLineCalculator
{
   public void update(RobotSide side, BipedSupportPolygons bipedSupportPolygons, FramePoint2d capturePointInSupportFootZUp, FramePoint finalDesiredSwingTarget);

   public void reset();

   public FrameLineSegment2d getGuideLine(RobotSide supportLeg);
}
