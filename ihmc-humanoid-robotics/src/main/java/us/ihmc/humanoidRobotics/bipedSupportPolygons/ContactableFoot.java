package us.ihmc.humanoidRobotics.bipedSupportPolygons;

import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.robotics.geometry.FrameLineSegment2d;

public interface ContactableFoot extends ContactablePlaneBody
{
   public abstract void getToeOffContactPoint(FramePoint2D contactPointToPack);

   public abstract void getToeOffContactLine(FrameLineSegment2d contactLineToPack);
}
