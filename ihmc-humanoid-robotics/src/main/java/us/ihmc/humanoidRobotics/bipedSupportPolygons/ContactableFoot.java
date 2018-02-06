package us.ihmc.humanoidRobotics.bipedSupportPolygons;

import us.ihmc.euclid.referenceFrame.FrameLineSegment2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;

public interface ContactableFoot extends ContactablePlaneBody
{
   public abstract void getToeOffContactPoint(FramePoint2D contactPointToPack);

   public abstract void getToeOffContactLine(FrameLineSegment2D contactLineToPack);
}
