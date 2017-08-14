package us.ihmc.humanoidRobotics.bipedSupportPolygons;

import us.ihmc.robotics.geometry.FrameLineSegment2d;
import us.ihmc.robotics.geometry.FramePoint2D;

public interface ContactableFoot extends ContactablePlaneBody
{
   public abstract void getToeOffContactPoint(FramePoint2D contactPointToPack);

   public abstract void getToeOffContactLine(FrameLineSegment2d contactLineToPack);
}
