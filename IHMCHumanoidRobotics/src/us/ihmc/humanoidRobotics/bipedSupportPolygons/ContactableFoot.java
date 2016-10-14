package us.ihmc.humanoidRobotics.bipedSupportPolygons;

import us.ihmc.robotics.geometry.FramePoint2d;

public interface ContactableFoot extends ContactablePlaneBody
{
   public abstract void getToeOffContactPoint(FramePoint2d contactPointToPack);
}
