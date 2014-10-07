package us.ihmc.commonWalkingControlModules.bipedSupportPolygons;

import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.yoUtilities.humanoidRobot.bipedSupportPolygons.ContactablePlaneBody;

public interface ContactableRollingBody extends ContactablePlaneBody
{
   public abstract FramePoint getCylinderOriginCopy(); 
   public abstract double getCylinderRadius();
}