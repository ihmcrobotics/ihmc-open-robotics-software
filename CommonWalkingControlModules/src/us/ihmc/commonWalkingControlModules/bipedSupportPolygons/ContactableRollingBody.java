package us.ihmc.commonWalkingControlModules.bipedSupportPolygons;

import us.ihmc.utilities.math.geometry.FramePoint;

public interface ContactableRollingBody extends ContactablePlaneBody
{
   public abstract FramePoint getCopyOfCylinderOriginInBodyFrame(); 
   public abstract double getCylinderRadius();
}