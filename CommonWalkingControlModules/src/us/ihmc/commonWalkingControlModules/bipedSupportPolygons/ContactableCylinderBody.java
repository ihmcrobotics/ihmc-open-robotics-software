package us.ihmc.commonWalkingControlModules.bipedSupportPolygons;

import us.ihmc.utilities.math.geometry.FramePoint;

public interface ContactableCylinderBody extends ContactablePlaneBody
{
   public abstract FramePoint getCopyOfCylinderOriginInBodyFrame(); 
   public abstract double getCylinderRadius();
}