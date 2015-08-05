package us.ihmc.commonWalkingControlModules.bipedSupportPolygons;

import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;

public interface ContactableRollingBody extends ContactablePlaneBody
{
   public abstract FramePoint getCylinderOriginCopy(); 
   public abstract double getCylinderRadius();
}