package us.ihmc.commonWalkingControlModules.bipedSupportPolygons;

import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;

public interface ContactPointInterface
{
   public boolean isInContact();

   public void setInContact(boolean inContact);

   // TODO Needs to go away
   public FramePoint2d getPosition2d();

   public FramePoint getPosition();

   public PlaneContactState getParentContactState();

}
