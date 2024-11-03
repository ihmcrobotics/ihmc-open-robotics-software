package us.ihmc.commonWalkingControlModules.bipedSupportPolygons;

import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;

public interface ContactPointBasics extends FixedFramePoint3DBasics
{
   boolean isInContact();

   void setInContact(boolean inContact);

   PlaneContactState getParentContactState();
}
