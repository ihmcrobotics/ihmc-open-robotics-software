package us.ihmc.commonWalkingControlModules.bipedSupportPolygons;

import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;

public class ContactPoint implements ContactPointInterface
{
   private boolean inContact = false;
   private final FramePoint position;
   // TODO Needs to go away
   private final FramePoint2d position2d;
   private final PlaneContactState parentContactState;

   public ContactPoint(FramePoint2d point2d, PlaneContactState parentContactState)
   {
      // TODO Needs to go away
      position2d = point2d;
      position = new FramePoint(position2d.getReferenceFrame(), position2d.getX(), position2d.getY(), 0.0);
      this.parentContactState = parentContactState;
   }

   public boolean isInContact()
   {
      return inContact;
   }

   public void setInContact(boolean inContact)
   {
      this.inContact = inContact;
   }

   // TODO Needs to go away
   public FramePoint2d getPosition2d()
   {
      return position2d;
   }

   public FramePoint getPosition()
   {
      return position;
   }

   public PlaneContactState getParentContactState()
   {
      return parentContactState;
   }
}
