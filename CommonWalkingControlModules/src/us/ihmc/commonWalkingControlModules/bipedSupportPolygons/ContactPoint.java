package us.ihmc.commonWalkingControlModules.bipedSupportPolygons;

import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;

public class ContactPoint
{
   private boolean isTrusted = false;
   private boolean inContact = false;
   private final FramePoint position;
   private final FramePoint2d position2d;

   public ContactPoint(FramePoint2d point2d)
   {
      position2d = point2d;
      position = new FramePoint(position2d.getReferenceFrame(), position2d.getX(), position2d.getY(), 0.0);
   }

   public boolean isInContact()
   {
      return inContact;
   }

   public void setInContact(boolean inContact)
   {
      this.inContact = inContact;
   }

   public boolean isTrusted()
   {
      return isTrusted;
   }

   /**
    * Specify if the contact point can be trusted for state estimation.
    * @param isTrusted
    */
   public void setTrusted(boolean isTrusted)
   {
      this.isTrusted = isTrusted;
   }

   public FramePoint2d getPosition2d()
   {
      return position2d;
   }

   public FramePoint getPosition()
   {
      return position;
   }
}
