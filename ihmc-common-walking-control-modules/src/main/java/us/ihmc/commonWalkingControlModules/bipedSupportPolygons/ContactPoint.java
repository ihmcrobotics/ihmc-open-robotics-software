package us.ihmc.commonWalkingControlModules.bipedSupportPolygons;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple2DReadOnly;

public class ContactPoint implements ContactPointBasics
{
   private boolean inContact = false;
   private double x, y, z;
   private final ReferenceFrame referenceFrame;
   private final PlaneContactState parentContactState;

   public ContactPoint(ReferenceFrame referenceFrame, PlaneContactState parentContactState)
   {
      this.referenceFrame = referenceFrame;
      this.parentContactState = parentContactState;
   }

   public ContactPoint(FrameTuple2DReadOnly position, PlaneContactState parentContactState)
   {
      this.referenceFrame = position.getReferenceFrame();
      set(position, 0.0);
      this.parentContactState = parentContactState;
   }

   @Override
   public boolean isInContact()
   {
      return inContact;
   }

   @Override
   public void setInContact(boolean inContact)
   {
      this.inContact = inContact;
   }

   @Override
   public PlaneContactState getParentContactState()
   {
      return parentContactState;
   }

   @Override
   public void setX(double x)
   {
      this.x = x;
   }

   @Override
   public void setY(double y)
   {
      this.y = y;
   }

   @Override
   public void setZ(double z)
   {
      this.z = z;
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   @Override
   public double getX()
   {
      return x;
   }

   @Override
   public double getY()
   {
      return y;
   }

   @Override
   public double getZ()
   {
      return z;
   }
}
