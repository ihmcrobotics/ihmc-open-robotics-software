package us.ihmc.robotics.sensors;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.screwTheory.GenericCRC32;

/**
 * Data holder used to pass center of mass position and velocity from the state estimator thread to
 * the controller thread.
 */
public class CenterOfMassDataHolder implements CenterOfMassDataHolderReadOnly
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private boolean hasPosition = false;
   private boolean hasVelocity = false;
   private final FramePoint3D centerOfMassPosition = new FramePoint3D(worldFrame);
   private final FrameVector3D centerOfMassVelocity = new FrameVector3D(worldFrame);

   public CenterOfMassDataHolder()
   {
      clear();
   }

   public void clear()
   {
      hasPosition = false;
      hasVelocity = false;
      centerOfMassPosition.setToNaN();
      centerOfMassVelocity.setToNaN();
   }

   public void setCenterOfMassPosition(ReferenceFrame referenceFrame, Point3DReadOnly centerOfMassPosition)
   {
      hasPosition = true;
      this.centerOfMassPosition.setReferenceFrame(worldFrame);
      this.centerOfMassPosition.setMatchingFrame(referenceFrame, centerOfMassPosition);
   }

   public void setCenterOfMassPosition(FramePoint3DReadOnly centerOfMassPosition)
   {
      setCenterOfMassPosition(centerOfMassPosition.getReferenceFrame(), centerOfMassPosition);
   }

   public void setCenterOfMassVelocity(ReferenceFrame referenceFrame, Vector3DReadOnly centerOfMassVelocity)
   {
      hasVelocity = true;
      this.centerOfMassVelocity.setReferenceFrame(worldFrame);
      this.centerOfMassVelocity.setMatchingFrame(referenceFrame, centerOfMassVelocity);
   }

   public void setCenterOfMassVelocity(FrameVector3DReadOnly centerOfMassVelocity)
   {
      setCenterOfMassVelocity(centerOfMassVelocity.getReferenceFrame(), centerOfMassVelocity);
   }

   public void set(CenterOfMassDataHolder other)
   {
      hasPosition = other.hasPosition;
      hasVelocity = other.hasVelocity;
      centerOfMassPosition.set(other.centerOfMassPosition);
      centerOfMassVelocity.set(other.centerOfMassVelocity);
   }

   @Override
   public boolean hasCenterOfMassPosition()
   {
      return hasPosition;
   }

   @Override
   public FramePoint3D getCenterOfMassPosition()
   {
      return centerOfMassPosition;
   }

   @Override
   public boolean hasCenterOfMassVelocity()
   {
      return hasVelocity;
   }

   @Override
   public FrameVector3D getCenterOfMassVelocity()
   {
      return centerOfMassVelocity;
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
      {
         return true;
      }
      else if (object instanceof CenterOfMassDataHolder)
      {
         CenterOfMassDataHolder other = (CenterOfMassDataHolder) object;
         if (hasPosition != other.hasPosition)
            return false;
         if (hasVelocity != other.hasVelocity)
            return false;
         if (hasCenterOfMassPosition() && !centerOfMassPosition.equals(other.centerOfMassPosition))
            return false;
         if (hasCenterOfMassVelocity() && !centerOfMassVelocity.equals(other.centerOfMassVelocity))
            return false;
         return true;
      }
      else
      {
         return false;
      }
   }

   public void calculateChecksum(GenericCRC32 checksum)
   {
      checksum.update(hasPosition);
      checksum.update(hasVelocity);
      checksum.update(centerOfMassPosition);
      checksum.update(centerOfMassVelocity);
   }
}
