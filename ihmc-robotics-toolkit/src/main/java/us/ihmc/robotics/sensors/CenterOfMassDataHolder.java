package us.ihmc.robotics.sensors;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.robotics.screwTheory.GenericCRC32;

public class CenterOfMassDataHolder implements CenterOfMassDataHolderReadOnly
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private boolean hasPosition = false;
   private boolean hasVelocity = false;
   private final FixedFramePoint3DBasics centerOfMassPosition = new FramePoint3D(worldFrame);
   private final FixedFrameVector3DBasics centerOfMassVelocity = new FrameVector3D(worldFrame);

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

   public void setCenterOfMassPosition(FramePoint3DReadOnly centerOfMassPosition)
   {
      hasPosition = true;
      this.centerOfMassPosition.setMatchingFrame(centerOfMassPosition);
   }

   public void setCenterOfMassVelocity(FrameVector3DReadOnly centerOfMassVelocity)
   {
      hasVelocity = true;
      this.centerOfMassVelocity.setMatchingFrame(centerOfMassVelocity);
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
   public FramePoint3DReadOnly getCenterOfMassPosition()
   {
      return centerOfMassPosition;
   }

   @Override
   public boolean hasCenterOfMassVelocity()
   {
      return hasVelocity;
   }

   @Override
   public FrameVector3DReadOnly getCenterOfMassVelocity()
   {
      return centerOfMassVelocity;
   }

   public void calculateChecksum(GenericCRC32 checksum)
   {
      checksum.update(hasPosition);
      checksum.update(hasVelocity);
      checksum.update(centerOfMassPosition);
      checksum.update(centerOfMassVelocity);
   }
}
