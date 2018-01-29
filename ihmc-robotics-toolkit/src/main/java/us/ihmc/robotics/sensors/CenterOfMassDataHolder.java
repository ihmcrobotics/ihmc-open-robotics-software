package us.ihmc.robotics.sensors;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.robotics.screwTheory.GenericCRC32;

public class CenterOfMassDataHolder implements CenterOfMassDataHolderReadOnly
{
   private final FrameVector3D centerOfMassVelocity = new FrameVector3D();
   private final FrameVector3D centerOfMassAcceleration = new FrameVector3D();

   public void setCenterOfMassVelocity(FrameVector3DReadOnly centerOfMassVelocity)
   {
      this.centerOfMassVelocity.setIncludingFrame(centerOfMassVelocity); 
   }

   public void setCenterOfMassAcceleration(FrameVector3DReadOnly centerOfMassAcceleration)
   {
      this.centerOfMassAcceleration.setIncludingFrame(centerOfMassAcceleration);
   }

   public void set(CenterOfMassDataHolder estimatorCenterOfMassDataHolder)
   {
      this.centerOfMassVelocity.setIncludingFrame(estimatorCenterOfMassDataHolder.centerOfMassVelocity);
      this.centerOfMassAcceleration.setIncludingFrame(estimatorCenterOfMassDataHolder.centerOfMassAcceleration);
   }

   @Override
   public void getCenterOfMassVelocity(FrameVector3D centerOfMassVelocityToPack)
   {
      centerOfMassVelocityToPack.setIncludingFrame(centerOfMassVelocity);
   }

   @Override
   public void getCenterOfMassAcceleration(FrameVector3D centerOfMassAccelerationToPack)
   {
      centerOfMassAccelerationToPack.setIncludingFrame(centerOfMassAcceleration);
   }

   public void calculateChecksum(GenericCRC32 checksum)
   {
      checksum.update(centerOfMassVelocity);
   }

}
