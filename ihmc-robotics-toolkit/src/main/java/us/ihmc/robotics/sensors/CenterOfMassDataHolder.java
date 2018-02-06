package us.ihmc.robotics.sensors;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.robotics.screwTheory.GenericCRC32;

public class CenterOfMassDataHolder implements CenterOfMassDataHolderReadOnly
{
   private final FrameVector3D centerOfMassVelocity = new FrameVector3D();
   
   public void setCenterOfMassVelocity(FrameVector3DReadOnly centerOfMassVelocity)
   {
      this.centerOfMassVelocity.setIncludingFrame(centerOfMassVelocity); 
   }

   public void set(CenterOfMassDataHolder estimatorCenterOfMassDataHolder)
   {
      this.centerOfMassVelocity.setIncludingFrame(estimatorCenterOfMassDataHolder.centerOfMassVelocity);
   }

   @Override
   public void getCenterOfMassVelocity(FrameVector3D centerOfMassVelocityToPack)
   {
      centerOfMassVelocityToPack.setIncludingFrame(centerOfMassVelocity);
   }

   public void calculateChecksum(GenericCRC32 checksum)
   {
      checksum.update(centerOfMassVelocity);
   }

}
