package us.ihmc.robotics.sensors;

import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.screwTheory.GenericCRC32;

public class CenterOfMassDataHolder implements CenterOfMassDataHolderReadOnly
{
   private final FrameVector centerOfMassVelocity = new FrameVector();
   
   public void setCenterOfMassVelocity(FrameVector centerOfMassVelocity)
   {
      this.centerOfMassVelocity.setIncludingFrame(centerOfMassVelocity); 
   }

   public void set(CenterOfMassDataHolder estimatorCenterOfMassDataHolder)
   {
      this.centerOfMassVelocity.setIncludingFrame(estimatorCenterOfMassDataHolder.centerOfMassVelocity);
   }

   @Override
   public void getCenterOfMassVelocity(FrameVector centerOfMassVelocityToPack)
   {
      centerOfMassVelocityToPack.setIncludingFrame(centerOfMassVelocity);
   }

   public void calculateChecksum(GenericCRC32 checksum)
   {
      checksum.update(centerOfMassVelocity.getVector());
   }

}
