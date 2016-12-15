package us.ihmc.communication.packets;

public class BodyPartsTrackingWeightsPacket extends StatusPacket<BodyPartsTrackingWeightsPacket>
{
   private double chestWeight;
   private double pelvisOrientationWeight;

   public BodyPartsTrackingWeightsPacket(double chestWeight, double pelvisOrientationWeight)
   {
      this.chestWeight = chestWeight;
      this.pelvisOrientationWeight = pelvisOrientationWeight;
   }

   public BodyPartsTrackingWeightsPacket()
   {
      this.chestWeight = 0.02;
      this.pelvisOrientationWeight = 0.02;
   }

   public void increaseBodyWeights()
   {
         this.chestWeight = 100.0;
         this.pelvisOrientationWeight = 100.0;
   }

   @Override
   public void set(BodyPartsTrackingWeightsPacket other)
   {
      this.chestWeight = other.chestWeight;
      this.pelvisOrientationWeight = other.pelvisOrientationWeight;
   }

   @Override
   public boolean epsilonEquals(BodyPartsTrackingWeightsPacket other, double epsilon)
   {
      return true;
   }

}
