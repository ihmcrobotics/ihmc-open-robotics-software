package us.ihmc.communication.packets;

public class RequestLidarScanMessage extends Packet<RequestLidarScanMessage>
{
   public boolean removeShadows = true;
   public boolean removeSelfCollisions = true;

   public RequestLidarScanMessage()
   {
   }

   public RequestLidarScanMessage(boolean removeShadows, boolean removeSelfCollisions)
   {
      this.removeShadows = removeShadows;
      this.removeSelfCollisions = removeSelfCollisions;
   }

   @Override
   public void set(RequestLidarScanMessage other)
   {
      removeShadows = other.removeShadows;
      removeSelfCollisions = other.removeSelfCollisions;
      setPacketInformation(other);
   }

   public boolean isRemoveShadows()
   {
      return removeShadows;
   }

   public void setRemoveShadows(boolean removeShadows)
   {
      this.removeShadows = removeShadows;
   }

   public boolean isRemoveSelfCollisions()
   {
      return removeSelfCollisions;
   }

   public void setRemoveSelfCollisions(boolean removeSelfCollisions)
   {
      this.removeSelfCollisions = removeSelfCollisions;
   }

   @Override
   public boolean epsilonEquals(RequestLidarScanMessage other, double epsilon)
   {
      if (removeShadows != other.removeShadows)
      {
         return false;
      }

      if (removeSelfCollisions != other.removeSelfCollisions)
      {
         return false;
      }
      return true;
   }
}
