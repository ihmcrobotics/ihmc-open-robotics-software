package us.ihmc.robotEnvironmentAwareness.communication.packets;

import java.util.Arrays;

import us.ihmc.communication.packets.Packet;
import us.ihmc.jOctoMap.key.OcTreeKey;
import us.ihmc.jOctoMap.key.OcTreeKeyReadOnly;

public class OcTreeKeyMessage extends Packet<OcTreeKeyMessage> implements OcTreeKeyReadOnly
{
   public int[] k;

   public OcTreeKeyMessage()
   {
   }

   public OcTreeKeyMessage(int k0, int k1, int k2)
   {
      set(k0, k1, k2);
   }

   public OcTreeKeyMessage(OcTreeKeyReadOnly other)
   {
      set(other);
   }

   @Override
   public void set(OcTreeKeyMessage other)
   {
      set(other.getKey(0), other.getKey(1), other.getKey(2));
      setPacketInformation(other);
   }

   public void set(OcTreeKeyReadOnly other)
   {
      set(other.getKey(0), other.getKey(1), other.getKey(2));
   }

   public void set(int k0, int k1, int k2)
   {
      if (k == null)
         k = new int[3];
      k[0] = k0;
      k[1] = k1;
      k[2] = k2;
   }

   @Override
   public int getKey(int index)
   {
      return k[index];
   }

   @Override
   public boolean equals(Object obj)
   {
      if (obj instanceof OcTreeKey)
         return equals((OcTreeKey) obj);
      if (obj instanceof OcTreeKeyMessage)
         return equals((OcTreeKeyMessage) obj);
      return false;
   }

   public boolean equals(OcTreeKeyMessage other)
   {
      return Arrays.equals(k, other.k);
   }

   @Override
   public boolean equals(OcTreeKey other)
   {
      return k[0] == other.getKey(0) && k[1] == other.getKey(1) && k[2] == other.getKey(2);
   }

   @Override
   public int hashCode()
   {
      return (int) (k[0] + 1447L * k[1] + 345637L * k[2]);
   }

   @Override
   public boolean epsilonEquals(OcTreeKeyMessage other, double epsilon)
   {
      return equals(other);
   }
}
