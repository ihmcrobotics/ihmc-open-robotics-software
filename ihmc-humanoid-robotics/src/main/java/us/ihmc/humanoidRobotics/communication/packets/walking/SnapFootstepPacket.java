package us.ihmc.humanoidRobotics.communication.packets.walking;

import java.util.ArrayList;
import java.util.Arrays;

import us.ihmc.communication.packets.Packet;

public class SnapFootstepPacket extends Packet<SnapFootstepPacket>
{
   //   public static final byte UNKOWN = 0;
   //   public static final byte VALID_UNCHANGED_STEP = 1;
   //   public static final byte VALID_SNAPPED_STEP = 2;
   //   public static final byte BAD_STEP = 3;
   public ArrayList<FootstepDataMessage> footstepData;
   public int[] footstepOrder;
   public byte[] flag;

   public SnapFootstepPacket()
   {
      // Empty constructor for deserialization
   }

   public SnapFootstepPacket(ArrayList<FootstepDataMessage> data, int[] footstepOrder, byte[] flag)
   {
      this.footstepData = data;
      this.footstepOrder = footstepOrder;
      this.flag = flag;
   }

   @Override
   public void set(SnapFootstepPacket other)
   {
      footstepData = new ArrayList<>();
      for (FootstepDataMessage otherStep : other.footstepData)
      {
         FootstepDataMessage step = new FootstepDataMessage();
         step.set(otherStep);
         footstepData.add(step);
      }

      footstepOrder = Arrays.copyOf(other.footstepOrder, other.footstepOrder.length);
      flag = Arrays.copyOf(other.flag, other.flag.length);
      setPacketInformation(other);
   }

   public ArrayList<FootstepDataMessage> getFootstepData()
   {
      return this.footstepData;
   }

   public int[] getFootstepOrder()
   {
      return footstepOrder;
   }

   public byte[] getFlag()
   {
      return flag;
   }

   public boolean equals(Object obj)
   {
      return ((obj instanceof SnapFootstepPacket) && this.epsilonEquals((SnapFootstepPacket) obj, 0));
   }

   @Override
   public boolean epsilonEquals(SnapFootstepPacket other, double epsilon)
   {
      boolean ret = true;
      if(this.footstepData.size() != other.footstepData.size())
      {
         return false;
      }

      for (int i = 0; i < footstepData.size(); i++)
      {
         ret &= this.footstepData.get(i).epsilonEquals(other.footstepData.get(i), epsilon);
         ret &= this.footstepOrder[i] == other.footstepOrder[i];
         ret &= this.flag[i] == other.flag[i];
      }

      return ret;
   }
}
