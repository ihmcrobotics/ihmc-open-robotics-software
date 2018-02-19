package us.ihmc.humanoidRobotics.communication.packets.walking;

import gnu.trove.list.array.TByteArrayList;
import gnu.trove.list.array.TIntArrayList;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.Packet;
import us.ihmc.idl.PreallocatedList;

public class SnapFootstepPacket extends Packet<SnapFootstepPacket>
{
   //   public static final byte UNKOWN = 0;
   //   public static final byte VALID_UNCHANGED_STEP = 1;
   //   public static final byte VALID_SNAPPED_STEP = 2;
   //   public static final byte BAD_STEP = 3;
   public PreallocatedList<FootstepDataMessage> footstepData = new PreallocatedList<>(FootstepDataMessage.class, FootstepDataMessage::new, 10);
   public TIntArrayList footstepOrder = new TIntArrayList();
   public TByteArrayList flag = new TByteArrayList();

   public SnapFootstepPacket()
   {
      // Empty constructor for deserialization
   }

   @Override
   public void set(SnapFootstepPacket other)
   {
      MessageTools.copyData(other.footstepData, footstepData);
      MessageTools.copyData(other.footstepOrder, footstepOrder);
      MessageTools.copyData(other.flag, flag);
      setPacketInformation(other);
   }

   public PreallocatedList<FootstepDataMessage> getFootstepData()
   {
      return this.footstepData;
   }

   public TIntArrayList getFootstepOrder()
   {
      return footstepOrder;
   }

   public TByteArrayList getFlag()
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
      if (!MessageTools.epsilonEquals(footstepData, other.footstepData, epsilon))
         return false;
      if (!footstepOrder.equals(other.footstepOrder))
         return false;
      if (!flag.equals(other.flag))
         return false;
      return true;
   }
}
