package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Settable;

/**
 * This message is part of the IHMC humanoid behavior module.
 */
public class BehaviorStatusPacket extends Packet<BehaviorStatusPacket> implements Settable<BehaviorStatusPacket>, EpsilonComparable<BehaviorStatusPacket>
{
   public static final byte NO_BEHAVIOR_RUNNING = (byte) 0;
   public static final byte BEHAVIOS_RUNNING = (byte) 1;
   public static final byte BEHAVIOR_PAUSED = (byte) 2;
   public byte current_behavior_status_ = (byte) 255;

   public BehaviorStatusPacket()
   {
   }

   public BehaviorStatusPacket(BehaviorStatusPacket other)
   {
      set(other);
   }

   public void set(BehaviorStatusPacket other)
   {
      current_behavior_status_ = other.current_behavior_status_;
   }

   public byte getCurrentBehaviorStatus()
   {
      return current_behavior_status_;
   }

   public void setCurrentBehaviorStatus(byte current_behavior_status)
   {
      current_behavior_status_ = current_behavior_status;
   }

   @Override
   public boolean epsilonEquals(BehaviorStatusPacket other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.current_behavior_status_, other.current_behavior_status_, epsilon))
         return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;
      if (!(other instanceof BehaviorStatusPacket))
         return false;

      BehaviorStatusPacket otherMyClass = (BehaviorStatusPacket) other;

      if (this.current_behavior_status_ != otherMyClass.current_behavior_status_)
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("BehaviorStatusPacket {");
      builder.append("current_behavior_status=");
      builder.append(this.current_behavior_status_);

      builder.append("}");
      return builder.toString();
   }
}