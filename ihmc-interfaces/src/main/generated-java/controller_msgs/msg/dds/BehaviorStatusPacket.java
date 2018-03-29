package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;

/**
 * This message is part of the IHMC humanoid behavior module.
 */
public class BehaviorStatusPacket extends Packet<BehaviorStatusPacket> implements Settable<BehaviorStatusPacket>, EpsilonComparable<BehaviorStatusPacket>
{
   public static final byte NO_BEHAVIOR_RUNNING = (byte) 0;
   public static final byte BEHAVIOS_RUNNING = (byte) 1;
   public static final byte BEHAVIOR_PAUSED = (byte) 2;
   /**
    * As of March 2018, the header for this message is only use for its sequence ID.
    */
   public std_msgs.msg.dds.Header header_;
   public byte current_behavior_status_ = (byte) 255;

   public BehaviorStatusPacket()
   {
      header_ = new std_msgs.msg.dds.Header();
   }

   public BehaviorStatusPacket(BehaviorStatusPacket other)
   {
      this();
      set(other);
   }

   public void set(BehaviorStatusPacket other)
   {
      std_msgs.msg.dds.HeaderPubSubType.staticCopy(other.header_, header_);
      current_behavior_status_ = other.current_behavior_status_;

   }

   /**
    * As of March 2018, the header for this message is only use for its sequence ID.
    */
   public std_msgs.msg.dds.Header getHeader()
   {
      return header_;
   }

   public void setCurrentBehaviorStatus(byte current_behavior_status)
   {
      current_behavior_status_ = current_behavior_status;
   }

   public byte getCurrentBehaviorStatus()
   {
      return current_behavior_status_;
   }

   @Override
   public boolean epsilonEquals(BehaviorStatusPacket other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!this.header_.epsilonEquals(other.header_, epsilon))
         return false;
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

      if (!this.header_.equals(otherMyClass.header_))
         return false;
      if (this.current_behavior_status_ != otherMyClass.current_behavior_status_)
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("BehaviorStatusPacket {");
      builder.append("header=");
      builder.append(this.header_);
      builder.append(", ");
      builder.append("current_behavior_status=");
      builder.append(this.current_behavior_status_);
      builder.append("}");
      return builder.toString();
   }
}
