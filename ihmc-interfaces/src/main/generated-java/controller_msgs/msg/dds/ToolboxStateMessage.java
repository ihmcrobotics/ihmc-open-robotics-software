package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;

/**
 * This message is part of the IHMC toolbox framework.
 */
public class ToolboxStateMessage extends Packet<ToolboxStateMessage> implements Settable<ToolboxStateMessage>, EpsilonComparable<ToolboxStateMessage>
{
   public static final byte WAKE_UP = (byte) 0;
   public static final byte REINITIALIZE = (byte) 1;
   public static final byte SLEEP = (byte) 2;
   /**
    * As of March 2018, the header for this message is only use for its sequence ID.
    */
   public std_msgs.msg.dds.Header header_;
   public byte requested_toolbox_state_ = (byte) 255;

   public ToolboxStateMessage()
   {
      header_ = new std_msgs.msg.dds.Header();
   }

   public ToolboxStateMessage(ToolboxStateMessage other)
   {
      this();
      set(other);
   }

   public void set(ToolboxStateMessage other)
   {
      std_msgs.msg.dds.HeaderPubSubType.staticCopy(other.header_, header_);
      requested_toolbox_state_ = other.requested_toolbox_state_;

   }

   /**
    * As of March 2018, the header for this message is only use for its sequence ID.
    */
   public std_msgs.msg.dds.Header getHeader()
   {
      return header_;
   }

   public void setRequestedToolboxState(byte requested_toolbox_state)
   {
      requested_toolbox_state_ = requested_toolbox_state;
   }

   public byte getRequestedToolboxState()
   {
      return requested_toolbox_state_;
   }

   @Override
   public boolean epsilonEquals(ToolboxStateMessage other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!this.header_.epsilonEquals(other.header_, epsilon))
         return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.requested_toolbox_state_, other.requested_toolbox_state_, epsilon))
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
      if (!(other instanceof ToolboxStateMessage))
         return false;

      ToolboxStateMessage otherMyClass = (ToolboxStateMessage) other;

      if (!this.header_.equals(otherMyClass.header_))
         return false;
      if (this.requested_toolbox_state_ != otherMyClass.requested_toolbox_state_)
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ToolboxStateMessage {");
      builder.append("header=");
      builder.append(this.header_);
      builder.append(", ");
      builder.append("requested_toolbox_state=");
      builder.append(this.requested_toolbox_state_);
      builder.append("}");
      return builder.toString();
   }
}
