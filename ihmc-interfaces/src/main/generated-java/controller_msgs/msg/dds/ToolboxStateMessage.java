package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Settable;

/**
 * This message is part of the IHMC toolbox framework.
 */
public class ToolboxStateMessage extends Packet<ToolboxStateMessage> implements Settable<ToolboxStateMessage>, EpsilonComparable<ToolboxStateMessage>
{
   public static final byte WAKE_UP = (byte) 0;
   public static final byte REINITIALIZE = (byte) 1;
   public static final byte SLEEP = (byte) 2;
   public byte requested_toolbox_state_ = (byte) 255;

   public ToolboxStateMessage()
   {
   }

   public ToolboxStateMessage(ToolboxStateMessage other)
   {
      set(other);
   }

   public void set(ToolboxStateMessage other)
   {
      requested_toolbox_state_ = other.requested_toolbox_state_;
   }

   public byte getRequestedToolboxState()
   {
      return requested_toolbox_state_;
   }

   public void setRequestedToolboxState(byte requested_toolbox_state)
   {
      requested_toolbox_state_ = requested_toolbox_state;
   }

   @Override
   public boolean epsilonEquals(ToolboxStateMessage other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

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

      if (this.requested_toolbox_state_ != otherMyClass.requested_toolbox_state_)
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ToolboxStateMessage {");
      builder.append("requested_toolbox_state=");
      builder.append(this.requested_toolbox_state_);

      builder.append("}");
      return builder.toString();
   }
}