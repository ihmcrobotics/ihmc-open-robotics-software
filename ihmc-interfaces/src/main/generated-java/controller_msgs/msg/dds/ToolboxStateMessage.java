package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC toolbox framework.
       */
public class ToolboxStateMessage extends Packet<ToolboxStateMessage> implements Settable<ToolboxStateMessage>, EpsilonComparable<ToolboxStateMessage>
{
   public static final byte WAKE_UP = (byte) 0;
   public static final byte REINITIALIZE = (byte) 1;
   public static final byte SLEEP = (byte) 2;
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   public byte requested_toolbox_state_ = (byte) 255;
   /**
            * If true and the toolbox supports logging, all messages to and from the toolbox are logged.
            * Can only be requested for WAKE_UP and REINITIALIZE. SLEEP will automatically end the toolbox log.
            */
   public boolean request_logging_;

   public ToolboxStateMessage()
   {
   }

   public ToolboxStateMessage(ToolboxStateMessage other)
   {
      this();
      set(other);
   }

   public void set(ToolboxStateMessage other)
   {
      sequence_id_ = other.sequence_id_;

      requested_toolbox_state_ = other.requested_toolbox_state_;

      request_logging_ = other.request_logging_;

   }

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public void setSequenceId(long sequence_id)
   {
      sequence_id_ = sequence_id;
   }
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long getSequenceId()
   {
      return sequence_id_;
   }

   public void setRequestedToolboxState(byte requested_toolbox_state)
   {
      requested_toolbox_state_ = requested_toolbox_state;
   }
   public byte getRequestedToolboxState()
   {
      return requested_toolbox_state_;
   }

   /**
            * If true and the toolbox supports logging, all messages to and from the toolbox are logged.
            * Can only be requested for WAKE_UP and REINITIALIZE. SLEEP will automatically end the toolbox log.
            */
   public void setRequestLogging(boolean request_logging)
   {
      request_logging_ = request_logging;
   }
   /**
            * If true and the toolbox supports logging, all messages to and from the toolbox are logged.
            * Can only be requested for WAKE_UP and REINITIALIZE. SLEEP will automatically end the toolbox log.
            */
   public boolean getRequestLogging()
   {
      return request_logging_;
   }


   public static Supplier<ToolboxStateMessagePubSubType> getPubSubType()
   {
      return ToolboxStateMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return ToolboxStateMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(ToolboxStateMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.requested_toolbox_state_, other.requested_toolbox_state_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.request_logging_, other.request_logging_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof ToolboxStateMessage)) return false;

      ToolboxStateMessage otherMyClass = (ToolboxStateMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.requested_toolbox_state_ != otherMyClass.requested_toolbox_state_) return false;

      if(this.request_logging_ != otherMyClass.request_logging_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ToolboxStateMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("requested_toolbox_state=");
      builder.append(this.requested_toolbox_state_);      builder.append(", ");
      builder.append("request_logging=");
      builder.append(this.request_logging_);
      builder.append("}");
      return builder.toString();
   }
}
