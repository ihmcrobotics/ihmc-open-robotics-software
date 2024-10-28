package toolbox_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the ReferenceSpreadingToolbox API.
       */
public class ReferenceSpreadingToolboxInputMessage extends Packet<ReferenceSpreadingToolboxInputMessage> implements Settable<ReferenceSpreadingToolboxInputMessage>, EpsilonComparable<ReferenceSpreadingToolboxInputMessage>
{
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   /**
            * The state which the controller should go into. Options are: [0: WAITING, 1: RECORDING, 2: PLAYBACK]
            */
   public byte state_;

   public ReferenceSpreadingToolboxInputMessage()
   {
   }

   public ReferenceSpreadingToolboxInputMessage(ReferenceSpreadingToolboxInputMessage other)
   {
      this();
      set(other);
   }

   public void set(ReferenceSpreadingToolboxInputMessage other)
   {
      sequence_id_ = other.sequence_id_;

      state_ = other.state_;

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

   /**
            * The state which the controller should go into. Options are: [0: WAITING, 1: RECORDING, 2: PLAYBACK]
            */
   public void setState(byte state)
   {
      state_ = state;
   }
   /**
            * The state which the controller should go into. Options are: [0: WAITING, 1: RECORDING, 2: PLAYBACK]
            */
   public byte getState()
   {
      return state_;
   }


   public static Supplier<ReferenceSpreadingToolboxInputMessagePubSubType> getPubSubType()
   {
      return ReferenceSpreadingToolboxInputMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return ReferenceSpreadingToolboxInputMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(ReferenceSpreadingToolboxInputMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.state_, other.state_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof ReferenceSpreadingToolboxInputMessage)) return false;

      ReferenceSpreadingToolboxInputMessage otherMyClass = (ReferenceSpreadingToolboxInputMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.state_ != otherMyClass.state_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ReferenceSpreadingToolboxInputMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("state=");
      builder.append(this.state_);
      builder.append("}");
      return builder.toString();
   }
}
