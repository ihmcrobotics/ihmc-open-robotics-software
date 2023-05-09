package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class ActionInformationMessage extends Packet<ActionInformationMessage> implements Settable<ActionInformationMessage>, EpsilonComparable<ActionInformationMessage>
{
   /**
            * Unique identifier of the sequence update used to make sure we got complete data
            */
   public long sequence_update_uuid_;
   /**
            * Index of the action within the sequence
            */
   public long action_index_;

   public ActionInformationMessage()
   {
   }

   public ActionInformationMessage(ActionInformationMessage other)
   {
      this();
      set(other);
   }

   public void set(ActionInformationMessage other)
   {
      sequence_update_uuid_ = other.sequence_update_uuid_;

      action_index_ = other.action_index_;

   }

   /**
            * Unique identifier of the sequence update used to make sure we got complete data
            */
   public void setSequenceUpdateUuid(long sequence_update_uuid)
   {
      sequence_update_uuid_ = sequence_update_uuid;
   }
   /**
            * Unique identifier of the sequence update used to make sure we got complete data
            */
   public long getSequenceUpdateUuid()
   {
      return sequence_update_uuid_;
   }

   /**
            * Index of the action within the sequence
            */
   public void setActionIndex(long action_index)
   {
      action_index_ = action_index;
   }
   /**
            * Index of the action within the sequence
            */
   public long getActionIndex()
   {
      return action_index_;
   }


   public static Supplier<ActionInformationMessagePubSubType> getPubSubType()
   {
      return ActionInformationMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return ActionInformationMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(ActionInformationMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_update_uuid_, other.sequence_update_uuid_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.action_index_, other.action_index_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof ActionInformationMessage)) return false;

      ActionInformationMessage otherMyClass = (ActionInformationMessage) other;

      if(this.sequence_update_uuid_ != otherMyClass.sequence_update_uuid_) return false;

      if(this.action_index_ != otherMyClass.action_index_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ActionInformationMessage {");
      builder.append("sequence_update_uuid=");
      builder.append(this.sequence_update_uuid_);      builder.append(", ");
      builder.append("action_index=");
      builder.append(this.action_index_);
      builder.append("}");
      return builder.toString();
   }
}
