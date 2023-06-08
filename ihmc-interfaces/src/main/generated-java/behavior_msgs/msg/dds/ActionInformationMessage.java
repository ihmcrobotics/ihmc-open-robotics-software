package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class ActionInformationMessage extends Packet<ActionInformationMessage> implements Settable<ActionInformationMessage>, EpsilonComparable<ActionInformationMessage>
{
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
      action_index_ = other.action_index_;

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

      if(this.action_index_ != otherMyClass.action_index_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ActionInformationMessage {");
      builder.append("action_index=");
      builder.append(this.action_index_);
      builder.append("}");
      return builder.toString();
   }
}
