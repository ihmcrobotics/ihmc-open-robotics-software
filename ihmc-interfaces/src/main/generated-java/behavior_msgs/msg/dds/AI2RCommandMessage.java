package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class AI2RCommandMessage extends Packet<AI2RCommandMessage> implements Settable<AI2RCommandMessage>, EpsilonComparable<AI2RCommandMessage>
{
   /**
            * Behavior to execute (checkpoint to jump to in the pre-loaded behavior collection)
            */
   public java.lang.StringBuilder behavior_to_execute_;

   public AI2RCommandMessage()
   {
      behavior_to_execute_ = new java.lang.StringBuilder(255);
   }

   public AI2RCommandMessage(AI2RCommandMessage other)
   {
      this();
      set(other);
   }

   public void set(AI2RCommandMessage other)
   {
      behavior_to_execute_.setLength(0);
      behavior_to_execute_.append(other.behavior_to_execute_);
   }

   /**
            * Behavior to execute (checkpoint to jump to in the pre-loaded behavior collection)
            */
   public void setBehaviorToExecute(java.lang.String behavior_to_execute)
   {
      behavior_to_execute_.setLength(0);
      behavior_to_execute_.append(behavior_to_execute);
   }

   /**
            * Behavior to execute (checkpoint to jump to in the pre-loaded behavior collection)
            */
   public java.lang.String getBehaviorToExecuteAsString()
   {
      return getBehaviorToExecute().toString();
   }
   /**
            * Behavior to execute (checkpoint to jump to in the pre-loaded behavior collection)
            */
   public java.lang.StringBuilder getBehaviorToExecute()
   {
      return behavior_to_execute_;
   }


   public static Supplier<AI2RCommandMessagePubSubType> getPubSubType()
   {
      return AI2RCommandMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return AI2RCommandMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(AI2RCommandMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.behavior_to_execute_, other.behavior_to_execute_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof AI2RCommandMessage)) return false;

      AI2RCommandMessage otherMyClass = (AI2RCommandMessage) other;

      if (!us.ihmc.idl.IDLTools.equals(this.behavior_to_execute_, otherMyClass.behavior_to_execute_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("AI2RCommandMessage {");
      builder.append("behavior_to_execute=");
      builder.append(this.behavior_to_execute_);
      builder.append("}");
      return builder.toString();
   }
}
