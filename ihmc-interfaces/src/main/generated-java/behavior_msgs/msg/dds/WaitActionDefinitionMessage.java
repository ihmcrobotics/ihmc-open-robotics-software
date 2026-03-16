package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class WaitActionDefinitionMessage extends Packet<WaitActionDefinitionMessage> implements Settable<WaitActionDefinitionMessage>, EpsilonComparable<WaitActionDefinitionMessage>
{
   /**
            * Parent definition fields
            */
   public behavior_msgs.msg.dds.ActionNodeDefinitionMessage definition_;
   /**
            * Wait duration
            */
   public double wait_duration_;

   public WaitActionDefinitionMessage()
   {
      definition_ = new behavior_msgs.msg.dds.ActionNodeDefinitionMessage();
   }

   public WaitActionDefinitionMessage(WaitActionDefinitionMessage other)
   {
      this();
      set(other);
   }

   public void set(WaitActionDefinitionMessage other)
   {
      behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType.staticCopy(other.definition_, definition_);
      wait_duration_ = other.wait_duration_;

   }


   /**
            * Parent definition fields
            */
   public behavior_msgs.msg.dds.ActionNodeDefinitionMessage getDefinition()
   {
      return definition_;
   }

   /**
            * Wait duration
            */
   public void setWaitDuration(double wait_duration)
   {
      wait_duration_ = wait_duration;
   }
   /**
            * Wait duration
            */
   public double getWaitDuration()
   {
      return wait_duration_;
   }


   public static Supplier<WaitActionDefinitionMessagePubSubType> getPubSubType()
   {
      return WaitActionDefinitionMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return WaitActionDefinitionMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(WaitActionDefinitionMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.definition_.epsilonEquals(other.definition_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.wait_duration_, other.wait_duration_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof WaitActionDefinitionMessage)) return false;

      WaitActionDefinitionMessage otherMyClass = (WaitActionDefinitionMessage) other;

      if (!this.definition_.equals(otherMyClass.definition_)) return false;
      if(this.wait_duration_ != otherMyClass.wait_duration_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("WaitActionDefinitionMessage {");
      builder.append("definition=");
      builder.append(this.definition_);      builder.append(", ");
      builder.append("wait_duration=");
      builder.append(this.wait_duration_);
      builder.append("}");
      return builder.toString();
   }
}
