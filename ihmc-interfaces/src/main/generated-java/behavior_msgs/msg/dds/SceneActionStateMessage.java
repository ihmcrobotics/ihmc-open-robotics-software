package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class SceneActionStateMessage extends Packet<SceneActionStateMessage> implements Settable<SceneActionStateMessage>, EpsilonComparable<SceneActionStateMessage>
{
   /**
            * Parent state fields
            */
   public behavior_msgs.msg.dds.ActionNodeStateMessage state_;
   /**
            * Definition
            */
   public behavior_msgs.msg.dds.SceneActionDefinitionMessage definition_;

   public SceneActionStateMessage()
   {
      state_ = new behavior_msgs.msg.dds.ActionNodeStateMessage();
      definition_ = new behavior_msgs.msg.dds.SceneActionDefinitionMessage();
   }

   public SceneActionStateMessage(SceneActionStateMessage other)
   {
      this();
      set(other);
   }

   public void set(SceneActionStateMessage other)
   {
      behavior_msgs.msg.dds.ActionNodeStateMessagePubSubType.staticCopy(other.state_, state_);
      behavior_msgs.msg.dds.SceneActionDefinitionMessagePubSubType.staticCopy(other.definition_, definition_);
   }


   /**
            * Parent state fields
            */
   public behavior_msgs.msg.dds.ActionNodeStateMessage getState()
   {
      return state_;
   }


   /**
            * Definition
            */
   public behavior_msgs.msg.dds.SceneActionDefinitionMessage getDefinition()
   {
      return definition_;
   }


   public static Supplier<SceneActionStateMessagePubSubType> getPubSubType()
   {
      return SceneActionStateMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return SceneActionStateMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(SceneActionStateMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.state_.epsilonEquals(other.state_, epsilon)) return false;
      if (!this.definition_.epsilonEquals(other.definition_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof SceneActionStateMessage)) return false;

      SceneActionStateMessage otherMyClass = (SceneActionStateMessage) other;

      if (!this.state_.equals(otherMyClass.state_)) return false;
      if (!this.definition_.equals(otherMyClass.definition_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("SceneActionStateMessage {");
      builder.append("state=");
      builder.append(this.state_);      builder.append(", ");
      builder.append("definition=");
      builder.append(this.definition_);
      builder.append("}");
      return builder.toString();
   }
}
