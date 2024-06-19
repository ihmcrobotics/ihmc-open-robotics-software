package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class BuildingExplorationDefinitionMessage extends Packet<BuildingExplorationDefinitionMessage> implements Settable<BuildingExplorationDefinitionMessage>, EpsilonComparable<BuildingExplorationDefinitionMessage>
{
   /**
            * Parent definition fields
            */
   public behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessage definition_;

   public BuildingExplorationDefinitionMessage()
   {
      definition_ = new behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessage();
   }

   public BuildingExplorationDefinitionMessage(BuildingExplorationDefinitionMessage other)
   {
      this();
      set(other);
   }

   public void set(BuildingExplorationDefinitionMessage other)
   {
      behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessagePubSubType.staticCopy(other.definition_, definition_);   }


   /**
            * Parent definition fields
            */
   public behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessage getDefinition()
   {
      return definition_;
   }


   public static Supplier<BuildingExplorationDefinitionMessagePubSubType> getPubSubType()
   {
      return BuildingExplorationDefinitionMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return BuildingExplorationDefinitionMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(BuildingExplorationDefinitionMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.definition_.epsilonEquals(other.definition_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof BuildingExplorationDefinitionMessage)) return false;

      BuildingExplorationDefinitionMessage otherMyClass = (BuildingExplorationDefinitionMessage) other;

      if (!this.definition_.equals(otherMyClass.definition_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("BuildingExplorationDefinitionMessage {");
      builder.append("definition=");
      builder.append(this.definition_);
      builder.append("}");
      return builder.toString();
   }
}
