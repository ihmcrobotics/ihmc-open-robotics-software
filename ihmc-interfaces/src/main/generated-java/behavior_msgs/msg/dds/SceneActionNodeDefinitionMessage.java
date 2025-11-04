package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class SceneActionNodeDefinitionMessage extends Packet<SceneActionNodeDefinitionMessage> implements Settable<SceneActionNodeDefinitionMessage>, EpsilonComparable<SceneActionNodeDefinitionMessage>
{
   /**
            * Parent definition fields
            */
   public behavior_msgs.msg.dds.ActionNodeDefinitionMessage definition_;
   /**
            * Object type (enum ordinal)
            */
   public int object_type_;

   public SceneActionNodeDefinitionMessage()
   {
      definition_ = new behavior_msgs.msg.dds.ActionNodeDefinitionMessage();
   }

   public SceneActionNodeDefinitionMessage(SceneActionNodeDefinitionMessage other)
   {
      this();
      set(other);
   }

   public void set(SceneActionNodeDefinitionMessage other)
   {
      behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType.staticCopy(other.definition_, definition_);
      object_type_ = other.object_type_;

   }


   /**
            * Parent definition fields
            */
   public behavior_msgs.msg.dds.ActionNodeDefinitionMessage getDefinition()
   {
      return definition_;
   }

   /**
            * Object type (enum ordinal)
            */
   public void setObjectType(int object_type)
   {
      object_type_ = object_type;
   }
   /**
            * Object type (enum ordinal)
            */
   public int getObjectType()
   {
      return object_type_;
   }


   public static Supplier<SceneActionNodeDefinitionMessagePubSubType> getPubSubType()
   {
      return SceneActionNodeDefinitionMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return SceneActionNodeDefinitionMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(SceneActionNodeDefinitionMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.definition_.epsilonEquals(other.definition_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.object_type_, other.object_type_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof SceneActionNodeDefinitionMessage)) return false;

      SceneActionNodeDefinitionMessage otherMyClass = (SceneActionNodeDefinitionMessage) other;

      if (!this.definition_.equals(otherMyClass.definition_)) return false;
      if(this.object_type_ != otherMyClass.object_type_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("SceneActionNodeDefinitionMessage {");
      builder.append("definition=");
      builder.append(this.definition_);      builder.append(", ");
      builder.append("object_type=");
      builder.append(this.object_type_);
      builder.append("}");
      return builder.toString();
   }
}
