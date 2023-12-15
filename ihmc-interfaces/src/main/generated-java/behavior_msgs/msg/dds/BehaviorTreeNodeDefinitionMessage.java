package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * The base description of a behavior tree node.
       * Note that descriptions have children however we can not put them
       * in this message because ROS 2 messages cannot contain themselves.
       * Instead we use a depth first ordered list and number of children
       * to send trees over. This is contained in another higher level message.
       */
public class BehaviorTreeNodeDefinitionMessage extends Packet<BehaviorTreeNodeDefinitionMessage> implements Settable<BehaviorTreeNodeDefinitionMessage>, EpsilonComparable<BehaviorTreeNodeDefinitionMessage>
{
   /**
            * A human readable description of what the node does
            */
   public java.lang.StringBuilder description_;
   /**
            * Number of children
            */
   public int number_of_children_;
   /**
            * JSON file name if this node is the root of a JSON file
            */
   public java.lang.StringBuilder json_file_name_;

   public BehaviorTreeNodeDefinitionMessage()
   {
      description_ = new java.lang.StringBuilder(255);
      json_file_name_ = new java.lang.StringBuilder(255);
   }

   public BehaviorTreeNodeDefinitionMessage(BehaviorTreeNodeDefinitionMessage other)
   {
      this();
      set(other);
   }

   public void set(BehaviorTreeNodeDefinitionMessage other)
   {
      description_.setLength(0);
      description_.append(other.description_);

      number_of_children_ = other.number_of_children_;

      json_file_name_.setLength(0);
      json_file_name_.append(other.json_file_name_);

   }

   /**
            * A human readable description of what the node does
            */
   public void setDescription(java.lang.String description)
   {
      description_.setLength(0);
      description_.append(description);
   }

   /**
            * A human readable description of what the node does
            */
   public java.lang.String getDescriptionAsString()
   {
      return getDescription().toString();
   }
   /**
            * A human readable description of what the node does
            */
   public java.lang.StringBuilder getDescription()
   {
      return description_;
   }

   /**
            * Number of children
            */
   public void setNumberOfChildren(int number_of_children)
   {
      number_of_children_ = number_of_children;
   }
   /**
            * Number of children
            */
   public int getNumberOfChildren()
   {
      return number_of_children_;
   }

   /**
            * JSON file name if this node is the root of a JSON file
            */
   public void setJsonFileName(java.lang.String json_file_name)
   {
      json_file_name_.setLength(0);
      json_file_name_.append(json_file_name);
   }

   /**
            * JSON file name if this node is the root of a JSON file
            */
   public java.lang.String getJsonFileNameAsString()
   {
      return getJsonFileName().toString();
   }
   /**
            * JSON file name if this node is the root of a JSON file
            */
   public java.lang.StringBuilder getJsonFileName()
   {
      return json_file_name_;
   }


   public static Supplier<BehaviorTreeNodeDefinitionMessagePubSubType> getPubSubType()
   {
      return BehaviorTreeNodeDefinitionMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return BehaviorTreeNodeDefinitionMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(BehaviorTreeNodeDefinitionMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.description_, other.description_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.number_of_children_, other.number_of_children_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.json_file_name_, other.json_file_name_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof BehaviorTreeNodeDefinitionMessage)) return false;

      BehaviorTreeNodeDefinitionMessage otherMyClass = (BehaviorTreeNodeDefinitionMessage) other;

      if (!us.ihmc.idl.IDLTools.equals(this.description_, otherMyClass.description_)) return false;

      if(this.number_of_children_ != otherMyClass.number_of_children_) return false;

      if (!us.ihmc.idl.IDLTools.equals(this.json_file_name_, otherMyClass.json_file_name_)) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("BehaviorTreeNodeDefinitionMessage {");
      builder.append("description=");
      builder.append(this.description_);      builder.append(", ");
      builder.append("number_of_children=");
      builder.append(this.number_of_children_);      builder.append(", ");
      builder.append("json_file_name=");
      builder.append(this.json_file_name_);
      builder.append("}");
      return builder.toString();
   }
}
