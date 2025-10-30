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
       * Long form notes about the node.
       * string notes
       */
public class BehaviorTreeNodeDefinitionMessage extends Packet<BehaviorTreeNodeDefinitionMessage> implements Settable<BehaviorTreeNodeDefinitionMessage>, EpsilonComparable<BehaviorTreeNodeDefinitionMessage>
{
   /**
            * The type of this node
            */
   public byte type_;
   /**
            * The timestamp and modifier ID of the latest modification to this node's data fields
            */
   public ihmc_common_msgs.msg.dds.LatestModificationMessage latest_modification_to_data_;
   /**
            * The timestamp and modifier ID of the latest modification to this node's children set
            */
   public ihmc_common_msgs.msg.dds.LatestModificationMessage latest_modification_to_children_;
   /**
            * The name of the node including .json if it's a JSON root node
            */
   public java.lang.StringBuilder name_;
   /**
            * Number of children
            */
   public int number_of_children_;

   public BehaviorTreeNodeDefinitionMessage()
   {
      latest_modification_to_data_ = new ihmc_common_msgs.msg.dds.LatestModificationMessage();
      latest_modification_to_children_ = new ihmc_common_msgs.msg.dds.LatestModificationMessage();
      name_ = new java.lang.StringBuilder(255);
   }

   public BehaviorTreeNodeDefinitionMessage(BehaviorTreeNodeDefinitionMessage other)
   {
      this();
      set(other);
   }

   public void set(BehaviorTreeNodeDefinitionMessage other)
   {
      type_ = other.type_;

      ihmc_common_msgs.msg.dds.LatestModificationMessagePubSubType.staticCopy(other.latest_modification_to_data_, latest_modification_to_data_);
      ihmc_common_msgs.msg.dds.LatestModificationMessagePubSubType.staticCopy(other.latest_modification_to_children_, latest_modification_to_children_);
      name_.setLength(0);
      name_.append(other.name_);

      number_of_children_ = other.number_of_children_;

   }

   /**
            * The type of this node
            */
   public void setType(byte type)
   {
      type_ = type;
   }
   /**
            * The type of this node
            */
   public byte getType()
   {
      return type_;
   }


   /**
            * The timestamp and modifier ID of the latest modification to this node's data fields
            */
   public ihmc_common_msgs.msg.dds.LatestModificationMessage getLatestModificationToData()
   {
      return latest_modification_to_data_;
   }


   /**
            * The timestamp and modifier ID of the latest modification to this node's children set
            */
   public ihmc_common_msgs.msg.dds.LatestModificationMessage getLatestModificationToChildren()
   {
      return latest_modification_to_children_;
   }

   /**
            * The name of the node including .json if it's a JSON root node
            */
   public void setName(java.lang.String name)
   {
      name_.setLength(0);
      name_.append(name);
   }

   /**
            * The name of the node including .json if it's a JSON root node
            */
   public java.lang.String getNameAsString()
   {
      return getName().toString();
   }
   /**
            * The name of the node including .json if it's a JSON root node
            */
   public java.lang.StringBuilder getName()
   {
      return name_;
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

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.type_, other.type_, epsilon)) return false;

      if (!this.latest_modification_to_data_.epsilonEquals(other.latest_modification_to_data_, epsilon)) return false;
      if (!this.latest_modification_to_children_.epsilonEquals(other.latest_modification_to_children_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.name_, other.name_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.number_of_children_, other.number_of_children_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof BehaviorTreeNodeDefinitionMessage)) return false;

      BehaviorTreeNodeDefinitionMessage otherMyClass = (BehaviorTreeNodeDefinitionMessage) other;

      if(this.type_ != otherMyClass.type_) return false;

      if (!this.latest_modification_to_data_.equals(otherMyClass.latest_modification_to_data_)) return false;
      if (!this.latest_modification_to_children_.equals(otherMyClass.latest_modification_to_children_)) return false;
      if (!us.ihmc.idl.IDLTools.equals(this.name_, otherMyClass.name_)) return false;

      if(this.number_of_children_ != otherMyClass.number_of_children_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("BehaviorTreeNodeDefinitionMessage {");
      builder.append("type=");
      builder.append(this.type_);      builder.append(", ");
      builder.append("latest_modification_to_data=");
      builder.append(this.latest_modification_to_data_);      builder.append(", ");
      builder.append("latest_modification_to_children=");
      builder.append(this.latest_modification_to_children_);      builder.append(", ");
      builder.append("name=");
      builder.append(this.name_);      builder.append(", ");
      builder.append("number_of_children=");
      builder.append(this.number_of_children_);
      builder.append("}");
      return builder.toString();
   }
}
