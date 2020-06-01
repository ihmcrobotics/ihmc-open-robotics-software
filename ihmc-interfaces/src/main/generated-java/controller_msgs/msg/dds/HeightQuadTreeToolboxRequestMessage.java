package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC height quad tree module
       */
public class HeightQuadTreeToolboxRequestMessage extends Packet<HeightQuadTreeToolboxRequestMessage> implements Settable<HeightQuadTreeToolboxRequestMessage>, EpsilonComparable<HeightQuadTreeToolboxRequestMessage>
{

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   public boolean request_clear_quad_tree_;

   public boolean request_quad_tree_update_;

   public HeightQuadTreeToolboxRequestMessage()
   {




   }

   public HeightQuadTreeToolboxRequestMessage(HeightQuadTreeToolboxRequestMessage other)
   {
      this();
      set(other);
   }

   public void set(HeightQuadTreeToolboxRequestMessage other)
   {

      sequence_id_ = other.sequence_id_;


      request_clear_quad_tree_ = other.request_clear_quad_tree_;


      request_quad_tree_update_ = other.request_quad_tree_update_;

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


   public void setRequestClearQuadTree(boolean request_clear_quad_tree)
   {
      request_clear_quad_tree_ = request_clear_quad_tree;
   }
   public boolean getRequestClearQuadTree()
   {
      return request_clear_quad_tree_;
   }


   public void setRequestQuadTreeUpdate(boolean request_quad_tree_update)
   {
      request_quad_tree_update_ = request_quad_tree_update;
   }
   public boolean getRequestQuadTreeUpdate()
   {
      return request_quad_tree_update_;
   }


   public static Supplier<HeightQuadTreeToolboxRequestMessagePubSubType> getPubSubType()
   {
      return HeightQuadTreeToolboxRequestMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return HeightQuadTreeToolboxRequestMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(HeightQuadTreeToolboxRequestMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.request_clear_quad_tree_, other.request_clear_quad_tree_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.request_quad_tree_update_, other.request_quad_tree_update_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof HeightQuadTreeToolboxRequestMessage)) return false;

      HeightQuadTreeToolboxRequestMessage otherMyClass = (HeightQuadTreeToolboxRequestMessage) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if(this.request_clear_quad_tree_ != otherMyClass.request_clear_quad_tree_) return false;


      if(this.request_quad_tree_update_ != otherMyClass.request_quad_tree_update_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("HeightQuadTreeToolboxRequestMessage {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("request_clear_quad_tree=");
      builder.append(this.request_clear_quad_tree_);      builder.append(", ");

      builder.append("request_quad_tree_update=");
      builder.append(this.request_quad_tree_update_);
      builder.append("}");
      return builder.toString();
   }
}
