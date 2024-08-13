package perception_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class GlobalMapMessage extends Packet<GlobalMapMessage> implements Settable<GlobalMapMessage>, EpsilonComparable<GlobalMapMessage>
{
   public us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.GlobalMapTileMessage>  global_map_;

   public GlobalMapMessage()
   {
      global_map_ = new us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.GlobalMapTileMessage> (100, new perception_msgs.msg.dds.GlobalMapTileMessagePubSubType());

   }

   public GlobalMapMessage(GlobalMapMessage other)
   {
      this();
      set(other);
   }

   public void set(GlobalMapMessage other)
   {
      global_map_.set(other.global_map_);
   }


   public us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.GlobalMapTileMessage>  getGlobalMap()
   {
      return global_map_;
   }


   public static Supplier<GlobalMapMessagePubSubType> getPubSubType()
   {
      return GlobalMapMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return GlobalMapMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(GlobalMapMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (this.global_map_.size() != other.global_map_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.global_map_.size(); i++)
         {  if (!this.global_map_.get(i).epsilonEquals(other.global_map_.get(i), epsilon)) return false; }
      }

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof GlobalMapMessage)) return false;

      GlobalMapMessage otherMyClass = (GlobalMapMessage) other;

      if (!this.global_map_.equals(otherMyClass.global_map_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("GlobalMapMessage {");
      builder.append("global_map=");
      builder.append(this.global_map_);
      builder.append("}");
      return builder.toString();
   }
}
