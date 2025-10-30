package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * The state of the behavior tree scene
       */
public class BehaviorTreeSceneStateMessage extends Packet<BehaviorTreeSceneStateMessage> implements Settable<BehaviorTreeSceneStateMessage>, EpsilonComparable<BehaviorTreeSceneStateMessage>
{
   /**
            * The timestamp and modifier ID of the latest modification to the list of objects
            */
   public ihmc_common_msgs.msg.dds.LatestModificationMessage latest_modification_to_list_;
   /**
            * Scene objects
            */
   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.BehaviorTreeSceneObjectStateMessage>  objects_;

   public BehaviorTreeSceneStateMessage()
   {
      latest_modification_to_list_ = new ihmc_common_msgs.msg.dds.LatestModificationMessage();
      objects_ = new us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.BehaviorTreeSceneObjectStateMessage> (100, new behavior_msgs.msg.dds.BehaviorTreeSceneObjectStateMessagePubSubType());

   }

   public BehaviorTreeSceneStateMessage(BehaviorTreeSceneStateMessage other)
   {
      this();
      set(other);
   }

   public void set(BehaviorTreeSceneStateMessage other)
   {
      ihmc_common_msgs.msg.dds.LatestModificationMessagePubSubType.staticCopy(other.latest_modification_to_list_, latest_modification_to_list_);
      objects_.set(other.objects_);
   }


   /**
            * The timestamp and modifier ID of the latest modification to the list of objects
            */
   public ihmc_common_msgs.msg.dds.LatestModificationMessage getLatestModificationToList()
   {
      return latest_modification_to_list_;
   }


   /**
            * Scene objects
            */
   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.BehaviorTreeSceneObjectStateMessage>  getObjects()
   {
      return objects_;
   }


   public static Supplier<BehaviorTreeSceneStateMessagePubSubType> getPubSubType()
   {
      return BehaviorTreeSceneStateMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return BehaviorTreeSceneStateMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(BehaviorTreeSceneStateMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.latest_modification_to_list_.epsilonEquals(other.latest_modification_to_list_, epsilon)) return false;
      if (this.objects_.size() != other.objects_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.objects_.size(); i++)
         {  if (!this.objects_.get(i).epsilonEquals(other.objects_.get(i), epsilon)) return false; }
      }


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof BehaviorTreeSceneStateMessage)) return false;

      BehaviorTreeSceneStateMessage otherMyClass = (BehaviorTreeSceneStateMessage) other;

      if (!this.latest_modification_to_list_.equals(otherMyClass.latest_modification_to_list_)) return false;
      if (!this.objects_.equals(otherMyClass.objects_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("BehaviorTreeSceneStateMessage {");
      builder.append("latest_modification_to_list=");
      builder.append(this.latest_modification_to_list_);      builder.append(", ");
      builder.append("objects=");
      builder.append(this.objects_);
      builder.append("}");
      return builder.toString();
   }
}
