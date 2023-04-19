package perception_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * Gives the current state of the complete collection of detectable scene objects.
       * Publishing all scene objects in one messages can simplify synchronization and
       * reduce the complexity of logic in figuring out when objects are currently under
       * consideraion.
       */
public class DetectableSceneObjectsMessage extends Packet<DetectableSceneObjectsMessage> implements Settable<DetectableSceneObjectsMessage>, EpsilonComparable<DetectableSceneObjectsMessage>
{
   /**
            * The current state of the complete collection of detectable scene objects
            */
   public us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.DetectableSceneObjectMessage>  detectable_scene_objects_;

   public DetectableSceneObjectsMessage()
   {
      detectable_scene_objects_ = new us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.DetectableSceneObjectMessage> (5000, new perception_msgs.msg.dds.DetectableSceneObjectMessagePubSubType());

   }

   public DetectableSceneObjectsMessage(DetectableSceneObjectsMessage other)
   {
      this();
      set(other);
   }

   public void set(DetectableSceneObjectsMessage other)
   {
      detectable_scene_objects_.set(other.detectable_scene_objects_);
   }


   /**
            * The current state of the complete collection of detectable scene objects
            */
   public us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.DetectableSceneObjectMessage>  getDetectableSceneObjects()
   {
      return detectable_scene_objects_;
   }


   public static Supplier<DetectableSceneObjectsMessagePubSubType> getPubSubType()
   {
      return DetectableSceneObjectsMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return DetectableSceneObjectsMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(DetectableSceneObjectsMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (this.detectable_scene_objects_.size() != other.detectable_scene_objects_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.detectable_scene_objects_.size(); i++)
         {  if (!this.detectable_scene_objects_.get(i).epsilonEquals(other.detectable_scene_objects_.get(i), epsilon)) return false; }
      }

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof DetectableSceneObjectsMessage)) return false;

      DetectableSceneObjectsMessage otherMyClass = (DetectableSceneObjectsMessage) other;

      if (!this.detectable_scene_objects_.equals(otherMyClass.detectable_scene_objects_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("DetectableSceneObjectsMessage {");
      builder.append("detectable_scene_objects=");
      builder.append(this.detectable_scene_objects_);
      builder.append("}");
      return builder.toString();
   }
}
