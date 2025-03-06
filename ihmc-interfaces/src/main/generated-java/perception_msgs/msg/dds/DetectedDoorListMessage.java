package perception_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class DetectedDoorListMessage extends Packet<DetectedDoorListMessage> implements Settable<DetectedDoorListMessage>, EpsilonComparable<DetectedDoorListMessage>
{
   /**
            * List of detected doors
            */
   public us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.DetectedDoorMessage>  detected_doors_;

   public DetectedDoorListMessage()
   {
      detected_doors_ = new us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.DetectedDoorMessage> (16, new perception_msgs.msg.dds.DetectedDoorMessagePubSubType());

   }

   public DetectedDoorListMessage(DetectedDoorListMessage other)
   {
      this();
      set(other);
   }

   public void set(DetectedDoorListMessage other)
   {
      detected_doors_.set(other.detected_doors_);
   }


   /**
            * List of detected doors
            */
   public us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.DetectedDoorMessage>  getDetectedDoors()
   {
      return detected_doors_;
   }


   public static Supplier<DetectedDoorListMessagePubSubType> getPubSubType()
   {
      return DetectedDoorListMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return DetectedDoorListMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(DetectedDoorListMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (this.detected_doors_.size() != other.detected_doors_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.detected_doors_.size(); i++)
         {  if (!this.detected_doors_.get(i).epsilonEquals(other.detected_doors_.get(i), epsilon)) return false; }
      }

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof DetectedDoorListMessage)) return false;

      DetectedDoorListMessage otherMyClass = (DetectedDoorListMessage) other;

      if (!this.detected_doors_.equals(otherMyClass.detected_doors_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("DetectedDoorListMessage {");
      builder.append("detected_doors=");
      builder.append(this.detected_doors_);
      builder.append("}");
      return builder.toString();
   }
}
