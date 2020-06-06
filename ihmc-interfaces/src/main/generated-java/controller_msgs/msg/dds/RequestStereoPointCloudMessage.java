package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * Use this message to request a new point cloud from the stereo camera.
       */
public class RequestStereoPointCloudMessage extends Packet<RequestStereoPointCloudMessage> implements Settable<RequestStereoPointCloudMessage>, EpsilonComparable<RequestStereoPointCloudMessage>
{

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   public RequestStereoPointCloudMessage()
   {


   }

   public RequestStereoPointCloudMessage(RequestStereoPointCloudMessage other)
   {
      this();
      set(other);
   }

   public void set(RequestStereoPointCloudMessage other)
   {

      sequence_id_ = other.sequence_id_;

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


   public static Supplier<RequestStereoPointCloudMessagePubSubType> getPubSubType()
   {
      return RequestStereoPointCloudMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return RequestStereoPointCloudMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(RequestStereoPointCloudMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof RequestStereoPointCloudMessage)) return false;

      RequestStereoPointCloudMessage otherMyClass = (RequestStereoPointCloudMessage) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("RequestStereoPointCloudMessage {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);
      builder.append("}");
      return builder.toString();
   }
}
