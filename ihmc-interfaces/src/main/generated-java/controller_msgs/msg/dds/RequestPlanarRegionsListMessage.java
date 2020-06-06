package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is obsolete and will be removed in the near future.
       * This message is part of the IHMC robot environment awareness (REA) module.
       * Use this message to request new planar regions
       * @deprecated REA always publishes planar regions when running. To request REA to clear its internal state, see REAStateRequestMessage.
       */
public class RequestPlanarRegionsListMessage extends Packet<RequestPlanarRegionsListMessage> implements Settable<RequestPlanarRegionsListMessage>, EpsilonComparable<RequestPlanarRegionsListMessage>
{

   public static final byte SINGLE_UPDATE = (byte) 0;

   public static final byte CONTINUOUS_UPDATE = (byte) 1;

   public static final byte STOP_UPDATE = (byte) 2;

   public static final byte CLEAR = (byte) 3;

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   public byte planar_regions_request_type_;

   public controller_msgs.msg.dds.BoundingBox3DMessage bounding_box_in_world_for_request_;

   public RequestPlanarRegionsListMessage()
   {



      bounding_box_in_world_for_request_ = new controller_msgs.msg.dds.BoundingBox3DMessage();

   }

   public RequestPlanarRegionsListMessage(RequestPlanarRegionsListMessage other)
   {
      this();
      set(other);
   }

   public void set(RequestPlanarRegionsListMessage other)
   {

      sequence_id_ = other.sequence_id_;


      planar_regions_request_type_ = other.planar_regions_request_type_;


      controller_msgs.msg.dds.BoundingBox3DMessagePubSubType.staticCopy(other.bounding_box_in_world_for_request_, bounding_box_in_world_for_request_);
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


   public void setPlanarRegionsRequestType(byte planar_regions_request_type)
   {
      planar_regions_request_type_ = planar_regions_request_type;
   }
   public byte getPlanarRegionsRequestType()
   {
      return planar_regions_request_type_;
   }



   public controller_msgs.msg.dds.BoundingBox3DMessage getBoundingBoxInWorldForRequest()
   {
      return bounding_box_in_world_for_request_;
   }


   public static Supplier<RequestPlanarRegionsListMessagePubSubType> getPubSubType()
   {
      return RequestPlanarRegionsListMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return RequestPlanarRegionsListMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(RequestPlanarRegionsListMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.planar_regions_request_type_, other.planar_regions_request_type_, epsilon)) return false;


      if (!this.bounding_box_in_world_for_request_.epsilonEquals(other.bounding_box_in_world_for_request_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof RequestPlanarRegionsListMessage)) return false;

      RequestPlanarRegionsListMessage otherMyClass = (RequestPlanarRegionsListMessage) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if(this.planar_regions_request_type_ != otherMyClass.planar_regions_request_type_) return false;


      if (!this.bounding_box_in_world_for_request_.equals(otherMyClass.bounding_box_in_world_for_request_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("RequestPlanarRegionsListMessage {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("planar_regions_request_type=");
      builder.append(this.planar_regions_request_type_);      builder.append(", ");

      builder.append("bounding_box_in_world_for_request=");
      builder.append(this.bounding_box_in_world_for_request_);
      builder.append("}");
      return builder.toString();
   }
}
