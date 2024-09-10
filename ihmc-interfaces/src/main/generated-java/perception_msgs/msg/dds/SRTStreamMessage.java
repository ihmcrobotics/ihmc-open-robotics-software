package perception_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * #############################
       * ## REQUEST MESSAGE FIELDS ###
       * #############################
       * #########################
       * ## ACK MESSAGE FIELDS ###
       * #########################
       * ############################
       * ## Status message fields ###
       * ############################
       */
public class SRTStreamMessage extends Packet<SRTStreamMessage> implements Settable<SRTStreamMessage>, EpsilonComparable<SRTStreamMessage>
{
   /**
            * UUID of message
            * Request: set to random UUID
            * Reply: match the request UUID
            * Status: 0
            */
   public ihmc_common_msgs.msg.dds.UUIDMessage id_;
   /**
            * Address to send the stream to
            */
   public java.lang.StringBuilder receiver_address_;
   /**
            * Port to send the stream to
            */
   public int receiver_port_;
   /**
            * Whether to connect or disconnect
            */
   public boolean connection_wanted_;
   /**
            * Camera intrinsics
            */
   public int image_width_;
   public int image_height_;
   public float fx_;
   public float fy_;
   public float cx_;
   public float cy_;
   public float depth_discretization_;
   /**
            * Position of the focal point at sensor data acquisition
            */
   public us.ihmc.euclid.tuple3D.Point3D position_;
   /**
            * Orientation of the focal point at sensor data acquisition
            */
   public us.ihmc.euclid.tuple4D.Quaternion orientation_;

   public SRTStreamMessage()
   {
      id_ = new ihmc_common_msgs.msg.dds.UUIDMessage();
      receiver_address_ = new java.lang.StringBuilder(255);
      position_ = new us.ihmc.euclid.tuple3D.Point3D();
      orientation_ = new us.ihmc.euclid.tuple4D.Quaternion();
   }

   public SRTStreamMessage(SRTStreamMessage other)
   {
      this();
      set(other);
   }

   public void set(SRTStreamMessage other)
   {
      ihmc_common_msgs.msg.dds.UUIDMessagePubSubType.staticCopy(other.id_, id_);
      receiver_address_.setLength(0);
      receiver_address_.append(other.receiver_address_);

      receiver_port_ = other.receiver_port_;

      connection_wanted_ = other.connection_wanted_;

      image_width_ = other.image_width_;

      image_height_ = other.image_height_;

      fx_ = other.fx_;

      fy_ = other.fy_;

      cx_ = other.cx_;

      cy_ = other.cy_;

      depth_discretization_ = other.depth_discretization_;

      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.position_, position_);
      geometry_msgs.msg.dds.QuaternionPubSubType.staticCopy(other.orientation_, orientation_);
   }


   /**
            * UUID of message
            * Request: set to random UUID
            * Reply: match the request UUID
            * Status: 0
            */
   public ihmc_common_msgs.msg.dds.UUIDMessage getId()
   {
      return id_;
   }

   /**
            * Address to send the stream to
            */
   public void setReceiverAddress(java.lang.String receiver_address)
   {
      receiver_address_.setLength(0);
      receiver_address_.append(receiver_address);
   }

   /**
            * Address to send the stream to
            */
   public java.lang.String getReceiverAddressAsString()
   {
      return getReceiverAddress().toString();
   }
   /**
            * Address to send the stream to
            */
   public java.lang.StringBuilder getReceiverAddress()
   {
      return receiver_address_;
   }

   /**
            * Port to send the stream to
            */
   public void setReceiverPort(int receiver_port)
   {
      receiver_port_ = receiver_port;
   }
   /**
            * Port to send the stream to
            */
   public int getReceiverPort()
   {
      return receiver_port_;
   }

   /**
            * Whether to connect or disconnect
            */
   public void setConnectionWanted(boolean connection_wanted)
   {
      connection_wanted_ = connection_wanted;
   }
   /**
            * Whether to connect or disconnect
            */
   public boolean getConnectionWanted()
   {
      return connection_wanted_;
   }

   /**
            * Camera intrinsics
            */
   public void setImageWidth(int image_width)
   {
      image_width_ = image_width;
   }
   /**
            * Camera intrinsics
            */
   public int getImageWidth()
   {
      return image_width_;
   }

   public void setImageHeight(int image_height)
   {
      image_height_ = image_height;
   }
   public int getImageHeight()
   {
      return image_height_;
   }

   public void setFx(float fx)
   {
      fx_ = fx;
   }
   public float getFx()
   {
      return fx_;
   }

   public void setFy(float fy)
   {
      fy_ = fy;
   }
   public float getFy()
   {
      return fy_;
   }

   public void setCx(float cx)
   {
      cx_ = cx;
   }
   public float getCx()
   {
      return cx_;
   }

   public void setCy(float cy)
   {
      cy_ = cy;
   }
   public float getCy()
   {
      return cy_;
   }

   public void setDepthDiscretization(float depth_discretization)
   {
      depth_discretization_ = depth_discretization;
   }
   public float getDepthDiscretization()
   {
      return depth_discretization_;
   }


   /**
            * Position of the focal point at sensor data acquisition
            */
   public us.ihmc.euclid.tuple3D.Point3D getPosition()
   {
      return position_;
   }


   /**
            * Orientation of the focal point at sensor data acquisition
            */
   public us.ihmc.euclid.tuple4D.Quaternion getOrientation()
   {
      return orientation_;
   }


   public static Supplier<SRTStreamMessagePubSubType> getPubSubType()
   {
      return SRTStreamMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return SRTStreamMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(SRTStreamMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.id_.epsilonEquals(other.id_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.receiver_address_, other.receiver_address_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.receiver_port_, other.receiver_port_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.connection_wanted_, other.connection_wanted_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.image_width_, other.image_width_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.image_height_, other.image_height_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.fx_, other.fx_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.fy_, other.fy_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.cx_, other.cx_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.cy_, other.cy_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.depth_discretization_, other.depth_discretization_, epsilon)) return false;

      if (!this.position_.epsilonEquals(other.position_, epsilon)) return false;
      if (!this.orientation_.epsilonEquals(other.orientation_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof SRTStreamMessage)) return false;

      SRTStreamMessage otherMyClass = (SRTStreamMessage) other;

      if (!this.id_.equals(otherMyClass.id_)) return false;
      if (!us.ihmc.idl.IDLTools.equals(this.receiver_address_, otherMyClass.receiver_address_)) return false;

      if(this.receiver_port_ != otherMyClass.receiver_port_) return false;

      if(this.connection_wanted_ != otherMyClass.connection_wanted_) return false;

      if(this.image_width_ != otherMyClass.image_width_) return false;

      if(this.image_height_ != otherMyClass.image_height_) return false;

      if(this.fx_ != otherMyClass.fx_) return false;

      if(this.fy_ != otherMyClass.fy_) return false;

      if(this.cx_ != otherMyClass.cx_) return false;

      if(this.cy_ != otherMyClass.cy_) return false;

      if(this.depth_discretization_ != otherMyClass.depth_discretization_) return false;

      if (!this.position_.equals(otherMyClass.position_)) return false;
      if (!this.orientation_.equals(otherMyClass.orientation_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("SRTStreamMessage {");
      builder.append("id=");
      builder.append(this.id_);      builder.append(", ");
      builder.append("receiver_address=");
      builder.append(this.receiver_address_);      builder.append(", ");
      builder.append("receiver_port=");
      builder.append(this.receiver_port_);      builder.append(", ");
      builder.append("connection_wanted=");
      builder.append(this.connection_wanted_);      builder.append(", ");
      builder.append("image_width=");
      builder.append(this.image_width_);      builder.append(", ");
      builder.append("image_height=");
      builder.append(this.image_height_);      builder.append(", ");
      builder.append("fx=");
      builder.append(this.fx_);      builder.append(", ");
      builder.append("fy=");
      builder.append(this.fy_);      builder.append(", ");
      builder.append("cx=");
      builder.append(this.cx_);      builder.append(", ");
      builder.append("cy=");
      builder.append(this.cy_);      builder.append(", ");
      builder.append("depth_discretization=");
      builder.append(this.depth_discretization_);      builder.append(", ");
      builder.append("position=");
      builder.append(this.position_);      builder.append(", ");
      builder.append("orientation=");
      builder.append(this.orientation_);
      builder.append("}");
      return builder.toString();
   }
}
