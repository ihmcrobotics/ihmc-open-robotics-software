package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * Should disappear for the ROS equivalent.
       */
public class StereoVisionPointCloudMessage extends Packet<StereoVisionPointCloudMessage> implements Settable<StereoVisionPointCloudMessage>, EpsilonComparable<StereoVisionPointCloudMessage>
{
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   public long robot_timestamp_;
   public us.ihmc.idl.IDLSequence.Float  point_cloud_;
   public us.ihmc.idl.IDLSequence.Integer  colors_;

   public StereoVisionPointCloudMessage()
   {
      point_cloud_ = new us.ihmc.idl.IDLSequence.Float (600000, "type_5");

      colors_ = new us.ihmc.idl.IDLSequence.Integer (200000, "type_2");

   }

   public StereoVisionPointCloudMessage(StereoVisionPointCloudMessage other)
   {
      this();
      set(other);
   }

   public void set(StereoVisionPointCloudMessage other)
   {
      sequence_id_ = other.sequence_id_;

      robot_timestamp_ = other.robot_timestamp_;

      point_cloud_.set(other.point_cloud_);
      colors_.set(other.colors_);
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

   public void setRobotTimestamp(long robot_timestamp)
   {
      robot_timestamp_ = robot_timestamp;
   }
   public long getRobotTimestamp()
   {
      return robot_timestamp_;
   }


   public us.ihmc.idl.IDLSequence.Float  getPointCloud()
   {
      return point_cloud_;
   }


   public us.ihmc.idl.IDLSequence.Integer  getColors()
   {
      return colors_;
   }


   public static Supplier<StereoVisionPointCloudMessagePubSubType> getPubSubType()
   {
      return StereoVisionPointCloudMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return StereoVisionPointCloudMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(StereoVisionPointCloudMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.robot_timestamp_, other.robot_timestamp_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsFloatSequence(this.point_cloud_, other.point_cloud_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsIntegerSequence(this.colors_, other.colors_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof StereoVisionPointCloudMessage)) return false;

      StereoVisionPointCloudMessage otherMyClass = (StereoVisionPointCloudMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.robot_timestamp_ != otherMyClass.robot_timestamp_) return false;

      if (!this.point_cloud_.equals(otherMyClass.point_cloud_)) return false;
      if (!this.colors_.equals(otherMyClass.colors_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("StereoVisionPointCloudMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("robot_timestamp=");
      builder.append(this.robot_timestamp_);      builder.append(", ");
      builder.append("point_cloud=");
      builder.append(this.point_cloud_);      builder.append(", ");
      builder.append("colors=");
      builder.append(this.colors_);
      builder.append("}");
      return builder.toString();
   }
}
