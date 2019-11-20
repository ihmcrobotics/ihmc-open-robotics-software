package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC localization module.
       */
public class StampedPosePacket extends Packet<StampedPosePacket> implements Settable<StampedPosePacket>, EpsilonComparable<StampedPosePacket>
{
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   public us.ihmc.euclid.geometry.Pose3D pose_;
   public us.ihmc.euclid.tuple3D.Vector3D linear_velocity_;
   public us.ihmc.euclid.tuple3D.Vector3D angular_velocity_;
   public long timestamp_;
   public double confidence_factor_;
   public java.lang.StringBuilder frame_id_;

   public StampedPosePacket()
   {
      pose_ = new us.ihmc.euclid.geometry.Pose3D();
      linear_velocity_ = new us.ihmc.euclid.tuple3D.Vector3D();
      angular_velocity_ = new us.ihmc.euclid.tuple3D.Vector3D();
      frame_id_ = new java.lang.StringBuilder(255);
   }

   public StampedPosePacket(StampedPosePacket other)
   {
      this();
      set(other);
   }

   public void set(StampedPosePacket other)
   {
      sequence_id_ = other.sequence_id_;

      geometry_msgs.msg.dds.PosePubSubType.staticCopy(other.pose_, pose_);
      geometry_msgs.msg.dds.Vector3PubSubType.staticCopy(other.linear_velocity_, linear_velocity_);
      geometry_msgs.msg.dds.Vector3PubSubType.staticCopy(other.angular_velocity_, angular_velocity_);
      timestamp_ = other.timestamp_;

      confidence_factor_ = other.confidence_factor_;

      frame_id_.setLength(0);
      frame_id_.append(other.frame_id_);

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


   public us.ihmc.euclid.geometry.Pose3D getPose()
   {
      return pose_;
   }


   public us.ihmc.euclid.tuple3D.Vector3D getLinearVelocity()
   {
      return linear_velocity_;
   }


   public us.ihmc.euclid.tuple3D.Vector3D getAngularVelocity()
   {
      return angular_velocity_;
   }

   public void setTimestamp(long timestamp)
   {
      timestamp_ = timestamp;
   }
   public long getTimestamp()
   {
      return timestamp_;
   }

   public void setConfidenceFactor(double confidence_factor)
   {
      confidence_factor_ = confidence_factor;
   }
   public double getConfidenceFactor()
   {
      return confidence_factor_;
   }

   public void setFrameId(java.lang.String frame_id)
   {
      frame_id_.setLength(0);
      frame_id_.append(frame_id);
   }

   public java.lang.String getFrameIdAsString()
   {
      return getFrameId().toString();
   }
   public java.lang.StringBuilder getFrameId()
   {
      return frame_id_;
   }


   public static Supplier<StampedPosePacketPubSubType> getPubSubType()
   {
      return StampedPosePacketPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return StampedPosePacketPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(StampedPosePacket other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!this.pose_.epsilonEquals(other.pose_, epsilon)) return false;
      if (!this.linear_velocity_.epsilonEquals(other.linear_velocity_, epsilon)) return false;
      if (!this.angular_velocity_.epsilonEquals(other.angular_velocity_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.timestamp_, other.timestamp_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.confidence_factor_, other.confidence_factor_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.frame_id_, other.frame_id_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof StampedPosePacket)) return false;

      StampedPosePacket otherMyClass = (StampedPosePacket) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if (!this.pose_.equals(otherMyClass.pose_)) return false;
      if (!this.linear_velocity_.equals(otherMyClass.linear_velocity_)) return false;
      if (!this.angular_velocity_.equals(otherMyClass.angular_velocity_)) return false;
      if(this.timestamp_ != otherMyClass.timestamp_) return false;

      if(this.confidence_factor_ != otherMyClass.confidence_factor_) return false;

      if (!us.ihmc.idl.IDLTools.equals(this.frame_id_, otherMyClass.frame_id_)) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("StampedPosePacket {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("pose=");
      builder.append(this.pose_);      builder.append(", ");
      builder.append("linear_velocity=");
      builder.append(this.linear_velocity_);      builder.append(", ");
      builder.append("angular_velocity=");
      builder.append(this.angular_velocity_);      builder.append(", ");
      builder.append("timestamp=");
      builder.append(this.timestamp_);      builder.append(", ");
      builder.append("confidence_factor=");
      builder.append(this.confidence_factor_);      builder.append(", ");
      builder.append("frame_id=");
      builder.append(this.frame_id_);
      builder.append("}");
      return builder.toString();
   }
}
