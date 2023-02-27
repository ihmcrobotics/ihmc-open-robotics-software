package ihmc_common_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message carries robot's joints, IMU, and force sensors' reference frame information.
       * It is published frequently from the IHMC state estimator along with robot configuration data.
       */
public class RobotFrameData extends Packet<RobotFrameData> implements Settable<RobotFrameData>, EpsilonComparable<RobotFrameData>
{
   /**
            * Platform dependent.
            * Time signal in nanoseconds that can be used to synchronize two time sources.
            */
   public long timestamp_;
   /**
            * Hash code of this frame.
            */
   public int frame_name_hash_;
   /**
            * Pose3D of this frame in the world frame.
            */
   public us.ihmc.euclid.geometry.Pose3D frame_pose_in_world_;
   /**
            * Name of this frame.
            */
   public java.lang.StringBuilder frame_name_;

   public RobotFrameData()
   {
      frame_pose_in_world_ = new us.ihmc.euclid.geometry.Pose3D();
      frame_name_ = new java.lang.StringBuilder(255);
   }

   public RobotFrameData(RobotFrameData other)
   {
      this();
      set(other);
   }

   public void set(RobotFrameData other)
   {
      timestamp_ = other.timestamp_;

      frame_name_hash_ = other.frame_name_hash_;

      geometry_msgs.msg.dds.PosePubSubType.staticCopy(other.frame_pose_in_world_, frame_pose_in_world_);
      frame_name_.setLength(0);
      frame_name_.append(other.frame_name_);

   }

   /**
            * Platform dependent.
            * Time signal in nanoseconds that can be used to synchronize two time sources.
            */
   public void setTimestamp(long timestamp)
   {
      timestamp_ = timestamp;
   }
   /**
            * Platform dependent.
            * Time signal in nanoseconds that can be used to synchronize two time sources.
            */
   public long getTimestamp()
   {
      return timestamp_;
   }

   /**
            * Hash code of this frame.
            */
   public void setFrameNameHash(int frame_name_hash)
   {
      frame_name_hash_ = frame_name_hash;
   }
   /**
            * Hash code of this frame.
            */
   public int getFrameNameHash()
   {
      return frame_name_hash_;
   }


   /**
            * Pose3D of this frame in the world frame.
            */
   public us.ihmc.euclid.geometry.Pose3D getFramePoseInWorld()
   {
      return frame_pose_in_world_;
   }

   /**
            * Name of this frame.
            */
   public void setFrameName(java.lang.String frame_name)
   {
      frame_name_.setLength(0);
      frame_name_.append(frame_name);
   }

   /**
            * Name of this frame.
            */
   public java.lang.String getFrameNameAsString()
   {
      return getFrameName().toString();
   }
   /**
            * Name of this frame.
            */
   public java.lang.StringBuilder getFrameName()
   {
      return frame_name_;
   }


   public static Supplier<RobotFrameDataPubSubType> getPubSubType()
   {
      return RobotFrameDataPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return RobotFrameDataPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(RobotFrameData other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.timestamp_, other.timestamp_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.frame_name_hash_, other.frame_name_hash_, epsilon)) return false;

      if (!this.frame_pose_in_world_.epsilonEquals(other.frame_pose_in_world_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.frame_name_, other.frame_name_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof RobotFrameData)) return false;

      RobotFrameData otherMyClass = (RobotFrameData) other;

      if(this.timestamp_ != otherMyClass.timestamp_) return false;

      if(this.frame_name_hash_ != otherMyClass.frame_name_hash_) return false;

      if (!this.frame_pose_in_world_.equals(otherMyClass.frame_pose_in_world_)) return false;
      if (!us.ihmc.idl.IDLTools.equals(this.frame_name_, otherMyClass.frame_name_)) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("RobotFrameData {");
      builder.append("timestamp=");
      builder.append(this.timestamp_);      builder.append(", ");
      builder.append("frame_name_hash=");
      builder.append(this.frame_name_hash_);      builder.append(", ");
      builder.append("frame_pose_in_world=");
      builder.append(this.frame_pose_in_world_);      builder.append(", ");
      builder.append("frame_name=");
      builder.append(this.frame_name_);
      builder.append("}");
      return builder.toString();
   }
}
