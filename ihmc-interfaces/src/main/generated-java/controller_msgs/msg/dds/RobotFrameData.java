package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class RobotFrameData extends Packet<RobotFrameData> implements Settable<RobotFrameData>, EpsilonComparable<RobotFrameData>
{
   public long timestamp_;
   public int frame_name_hash_;
   public us.ihmc.euclid.geometry.Pose3D frame_pose_in_world_;
   public us.ihmc.idl.IDLSequence.StringBuilderHolder  frame_name_;

   public RobotFrameData()
   {
      frame_pose_in_world_ = new us.ihmc.euclid.geometry.Pose3D();
      frame_name_ = new us.ihmc.idl.IDLSequence.StringBuilderHolder (2056, "type_d");
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
      frame_name_.set(other.frame_name_);
   }

   public void setTimestamp(long timestamp)
   {
      timestamp_ = timestamp;
   }
   public long getTimestamp()
   {
      return timestamp_;
   }

   public void setFrameNameHash(int frame_name_hash)
   {
      frame_name_hash_ = frame_name_hash;
   }
   public int getFrameNameHash()
   {
      return frame_name_hash_;
   }


   public us.ihmc.euclid.geometry.Pose3D getFramePoseInWorld()
   {
      return frame_pose_in_world_;
   }


   public us.ihmc.idl.IDLSequence.StringBuilderHolder  getFrameName()
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
      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilderSequence(this.frame_name_, other.frame_name_, epsilon)) return false;


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
      if (!this.frame_name_.equals(otherMyClass.frame_name_)) return false;

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
