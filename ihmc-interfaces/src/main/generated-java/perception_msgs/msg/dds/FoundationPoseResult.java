package perception_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class FoundationPoseResult extends Packet<FoundationPoseResult> implements Settable<FoundationPoseResult>, EpsilonComparable<FoundationPoseResult>
{
   /**
            * ID of the detected object
            */
   public java.lang.StringBuilder object_id_;
   /**
            * Pose of the object
            */
   public us.ihmc.euclid.geometry.Pose3D object_pose_;

   public FoundationPoseResult()
   {
      object_id_ = new java.lang.StringBuilder(255);
      object_pose_ = new us.ihmc.euclid.geometry.Pose3D();
   }

   public FoundationPoseResult(FoundationPoseResult other)
   {
      this();
      set(other);
   }

   public void set(FoundationPoseResult other)
   {
      object_id_.setLength(0);
      object_id_.append(other.object_id_);

      geometry_msgs.msg.dds.PosePubSubType.staticCopy(other.object_pose_, object_pose_);
   }

   /**
            * ID of the detected object
            */
   public void setObjectId(java.lang.String object_id)
   {
      object_id_.setLength(0);
      object_id_.append(object_id);
   }

   /**
            * ID of the detected object
            */
   public java.lang.String getObjectIdAsString()
   {
      return getObjectId().toString();
   }
   /**
            * ID of the detected object
            */
   public java.lang.StringBuilder getObjectId()
   {
      return object_id_;
   }


   /**
            * Pose of the object
            */
   public us.ihmc.euclid.geometry.Pose3D getObjectPose()
   {
      return object_pose_;
   }


   public static Supplier<FoundationPoseResultPubSubType> getPubSubType()
   {
      return FoundationPoseResultPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return FoundationPoseResultPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(FoundationPoseResult other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.object_id_, other.object_id_, epsilon)) return false;

      if (!this.object_pose_.epsilonEquals(other.object_pose_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof FoundationPoseResult)) return false;

      FoundationPoseResult otherMyClass = (FoundationPoseResult) other;

      if (!us.ihmc.idl.IDLTools.equals(this.object_id_, otherMyClass.object_id_)) return false;

      if (!this.object_pose_.equals(otherMyClass.object_pose_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("FoundationPoseResult {");
      builder.append("object_id=");
      builder.append(this.object_id_);      builder.append(", ");
      builder.append("object_pose=");
      builder.append(this.object_pose_);
      builder.append("}");
      return builder.toString();
   }
}
