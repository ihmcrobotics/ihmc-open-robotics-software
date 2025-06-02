package perception_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class FoundationPoseResult extends Packet<FoundationPoseResult> implements Settable<FoundationPoseResult>, EpsilonComparable<FoundationPoseResult>
{
   /**
            * Detection timestamp. Should be the acquisition time of the image used for detection
            */
   public ihmc_common_msgs.msg.dds.InstantMessage timestamp_;
   /**
            * ID of the detected object
            */
   public java.lang.StringBuilder object_id_;
   /**
            * Mesh file name of the object (to identify it's class)
            */
   public java.lang.StringBuilder mesh_file_;
   /**
            * Pose of the object
            */
   public us.ihmc.euclid.geometry.Pose3D object_pose_;

   public FoundationPoseResult()
   {
      timestamp_ = new ihmc_common_msgs.msg.dds.InstantMessage();
      object_id_ = new java.lang.StringBuilder(255);
      mesh_file_ = new java.lang.StringBuilder(255);
      object_pose_ = new us.ihmc.euclid.geometry.Pose3D();
   }

   public FoundationPoseResult(FoundationPoseResult other)
   {
      this();
      set(other);
   }

   public void set(FoundationPoseResult other)
   {
      ihmc_common_msgs.msg.dds.InstantMessagePubSubType.staticCopy(other.timestamp_, timestamp_);
      object_id_.setLength(0);
      object_id_.append(other.object_id_);

      mesh_file_.setLength(0);
      mesh_file_.append(other.mesh_file_);

      geometry_msgs.msg.dds.PosePubSubType.staticCopy(other.object_pose_, object_pose_);
   }


   /**
            * Detection timestamp. Should be the acquisition time of the image used for detection
            */
   public ihmc_common_msgs.msg.dds.InstantMessage getTimestamp()
   {
      return timestamp_;
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
            * Mesh file name of the object (to identify it's class)
            */
   public void setMeshFile(java.lang.String mesh_file)
   {
      mesh_file_.setLength(0);
      mesh_file_.append(mesh_file);
   }

   /**
            * Mesh file name of the object (to identify it's class)
            */
   public java.lang.String getMeshFileAsString()
   {
      return getMeshFile().toString();
   }
   /**
            * Mesh file name of the object (to identify it's class)
            */
   public java.lang.StringBuilder getMeshFile()
   {
      return mesh_file_;
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

      if (!this.timestamp_.epsilonEquals(other.timestamp_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.object_id_, other.object_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.mesh_file_, other.mesh_file_, epsilon)) return false;

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

      if (!this.timestamp_.equals(otherMyClass.timestamp_)) return false;
      if (!us.ihmc.idl.IDLTools.equals(this.object_id_, otherMyClass.object_id_)) return false;

      if (!us.ihmc.idl.IDLTools.equals(this.mesh_file_, otherMyClass.mesh_file_)) return false;

      if (!this.object_pose_.equals(otherMyClass.object_pose_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("FoundationPoseResult {");
      builder.append("timestamp=");
      builder.append(this.timestamp_);      builder.append(", ");
      builder.append("object_id=");
      builder.append(this.object_id_);      builder.append(", ");
      builder.append("mesh_file=");
      builder.append(this.mesh_file_);      builder.append(", ");
      builder.append("object_pose=");
      builder.append(this.object_pose_);
      builder.append("}");
      return builder.toString();
   }
}
