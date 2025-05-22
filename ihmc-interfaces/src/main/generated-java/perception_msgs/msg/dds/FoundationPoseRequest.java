package perception_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class FoundationPoseRequest extends Packet<FoundationPoseRequest> implements Settable<FoundationPoseRequest>, EpsilonComparable<FoundationPoseRequest>
{
   /**
            * ID that will be assigned to the object
            */
   public java.lang.StringBuilder object_id_;
   /**
            * Name of the mesh file for the object
            */
   public java.lang.StringBuilder mesh_file_;
   /**
            * Color image containing the object
            */
   public perception_msgs.msg.dds.ImageMessage color_;
   /**
            * Depth image containing the object
            */
   public perception_msgs.msg.dds.ImageMessage depth_;
   /**
            * Mask of the object in the image
            */
   public perception_msgs.msg.dds.ImageMessage object_mask_;

   public FoundationPoseRequest()
   {
      object_id_ = new java.lang.StringBuilder(255);
      mesh_file_ = new java.lang.StringBuilder(255);
      color_ = new perception_msgs.msg.dds.ImageMessage();
      depth_ = new perception_msgs.msg.dds.ImageMessage();
      object_mask_ = new perception_msgs.msg.dds.ImageMessage();
   }

   public FoundationPoseRequest(FoundationPoseRequest other)
   {
      this();
      set(other);
   }

   public void set(FoundationPoseRequest other)
   {
      object_id_.setLength(0);
      object_id_.append(other.object_id_);

      mesh_file_.setLength(0);
      mesh_file_.append(other.mesh_file_);

      perception_msgs.msg.dds.ImageMessagePubSubType.staticCopy(other.color_, color_);
      perception_msgs.msg.dds.ImageMessagePubSubType.staticCopy(other.depth_, depth_);
      perception_msgs.msg.dds.ImageMessagePubSubType.staticCopy(other.object_mask_, object_mask_);
   }

   /**
            * ID that will be assigned to the object
            */
   public void setObjectId(java.lang.String object_id)
   {
      object_id_.setLength(0);
      object_id_.append(object_id);
   }

   /**
            * ID that will be assigned to the object
            */
   public java.lang.String getObjectIdAsString()
   {
      return getObjectId().toString();
   }
   /**
            * ID that will be assigned to the object
            */
   public java.lang.StringBuilder getObjectId()
   {
      return object_id_;
   }

   /**
            * Name of the mesh file for the object
            */
   public void setMeshFile(java.lang.String mesh_file)
   {
      mesh_file_.setLength(0);
      mesh_file_.append(mesh_file);
   }

   /**
            * Name of the mesh file for the object
            */
   public java.lang.String getMeshFileAsString()
   {
      return getMeshFile().toString();
   }
   /**
            * Name of the mesh file for the object
            */
   public java.lang.StringBuilder getMeshFile()
   {
      return mesh_file_;
   }


   /**
            * Color image containing the object
            */
   public perception_msgs.msg.dds.ImageMessage getColor()
   {
      return color_;
   }


   /**
            * Depth image containing the object
            */
   public perception_msgs.msg.dds.ImageMessage getDepth()
   {
      return depth_;
   }


   /**
            * Mask of the object in the image
            */
   public perception_msgs.msg.dds.ImageMessage getObjectMask()
   {
      return object_mask_;
   }


   public static Supplier<FoundationPoseRequestPubSubType> getPubSubType()
   {
      return FoundationPoseRequestPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return FoundationPoseRequestPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(FoundationPoseRequest other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.object_id_, other.object_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.mesh_file_, other.mesh_file_, epsilon)) return false;

      if (!this.color_.epsilonEquals(other.color_, epsilon)) return false;
      if (!this.depth_.epsilonEquals(other.depth_, epsilon)) return false;
      if (!this.object_mask_.epsilonEquals(other.object_mask_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof FoundationPoseRequest)) return false;

      FoundationPoseRequest otherMyClass = (FoundationPoseRequest) other;

      if (!us.ihmc.idl.IDLTools.equals(this.object_id_, otherMyClass.object_id_)) return false;

      if (!us.ihmc.idl.IDLTools.equals(this.mesh_file_, otherMyClass.mesh_file_)) return false;

      if (!this.color_.equals(otherMyClass.color_)) return false;
      if (!this.depth_.equals(otherMyClass.depth_)) return false;
      if (!this.object_mask_.equals(otherMyClass.object_mask_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("FoundationPoseRequest {");
      builder.append("object_id=");
      builder.append(this.object_id_);      builder.append(", ");
      builder.append("mesh_file=");
      builder.append(this.mesh_file_);      builder.append(", ");
      builder.append("color=");
      builder.append(this.color_);      builder.append(", ");
      builder.append("depth=");
      builder.append(this.depth_);      builder.append(", ");
      builder.append("object_mask=");
      builder.append(this.object_mask_);
      builder.append("}");
      return builder.toString();
   }
}
