package perception_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * An perception scene node for an ArUco marker
       * The topic name identifies the node.
       */
public class CenterposeNodeMessage extends Packet<CenterposeNodeMessage> implements Settable<CenterposeNodeMessage>, EpsilonComparable<CenterposeNodeMessage>
{
   /**
            * The detectable scene node information
            */
   public perception_msgs.msg.dds.DetectableSceneNodeMessage detectable_scene_node_;
   public java.lang.StringBuilder object_type_;
   public int object_id_;
   public double confidence_;
   public us.ihmc.euclid.tuple3D.Point3D[] bounding_box_vertices_;
   public us.ihmc.euclid.tuple3D.Point3D[] bounding_box_vertices_2d_;
   public boolean enable_tracking_;

   public CenterposeNodeMessage()
   {
      detectable_scene_node_ = new perception_msgs.msg.dds.DetectableSceneNodeMessage();
      object_type_ = new java.lang.StringBuilder(255);
      bounding_box_vertices_ = new us.ihmc.euclid.tuple3D.Point3D[8];

      for(int i1 = 0; i1 < bounding_box_vertices_.length; ++i1)
      {
          bounding_box_vertices_[i1] = new us.ihmc.euclid.tuple3D.Point3D();
      }
      bounding_box_vertices_2d_ = new us.ihmc.euclid.tuple3D.Point3D[8];

      for(int i3 = 0; i3 < bounding_box_vertices_2d_.length; ++i3)
      {
          bounding_box_vertices_2d_[i3] = new us.ihmc.euclid.tuple3D.Point3D();
      }
   }

   public CenterposeNodeMessage(CenterposeNodeMessage other)
   {
      this();
      set(other);
   }

   public void set(CenterposeNodeMessage other)
   {
      perception_msgs.msg.dds.DetectableSceneNodeMessagePubSubType.staticCopy(other.detectable_scene_node_, detectable_scene_node_);
      object_type_.setLength(0);
      object_type_.append(other.object_type_);

      object_id_ = other.object_id_;

      confidence_ = other.confidence_;

      for(int i5 = 0; i5 < bounding_box_vertices_.length; ++i5)
      {
            geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.bounding_box_vertices_[i5], bounding_box_vertices_[i5]);}

      for(int i7 = 0; i7 < bounding_box_vertices_2d_.length; ++i7)
      {
            geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.bounding_box_vertices_2d_[i7], bounding_box_vertices_2d_[i7]);}

      enable_tracking_ = other.enable_tracking_;

   }


   /**
            * The detectable scene node information
            */
   public perception_msgs.msg.dds.DetectableSceneNodeMessage getDetectableSceneNode()
   {
      return detectable_scene_node_;
   }

   public void setObjectType(java.lang.String object_type)
   {
      object_type_.setLength(0);
      object_type_.append(object_type);
   }

   public java.lang.String getObjectTypeAsString()
   {
      return getObjectType().toString();
   }
   public java.lang.StringBuilder getObjectType()
   {
      return object_type_;
   }

   public void setObjectId(int object_id)
   {
      object_id_ = object_id;
   }
   public int getObjectId()
   {
      return object_id_;
   }

   public void setConfidence(double confidence)
   {
      confidence_ = confidence;
   }
   public double getConfidence()
   {
      return confidence_;
   }


   public us.ihmc.euclid.tuple3D.Point3D[] getBoundingBoxVertices()
   {
      return bounding_box_vertices_;
   }


   public us.ihmc.euclid.tuple3D.Point3D[] getBoundingBoxVertices2d()
   {
      return bounding_box_vertices_2d_;
   }

   public void setEnableTracking(boolean enable_tracking)
   {
      enable_tracking_ = enable_tracking;
   }
   public boolean getEnableTracking()
   {
      return enable_tracking_;
   }


   public static Supplier<CenterposeNodeMessagePubSubType> getPubSubType()
   {
      return CenterposeNodeMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return CenterposeNodeMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(CenterposeNodeMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.detectable_scene_node_.epsilonEquals(other.detectable_scene_node_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.object_type_, other.object_type_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.object_id_, other.object_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.confidence_, other.confidence_, epsilon)) return false;

      for(int i9 = 0; i9 < bounding_box_vertices_.length; ++i9)
      {
              if (!this.bounding_box_vertices_[i9].epsilonEquals(other.bounding_box_vertices_[i9], epsilon)) return false;
      }

      for(int i11 = 0; i11 < bounding_box_vertices_2d_.length; ++i11)
      {
              if (!this.bounding_box_vertices_2d_[i11].epsilonEquals(other.bounding_box_vertices_2d_[i11], epsilon)) return false;
      }

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.enable_tracking_, other.enable_tracking_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof CenterposeNodeMessage)) return false;

      CenterposeNodeMessage otherMyClass = (CenterposeNodeMessage) other;

      if (!this.detectable_scene_node_.equals(otherMyClass.detectable_scene_node_)) return false;
      if (!us.ihmc.idl.IDLTools.equals(this.object_type_, otherMyClass.object_type_)) return false;

      if(this.object_id_ != otherMyClass.object_id_) return false;

      if(this.confidence_ != otherMyClass.confidence_) return false;

      for(int i13 = 0; i13 < bounding_box_vertices_.length; ++i13)
      {
                if (!this.bounding_box_vertices_[i13].equals(otherMyClass.bounding_box_vertices_[i13])) return false;
      }
      for(int i15 = 0; i15 < bounding_box_vertices_2d_.length; ++i15)
      {
                if (!this.bounding_box_vertices_2d_[i15].equals(otherMyClass.bounding_box_vertices_2d_[i15])) return false;
      }
      if(this.enable_tracking_ != otherMyClass.enable_tracking_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("CenterposeNodeMessage {");
      builder.append("detectable_scene_node=");
      builder.append(this.detectable_scene_node_);      builder.append(", ");
      builder.append("object_type=");
      builder.append(this.object_type_);      builder.append(", ");
      builder.append("object_id=");
      builder.append(this.object_id_);      builder.append(", ");
      builder.append("confidence=");
      builder.append(this.confidence_);      builder.append(", ");
      builder.append("bounding_box_vertices=");
      builder.append(java.util.Arrays.toString(this.bounding_box_vertices_));      builder.append(", ");
      builder.append("bounding_box_vertices_2d=");
      builder.append(java.util.Arrays.toString(this.bounding_box_vertices_2d_));      builder.append(", ");
      builder.append("enable_tracking=");
      builder.append(this.enable_tracking_);
      builder.append("}");
      return builder.toString();
   }
}
