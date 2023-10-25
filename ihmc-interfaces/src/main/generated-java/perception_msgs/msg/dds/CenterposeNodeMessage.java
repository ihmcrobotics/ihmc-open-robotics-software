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
   /**
            * Object ID
            */
   public int object_id_;
   /**
            * Position and Orientation of the object
            */
   public us.ihmc.euclid.geometry.Pose3D pose_;
   /**
            * How confident are we about what object it is. Mostly used to NN based detections
            */
   public double confidence_;
   /**
            * Object Category
            */
   public java.lang.StringBuilder object_type_;
   /**
            * 2D Vertices of the 3d object bounding box projected onto image plane
            */
   public us.ihmc.euclid.tuple3D.Point3D[] bounding_box_2d_vertices_;
   /**
            * 3d Vertices of the 3d object Bounding box
            */
   public us.ihmc.euclid.tuple3D.Point3D[] bounding_box_vertices_;
   /**
            * Break frequency filter value for nodes that are alpha filtered
            */
   public float break_frequency_;

   public CenterposeNodeMessage()
   {
      detectable_scene_node_ = new perception_msgs.msg.dds.DetectableSceneNodeMessage();
      pose_ = new us.ihmc.euclid.geometry.Pose3D();
      object_type_ = new java.lang.StringBuilder(255);
      bounding_box_2d_vertices_ = new us.ihmc.euclid.tuple3D.Point3D[8];

      for(int i1 = 0; i1 < bounding_box_2d_vertices_.length; ++i1)
      {
          bounding_box_2d_vertices_[i1] = new us.ihmc.euclid.tuple3D.Point3D();
      }
      bounding_box_vertices_ = new us.ihmc.euclid.tuple3D.Point3D[8];

      for(int i3 = 0; i3 < bounding_box_vertices_.length; ++i3)
      {
          bounding_box_vertices_[i3] = new us.ihmc.euclid.tuple3D.Point3D();
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
      object_id_ = other.object_id_;

      geometry_msgs.msg.dds.PosePubSubType.staticCopy(other.pose_, pose_);
      confidence_ = other.confidence_;

      object_type_.setLength(0);
      object_type_.append(other.object_type_);

      for(int i5 = 0; i5 < bounding_box_2d_vertices_.length; ++i5)
      {
            geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.bounding_box_2d_vertices_[i5], bounding_box_2d_vertices_[i5]);}

      for(int i7 = 0; i7 < bounding_box_vertices_.length; ++i7)
      {
            geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.bounding_box_vertices_[i7], bounding_box_vertices_[i7]);}

      break_frequency_ = other.break_frequency_;

   }


   /**
            * The detectable scene node information
            */
   public perception_msgs.msg.dds.DetectableSceneNodeMessage getDetectableSceneNode()
   {
      return detectable_scene_node_;
   }

   /**
            * Object ID
            */
   public void setObjectId(int object_id)
   {
      object_id_ = object_id;
   }
   /**
            * Object ID
            */
   public int getObjectId()
   {
      return object_id_;
   }


   /**
            * Position and Orientation of the object
            */
   public us.ihmc.euclid.geometry.Pose3D getPose()
   {
      return pose_;
   }

   /**
            * How confident are we about what object it is. Mostly used to NN based detections
            */
   public void setConfidence(double confidence)
   {
      confidence_ = confidence;
   }
   /**
            * How confident are we about what object it is. Mostly used to NN based detections
            */
   public double getConfidence()
   {
      return confidence_;
   }

   /**
            * Object Category
            */
   public void setObjectType(java.lang.String object_type)
   {
      object_type_.setLength(0);
      object_type_.append(object_type);
   }

   /**
            * Object Category
            */
   public java.lang.String getObjectTypeAsString()
   {
      return getObjectType().toString();
   }
   /**
            * Object Category
            */
   public java.lang.StringBuilder getObjectType()
   {
      return object_type_;
   }


   /**
            * 2D Vertices of the 3d object bounding box projected onto image plane
            */
   public us.ihmc.euclid.tuple3D.Point3D[] getBoundingBox2dVertices()
   {
      return bounding_box_2d_vertices_;
   }


   /**
            * 3d Vertices of the 3d object Bounding box
            */
   public us.ihmc.euclid.tuple3D.Point3D[] getBoundingBoxVertices()
   {
      return bounding_box_vertices_;
   }

   /**
            * Break frequency filter value for nodes that are alpha filtered
            */
   public void setBreakFrequency(float break_frequency)
   {
      break_frequency_ = break_frequency;
   }
   /**
            * Break frequency filter value for nodes that are alpha filtered
            */
   public float getBreakFrequency()
   {
      return break_frequency_;
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
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.object_id_, other.object_id_, epsilon)) return false;

      if (!this.pose_.epsilonEquals(other.pose_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.confidence_, other.confidence_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.object_type_, other.object_type_, epsilon)) return false;

      for(int i9 = 0; i9 < bounding_box_2d_vertices_.length; ++i9)
      {
              if (!this.bounding_box_2d_vertices_[i9].epsilonEquals(other.bounding_box_2d_vertices_[i9], epsilon)) return false;
      }

      for(int i11 = 0; i11 < bounding_box_vertices_.length; ++i11)
      {
              if (!this.bounding_box_vertices_[i11].epsilonEquals(other.bounding_box_vertices_[i11], epsilon)) return false;
      }

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.break_frequency_, other.break_frequency_, epsilon)) return false;


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
      if(this.object_id_ != otherMyClass.object_id_) return false;

      if (!this.pose_.equals(otherMyClass.pose_)) return false;
      if(this.confidence_ != otherMyClass.confidence_) return false;

      if (!us.ihmc.idl.IDLTools.equals(this.object_type_, otherMyClass.object_type_)) return false;

      for(int i13 = 0; i13 < bounding_box_2d_vertices_.length; ++i13)
      {
                if (!this.bounding_box_2d_vertices_[i13].equals(otherMyClass.bounding_box_2d_vertices_[i13])) return false;
      }
      for(int i15 = 0; i15 < bounding_box_vertices_.length; ++i15)
      {
                if (!this.bounding_box_vertices_[i15].equals(otherMyClass.bounding_box_vertices_[i15])) return false;
      }
      if(this.break_frequency_ != otherMyClass.break_frequency_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("CenterposeNodeMessage {");
      builder.append("detectable_scene_node=");
      builder.append(this.detectable_scene_node_);      builder.append(", ");
      builder.append("object_id=");
      builder.append(this.object_id_);      builder.append(", ");
      builder.append("pose=");
      builder.append(this.pose_);      builder.append(", ");
      builder.append("confidence=");
      builder.append(this.confidence_);      builder.append(", ");
      builder.append("object_type=");
      builder.append(this.object_type_);      builder.append(", ");
      builder.append("bounding_box_2d_vertices=");
      builder.append(java.util.Arrays.toString(this.bounding_box_2d_vertices_));      builder.append(", ");
      builder.append("bounding_box_vertices=");
      builder.append(java.util.Arrays.toString(this.bounding_box_vertices_));      builder.append(", ");
      builder.append("break_frequency=");
      builder.append(this.break_frequency_);
      builder.append("}");
      return builder.toString();
   }
}
