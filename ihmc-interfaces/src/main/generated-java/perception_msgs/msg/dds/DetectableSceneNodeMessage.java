package perception_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * A detectable perception scene node
       * The topic name identifies the node.
       */
public class DetectableSceneNodeMessage extends Packet<DetectableSceneNodeMessage> implements Settable<DetectableSceneNodeMessage>, EpsilonComparable<DetectableSceneNodeMessage>
{
   /**
            * The name of the scene node
            */
   public java.lang.StringBuilder name_;
   /**
            * Whether or not the node is currently detected
            */
   public boolean currently_detected_;
   /**
            * Transform of the node's frame to world frame
            */
   public controller_msgs.msg.dds.RigidBodyTransformMessage transform_to_world_;
   /**
            * If this node is trackable via an ArUco maker, this is the ArUco marker's
            * transform to world frame. This is so we can reset overriden node
            * poses back to ArUco relative ones.
            */
   public controller_msgs.msg.dds.RigidBodyTransformMessage aruco_marker_transform_to_world_;
   /**
            * Keeps track of when the operator has overridden the pose of this node
            */
   public boolean is_pose_overridden_by_operator_;
   /**
            * For static ArUco relative nodes, if it's currently locked in
            */
   public boolean is_static_relative_pose_locked_in_;

   public DetectableSceneNodeMessage()
   {
      name_ = new java.lang.StringBuilder(255);
      transform_to_world_ = new controller_msgs.msg.dds.RigidBodyTransformMessage();
      aruco_marker_transform_to_world_ = new controller_msgs.msg.dds.RigidBodyTransformMessage();
   }

   public DetectableSceneNodeMessage(DetectableSceneNodeMessage other)
   {
      this();
      set(other);
   }

   public void set(DetectableSceneNodeMessage other)
   {
      name_.setLength(0);
      name_.append(other.name_);

      currently_detected_ = other.currently_detected_;

      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.staticCopy(other.transform_to_world_, transform_to_world_);
      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.staticCopy(other.aruco_marker_transform_to_world_, aruco_marker_transform_to_world_);
      is_pose_overridden_by_operator_ = other.is_pose_overridden_by_operator_;

      is_static_relative_pose_locked_in_ = other.is_static_relative_pose_locked_in_;

   }

   /**
            * The name of the scene node
            */
   public void setName(java.lang.String name)
   {
      name_.setLength(0);
      name_.append(name);
   }

   /**
            * The name of the scene node
            */
   public java.lang.String getNameAsString()
   {
      return getName().toString();
   }
   /**
            * The name of the scene node
            */
   public java.lang.StringBuilder getName()
   {
      return name_;
   }

   /**
            * Whether or not the node is currently detected
            */
   public void setCurrentlyDetected(boolean currently_detected)
   {
      currently_detected_ = currently_detected;
   }
   /**
            * Whether or not the node is currently detected
            */
   public boolean getCurrentlyDetected()
   {
      return currently_detected_;
   }


   /**
            * Transform of the node's frame to world frame
            */
   public controller_msgs.msg.dds.RigidBodyTransformMessage getTransformToWorld()
   {
      return transform_to_world_;
   }


   /**
            * If this node is trackable via an ArUco maker, this is the ArUco marker's
            * transform to world frame. This is so we can reset overriden node
            * poses back to ArUco relative ones.
            */
   public controller_msgs.msg.dds.RigidBodyTransformMessage getArucoMarkerTransformToWorld()
   {
      return aruco_marker_transform_to_world_;
   }

   /**
            * Keeps track of when the operator has overridden the pose of this node
            */
   public void setIsPoseOverriddenByOperator(boolean is_pose_overridden_by_operator)
   {
      is_pose_overridden_by_operator_ = is_pose_overridden_by_operator;
   }
   /**
            * Keeps track of when the operator has overridden the pose of this node
            */
   public boolean getIsPoseOverriddenByOperator()
   {
      return is_pose_overridden_by_operator_;
   }

   /**
            * For static ArUco relative nodes, if it's currently locked in
            */
   public void setIsStaticRelativePoseLockedIn(boolean is_static_relative_pose_locked_in)
   {
      is_static_relative_pose_locked_in_ = is_static_relative_pose_locked_in;
   }
   /**
            * For static ArUco relative nodes, if it's currently locked in
            */
   public boolean getIsStaticRelativePoseLockedIn()
   {
      return is_static_relative_pose_locked_in_;
   }


   public static Supplier<DetectableSceneNodeMessagePubSubType> getPubSubType()
   {
      return DetectableSceneNodeMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return DetectableSceneNodeMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(DetectableSceneNodeMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.name_, other.name_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.currently_detected_, other.currently_detected_, epsilon)) return false;

      if (!this.transform_to_world_.epsilonEquals(other.transform_to_world_, epsilon)) return false;
      if (!this.aruco_marker_transform_to_world_.epsilonEquals(other.aruco_marker_transform_to_world_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.is_pose_overridden_by_operator_, other.is_pose_overridden_by_operator_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.is_static_relative_pose_locked_in_, other.is_static_relative_pose_locked_in_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof DetectableSceneNodeMessage)) return false;

      DetectableSceneNodeMessage otherMyClass = (DetectableSceneNodeMessage) other;

      if (!us.ihmc.idl.IDLTools.equals(this.name_, otherMyClass.name_)) return false;

      if(this.currently_detected_ != otherMyClass.currently_detected_) return false;

      if (!this.transform_to_world_.equals(otherMyClass.transform_to_world_)) return false;
      if (!this.aruco_marker_transform_to_world_.equals(otherMyClass.aruco_marker_transform_to_world_)) return false;
      if(this.is_pose_overridden_by_operator_ != otherMyClass.is_pose_overridden_by_operator_) return false;

      if(this.is_static_relative_pose_locked_in_ != otherMyClass.is_static_relative_pose_locked_in_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("DetectableSceneNodeMessage {");
      builder.append("name=");
      builder.append(this.name_);      builder.append(", ");
      builder.append("currently_detected=");
      builder.append(this.currently_detected_);      builder.append(", ");
      builder.append("transform_to_world=");
      builder.append(this.transform_to_world_);      builder.append(", ");
      builder.append("aruco_marker_transform_to_world=");
      builder.append(this.aruco_marker_transform_to_world_);      builder.append(", ");
      builder.append("is_pose_overridden_by_operator=");
      builder.append(this.is_pose_overridden_by_operator_);      builder.append(", ");
      builder.append("is_static_relative_pose_locked_in=");
      builder.append(this.is_static_relative_pose_locked_in_);
      builder.append("}");
      return builder.toString();
   }
}
