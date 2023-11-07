package perception_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * Gives the current state of the complete collection of detectable scene nodes.
       * Publishing all scene nodes in one messages can simplify synchronization and
       * reduce the complexity of logic in figuring out when nodes are currently under
       * consideration.
       */
public class SceneGraphMessage extends Packet<SceneGraphMessage> implements Settable<SceneGraphMessage>, EpsilonComparable<SceneGraphMessage>
{
   public static final byte SCENE_NODE_TYPE = (byte) 0;
   public static final byte DETECTABLE_SCENE_NODE_TYPE = (byte) 1;
   public static final byte PREDEFINED_RIGID_BODY_NODE_TYPE = (byte) 2;
   public static final byte ARUCO_MARKER_NODE_TYPE = (byte) 3;
   public static final byte CENTERPOSE_NODE_TYPE = (byte) 4;
   public static final byte STATIC_RELATIVE_NODE_TYPE = (byte) 5;
   public static final byte PRIMITIVE_RIGID_BODY_NODE_TYPE = (byte) 6;
   /**
            * The ID to assign to the next instantiated node
            */
   public long next_id_;
   /**
            * A depth first ordered list of types.
            */
   public us.ihmc.idl.IDLSequence.Byte  scene_tree_types_;
   /**
            * A depth first ordered list of node indexes.
            * The index is of that node in it's respective list for
            * it's type.
            */
   public us.ihmc.idl.IDLSequence.Long  scene_tree_indices_;
   /**
            * Basic scene nodes
            */
   public us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.SceneNodeMessage>  scene_nodes_;
   /**
            * Detectable scene nodes
            */
   public us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.DetectableSceneNodeMessage>  detectable_scene_nodes_;
   /**
            * Predefined rigid body scene nodes
            */
   public us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.PredefinedRigidBodySceneNodeMessage>  predefined_rigid_body_scene_nodes_;
   /**
            * ArUco marker scene nodes
            */
   public us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.ArUcoMarkerNodeMessage>  aruco_marker_scene_nodes_;
   /**
            * Centerpose scene nodes
            */
   public us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.CenterposeNodeMessage>  centerpose_scene_nodes_;
   /**
            * Static relative scene nodes
            */
   public us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.StaticRelativeSceneNodeMessage>  static_relative_scene_nodes_;
   /**
            * Reshapable rigid body scene nodes
            */
   public us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.PrimitiveRigidBodySceneNodeMessage>  primitive_rigid_body_scene_nodes_;

   public SceneGraphMessage()
   {
      scene_tree_types_ = new us.ihmc.idl.IDLSequence.Byte (1000, "type_9");

      scene_tree_indices_ = new us.ihmc.idl.IDLSequence.Long (1000, "type_4");

      scene_nodes_ = new us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.SceneNodeMessage> (200, new perception_msgs.msg.dds.SceneNodeMessagePubSubType());
      detectable_scene_nodes_ = new us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.DetectableSceneNodeMessage> (200, new perception_msgs.msg.dds.DetectableSceneNodeMessagePubSubType());
      predefined_rigid_body_scene_nodes_ = new us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.PredefinedRigidBodySceneNodeMessage> (200, new perception_msgs.msg.dds.PredefinedRigidBodySceneNodeMessagePubSubType());
      aruco_marker_scene_nodes_ = new us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.ArUcoMarkerNodeMessage> (200, new perception_msgs.msg.dds.ArUcoMarkerNodeMessagePubSubType());
      centerpose_scene_nodes_ = new us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.CenterposeNodeMessage> (200, new perception_msgs.msg.dds.CenterposeNodeMessagePubSubType());
      static_relative_scene_nodes_ = new us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.StaticRelativeSceneNodeMessage> (200, new perception_msgs.msg.dds.StaticRelativeSceneNodeMessagePubSubType());
      primitive_rigid_body_scene_nodes_ = new us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.PrimitiveRigidBodySceneNodeMessage> (200, new perception_msgs.msg.dds.PrimitiveRigidBodySceneNodeMessagePubSubType());

   }

   public SceneGraphMessage(SceneGraphMessage other)
   {
      this();
      set(other);
   }

   public void set(SceneGraphMessage other)
   {
      next_id_ = other.next_id_;

      scene_tree_types_.set(other.scene_tree_types_);
      scene_tree_indices_.set(other.scene_tree_indices_);
      scene_nodes_.set(other.scene_nodes_);
      detectable_scene_nodes_.set(other.detectable_scene_nodes_);
      predefined_rigid_body_scene_nodes_.set(other.predefined_rigid_body_scene_nodes_);
      aruco_marker_scene_nodes_.set(other.aruco_marker_scene_nodes_);
      centerpose_scene_nodes_.set(other.centerpose_scene_nodes_);
      static_relative_scene_nodes_.set(other.static_relative_scene_nodes_);
      primitive_rigid_body_scene_nodes_.set(other.primitive_rigid_body_scene_nodes_);
   }

   /**
            * The ID to assign to the next instantiated node
            */
   public void setNextId(long next_id)
   {
      next_id_ = next_id;
   }
   /**
            * The ID to assign to the next instantiated node
            */
   public long getNextId()
   {
      return next_id_;
   }


   /**
            * A depth first ordered list of types.
            */
   public us.ihmc.idl.IDLSequence.Byte  getSceneTreeTypes()
   {
      return scene_tree_types_;
   }


   /**
            * A depth first ordered list of node indexes.
            * The index is of that node in it's respective list for
            * it's type.
            */
   public us.ihmc.idl.IDLSequence.Long  getSceneTreeIndices()
   {
      return scene_tree_indices_;
   }


   /**
            * Basic scene nodes
            */
   public us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.SceneNodeMessage>  getSceneNodes()
   {
      return scene_nodes_;
   }


   /**
            * Detectable scene nodes
            */
   public us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.DetectableSceneNodeMessage>  getDetectableSceneNodes()
   {
      return detectable_scene_nodes_;
   }


   /**
            * Predefined rigid body scene nodes
            */
   public us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.PredefinedRigidBodySceneNodeMessage>  getPredefinedRigidBodySceneNodes()
   {
      return predefined_rigid_body_scene_nodes_;
   }


   /**
            * ArUco marker scene nodes
            */
   public us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.ArUcoMarkerNodeMessage>  getArucoMarkerSceneNodes()
   {
      return aruco_marker_scene_nodes_;
   }


   /**
            * Centerpose scene nodes
            */
   public us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.CenterposeNodeMessage>  getCenterposeSceneNodes()
   {
      return centerpose_scene_nodes_;
   }


   /**
            * Static relative scene nodes
            */
   public us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.StaticRelativeSceneNodeMessage>  getStaticRelativeSceneNodes()
   {
      return static_relative_scene_nodes_;
   }


   /**
            * Reshapable rigid body scene nodes
            */
   public us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.PrimitiveRigidBodySceneNodeMessage>  getPrimitiveRigidBodySceneNodes()
   {
      return primitive_rigid_body_scene_nodes_;
   }


   public static Supplier<SceneGraphMessagePubSubType> getPubSubType()
   {
      return SceneGraphMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return SceneGraphMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(SceneGraphMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.next_id_, other.next_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsByteSequence(this.scene_tree_types_, other.scene_tree_types_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsLongSequence(this.scene_tree_indices_, other.scene_tree_indices_, epsilon)) return false;

      if (this.scene_nodes_.size() != other.scene_nodes_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.scene_nodes_.size(); i++)
         {  if (!this.scene_nodes_.get(i).epsilonEquals(other.scene_nodes_.get(i), epsilon)) return false; }
      }

      if (this.detectable_scene_nodes_.size() != other.detectable_scene_nodes_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.detectable_scene_nodes_.size(); i++)
         {  if (!this.detectable_scene_nodes_.get(i).epsilonEquals(other.detectable_scene_nodes_.get(i), epsilon)) return false; }
      }

      if (this.predefined_rigid_body_scene_nodes_.size() != other.predefined_rigid_body_scene_nodes_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.predefined_rigid_body_scene_nodes_.size(); i++)
         {  if (!this.predefined_rigid_body_scene_nodes_.get(i).epsilonEquals(other.predefined_rigid_body_scene_nodes_.get(i), epsilon)) return false; }
      }

      if (this.aruco_marker_scene_nodes_.size() != other.aruco_marker_scene_nodes_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.aruco_marker_scene_nodes_.size(); i++)
         {  if (!this.aruco_marker_scene_nodes_.get(i).epsilonEquals(other.aruco_marker_scene_nodes_.get(i), epsilon)) return false; }
      }

      if (this.centerpose_scene_nodes_.size() != other.centerpose_scene_nodes_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.centerpose_scene_nodes_.size(); i++)
         {  if (!this.centerpose_scene_nodes_.get(i).epsilonEquals(other.centerpose_scene_nodes_.get(i), epsilon)) return false; }
      }

      if (this.static_relative_scene_nodes_.size() != other.static_relative_scene_nodes_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.static_relative_scene_nodes_.size(); i++)
         {  if (!this.static_relative_scene_nodes_.get(i).epsilonEquals(other.static_relative_scene_nodes_.get(i), epsilon)) return false; }
      }

      if (this.primitive_rigid_body_scene_nodes_.size() != other.primitive_rigid_body_scene_nodes_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.primitive_rigid_body_scene_nodes_.size(); i++)
         {  if (!this.primitive_rigid_body_scene_nodes_.get(i).epsilonEquals(other.primitive_rigid_body_scene_nodes_.get(i), epsilon)) return false; }
      }


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof SceneGraphMessage)) return false;

      SceneGraphMessage otherMyClass = (SceneGraphMessage) other;

      if(this.next_id_ != otherMyClass.next_id_) return false;

      if (!this.scene_tree_types_.equals(otherMyClass.scene_tree_types_)) return false;
      if (!this.scene_tree_indices_.equals(otherMyClass.scene_tree_indices_)) return false;
      if (!this.scene_nodes_.equals(otherMyClass.scene_nodes_)) return false;
      if (!this.detectable_scene_nodes_.equals(otherMyClass.detectable_scene_nodes_)) return false;
      if (!this.predefined_rigid_body_scene_nodes_.equals(otherMyClass.predefined_rigid_body_scene_nodes_)) return false;
      if (!this.aruco_marker_scene_nodes_.equals(otherMyClass.aruco_marker_scene_nodes_)) return false;
      if (!this.centerpose_scene_nodes_.equals(otherMyClass.centerpose_scene_nodes_)) return false;
      if (!this.static_relative_scene_nodes_.equals(otherMyClass.static_relative_scene_nodes_)) return false;
      if (!this.primitive_rigid_body_scene_nodes_.equals(otherMyClass.primitive_rigid_body_scene_nodes_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("SceneGraphMessage {");
      builder.append("next_id=");
      builder.append(this.next_id_);      builder.append(", ");
      builder.append("scene_tree_types=");
      builder.append(this.scene_tree_types_);      builder.append(", ");
      builder.append("scene_tree_indices=");
      builder.append(this.scene_tree_indices_);      builder.append(", ");
      builder.append("scene_nodes=");
      builder.append(this.scene_nodes_);      builder.append(", ");
      builder.append("detectable_scene_nodes=");
      builder.append(this.detectable_scene_nodes_);      builder.append(", ");
      builder.append("predefined_rigid_body_scene_nodes=");
      builder.append(this.predefined_rigid_body_scene_nodes_);      builder.append(", ");
      builder.append("aruco_marker_scene_nodes=");
      builder.append(this.aruco_marker_scene_nodes_);      builder.append(", ");
      builder.append("centerpose_scene_nodes=");
      builder.append(this.centerpose_scene_nodes_);      builder.append(", ");
      builder.append("static_relative_scene_nodes=");
      builder.append(this.static_relative_scene_nodes_);      builder.append(", ");
      builder.append("primitive_rigid_body_scene_nodes=");
      builder.append(this.primitive_rigid_body_scene_nodes_);
      builder.append("}");
      return builder.toString();
   }
}
