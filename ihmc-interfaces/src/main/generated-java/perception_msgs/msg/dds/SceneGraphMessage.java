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
   /**
            * The ID to assign to the next instantiated node
            */
   public long next_id_;
   /**
            * A depth first ordered list of node indexes.
            * The indexes are the index of that node in it's
            * respective list for it's type.
            */
   public us.ihmc.idl.IDLSequence.Long  scene_tree_;
   /**
            * Basic scene nodes
            */
   public us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.SceneNodeMessage>  scene_nodes_;
   /**
            * Detectable scene nodes
            */
   public us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.DetectableSceneNodeMessage>  detectable_scene_nodes_;
   /**
            * ArUco marker scene nodes
            */
   public us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.ArUcoMarkerNodeMessage>  aruco_marker_scene_nodes_;
   /**
            * Static relative scene nodes
            */
   public us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.StaticRelativeSceneNodeMessage>  static_relative_scene_nodes_;

   public SceneGraphMessage()
   {
      scene_tree_ = new us.ihmc.idl.IDLSequence.Long (1000, "type_4");

      scene_nodes_ = new us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.SceneNodeMessage> (200, new perception_msgs.msg.dds.SceneNodeMessagePubSubType());
      detectable_scene_nodes_ = new us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.DetectableSceneNodeMessage> (200, new perception_msgs.msg.dds.DetectableSceneNodeMessagePubSubType());
      aruco_marker_scene_nodes_ = new us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.ArUcoMarkerNodeMessage> (200, new perception_msgs.msg.dds.ArUcoMarkerNodeMessagePubSubType());
      static_relative_scene_nodes_ = new us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.StaticRelativeSceneNodeMessage> (200, new perception_msgs.msg.dds.StaticRelativeSceneNodeMessagePubSubType());

   }

   public SceneGraphMessage(SceneGraphMessage other)
   {
      this();
      set(other);
   }

   public void set(SceneGraphMessage other)
   {
      next_id_ = other.next_id_;

      scene_tree_.set(other.scene_tree_);
      scene_nodes_.set(other.scene_nodes_);
      detectable_scene_nodes_.set(other.detectable_scene_nodes_);
      aruco_marker_scene_nodes_.set(other.aruco_marker_scene_nodes_);
      static_relative_scene_nodes_.set(other.static_relative_scene_nodes_);
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
            * A depth first ordered list of node indexes.
            * The indexes are the index of that node in it's
            * respective list for it's type.
            */
   public us.ihmc.idl.IDLSequence.Long  getSceneTree()
   {
      return scene_tree_;
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
            * ArUco marker scene nodes
            */
   public us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.ArUcoMarkerNodeMessage>  getArucoMarkerSceneNodes()
   {
      return aruco_marker_scene_nodes_;
   }


   /**
            * Static relative scene nodes
            */
   public us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.StaticRelativeSceneNodeMessage>  getStaticRelativeSceneNodes()
   {
      return static_relative_scene_nodes_;
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

      if (!us.ihmc.idl.IDLTools.epsilonEqualsLongSequence(this.scene_tree_, other.scene_tree_, epsilon)) return false;

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

      if (this.aruco_marker_scene_nodes_.size() != other.aruco_marker_scene_nodes_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.aruco_marker_scene_nodes_.size(); i++)
         {  if (!this.aruco_marker_scene_nodes_.get(i).epsilonEquals(other.aruco_marker_scene_nodes_.get(i), epsilon)) return false; }
      }

      if (this.static_relative_scene_nodes_.size() != other.static_relative_scene_nodes_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.static_relative_scene_nodes_.size(); i++)
         {  if (!this.static_relative_scene_nodes_.get(i).epsilonEquals(other.static_relative_scene_nodes_.get(i), epsilon)) return false; }
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

      if (!this.scene_tree_.equals(otherMyClass.scene_tree_)) return false;
      if (!this.scene_nodes_.equals(otherMyClass.scene_nodes_)) return false;
      if (!this.detectable_scene_nodes_.equals(otherMyClass.detectable_scene_nodes_)) return false;
      if (!this.aruco_marker_scene_nodes_.equals(otherMyClass.aruco_marker_scene_nodes_)) return false;
      if (!this.static_relative_scene_nodes_.equals(otherMyClass.static_relative_scene_nodes_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("SceneGraphMessage {");
      builder.append("next_id=");
      builder.append(this.next_id_);      builder.append(", ");
      builder.append("scene_tree=");
      builder.append(this.scene_tree_);      builder.append(", ");
      builder.append("scene_nodes=");
      builder.append(this.scene_nodes_);      builder.append(", ");
      builder.append("detectable_scene_nodes=");
      builder.append(this.detectable_scene_nodes_);      builder.append(", ");
      builder.append("aruco_marker_scene_nodes=");
      builder.append(this.aruco_marker_scene_nodes_);      builder.append(", ");
      builder.append("static_relative_scene_nodes=");
      builder.append(this.static_relative_scene_nodes_);
      builder.append("}");
      return builder.toString();
   }
}
