package perception_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * A static relative heuristic scene node
       * The topic name identifies the node.
       */
public class StaticRelativeSceneNodeMessage extends Packet<StaticRelativeSceneNodeMessage> implements Settable<StaticRelativeSceneNodeMessage>, EpsilonComparable<StaticRelativeSceneNodeMessage>
{
   /**
            * The predefined rigid body information that this extends
            */
   public perception_msgs.msg.dds.PredefinedRigidBodySceneNodeMessage predefined_rigid_body_scene_node_;
   /**
            * Distance to robot to disable tracking.
            * Used for automatically disabling tracking for static relative objects.
            */
   public float distance_to_disable_tracking_;
   /**
            * Current distance to robot.
            * Used for automatically disabling tracking for static relative objects.
            */
   public float current_distance_to_robot_;

   public StaticRelativeSceneNodeMessage()
   {
      predefined_rigid_body_scene_node_ = new perception_msgs.msg.dds.PredefinedRigidBodySceneNodeMessage();
   }

   public StaticRelativeSceneNodeMessage(StaticRelativeSceneNodeMessage other)
   {
      this();
      set(other);
   }

   public void set(StaticRelativeSceneNodeMessage other)
   {
      perception_msgs.msg.dds.PredefinedRigidBodySceneNodeMessagePubSubType.staticCopy(other.predefined_rigid_body_scene_node_, predefined_rigid_body_scene_node_);
      distance_to_disable_tracking_ = other.distance_to_disable_tracking_;

      current_distance_to_robot_ = other.current_distance_to_robot_;

   }


   /**
            * The predefined rigid body information that this extends
            */
   public perception_msgs.msg.dds.PredefinedRigidBodySceneNodeMessage getPredefinedRigidBodySceneNode()
   {
      return predefined_rigid_body_scene_node_;
   }

   /**
            * Distance to robot to disable tracking.
            * Used for automatically disabling tracking for static relative objects.
            */
   public void setDistanceToDisableTracking(float distance_to_disable_tracking)
   {
      distance_to_disable_tracking_ = distance_to_disable_tracking;
   }
   /**
            * Distance to robot to disable tracking.
            * Used for automatically disabling tracking for static relative objects.
            */
   public float getDistanceToDisableTracking()
   {
      return distance_to_disable_tracking_;
   }

   /**
            * Current distance to robot.
            * Used for automatically disabling tracking for static relative objects.
            */
   public void setCurrentDistanceToRobot(float current_distance_to_robot)
   {
      current_distance_to_robot_ = current_distance_to_robot;
   }
   /**
            * Current distance to robot.
            * Used for automatically disabling tracking for static relative objects.
            */
   public float getCurrentDistanceToRobot()
   {
      return current_distance_to_robot_;
   }


   public static Supplier<StaticRelativeSceneNodeMessagePubSubType> getPubSubType()
   {
      return StaticRelativeSceneNodeMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return StaticRelativeSceneNodeMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(StaticRelativeSceneNodeMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.predefined_rigid_body_scene_node_.epsilonEquals(other.predefined_rigid_body_scene_node_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.distance_to_disable_tracking_, other.distance_to_disable_tracking_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.current_distance_to_robot_, other.current_distance_to_robot_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof StaticRelativeSceneNodeMessage)) return false;

      StaticRelativeSceneNodeMessage otherMyClass = (StaticRelativeSceneNodeMessage) other;

      if (!this.predefined_rigid_body_scene_node_.equals(otherMyClass.predefined_rigid_body_scene_node_)) return false;
      if(this.distance_to_disable_tracking_ != otherMyClass.distance_to_disable_tracking_) return false;

      if(this.current_distance_to_robot_ != otherMyClass.current_distance_to_robot_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("StaticRelativeSceneNodeMessage {");
      builder.append("predefined_rigid_body_scene_node=");
      builder.append(this.predefined_rigid_body_scene_node_);      builder.append(", ");
      builder.append("distance_to_disable_tracking=");
      builder.append(this.distance_to_disable_tracking_);      builder.append(", ");
      builder.append("current_distance_to_robot=");
      builder.append(this.current_distance_to_robot_);
      builder.append("}");
      return builder.toString();
   }
}
