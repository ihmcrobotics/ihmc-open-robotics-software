package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class WalkActionDefinitionMessage extends Packet<WalkActionDefinitionMessage> implements Settable<WalkActionDefinitionMessage>, EpsilonComparable<WalkActionDefinitionMessage>
{
   /**
            * Parent definition fields
            */
   public behavior_msgs.msg.dds.ActionNodeDefinitionMessage definition_;
   public behavior_msgs.msg.dds.FootstepPlanActionDefinitionBasicsMessage definition_basics_;
   /**
            * Transform that expresses the walk gizmo pose in the parent frame
            */
   public controller_msgs.msg.dds.RigidBodyTransformMessage transform_to_parent_;
   /**
            * Left goal foot transform to the walk gizmo
            */
   public controller_msgs.msg.dds.RigidBodyTransformMessage left_goal_foot_transform_to_gizmo_;
   /**
            * Right goal foot transform to the walk gizmo
            */
   public controller_msgs.msg.dds.RigidBodyTransformMessage right_goal_foot_transform_to_gizmo_;

   public WalkActionDefinitionMessage()
   {
      definition_ = new behavior_msgs.msg.dds.ActionNodeDefinitionMessage();
      definition_basics_ = new behavior_msgs.msg.dds.FootstepPlanActionDefinitionBasicsMessage();
      transform_to_parent_ = new controller_msgs.msg.dds.RigidBodyTransformMessage();
      left_goal_foot_transform_to_gizmo_ = new controller_msgs.msg.dds.RigidBodyTransformMessage();
      right_goal_foot_transform_to_gizmo_ = new controller_msgs.msg.dds.RigidBodyTransformMessage();
   }

   public WalkActionDefinitionMessage(WalkActionDefinitionMessage other)
   {
      this();
      set(other);
   }

   public void set(WalkActionDefinitionMessage other)
   {
      behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType.staticCopy(other.definition_, definition_);
      behavior_msgs.msg.dds.FootstepPlanActionDefinitionBasicsMessagePubSubType.staticCopy(other.definition_basics_, definition_basics_);
      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.staticCopy(other.transform_to_parent_, transform_to_parent_);
      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.staticCopy(other.left_goal_foot_transform_to_gizmo_, left_goal_foot_transform_to_gizmo_);
      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.staticCopy(other.right_goal_foot_transform_to_gizmo_, right_goal_foot_transform_to_gizmo_);
   }


   /**
            * Parent definition fields
            */
   public behavior_msgs.msg.dds.ActionNodeDefinitionMessage getDefinition()
   {
      return definition_;
   }


   public behavior_msgs.msg.dds.FootstepPlanActionDefinitionBasicsMessage getDefinitionBasics()
   {
      return definition_basics_;
   }


   /**
            * Transform that expresses the walk gizmo pose in the parent frame
            */
   public controller_msgs.msg.dds.RigidBodyTransformMessage getTransformToParent()
   {
      return transform_to_parent_;
   }


   /**
            * Left goal foot transform to the walk gizmo
            */
   public controller_msgs.msg.dds.RigidBodyTransformMessage getLeftGoalFootTransformToGizmo()
   {
      return left_goal_foot_transform_to_gizmo_;
   }


   /**
            * Right goal foot transform to the walk gizmo
            */
   public controller_msgs.msg.dds.RigidBodyTransformMessage getRightGoalFootTransformToGizmo()
   {
      return right_goal_foot_transform_to_gizmo_;
   }


   public static Supplier<WalkActionDefinitionMessagePubSubType> getPubSubType()
   {
      return WalkActionDefinitionMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return WalkActionDefinitionMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(WalkActionDefinitionMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.definition_.epsilonEquals(other.definition_, epsilon)) return false;
      if (!this.definition_basics_.epsilonEquals(other.definition_basics_, epsilon)) return false;
      if (!this.transform_to_parent_.epsilonEquals(other.transform_to_parent_, epsilon)) return false;
      if (!this.left_goal_foot_transform_to_gizmo_.epsilonEquals(other.left_goal_foot_transform_to_gizmo_, epsilon)) return false;
      if (!this.right_goal_foot_transform_to_gizmo_.epsilonEquals(other.right_goal_foot_transform_to_gizmo_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof WalkActionDefinitionMessage)) return false;

      WalkActionDefinitionMessage otherMyClass = (WalkActionDefinitionMessage) other;

      if (!this.definition_.equals(otherMyClass.definition_)) return false;
      if (!this.definition_basics_.equals(otherMyClass.definition_basics_)) return false;
      if (!this.transform_to_parent_.equals(otherMyClass.transform_to_parent_)) return false;
      if (!this.left_goal_foot_transform_to_gizmo_.equals(otherMyClass.left_goal_foot_transform_to_gizmo_)) return false;
      if (!this.right_goal_foot_transform_to_gizmo_.equals(otherMyClass.right_goal_foot_transform_to_gizmo_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("WalkActionDefinitionMessage {");
      builder.append("definition=");
      builder.append(this.definition_);      builder.append(", ");
      builder.append("definition_basics=");
      builder.append(this.definition_basics_);      builder.append(", ");
      builder.append("transform_to_parent=");
      builder.append(this.transform_to_parent_);      builder.append(", ");
      builder.append("left_goal_foot_transform_to_gizmo=");
      builder.append(this.left_goal_foot_transform_to_gizmo_);      builder.append(", ");
      builder.append("right_goal_foot_transform_to_gizmo=");
      builder.append(this.right_goal_foot_transform_to_gizmo_);
      builder.append("}");
      return builder.toString();
   }
}
