package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class SpineActionStateMessage extends Packet<SpineActionStateMessage> implements Settable<SpineActionStateMessage>, EpsilonComparable<SpineActionStateMessage>
{
   /**
            * Parent state fields
            */
   public behavior_msgs.msg.dds.ActionNodeStateMessage state_;
   /**
            * Definition
            */
   public behavior_msgs.msg.dds.SpineActionDefinitionMessage definition_;
   /**
            * This is the estimated goal pelvis frame as the robot executes a potential whole body action.
            */
   public controller_msgs.msg.dds.RigidBodyTransformMessage goal_pelvis_transform_to_world_;

   public SpineActionStateMessage()
   {
      state_ = new behavior_msgs.msg.dds.ActionNodeStateMessage();
      definition_ = new behavior_msgs.msg.dds.SpineActionDefinitionMessage();
      goal_pelvis_transform_to_world_ = new controller_msgs.msg.dds.RigidBodyTransformMessage();
   }

   public SpineActionStateMessage(SpineActionStateMessage other)
   {
      this();
      set(other);
   }

   public void set(SpineActionStateMessage other)
   {
      behavior_msgs.msg.dds.ActionNodeStateMessagePubSubType.staticCopy(other.state_, state_);
      behavior_msgs.msg.dds.SpineActionDefinitionMessagePubSubType.staticCopy(other.definition_, definition_);
      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.staticCopy(other.goal_pelvis_transform_to_world_, goal_pelvis_transform_to_world_);
   }


   /**
            * Parent state fields
            */
   public behavior_msgs.msg.dds.ActionNodeStateMessage getState()
   {
      return state_;
   }


   /**
            * Definition
            */
   public behavior_msgs.msg.dds.SpineActionDefinitionMessage getDefinition()
   {
      return definition_;
   }


   /**
            * This is the estimated goal pelvis frame as the robot executes a potential whole body action.
            */
   public controller_msgs.msg.dds.RigidBodyTransformMessage getGoalPelvisTransformToWorld()
   {
      return goal_pelvis_transform_to_world_;
   }


   public static Supplier<SpineActionStateMessagePubSubType> getPubSubType()
   {
      return SpineActionStateMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return SpineActionStateMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(SpineActionStateMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.state_.epsilonEquals(other.state_, epsilon)) return false;
      if (!this.definition_.epsilonEquals(other.definition_, epsilon)) return false;
      if (!this.goal_pelvis_transform_to_world_.epsilonEquals(other.goal_pelvis_transform_to_world_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof SpineActionStateMessage)) return false;

      SpineActionStateMessage otherMyClass = (SpineActionStateMessage) other;

      if (!this.state_.equals(otherMyClass.state_)) return false;
      if (!this.definition_.equals(otherMyClass.definition_)) return false;
      if (!this.goal_pelvis_transform_to_world_.equals(otherMyClass.goal_pelvis_transform_to_world_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("SpineActionStateMessage {");
      builder.append("state=");
      builder.append(this.state_);      builder.append(", ");
      builder.append("definition=");
      builder.append(this.definition_);      builder.append(", ");
      builder.append("goal_pelvis_transform_to_world=");
      builder.append(this.goal_pelvis_transform_to_world_);
      builder.append("}");
      return builder.toString();
   }
}
