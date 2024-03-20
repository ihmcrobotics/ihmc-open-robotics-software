package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class FootstepPlanActionDefinitionMessage extends Packet<FootstepPlanActionDefinitionMessage> implements Settable<FootstepPlanActionDefinitionMessage>, EpsilonComparable<FootstepPlanActionDefinitionMessage>
{
   /**
            * Parent definition fields
            */
   public behavior_msgs.msg.dds.ActionNodeDefinitionMessage definition_;
   /**
            * Name of the parent frame the footsteps are expressed in
            */
   public java.lang.StringBuilder parent_frame_name_;
   /**
            * Swing duration
            */
   public double swing_duration_;
   /**
            * Transfer duration
            */
   public double transfer_duration_;
   /**
            * Is using the footstep planner or manually placing
            */
   public boolean is_manually_placed_;
   /**
            * The footsteps, with a maximum of 50
            */
   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.FootstepPlanActionFootstepDefinitionMessage>  footsteps_;
   /**
            * The position to which the goal stance is aligned
            */
   public us.ihmc.euclid.tuple3D.Point3D goal_stance_point_;
   /**
            * The point that the robot should be facing in the goal stance
            */
   public us.ihmc.euclid.tuple3D.Point3D goal_focal_point_;
   /**
            * Left goal foot X to the goal pose
            */
   public double left_goal_foot_x_to_gizmo_;
   /**
            * Left goal foot Y to the goal pose
            */
   public double left_goal_foot_y_to_gizmo_;
   /**
            * Left goal foot Yaw to the goal pose
            */
   public double left_goal_foot_yaw_to_gizmo_;
   /**
            * Right goal foot X to the goal pose
            */
   public double right_goal_foot_x_to_gizmo_;
   /**
            * Right goal foot Y to the goal pose
            */
   public double right_goal_foot_y_to_gizmo_;
   /**
            * Right goal foot Yaw to the goal pose
            */
   public double right_goal_foot_yaw_to_gizmo_;

   public FootstepPlanActionDefinitionMessage()
   {
      definition_ = new behavior_msgs.msg.dds.ActionNodeDefinitionMessage();
      parent_frame_name_ = new java.lang.StringBuilder(255);
      footsteps_ = new us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.FootstepPlanActionFootstepDefinitionMessage> (50, new behavior_msgs.msg.dds.FootstepPlanActionFootstepDefinitionMessagePubSubType());
      goal_stance_point_ = new us.ihmc.euclid.tuple3D.Point3D();
      goal_focal_point_ = new us.ihmc.euclid.tuple3D.Point3D();

   }

   public FootstepPlanActionDefinitionMessage(FootstepPlanActionDefinitionMessage other)
   {
      this();
      set(other);
   }

   public void set(FootstepPlanActionDefinitionMessage other)
   {
      behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType.staticCopy(other.definition_, definition_);
      parent_frame_name_.setLength(0);
      parent_frame_name_.append(other.parent_frame_name_);

      swing_duration_ = other.swing_duration_;

      transfer_duration_ = other.transfer_duration_;

      is_manually_placed_ = other.is_manually_placed_;

      footsteps_.set(other.footsteps_);
      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.goal_stance_point_, goal_stance_point_);
      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.goal_focal_point_, goal_focal_point_);
      left_goal_foot_x_to_gizmo_ = other.left_goal_foot_x_to_gizmo_;

      left_goal_foot_y_to_gizmo_ = other.left_goal_foot_y_to_gizmo_;

      left_goal_foot_yaw_to_gizmo_ = other.left_goal_foot_yaw_to_gizmo_;

      right_goal_foot_x_to_gizmo_ = other.right_goal_foot_x_to_gizmo_;

      right_goal_foot_y_to_gizmo_ = other.right_goal_foot_y_to_gizmo_;

      right_goal_foot_yaw_to_gizmo_ = other.right_goal_foot_yaw_to_gizmo_;

   }


   /**
            * Parent definition fields
            */
   public behavior_msgs.msg.dds.ActionNodeDefinitionMessage getDefinition()
   {
      return definition_;
   }

   /**
            * Name of the parent frame the footsteps are expressed in
            */
   public void setParentFrameName(java.lang.String parent_frame_name)
   {
      parent_frame_name_.setLength(0);
      parent_frame_name_.append(parent_frame_name);
   }

   /**
            * Name of the parent frame the footsteps are expressed in
            */
   public java.lang.String getParentFrameNameAsString()
   {
      return getParentFrameName().toString();
   }
   /**
            * Name of the parent frame the footsteps are expressed in
            */
   public java.lang.StringBuilder getParentFrameName()
   {
      return parent_frame_name_;
   }

   /**
            * Swing duration
            */
   public void setSwingDuration(double swing_duration)
   {
      swing_duration_ = swing_duration;
   }
   /**
            * Swing duration
            */
   public double getSwingDuration()
   {
      return swing_duration_;
   }

   /**
            * Transfer duration
            */
   public void setTransferDuration(double transfer_duration)
   {
      transfer_duration_ = transfer_duration;
   }
   /**
            * Transfer duration
            */
   public double getTransferDuration()
   {
      return transfer_duration_;
   }

   /**
            * Is using the footstep planner or manually placing
            */
   public void setIsManuallyPlaced(boolean is_manually_placed)
   {
      is_manually_placed_ = is_manually_placed;
   }
   /**
            * Is using the footstep planner or manually placing
            */
   public boolean getIsManuallyPlaced()
   {
      return is_manually_placed_;
   }


   /**
            * The footsteps, with a maximum of 50
            */
   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.FootstepPlanActionFootstepDefinitionMessage>  getFootsteps()
   {
      return footsteps_;
   }


   /**
            * The position to which the goal stance is aligned
            */
   public us.ihmc.euclid.tuple3D.Point3D getGoalStancePoint()
   {
      return goal_stance_point_;
   }


   /**
            * The point that the robot should be facing in the goal stance
            */
   public us.ihmc.euclid.tuple3D.Point3D getGoalFocalPoint()
   {
      return goal_focal_point_;
   }

   /**
            * Left goal foot X to the goal pose
            */
   public void setLeftGoalFootXToGizmo(double left_goal_foot_x_to_gizmo)
   {
      left_goal_foot_x_to_gizmo_ = left_goal_foot_x_to_gizmo;
   }
   /**
            * Left goal foot X to the goal pose
            */
   public double getLeftGoalFootXToGizmo()
   {
      return left_goal_foot_x_to_gizmo_;
   }

   /**
            * Left goal foot Y to the goal pose
            */
   public void setLeftGoalFootYToGizmo(double left_goal_foot_y_to_gizmo)
   {
      left_goal_foot_y_to_gizmo_ = left_goal_foot_y_to_gizmo;
   }
   /**
            * Left goal foot Y to the goal pose
            */
   public double getLeftGoalFootYToGizmo()
   {
      return left_goal_foot_y_to_gizmo_;
   }

   /**
            * Left goal foot Yaw to the goal pose
            */
   public void setLeftGoalFootYawToGizmo(double left_goal_foot_yaw_to_gizmo)
   {
      left_goal_foot_yaw_to_gizmo_ = left_goal_foot_yaw_to_gizmo;
   }
   /**
            * Left goal foot Yaw to the goal pose
            */
   public double getLeftGoalFootYawToGizmo()
   {
      return left_goal_foot_yaw_to_gizmo_;
   }

   /**
            * Right goal foot X to the goal pose
            */
   public void setRightGoalFootXToGizmo(double right_goal_foot_x_to_gizmo)
   {
      right_goal_foot_x_to_gizmo_ = right_goal_foot_x_to_gizmo;
   }
   /**
            * Right goal foot X to the goal pose
            */
   public double getRightGoalFootXToGizmo()
   {
      return right_goal_foot_x_to_gizmo_;
   }

   /**
            * Right goal foot Y to the goal pose
            */
   public void setRightGoalFootYToGizmo(double right_goal_foot_y_to_gizmo)
   {
      right_goal_foot_y_to_gizmo_ = right_goal_foot_y_to_gizmo;
   }
   /**
            * Right goal foot Y to the goal pose
            */
   public double getRightGoalFootYToGizmo()
   {
      return right_goal_foot_y_to_gizmo_;
   }

   /**
            * Right goal foot Yaw to the goal pose
            */
   public void setRightGoalFootYawToGizmo(double right_goal_foot_yaw_to_gizmo)
   {
      right_goal_foot_yaw_to_gizmo_ = right_goal_foot_yaw_to_gizmo;
   }
   /**
            * Right goal foot Yaw to the goal pose
            */
   public double getRightGoalFootYawToGizmo()
   {
      return right_goal_foot_yaw_to_gizmo_;
   }


   public static Supplier<FootstepPlanActionDefinitionMessagePubSubType> getPubSubType()
   {
      return FootstepPlanActionDefinitionMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return FootstepPlanActionDefinitionMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(FootstepPlanActionDefinitionMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.definition_.epsilonEquals(other.definition_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.parent_frame_name_, other.parent_frame_name_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.swing_duration_, other.swing_duration_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.transfer_duration_, other.transfer_duration_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.is_manually_placed_, other.is_manually_placed_, epsilon)) return false;

      if (this.footsteps_.size() != other.footsteps_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.footsteps_.size(); i++)
         {  if (!this.footsteps_.get(i).epsilonEquals(other.footsteps_.get(i), epsilon)) return false; }
      }

      if (!this.goal_stance_point_.epsilonEquals(other.goal_stance_point_, epsilon)) return false;
      if (!this.goal_focal_point_.epsilonEquals(other.goal_focal_point_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.left_goal_foot_x_to_gizmo_, other.left_goal_foot_x_to_gizmo_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.left_goal_foot_y_to_gizmo_, other.left_goal_foot_y_to_gizmo_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.left_goal_foot_yaw_to_gizmo_, other.left_goal_foot_yaw_to_gizmo_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.right_goal_foot_x_to_gizmo_, other.right_goal_foot_x_to_gizmo_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.right_goal_foot_y_to_gizmo_, other.right_goal_foot_y_to_gizmo_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.right_goal_foot_yaw_to_gizmo_, other.right_goal_foot_yaw_to_gizmo_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof FootstepPlanActionDefinitionMessage)) return false;

      FootstepPlanActionDefinitionMessage otherMyClass = (FootstepPlanActionDefinitionMessage) other;

      if (!this.definition_.equals(otherMyClass.definition_)) return false;
      if (!us.ihmc.idl.IDLTools.equals(this.parent_frame_name_, otherMyClass.parent_frame_name_)) return false;

      if(this.swing_duration_ != otherMyClass.swing_duration_) return false;

      if(this.transfer_duration_ != otherMyClass.transfer_duration_) return false;

      if(this.is_manually_placed_ != otherMyClass.is_manually_placed_) return false;

      if (!this.footsteps_.equals(otherMyClass.footsteps_)) return false;
      if (!this.goal_stance_point_.equals(otherMyClass.goal_stance_point_)) return false;
      if (!this.goal_focal_point_.equals(otherMyClass.goal_focal_point_)) return false;
      if(this.left_goal_foot_x_to_gizmo_ != otherMyClass.left_goal_foot_x_to_gizmo_) return false;

      if(this.left_goal_foot_y_to_gizmo_ != otherMyClass.left_goal_foot_y_to_gizmo_) return false;

      if(this.left_goal_foot_yaw_to_gizmo_ != otherMyClass.left_goal_foot_yaw_to_gizmo_) return false;

      if(this.right_goal_foot_x_to_gizmo_ != otherMyClass.right_goal_foot_x_to_gizmo_) return false;

      if(this.right_goal_foot_y_to_gizmo_ != otherMyClass.right_goal_foot_y_to_gizmo_) return false;

      if(this.right_goal_foot_yaw_to_gizmo_ != otherMyClass.right_goal_foot_yaw_to_gizmo_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("FootstepPlanActionDefinitionMessage {");
      builder.append("definition=");
      builder.append(this.definition_);      builder.append(", ");
      builder.append("parent_frame_name=");
      builder.append(this.parent_frame_name_);      builder.append(", ");
      builder.append("swing_duration=");
      builder.append(this.swing_duration_);      builder.append(", ");
      builder.append("transfer_duration=");
      builder.append(this.transfer_duration_);      builder.append(", ");
      builder.append("is_manually_placed=");
      builder.append(this.is_manually_placed_);      builder.append(", ");
      builder.append("footsteps=");
      builder.append(this.footsteps_);      builder.append(", ");
      builder.append("goal_stance_point=");
      builder.append(this.goal_stance_point_);      builder.append(", ");
      builder.append("goal_focal_point=");
      builder.append(this.goal_focal_point_);      builder.append(", ");
      builder.append("left_goal_foot_x_to_gizmo=");
      builder.append(this.left_goal_foot_x_to_gizmo_);      builder.append(", ");
      builder.append("left_goal_foot_y_to_gizmo=");
      builder.append(this.left_goal_foot_y_to_gizmo_);      builder.append(", ");
      builder.append("left_goal_foot_yaw_to_gizmo=");
      builder.append(this.left_goal_foot_yaw_to_gizmo_);      builder.append(", ");
      builder.append("right_goal_foot_x_to_gizmo=");
      builder.append(this.right_goal_foot_x_to_gizmo_);      builder.append(", ");
      builder.append("right_goal_foot_y_to_gizmo=");
      builder.append(this.right_goal_foot_y_to_gizmo_);      builder.append(", ");
      builder.append("right_goal_foot_yaw_to_gizmo=");
      builder.append(this.right_goal_foot_yaw_to_gizmo_);
      builder.append("}");
      return builder.toString();
   }
}
