package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class FootstepPlanActionStateMessage extends Packet<FootstepPlanActionStateMessage> implements Settable<FootstepPlanActionStateMessage>, EpsilonComparable<FootstepPlanActionStateMessage>
{
   public static final byte FOOTSTEP_PLANNING = (byte) 0;
   public static final byte PLANNING_FAILED = (byte) 1;
   public static final byte PLANNING_SUCCEEDED = (byte) 2;
   public static final byte PLAN_COMMANDED = (byte) 3;
   /**
            * Parent state fields
            */
   public behavior_msgs.msg.dds.ActionNodeStateMessage state_;
   /**
            * Definition
            */
   public behavior_msgs.msg.dds.FootstepPlanActionDefinitionMessage definition_;
   /**
            * The footsteps, with a maximum of 50
            */
   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.FootstepPlanActionFootstepStateMessage>  footsteps_;
   /**
            * Transform from the planning goal to the action's parent frame
            */
   public controller_msgs.msg.dds.RigidBodyTransformMessage goal_transform_to_parent_;
   /**
            * Execution state
            */
   public byte execution_state_;
   /**
            * Total number of footsteps; used for walking actions
            */
   public int total_number_of_footsteps_;
   /**
            * Incomplete footsteps; used for walking actions
            */
   public int number_of_incomplete_footsteps_;
   /**
            * Desired left footsteps
            */
   public us.ihmc.idl.IDLSequence.Object<ihmc_common_msgs.msg.dds.SE3TrajectoryPointMessage>  desired_left_footsteps_;
   /**
            * Desired right footsteps
            */
   public us.ihmc.idl.IDLSequence.Object<ihmc_common_msgs.msg.dds.SE3TrajectoryPointMessage>  desired_right_footsteps_;
   /**
            * Current left pose
            */
   public us.ihmc.euclid.geometry.Pose3D current_left_foot_pose_;
   /**
            * Current right pose
            */
   public us.ihmc.euclid.geometry.Pose3D current_right_foot_pose_;
   /**
            * Preview footstep plan, with a maximum of 50
            */
   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.FootstepPlanActionFootstepDefinitionMessage>  preview_footsteps_;

   public FootstepPlanActionStateMessage()
   {
      state_ = new behavior_msgs.msg.dds.ActionNodeStateMessage();
      definition_ = new behavior_msgs.msg.dds.FootstepPlanActionDefinitionMessage();
      footsteps_ = new us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.FootstepPlanActionFootstepStateMessage> (50, new behavior_msgs.msg.dds.FootstepPlanActionFootstepStateMessagePubSubType());
      goal_transform_to_parent_ = new controller_msgs.msg.dds.RigidBodyTransformMessage();
      desired_left_footsteps_ = new us.ihmc.idl.IDLSequence.Object<ihmc_common_msgs.msg.dds.SE3TrajectoryPointMessage> (50, new ihmc_common_msgs.msg.dds.SE3TrajectoryPointMessagePubSubType());
      desired_right_footsteps_ = new us.ihmc.idl.IDLSequence.Object<ihmc_common_msgs.msg.dds.SE3TrajectoryPointMessage> (50, new ihmc_common_msgs.msg.dds.SE3TrajectoryPointMessagePubSubType());
      current_left_foot_pose_ = new us.ihmc.euclid.geometry.Pose3D();
      current_right_foot_pose_ = new us.ihmc.euclid.geometry.Pose3D();
      preview_footsteps_ = new us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.FootstepPlanActionFootstepDefinitionMessage> (50, new behavior_msgs.msg.dds.FootstepPlanActionFootstepDefinitionMessagePubSubType());

   }

   public FootstepPlanActionStateMessage(FootstepPlanActionStateMessage other)
   {
      this();
      set(other);
   }

   public void set(FootstepPlanActionStateMessage other)
   {
      behavior_msgs.msg.dds.ActionNodeStateMessagePubSubType.staticCopy(other.state_, state_);
      behavior_msgs.msg.dds.FootstepPlanActionDefinitionMessagePubSubType.staticCopy(other.definition_, definition_);
      footsteps_.set(other.footsteps_);
      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.staticCopy(other.goal_transform_to_parent_, goal_transform_to_parent_);
      execution_state_ = other.execution_state_;

      total_number_of_footsteps_ = other.total_number_of_footsteps_;

      number_of_incomplete_footsteps_ = other.number_of_incomplete_footsteps_;

      desired_left_footsteps_.set(other.desired_left_footsteps_);
      desired_right_footsteps_.set(other.desired_right_footsteps_);
      geometry_msgs.msg.dds.PosePubSubType.staticCopy(other.current_left_foot_pose_, current_left_foot_pose_);
      geometry_msgs.msg.dds.PosePubSubType.staticCopy(other.current_right_foot_pose_, current_right_foot_pose_);
      preview_footsteps_.set(other.preview_footsteps_);
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
   public behavior_msgs.msg.dds.FootstepPlanActionDefinitionMessage getDefinition()
   {
      return definition_;
   }


   /**
            * The footsteps, with a maximum of 50
            */
   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.FootstepPlanActionFootstepStateMessage>  getFootsteps()
   {
      return footsteps_;
   }


   /**
            * Transform from the planning goal to the action's parent frame
            */
   public controller_msgs.msg.dds.RigidBodyTransformMessage getGoalTransformToParent()
   {
      return goal_transform_to_parent_;
   }

   /**
            * Execution state
            */
   public void setExecutionState(byte execution_state)
   {
      execution_state_ = execution_state;
   }
   /**
            * Execution state
            */
   public byte getExecutionState()
   {
      return execution_state_;
   }

   /**
            * Total number of footsteps; used for walking actions
            */
   public void setTotalNumberOfFootsteps(int total_number_of_footsteps)
   {
      total_number_of_footsteps_ = total_number_of_footsteps;
   }
   /**
            * Total number of footsteps; used for walking actions
            */
   public int getTotalNumberOfFootsteps()
   {
      return total_number_of_footsteps_;
   }

   /**
            * Incomplete footsteps; used for walking actions
            */
   public void setNumberOfIncompleteFootsteps(int number_of_incomplete_footsteps)
   {
      number_of_incomplete_footsteps_ = number_of_incomplete_footsteps;
   }
   /**
            * Incomplete footsteps; used for walking actions
            */
   public int getNumberOfIncompleteFootsteps()
   {
      return number_of_incomplete_footsteps_;
   }


   /**
            * Desired left footsteps
            */
   public us.ihmc.idl.IDLSequence.Object<ihmc_common_msgs.msg.dds.SE3TrajectoryPointMessage>  getDesiredLeftFootsteps()
   {
      return desired_left_footsteps_;
   }


   /**
            * Desired right footsteps
            */
   public us.ihmc.idl.IDLSequence.Object<ihmc_common_msgs.msg.dds.SE3TrajectoryPointMessage>  getDesiredRightFootsteps()
   {
      return desired_right_footsteps_;
   }


   /**
            * Current left pose
            */
   public us.ihmc.euclid.geometry.Pose3D getCurrentLeftFootPose()
   {
      return current_left_foot_pose_;
   }


   /**
            * Current right pose
            */
   public us.ihmc.euclid.geometry.Pose3D getCurrentRightFootPose()
   {
      return current_right_foot_pose_;
   }


   /**
            * Preview footstep plan, with a maximum of 50
            */
   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.FootstepPlanActionFootstepDefinitionMessage>  getPreviewFootsteps()
   {
      return preview_footsteps_;
   }


   public static Supplier<FootstepPlanActionStateMessagePubSubType> getPubSubType()
   {
      return FootstepPlanActionStateMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return FootstepPlanActionStateMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(FootstepPlanActionStateMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.state_.epsilonEquals(other.state_, epsilon)) return false;
      if (!this.definition_.epsilonEquals(other.definition_, epsilon)) return false;
      if (this.footsteps_.size() != other.footsteps_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.footsteps_.size(); i++)
         {  if (!this.footsteps_.get(i).epsilonEquals(other.footsteps_.get(i), epsilon)) return false; }
      }

      if (!this.goal_transform_to_parent_.epsilonEquals(other.goal_transform_to_parent_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.execution_state_, other.execution_state_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.total_number_of_footsteps_, other.total_number_of_footsteps_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.number_of_incomplete_footsteps_, other.number_of_incomplete_footsteps_, epsilon)) return false;

      if (this.desired_left_footsteps_.size() != other.desired_left_footsteps_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.desired_left_footsteps_.size(); i++)
         {  if (!this.desired_left_footsteps_.get(i).epsilonEquals(other.desired_left_footsteps_.get(i), epsilon)) return false; }
      }

      if (this.desired_right_footsteps_.size() != other.desired_right_footsteps_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.desired_right_footsteps_.size(); i++)
         {  if (!this.desired_right_footsteps_.get(i).epsilonEquals(other.desired_right_footsteps_.get(i), epsilon)) return false; }
      }

      if (!this.current_left_foot_pose_.epsilonEquals(other.current_left_foot_pose_, epsilon)) return false;
      if (!this.current_right_foot_pose_.epsilonEquals(other.current_right_foot_pose_, epsilon)) return false;
      if (this.preview_footsteps_.size() != other.preview_footsteps_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.preview_footsteps_.size(); i++)
         {  if (!this.preview_footsteps_.get(i).epsilonEquals(other.preview_footsteps_.get(i), epsilon)) return false; }
      }


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof FootstepPlanActionStateMessage)) return false;

      FootstepPlanActionStateMessage otherMyClass = (FootstepPlanActionStateMessage) other;

      if (!this.state_.equals(otherMyClass.state_)) return false;
      if (!this.definition_.equals(otherMyClass.definition_)) return false;
      if (!this.footsteps_.equals(otherMyClass.footsteps_)) return false;
      if (!this.goal_transform_to_parent_.equals(otherMyClass.goal_transform_to_parent_)) return false;
      if(this.execution_state_ != otherMyClass.execution_state_) return false;

      if(this.total_number_of_footsteps_ != otherMyClass.total_number_of_footsteps_) return false;

      if(this.number_of_incomplete_footsteps_ != otherMyClass.number_of_incomplete_footsteps_) return false;

      if (!this.desired_left_footsteps_.equals(otherMyClass.desired_left_footsteps_)) return false;
      if (!this.desired_right_footsteps_.equals(otherMyClass.desired_right_footsteps_)) return false;
      if (!this.current_left_foot_pose_.equals(otherMyClass.current_left_foot_pose_)) return false;
      if (!this.current_right_foot_pose_.equals(otherMyClass.current_right_foot_pose_)) return false;
      if (!this.preview_footsteps_.equals(otherMyClass.preview_footsteps_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("FootstepPlanActionStateMessage {");
      builder.append("state=");
      builder.append(this.state_);      builder.append(", ");
      builder.append("definition=");
      builder.append(this.definition_);      builder.append(", ");
      builder.append("footsteps=");
      builder.append(this.footsteps_);      builder.append(", ");
      builder.append("goal_transform_to_parent=");
      builder.append(this.goal_transform_to_parent_);      builder.append(", ");
      builder.append("execution_state=");
      builder.append(this.execution_state_);      builder.append(", ");
      builder.append("total_number_of_footsteps=");
      builder.append(this.total_number_of_footsteps_);      builder.append(", ");
      builder.append("number_of_incomplete_footsteps=");
      builder.append(this.number_of_incomplete_footsteps_);      builder.append(", ");
      builder.append("desired_left_footsteps=");
      builder.append(this.desired_left_footsteps_);      builder.append(", ");
      builder.append("desired_right_footsteps=");
      builder.append(this.desired_right_footsteps_);      builder.append(", ");
      builder.append("current_left_foot_pose=");
      builder.append(this.current_left_foot_pose_);      builder.append(", ");
      builder.append("current_right_foot_pose=");
      builder.append(this.current_right_foot_pose_);      builder.append(", ");
      builder.append("preview_footsteps=");
      builder.append(this.preview_footsteps_);
      builder.append("}");
      return builder.toString();
   }
}
