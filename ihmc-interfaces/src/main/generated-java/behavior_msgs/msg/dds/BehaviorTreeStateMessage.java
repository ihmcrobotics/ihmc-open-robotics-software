package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * Gives the current state of the complete collection of behavior tree nodes.
       * Publishing all behavior tree nodes in one message can simplify synchronization and
       * reduce the complexity of logic in figuring out when nodes are currently under
       * consideration.
       */
public class BehaviorTreeStateMessage extends Packet<BehaviorTreeStateMessage> implements Settable<BehaviorTreeStateMessage>, EpsilonComparable<BehaviorTreeStateMessage>
{
   public static final byte BASIC_NODE = (byte) 0;
   public static final byte ACTION_SEQUENCE = (byte) 1;
   public static final byte DOOR_TRAVERSAL = (byte) 2;
   public static final byte ROOM_EXPLORATION = (byte) 3;
   public static final byte CHEST_ORIENTATION_ACTION = (byte) 10;
   public static final byte FOOTSTEP_PLAN_ACTION = (byte) 11;
   public static final byte SAKE_HAND_COMMAND_ACTION = (byte) 12;
   public static final byte HAND_POSE_ACTION = (byte) 13;
   public static final byte HAND_WRENCH_ACTION = (byte) 14;
   public static final byte SCREW_PRIMITIVE_ACTION = (byte) 15;
   public static final byte PELVIS_HEIGHT_PITCH_ACTION = (byte) 16;
   public static final byte WAIT_DURATION_ACTION = (byte) 17;
   /**
            * Monotonically increasing message ID that matches the CRDTInfo update number
            */
   public long sequence_id_;
   /**
            * The ID to assign to the next instantiated node
            */
   public long next_id_;
   /**
            * A mechanism for confirming and ending a freeze early
            */
   public ihmc_common_msgs.msg.dds.ConfirmableRequestMessage confirmable_request_;
   /**
            * A depth first ordered list of types.
            */
   public us.ihmc.idl.IDLSequence.Byte  behavior_tree_types_;
   /**
            * A depth first ordered list of node indexes.
            * The index is of that node in it's respective list for
            * it's type.
            */
   public us.ihmc.idl.IDLSequence.Long  behavior_tree_indices_;
   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.BasicNodeStateMessage>  basic_nodes_;
   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.ActionSequenceStateMessage>  action_sequences_;
   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.DoorTraversalStateMessage>  door_traversals_;
   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.RoomExplorationStateMessage>  room_explorations_;
   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.ChestOrientationActionStateMessage>  chest_orientation_actions_;
   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.FootstepPlanActionStateMessage>  footstep_plan_actions_;
   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.SakeHandCommandActionStateMessage>  sake_hand_command_actions_;
   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.HandPoseActionStateMessage>  hand_pose_actions_;
   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.HandWrenchActionStateMessage>  hand_wrench_actions_;
   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.ScrewPrimitiveActionStateMessage>  screw_primitive_actions_;
   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.PelvisHeightPitchActionStateMessage>  pelvis_height_actions_;
   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.WaitDurationActionStateMessage>  wait_duration_actions_;

   public BehaviorTreeStateMessage()
   {
      confirmable_request_ = new ihmc_common_msgs.msg.dds.ConfirmableRequestMessage();
      behavior_tree_types_ = new us.ihmc.idl.IDLSequence.Byte (1000, "type_9");

      behavior_tree_indices_ = new us.ihmc.idl.IDLSequence.Long (1000, "type_4");

      basic_nodes_ = new us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.BasicNodeStateMessage> (200, new behavior_msgs.msg.dds.BasicNodeStateMessagePubSubType());
      action_sequences_ = new us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.ActionSequenceStateMessage> (200, new behavior_msgs.msg.dds.ActionSequenceStateMessagePubSubType());
      door_traversals_ = new us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.DoorTraversalStateMessage> (200, new behavior_msgs.msg.dds.DoorTraversalStateMessagePubSubType());
      room_explorations_ = new us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.RoomExplorationStateMessage> (200, new behavior_msgs.msg.dds.RoomExplorationStateMessagePubSubType());
      chest_orientation_actions_ = new us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.ChestOrientationActionStateMessage> (200, new behavior_msgs.msg.dds.ChestOrientationActionStateMessagePubSubType());
      footstep_plan_actions_ = new us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.FootstepPlanActionStateMessage> (200, new behavior_msgs.msg.dds.FootstepPlanActionStateMessagePubSubType());
      sake_hand_command_actions_ = new us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.SakeHandCommandActionStateMessage> (200, new behavior_msgs.msg.dds.SakeHandCommandActionStateMessagePubSubType());
      hand_pose_actions_ = new us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.HandPoseActionStateMessage> (200, new behavior_msgs.msg.dds.HandPoseActionStateMessagePubSubType());
      hand_wrench_actions_ = new us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.HandWrenchActionStateMessage> (200, new behavior_msgs.msg.dds.HandWrenchActionStateMessagePubSubType());
      screw_primitive_actions_ = new us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.ScrewPrimitiveActionStateMessage> (200, new behavior_msgs.msg.dds.ScrewPrimitiveActionStateMessagePubSubType());
      pelvis_height_actions_ = new us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.PelvisHeightPitchActionStateMessage> (200, new behavior_msgs.msg.dds.PelvisHeightPitchActionStateMessagePubSubType());
      wait_duration_actions_ = new us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.WaitDurationActionStateMessage> (200, new behavior_msgs.msg.dds.WaitDurationActionStateMessagePubSubType());

   }

   public BehaviorTreeStateMessage(BehaviorTreeStateMessage other)
   {
      this();
      set(other);
   }

   public void set(BehaviorTreeStateMessage other)
   {
      sequence_id_ = other.sequence_id_;

      next_id_ = other.next_id_;

      ihmc_common_msgs.msg.dds.ConfirmableRequestMessagePubSubType.staticCopy(other.confirmable_request_, confirmable_request_);
      behavior_tree_types_.set(other.behavior_tree_types_);
      behavior_tree_indices_.set(other.behavior_tree_indices_);
      basic_nodes_.set(other.basic_nodes_);
      action_sequences_.set(other.action_sequences_);
      door_traversals_.set(other.door_traversals_);
      room_explorations_.set(other.room_explorations_);
      chest_orientation_actions_.set(other.chest_orientation_actions_);
      footstep_plan_actions_.set(other.footstep_plan_actions_);
      sake_hand_command_actions_.set(other.sake_hand_command_actions_);
      hand_pose_actions_.set(other.hand_pose_actions_);
      hand_wrench_actions_.set(other.hand_wrench_actions_);
      screw_primitive_actions_.set(other.screw_primitive_actions_);
      pelvis_height_actions_.set(other.pelvis_height_actions_);
      wait_duration_actions_.set(other.wait_duration_actions_);
   }

   /**
            * Monotonically increasing message ID that matches the CRDTInfo update number
            */
   public void setSequenceId(long sequence_id)
   {
      sequence_id_ = sequence_id;
   }
   /**
            * Monotonically increasing message ID that matches the CRDTInfo update number
            */
   public long getSequenceId()
   {
      return sequence_id_;
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
            * A mechanism for confirming and ending a freeze early
            */
   public ihmc_common_msgs.msg.dds.ConfirmableRequestMessage getConfirmableRequest()
   {
      return confirmable_request_;
   }


   /**
            * A depth first ordered list of types.
            */
   public us.ihmc.idl.IDLSequence.Byte  getBehaviorTreeTypes()
   {
      return behavior_tree_types_;
   }


   /**
            * A depth first ordered list of node indexes.
            * The index is of that node in it's respective list for
            * it's type.
            */
   public us.ihmc.idl.IDLSequence.Long  getBehaviorTreeIndices()
   {
      return behavior_tree_indices_;
   }


   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.BasicNodeStateMessage>  getBasicNodes()
   {
      return basic_nodes_;
   }


   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.ActionSequenceStateMessage>  getActionSequences()
   {
      return action_sequences_;
   }


   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.DoorTraversalStateMessage>  getDoorTraversals()
   {
      return door_traversals_;
   }


   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.RoomExplorationStateMessage>  getRoomExplorations()
   {
      return room_explorations_;
   }


   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.ChestOrientationActionStateMessage>  getChestOrientationActions()
   {
      return chest_orientation_actions_;
   }


   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.FootstepPlanActionStateMessage>  getFootstepPlanActions()
   {
      return footstep_plan_actions_;
   }


   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.SakeHandCommandActionStateMessage>  getSakeHandCommandActions()
   {
      return sake_hand_command_actions_;
   }


   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.HandPoseActionStateMessage>  getHandPoseActions()
   {
      return hand_pose_actions_;
   }


   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.HandWrenchActionStateMessage>  getHandWrenchActions()
   {
      return hand_wrench_actions_;
   }


   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.ScrewPrimitiveActionStateMessage>  getScrewPrimitiveActions()
   {
      return screw_primitive_actions_;
   }


   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.PelvisHeightPitchActionStateMessage>  getPelvisHeightActions()
   {
      return pelvis_height_actions_;
   }


   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.WaitDurationActionStateMessage>  getWaitDurationActions()
   {
      return wait_duration_actions_;
   }


   public static Supplier<BehaviorTreeStateMessagePubSubType> getPubSubType()
   {
      return BehaviorTreeStateMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return BehaviorTreeStateMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(BehaviorTreeStateMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.next_id_, other.next_id_, epsilon)) return false;

      if (!this.confirmable_request_.epsilonEquals(other.confirmable_request_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsByteSequence(this.behavior_tree_types_, other.behavior_tree_types_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsLongSequence(this.behavior_tree_indices_, other.behavior_tree_indices_, epsilon)) return false;

      if (this.basic_nodes_.size() != other.basic_nodes_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.basic_nodes_.size(); i++)
         {  if (!this.basic_nodes_.get(i).epsilonEquals(other.basic_nodes_.get(i), epsilon)) return false; }
      }

      if (this.action_sequences_.size() != other.action_sequences_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.action_sequences_.size(); i++)
         {  if (!this.action_sequences_.get(i).epsilonEquals(other.action_sequences_.get(i), epsilon)) return false; }
      }

      if (this.door_traversals_.size() != other.door_traversals_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.door_traversals_.size(); i++)
         {  if (!this.door_traversals_.get(i).epsilonEquals(other.door_traversals_.get(i), epsilon)) return false; }
      }

      if (this.room_explorations_.size() != other.room_explorations_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.room_explorations_.size(); i++)
         {  if (!this.room_explorations_.get(i).epsilonEquals(other.room_explorations_.get(i), epsilon)) return false; }
      }

      if (this.chest_orientation_actions_.size() != other.chest_orientation_actions_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.chest_orientation_actions_.size(); i++)
         {  if (!this.chest_orientation_actions_.get(i).epsilonEquals(other.chest_orientation_actions_.get(i), epsilon)) return false; }
      }

      if (this.footstep_plan_actions_.size() != other.footstep_plan_actions_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.footstep_plan_actions_.size(); i++)
         {  if (!this.footstep_plan_actions_.get(i).epsilonEquals(other.footstep_plan_actions_.get(i), epsilon)) return false; }
      }

      if (this.sake_hand_command_actions_.size() != other.sake_hand_command_actions_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.sake_hand_command_actions_.size(); i++)
         {  if (!this.sake_hand_command_actions_.get(i).epsilonEquals(other.sake_hand_command_actions_.get(i), epsilon)) return false; }
      }

      if (this.hand_pose_actions_.size() != other.hand_pose_actions_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.hand_pose_actions_.size(); i++)
         {  if (!this.hand_pose_actions_.get(i).epsilonEquals(other.hand_pose_actions_.get(i), epsilon)) return false; }
      }

      if (this.hand_wrench_actions_.size() != other.hand_wrench_actions_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.hand_wrench_actions_.size(); i++)
         {  if (!this.hand_wrench_actions_.get(i).epsilonEquals(other.hand_wrench_actions_.get(i), epsilon)) return false; }
      }

      if (this.screw_primitive_actions_.size() != other.screw_primitive_actions_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.screw_primitive_actions_.size(); i++)
         {  if (!this.screw_primitive_actions_.get(i).epsilonEquals(other.screw_primitive_actions_.get(i), epsilon)) return false; }
      }

      if (this.pelvis_height_actions_.size() != other.pelvis_height_actions_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.pelvis_height_actions_.size(); i++)
         {  if (!this.pelvis_height_actions_.get(i).epsilonEquals(other.pelvis_height_actions_.get(i), epsilon)) return false; }
      }

      if (this.wait_duration_actions_.size() != other.wait_duration_actions_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.wait_duration_actions_.size(); i++)
         {  if (!this.wait_duration_actions_.get(i).epsilonEquals(other.wait_duration_actions_.get(i), epsilon)) return false; }
      }


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof BehaviorTreeStateMessage)) return false;

      BehaviorTreeStateMessage otherMyClass = (BehaviorTreeStateMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.next_id_ != otherMyClass.next_id_) return false;

      if (!this.confirmable_request_.equals(otherMyClass.confirmable_request_)) return false;
      if (!this.behavior_tree_types_.equals(otherMyClass.behavior_tree_types_)) return false;
      if (!this.behavior_tree_indices_.equals(otherMyClass.behavior_tree_indices_)) return false;
      if (!this.basic_nodes_.equals(otherMyClass.basic_nodes_)) return false;
      if (!this.action_sequences_.equals(otherMyClass.action_sequences_)) return false;
      if (!this.door_traversals_.equals(otherMyClass.door_traversals_)) return false;
      if (!this.room_explorations_.equals(otherMyClass.room_explorations_)) return false;
      if (!this.chest_orientation_actions_.equals(otherMyClass.chest_orientation_actions_)) return false;
      if (!this.footstep_plan_actions_.equals(otherMyClass.footstep_plan_actions_)) return false;
      if (!this.sake_hand_command_actions_.equals(otherMyClass.sake_hand_command_actions_)) return false;
      if (!this.hand_pose_actions_.equals(otherMyClass.hand_pose_actions_)) return false;
      if (!this.hand_wrench_actions_.equals(otherMyClass.hand_wrench_actions_)) return false;
      if (!this.screw_primitive_actions_.equals(otherMyClass.screw_primitive_actions_)) return false;
      if (!this.pelvis_height_actions_.equals(otherMyClass.pelvis_height_actions_)) return false;
      if (!this.wait_duration_actions_.equals(otherMyClass.wait_duration_actions_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("BehaviorTreeStateMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("next_id=");
      builder.append(this.next_id_);      builder.append(", ");
      builder.append("confirmable_request=");
      builder.append(this.confirmable_request_);      builder.append(", ");
      builder.append("behavior_tree_types=");
      builder.append(this.behavior_tree_types_);      builder.append(", ");
      builder.append("behavior_tree_indices=");
      builder.append(this.behavior_tree_indices_);      builder.append(", ");
      builder.append("basic_nodes=");
      builder.append(this.basic_nodes_);      builder.append(", ");
      builder.append("action_sequences=");
      builder.append(this.action_sequences_);      builder.append(", ");
      builder.append("door_traversals=");
      builder.append(this.door_traversals_);      builder.append(", ");
      builder.append("room_explorations=");
      builder.append(this.room_explorations_);      builder.append(", ");
      builder.append("chest_orientation_actions=");
      builder.append(this.chest_orientation_actions_);      builder.append(", ");
      builder.append("footstep_plan_actions=");
      builder.append(this.footstep_plan_actions_);      builder.append(", ");
      builder.append("sake_hand_command_actions=");
      builder.append(this.sake_hand_command_actions_);      builder.append(", ");
      builder.append("hand_pose_actions=");
      builder.append(this.hand_pose_actions_);      builder.append(", ");
      builder.append("hand_wrench_actions=");
      builder.append(this.hand_wrench_actions_);      builder.append(", ");
      builder.append("screw_primitive_actions=");
      builder.append(this.screw_primitive_actions_);      builder.append(", ");
      builder.append("pelvis_height_actions=");
      builder.append(this.pelvis_height_actions_);      builder.append(", ");
      builder.append("wait_duration_actions=");
      builder.append(this.wait_duration_actions_);
      builder.append("}");
      return builder.toString();
   }
}
