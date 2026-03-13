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
   /**
          * Used to minimize bandwidth, nodes that are send
          * without their full data.
          */
   public static final byte PARTIAL_DATA = (byte) 0;
   public static final byte ROOT_NODE = (byte) 1;
   public static final byte BASIC_NODE = (byte) 2;
   public static final byte ACTION_SEQUENCE = (byte) 3;
   public static final byte FALLBACK_NODE = (byte) 4;
   public static final byte CONDITION_NODE = (byte) 5;
   public static final byte GOTO_NODE = (byte) 6;
   public static final byte CHECKPOINT_NODE = (byte) 7;
   public static final byte SCENE_ACTION = (byte) 8;
   public static final byte AI2R_NODE = (byte) 9;
   public static final byte DOOR_TRAVERSAL = (byte) 10;
   public static final byte BUILDING_EXPLORATION = (byte) 11;
   public static final byte NECK_ACTION = (byte) 12;
   public static final byte CHEST_ORIENTATION_ACTION = (byte) 13;
   public static final byte FOOTSTEP_PLAN_ACTION = (byte) 14;
   public static final byte SAKE_HAND_COMMAND_ACTION = (byte) 15;
   public static final byte ABILITY_HAND_ACTION = (byte) 16;
   public static final byte HAND_POSE_ACTION = (byte) 17;
   public static final byte HAND_WRENCH_ACTION = (byte) 18;
   public static final byte SCREW_PRIMITIVE_ACTION = (byte) 19;
   public static final byte PELVIS_HEIGHT_ORIENTATION_ACTION = (byte) 20;
   public static final byte WAIT_DURATION_ACTION = (byte) 21;
   public static final byte FOOT_POSE_ACTION = (byte) 22;
   /**
            * Monotonically increasing message ID that matches the CRDTInfo update number
            */
   public long sequence_id_;
   /**
            * The ID to assign to the next instantiated node
            */
   public long next_id_;
   /**
            * The timestamp and modifier ID of the most recent time the root node was replaced or removed
            */
   public ihmc_common_msgs.msg.dds.LatestModificationMessage latest_modification_to_root_reference_;
   /**
            * The timestamp and modifier ID of the latest modification of the tree data fields
            */
   public ihmc_common_msgs.msg.dds.LatestModificationMessage latest_modification_to_data_;
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
   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.BasicNodeStateMessage>  partial_data_nodes_;
   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.BehaviorTreeRootNodeStateMessage>  root_nodes_;
   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.BasicNodeStateMessage>  basic_nodes_;
   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.ActionSequenceStateMessage>  action_sequences_;
   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.FallbackNodeStateMessage>  fallback_nodes_;
   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.ConditionNodeStateMessage>  condition_nodes_;
   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.GotoNodeStateMessage>  goto_nodes_;
   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.CheckPointNodeStateMessage>  checkpoint_nodes_;
   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.SceneActionNodeStateMessage>  scene_actions_;
   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.AI2RNodeStateMessage>  ai2r_nodes_;
   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.DoorTraversalStateMessage>  door_traversals_;
   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.BuildingExplorationStateMessage>  building_explorations_;
   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.NeckActionStateMessage>  neck_actions_;
   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.ChestOrientationActionStateMessage>  chest_orientation_actions_;
   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.FootstepPlanActionStateMessage>  footstep_plan_actions_;
   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.SakeHandCommandActionStateMessage>  sake_hand_command_actions_;
   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.AbilityHandActionStateMessage>  ability_hand_actions_;
   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.HandPoseActionStateMessage>  hand_pose_actions_;
   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.HandWrenchActionStateMessage>  hand_wrench_actions_;
   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.ScrewPrimitiveActionStateMessage>  screw_primitive_actions_;
   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.PelvisHeightOrientationActionStateMessage>  pelvis_height_actions_;
   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.WaitDurationActionStateMessage>  wait_duration_actions_;
   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.FootPoseActionStateMessage>  foot_pose_actions_;

   public BehaviorTreeStateMessage()
   {
      latest_modification_to_root_reference_ = new ihmc_common_msgs.msg.dds.LatestModificationMessage();
      latest_modification_to_data_ = new ihmc_common_msgs.msg.dds.LatestModificationMessage();
      behavior_tree_types_ = new us.ihmc.idl.IDLSequence.Byte (1000, "type_9");

      behavior_tree_indices_ = new us.ihmc.idl.IDLSequence.Long (1000, "type_4");

      partial_data_nodes_ = new us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.BasicNodeStateMessage> (300, new behavior_msgs.msg.dds.BasicNodeStateMessagePubSubType());
      root_nodes_ = new us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.BehaviorTreeRootNodeStateMessage> (1, new behavior_msgs.msg.dds.BehaviorTreeRootNodeStateMessagePubSubType());
      basic_nodes_ = new us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.BasicNodeStateMessage> (120, new behavior_msgs.msg.dds.BasicNodeStateMessagePubSubType());
      action_sequences_ = new us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.ActionSequenceStateMessage> (120, new behavior_msgs.msg.dds.ActionSequenceStateMessagePubSubType());
      fallback_nodes_ = new us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.FallbackNodeStateMessage> (120, new behavior_msgs.msg.dds.FallbackNodeStateMessagePubSubType());
      condition_nodes_ = new us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.ConditionNodeStateMessage> (120, new behavior_msgs.msg.dds.ConditionNodeStateMessagePubSubType());
      goto_nodes_ = new us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.GotoNodeStateMessage> (120, new behavior_msgs.msg.dds.GotoNodeStateMessagePubSubType());
      checkpoint_nodes_ = new us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.CheckPointNodeStateMessage> (120, new behavior_msgs.msg.dds.CheckPointNodeStateMessagePubSubType());
      scene_actions_ = new us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.SceneActionNodeStateMessage> (120, new behavior_msgs.msg.dds.SceneActionNodeStateMessagePubSubType());
      ai2r_nodes_ = new us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.AI2RNodeStateMessage> (1, new behavior_msgs.msg.dds.AI2RNodeStateMessagePubSubType());
      door_traversals_ = new us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.DoorTraversalStateMessage> (120, new behavior_msgs.msg.dds.DoorTraversalStateMessagePubSubType());
      building_explorations_ = new us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.BuildingExplorationStateMessage> (120, new behavior_msgs.msg.dds.BuildingExplorationStateMessagePubSubType());
      neck_actions_ = new us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.NeckActionStateMessage> (120, new behavior_msgs.msg.dds.NeckActionStateMessagePubSubType());
      chest_orientation_actions_ = new us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.ChestOrientationActionStateMessage> (120, new behavior_msgs.msg.dds.ChestOrientationActionStateMessagePubSubType());
      footstep_plan_actions_ = new us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.FootstepPlanActionStateMessage> (120, new behavior_msgs.msg.dds.FootstepPlanActionStateMessagePubSubType());
      sake_hand_command_actions_ = new us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.SakeHandCommandActionStateMessage> (120, new behavior_msgs.msg.dds.SakeHandCommandActionStateMessagePubSubType());
      ability_hand_actions_ = new us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.AbilityHandActionStateMessage> (120, new behavior_msgs.msg.dds.AbilityHandActionStateMessagePubSubType());
      hand_pose_actions_ = new us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.HandPoseActionStateMessage> (120, new behavior_msgs.msg.dds.HandPoseActionStateMessagePubSubType());
      hand_wrench_actions_ = new us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.HandWrenchActionStateMessage> (120, new behavior_msgs.msg.dds.HandWrenchActionStateMessagePubSubType());
      screw_primitive_actions_ = new us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.ScrewPrimitiveActionStateMessage> (120, new behavior_msgs.msg.dds.ScrewPrimitiveActionStateMessagePubSubType());
      pelvis_height_actions_ = new us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.PelvisHeightOrientationActionStateMessage> (120, new behavior_msgs.msg.dds.PelvisHeightOrientationActionStateMessagePubSubType());
      wait_duration_actions_ = new us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.WaitDurationActionStateMessage> (120, new behavior_msgs.msg.dds.WaitDurationActionStateMessagePubSubType());
      foot_pose_actions_ = new us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.FootPoseActionStateMessage> (120, new behavior_msgs.msg.dds.FootPoseActionStateMessagePubSubType());

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

      ihmc_common_msgs.msg.dds.LatestModificationMessagePubSubType.staticCopy(other.latest_modification_to_root_reference_, latest_modification_to_root_reference_);
      ihmc_common_msgs.msg.dds.LatestModificationMessagePubSubType.staticCopy(other.latest_modification_to_data_, latest_modification_to_data_);
      behavior_tree_types_.set(other.behavior_tree_types_);
      behavior_tree_indices_.set(other.behavior_tree_indices_);
      partial_data_nodes_.set(other.partial_data_nodes_);
      root_nodes_.set(other.root_nodes_);
      basic_nodes_.set(other.basic_nodes_);
      action_sequences_.set(other.action_sequences_);
      fallback_nodes_.set(other.fallback_nodes_);
      condition_nodes_.set(other.condition_nodes_);
      goto_nodes_.set(other.goto_nodes_);
      checkpoint_nodes_.set(other.checkpoint_nodes_);
      scene_actions_.set(other.scene_actions_);
      ai2r_nodes_.set(other.ai2r_nodes_);
      door_traversals_.set(other.door_traversals_);
      building_explorations_.set(other.building_explorations_);
      neck_actions_.set(other.neck_actions_);
      chest_orientation_actions_.set(other.chest_orientation_actions_);
      footstep_plan_actions_.set(other.footstep_plan_actions_);
      sake_hand_command_actions_.set(other.sake_hand_command_actions_);
      ability_hand_actions_.set(other.ability_hand_actions_);
      hand_pose_actions_.set(other.hand_pose_actions_);
      hand_wrench_actions_.set(other.hand_wrench_actions_);
      screw_primitive_actions_.set(other.screw_primitive_actions_);
      pelvis_height_actions_.set(other.pelvis_height_actions_);
      wait_duration_actions_.set(other.wait_duration_actions_);
      foot_pose_actions_.set(other.foot_pose_actions_);
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
            * The timestamp and modifier ID of the most recent time the root node was replaced or removed
            */
   public ihmc_common_msgs.msg.dds.LatestModificationMessage getLatestModificationToRootReference()
   {
      return latest_modification_to_root_reference_;
   }


   /**
            * The timestamp and modifier ID of the latest modification of the tree data fields
            */
   public ihmc_common_msgs.msg.dds.LatestModificationMessage getLatestModificationToData()
   {
      return latest_modification_to_data_;
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


   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.BasicNodeStateMessage>  getPartialDataNodes()
   {
      return partial_data_nodes_;
   }


   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.BehaviorTreeRootNodeStateMessage>  getRootNodes()
   {
      return root_nodes_;
   }


   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.BasicNodeStateMessage>  getBasicNodes()
   {
      return basic_nodes_;
   }


   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.ActionSequenceStateMessage>  getActionSequences()
   {
      return action_sequences_;
   }


   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.FallbackNodeStateMessage>  getFallbackNodes()
   {
      return fallback_nodes_;
   }


   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.ConditionNodeStateMessage>  getConditionNodes()
   {
      return condition_nodes_;
   }


   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.GotoNodeStateMessage>  getGotoNodes()
   {
      return goto_nodes_;
   }


   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.CheckPointNodeStateMessage>  getCheckpointNodes()
   {
      return checkpoint_nodes_;
   }


   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.SceneActionNodeStateMessage>  getSceneActions()
   {
      return scene_actions_;
   }


   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.AI2RNodeStateMessage>  getAi2rNodes()
   {
      return ai2r_nodes_;
   }


   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.DoorTraversalStateMessage>  getDoorTraversals()
   {
      return door_traversals_;
   }


   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.BuildingExplorationStateMessage>  getBuildingExplorations()
   {
      return building_explorations_;
   }


   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.NeckActionStateMessage>  getNeckActions()
   {
      return neck_actions_;
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


   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.AbilityHandActionStateMessage>  getAbilityHandActions()
   {
      return ability_hand_actions_;
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


   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.PelvisHeightOrientationActionStateMessage>  getPelvisHeightActions()
   {
      return pelvis_height_actions_;
   }


   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.WaitDurationActionStateMessage>  getWaitDurationActions()
   {
      return wait_duration_actions_;
   }


   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.FootPoseActionStateMessage>  getFootPoseActions()
   {
      return foot_pose_actions_;
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

      if (!this.latest_modification_to_root_reference_.epsilonEquals(other.latest_modification_to_root_reference_, epsilon)) return false;
      if (!this.latest_modification_to_data_.epsilonEquals(other.latest_modification_to_data_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsByteSequence(this.behavior_tree_types_, other.behavior_tree_types_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsLongSequence(this.behavior_tree_indices_, other.behavior_tree_indices_, epsilon)) return false;

      if (this.partial_data_nodes_.size() != other.partial_data_nodes_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.partial_data_nodes_.size(); i++)
         {  if (!this.partial_data_nodes_.get(i).epsilonEquals(other.partial_data_nodes_.get(i), epsilon)) return false; }
      }

      if (this.root_nodes_.size() != other.root_nodes_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.root_nodes_.size(); i++)
         {  if (!this.root_nodes_.get(i).epsilonEquals(other.root_nodes_.get(i), epsilon)) return false; }
      }

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

      if (this.fallback_nodes_.size() != other.fallback_nodes_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.fallback_nodes_.size(); i++)
         {  if (!this.fallback_nodes_.get(i).epsilonEquals(other.fallback_nodes_.get(i), epsilon)) return false; }
      }

      if (this.condition_nodes_.size() != other.condition_nodes_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.condition_nodes_.size(); i++)
         {  if (!this.condition_nodes_.get(i).epsilonEquals(other.condition_nodes_.get(i), epsilon)) return false; }
      }

      if (this.goto_nodes_.size() != other.goto_nodes_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.goto_nodes_.size(); i++)
         {  if (!this.goto_nodes_.get(i).epsilonEquals(other.goto_nodes_.get(i), epsilon)) return false; }
      }

      if (this.checkpoint_nodes_.size() != other.checkpoint_nodes_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.checkpoint_nodes_.size(); i++)
         {  if (!this.checkpoint_nodes_.get(i).epsilonEquals(other.checkpoint_nodes_.get(i), epsilon)) return false; }
      }

      if (this.scene_actions_.size() != other.scene_actions_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.scene_actions_.size(); i++)
         {  if (!this.scene_actions_.get(i).epsilonEquals(other.scene_actions_.get(i), epsilon)) return false; }
      }

      if (this.ai2r_nodes_.size() != other.ai2r_nodes_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.ai2r_nodes_.size(); i++)
         {  if (!this.ai2r_nodes_.get(i).epsilonEquals(other.ai2r_nodes_.get(i), epsilon)) return false; }
      }

      if (this.door_traversals_.size() != other.door_traversals_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.door_traversals_.size(); i++)
         {  if (!this.door_traversals_.get(i).epsilonEquals(other.door_traversals_.get(i), epsilon)) return false; }
      }

      if (this.building_explorations_.size() != other.building_explorations_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.building_explorations_.size(); i++)
         {  if (!this.building_explorations_.get(i).epsilonEquals(other.building_explorations_.get(i), epsilon)) return false; }
      }

      if (this.neck_actions_.size() != other.neck_actions_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.neck_actions_.size(); i++)
         {  if (!this.neck_actions_.get(i).epsilonEquals(other.neck_actions_.get(i), epsilon)) return false; }
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

      if (this.ability_hand_actions_.size() != other.ability_hand_actions_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.ability_hand_actions_.size(); i++)
         {  if (!this.ability_hand_actions_.get(i).epsilonEquals(other.ability_hand_actions_.get(i), epsilon)) return false; }
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

      if (this.foot_pose_actions_.size() != other.foot_pose_actions_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.foot_pose_actions_.size(); i++)
         {  if (!this.foot_pose_actions_.get(i).epsilonEquals(other.foot_pose_actions_.get(i), epsilon)) return false; }
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

      if (!this.latest_modification_to_root_reference_.equals(otherMyClass.latest_modification_to_root_reference_)) return false;
      if (!this.latest_modification_to_data_.equals(otherMyClass.latest_modification_to_data_)) return false;
      if (!this.behavior_tree_types_.equals(otherMyClass.behavior_tree_types_)) return false;
      if (!this.behavior_tree_indices_.equals(otherMyClass.behavior_tree_indices_)) return false;
      if (!this.partial_data_nodes_.equals(otherMyClass.partial_data_nodes_)) return false;
      if (!this.root_nodes_.equals(otherMyClass.root_nodes_)) return false;
      if (!this.basic_nodes_.equals(otherMyClass.basic_nodes_)) return false;
      if (!this.action_sequences_.equals(otherMyClass.action_sequences_)) return false;
      if (!this.fallback_nodes_.equals(otherMyClass.fallback_nodes_)) return false;
      if (!this.condition_nodes_.equals(otherMyClass.condition_nodes_)) return false;
      if (!this.goto_nodes_.equals(otherMyClass.goto_nodes_)) return false;
      if (!this.checkpoint_nodes_.equals(otherMyClass.checkpoint_nodes_)) return false;
      if (!this.scene_actions_.equals(otherMyClass.scene_actions_)) return false;
      if (!this.ai2r_nodes_.equals(otherMyClass.ai2r_nodes_)) return false;
      if (!this.door_traversals_.equals(otherMyClass.door_traversals_)) return false;
      if (!this.building_explorations_.equals(otherMyClass.building_explorations_)) return false;
      if (!this.neck_actions_.equals(otherMyClass.neck_actions_)) return false;
      if (!this.chest_orientation_actions_.equals(otherMyClass.chest_orientation_actions_)) return false;
      if (!this.footstep_plan_actions_.equals(otherMyClass.footstep_plan_actions_)) return false;
      if (!this.sake_hand_command_actions_.equals(otherMyClass.sake_hand_command_actions_)) return false;
      if (!this.ability_hand_actions_.equals(otherMyClass.ability_hand_actions_)) return false;
      if (!this.hand_pose_actions_.equals(otherMyClass.hand_pose_actions_)) return false;
      if (!this.hand_wrench_actions_.equals(otherMyClass.hand_wrench_actions_)) return false;
      if (!this.screw_primitive_actions_.equals(otherMyClass.screw_primitive_actions_)) return false;
      if (!this.pelvis_height_actions_.equals(otherMyClass.pelvis_height_actions_)) return false;
      if (!this.wait_duration_actions_.equals(otherMyClass.wait_duration_actions_)) return false;
      if (!this.foot_pose_actions_.equals(otherMyClass.foot_pose_actions_)) return false;

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
      builder.append("latest_modification_to_root_reference=");
      builder.append(this.latest_modification_to_root_reference_);      builder.append(", ");
      builder.append("latest_modification_to_data=");
      builder.append(this.latest_modification_to_data_);      builder.append(", ");
      builder.append("behavior_tree_types=");
      builder.append(this.behavior_tree_types_);      builder.append(", ");
      builder.append("behavior_tree_indices=");
      builder.append(this.behavior_tree_indices_);      builder.append(", ");
      builder.append("partial_data_nodes=");
      builder.append(this.partial_data_nodes_);      builder.append(", ");
      builder.append("root_nodes=");
      builder.append(this.root_nodes_);      builder.append(", ");
      builder.append("basic_nodes=");
      builder.append(this.basic_nodes_);      builder.append(", ");
      builder.append("action_sequences=");
      builder.append(this.action_sequences_);      builder.append(", ");
      builder.append("fallback_nodes=");
      builder.append(this.fallback_nodes_);      builder.append(", ");
      builder.append("condition_nodes=");
      builder.append(this.condition_nodes_);      builder.append(", ");
      builder.append("goto_nodes=");
      builder.append(this.goto_nodes_);      builder.append(", ");
      builder.append("checkpoint_nodes=");
      builder.append(this.checkpoint_nodes_);      builder.append(", ");
      builder.append("scene_actions=");
      builder.append(this.scene_actions_);      builder.append(", ");
      builder.append("ai2r_nodes=");
      builder.append(this.ai2r_nodes_);      builder.append(", ");
      builder.append("door_traversals=");
      builder.append(this.door_traversals_);      builder.append(", ");
      builder.append("building_explorations=");
      builder.append(this.building_explorations_);      builder.append(", ");
      builder.append("neck_actions=");
      builder.append(this.neck_actions_);      builder.append(", ");
      builder.append("chest_orientation_actions=");
      builder.append(this.chest_orientation_actions_);      builder.append(", ");
      builder.append("footstep_plan_actions=");
      builder.append(this.footstep_plan_actions_);      builder.append(", ");
      builder.append("sake_hand_command_actions=");
      builder.append(this.sake_hand_command_actions_);      builder.append(", ");
      builder.append("ability_hand_actions=");
      builder.append(this.ability_hand_actions_);      builder.append(", ");
      builder.append("hand_pose_actions=");
      builder.append(this.hand_pose_actions_);      builder.append(", ");
      builder.append("hand_wrench_actions=");
      builder.append(this.hand_wrench_actions_);      builder.append(", ");
      builder.append("screw_primitive_actions=");
      builder.append(this.screw_primitive_actions_);      builder.append(", ");
      builder.append("pelvis_height_actions=");
      builder.append(this.pelvis_height_actions_);      builder.append(", ");
      builder.append("wait_duration_actions=");
      builder.append(this.wait_duration_actions_);      builder.append(", ");
      builder.append("foot_pose_actions=");
      builder.append(this.foot_pose_actions_);
      builder.append("}");
      return builder.toString();
   }
}
