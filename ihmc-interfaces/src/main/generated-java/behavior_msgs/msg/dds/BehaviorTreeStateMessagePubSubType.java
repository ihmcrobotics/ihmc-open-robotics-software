package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "BehaviorTreeStateMessage" defined in "BehaviorTreeStateMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from BehaviorTreeStateMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit BehaviorTreeStateMessage_.idl instead.
*
*/
public class BehaviorTreeStateMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.BehaviorTreeStateMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::BehaviorTreeStateMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "6aafa11bd7c734422722f6e88c0f92804aa349180ca9263cbfc70af7960a7906";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.BehaviorTreeStateMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.BehaviorTreeStateMessage data) throws java.io.IOException
   {
      deserializeCDR.deserialize(serializedPayload);
      read(data, deserializeCDR);
      deserializeCDR.finishDeserialize();
   }

   public static int getMaxCdrSerializedSize()
   {
      return getMaxCdrSerializedSize(0);
   }

   public static int getMaxCdrSerializedSize(int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += ihmc_common_msgs.msg.dds.LatestModificationMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += ihmc_common_msgs.msg.dds.LatestModificationMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (1000 * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (1000 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 300; ++i0)
      {
          current_alignment += behavior_msgs.msg.dds.BasicNodeStateMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 1; ++i0)
      {
          current_alignment += behavior_msgs.msg.dds.BehaviorTreeRootNodeStateMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 120; ++i0)
      {
          current_alignment += behavior_msgs.msg.dds.BasicNodeStateMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 120; ++i0)
      {
          current_alignment += behavior_msgs.msg.dds.ActionSequenceStateMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 120; ++i0)
      {
          current_alignment += behavior_msgs.msg.dds.FallbackNodeStateMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 120; ++i0)
      {
          current_alignment += behavior_msgs.msg.dds.ConditionNodeStateMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 120; ++i0)
      {
          current_alignment += behavior_msgs.msg.dds.GotoNodeStateMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 120; ++i0)
      {
          current_alignment += behavior_msgs.msg.dds.CheckpointNodeStateMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 120; ++i0)
      {
          current_alignment += behavior_msgs.msg.dds.SceneActionStateMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 1; ++i0)
      {
          current_alignment += behavior_msgs.msg.dds.AI2RNodeStateMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 120; ++i0)
      {
          current_alignment += behavior_msgs.msg.dds.DoorTraversalStateMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 120; ++i0)
      {
          current_alignment += behavior_msgs.msg.dds.BuildingExplorationStateMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 120; ++i0)
      {
          current_alignment += behavior_msgs.msg.dds.NeckActionStateMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 120; ++i0)
      {
          current_alignment += behavior_msgs.msg.dds.SpineActionStateMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 120; ++i0)
      {
          current_alignment += behavior_msgs.msg.dds.WalkActionStateMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 120; ++i0)
      {
          current_alignment += behavior_msgs.msg.dds.EZGripperActionStateMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 120; ++i0)
      {
          current_alignment += behavior_msgs.msg.dds.AbilityHandActionStateMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 120; ++i0)
      {
          current_alignment += behavior_msgs.msg.dds.ArmActionStateMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 120; ++i0)
      {
          current_alignment += behavior_msgs.msg.dds.HandWrenchActionStateMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 120; ++i0)
      {
          current_alignment += behavior_msgs.msg.dds.ScrewPrimitiveActionStateMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 120; ++i0)
      {
          current_alignment += behavior_msgs.msg.dds.PelvisActionStateMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 120; ++i0)
      {
          current_alignment += behavior_msgs.msg.dds.WaitActionStateMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 120; ++i0)
      {
          current_alignment += behavior_msgs.msg.dds.LegActionStateMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.BehaviorTreeStateMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.BehaviorTreeStateMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += ihmc_common_msgs.msg.dds.LatestModificationMessagePubSubType.getCdrSerializedSize(data.getLatestModificationToRootReference(), current_alignment);

      current_alignment += ihmc_common_msgs.msg.dds.LatestModificationMessagePubSubType.getCdrSerializedSize(data.getLatestModificationToData(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getBehaviorTreeTypes().size() * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getBehaviorTreeIndices().size() * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getPartialDataNodes().size(); ++i0)
      {
          current_alignment += behavior_msgs.msg.dds.BasicNodeStateMessagePubSubType.getCdrSerializedSize(data.getPartialDataNodes().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getRootNodes().size(); ++i0)
      {
          current_alignment += behavior_msgs.msg.dds.BehaviorTreeRootNodeStateMessagePubSubType.getCdrSerializedSize(data.getRootNodes().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getBasicNodes().size(); ++i0)
      {
          current_alignment += behavior_msgs.msg.dds.BasicNodeStateMessagePubSubType.getCdrSerializedSize(data.getBasicNodes().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getActionSequences().size(); ++i0)
      {
          current_alignment += behavior_msgs.msg.dds.ActionSequenceStateMessagePubSubType.getCdrSerializedSize(data.getActionSequences().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getFallbackNodes().size(); ++i0)
      {
          current_alignment += behavior_msgs.msg.dds.FallbackNodeStateMessagePubSubType.getCdrSerializedSize(data.getFallbackNodes().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getConditionNodes().size(); ++i0)
      {
          current_alignment += behavior_msgs.msg.dds.ConditionNodeStateMessagePubSubType.getCdrSerializedSize(data.getConditionNodes().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getGotoNodes().size(); ++i0)
      {
          current_alignment += behavior_msgs.msg.dds.GotoNodeStateMessagePubSubType.getCdrSerializedSize(data.getGotoNodes().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getCheckpointNodes().size(); ++i0)
      {
          current_alignment += behavior_msgs.msg.dds.CheckpointNodeStateMessagePubSubType.getCdrSerializedSize(data.getCheckpointNodes().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getSceneActions().size(); ++i0)
      {
          current_alignment += behavior_msgs.msg.dds.SceneActionStateMessagePubSubType.getCdrSerializedSize(data.getSceneActions().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getAi2rNodes().size(); ++i0)
      {
          current_alignment += behavior_msgs.msg.dds.AI2RNodeStateMessagePubSubType.getCdrSerializedSize(data.getAi2rNodes().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getDoorTraversals().size(); ++i0)
      {
          current_alignment += behavior_msgs.msg.dds.DoorTraversalStateMessagePubSubType.getCdrSerializedSize(data.getDoorTraversals().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getBuildingExplorations().size(); ++i0)
      {
          current_alignment += behavior_msgs.msg.dds.BuildingExplorationStateMessagePubSubType.getCdrSerializedSize(data.getBuildingExplorations().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getNeckActions().size(); ++i0)
      {
          current_alignment += behavior_msgs.msg.dds.NeckActionStateMessagePubSubType.getCdrSerializedSize(data.getNeckActions().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getSpineActions().size(); ++i0)
      {
          current_alignment += behavior_msgs.msg.dds.SpineActionStateMessagePubSubType.getCdrSerializedSize(data.getSpineActions().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getWalkActions().size(); ++i0)
      {
          current_alignment += behavior_msgs.msg.dds.WalkActionStateMessagePubSubType.getCdrSerializedSize(data.getWalkActions().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getEzGripperActions().size(); ++i0)
      {
          current_alignment += behavior_msgs.msg.dds.EZGripperActionStateMessagePubSubType.getCdrSerializedSize(data.getEzGripperActions().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getAbilityHandActions().size(); ++i0)
      {
          current_alignment += behavior_msgs.msg.dds.AbilityHandActionStateMessagePubSubType.getCdrSerializedSize(data.getAbilityHandActions().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getArmActions().size(); ++i0)
      {
          current_alignment += behavior_msgs.msg.dds.ArmActionStateMessagePubSubType.getCdrSerializedSize(data.getArmActions().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getHandWrenchActions().size(); ++i0)
      {
          current_alignment += behavior_msgs.msg.dds.HandWrenchActionStateMessagePubSubType.getCdrSerializedSize(data.getHandWrenchActions().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getScrewPrimitiveActions().size(); ++i0)
      {
          current_alignment += behavior_msgs.msg.dds.ScrewPrimitiveActionStateMessagePubSubType.getCdrSerializedSize(data.getScrewPrimitiveActions().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getPelvisActions().size(); ++i0)
      {
          current_alignment += behavior_msgs.msg.dds.PelvisActionStateMessagePubSubType.getCdrSerializedSize(data.getPelvisActions().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getWaitActions().size(); ++i0)
      {
          current_alignment += behavior_msgs.msg.dds.WaitActionStateMessagePubSubType.getCdrSerializedSize(data.getWaitActions().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getLegActions().size(); ++i0)
      {
          current_alignment += behavior_msgs.msg.dds.LegActionStateMessagePubSubType.getCdrSerializedSize(data.getLegActions().get(i0), current_alignment);}


      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.BehaviorTreeStateMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_12(data.getSequenceId());

      cdr.write_type_4(data.getNextId());

      ihmc_common_msgs.msg.dds.LatestModificationMessagePubSubType.write(data.getLatestModificationToRootReference(), cdr);
      ihmc_common_msgs.msg.dds.LatestModificationMessagePubSubType.write(data.getLatestModificationToData(), cdr);
      if(data.getBehaviorTreeTypes().size() <= 1000)
      cdr.write_type_e(data.getBehaviorTreeTypes());else
          throw new RuntimeException("behavior_tree_types field exceeds the maximum length: %d > %d".formatted(data.getBehaviorTreeTypes().size(), 1000));

      if(data.getBehaviorTreeIndices().size() <= 1000)
      cdr.write_type_e(data.getBehaviorTreeIndices());else
          throw new RuntimeException("behavior_tree_indices field exceeds the maximum length: %d > %d".formatted(data.getBehaviorTreeIndices().size(), 1000));

      if(data.getPartialDataNodes().size() <= 300)
      cdr.write_type_e(data.getPartialDataNodes());else
          throw new RuntimeException("partial_data_nodes field exceeds the maximum length: %d > %d".formatted(data.getPartialDataNodes().size(), 300));

      if(data.getRootNodes().size() <= 1)
      cdr.write_type_e(data.getRootNodes());else
          throw new RuntimeException("root_nodes field exceeds the maximum length: %d > %d".formatted(data.getRootNodes().size(), 1));

      if(data.getBasicNodes().size() <= 120)
      cdr.write_type_e(data.getBasicNodes());else
          throw new RuntimeException("basic_nodes field exceeds the maximum length: %d > %d".formatted(data.getBasicNodes().size(), 120));

      if(data.getActionSequences().size() <= 120)
      cdr.write_type_e(data.getActionSequences());else
          throw new RuntimeException("action_sequences field exceeds the maximum length: %d > %d".formatted(data.getActionSequences().size(), 120));

      if(data.getFallbackNodes().size() <= 120)
      cdr.write_type_e(data.getFallbackNodes());else
          throw new RuntimeException("fallback_nodes field exceeds the maximum length: %d > %d".formatted(data.getFallbackNodes().size(), 120));

      if(data.getConditionNodes().size() <= 120)
      cdr.write_type_e(data.getConditionNodes());else
          throw new RuntimeException("condition_nodes field exceeds the maximum length: %d > %d".formatted(data.getConditionNodes().size(), 120));

      if(data.getGotoNodes().size() <= 120)
      cdr.write_type_e(data.getGotoNodes());else
          throw new RuntimeException("goto_nodes field exceeds the maximum length: %d > %d".formatted(data.getGotoNodes().size(), 120));

      if(data.getCheckpointNodes().size() <= 120)
      cdr.write_type_e(data.getCheckpointNodes());else
          throw new RuntimeException("checkpoint_nodes field exceeds the maximum length: %d > %d".formatted(data.getCheckpointNodes().size(), 120));

      if(data.getSceneActions().size() <= 120)
      cdr.write_type_e(data.getSceneActions());else
          throw new RuntimeException("scene_actions field exceeds the maximum length: %d > %d".formatted(data.getSceneActions().size(), 120));

      if(data.getAi2rNodes().size() <= 1)
      cdr.write_type_e(data.getAi2rNodes());else
          throw new RuntimeException("ai2r_nodes field exceeds the maximum length: %d > %d".formatted(data.getAi2rNodes().size(), 1));

      if(data.getDoorTraversals().size() <= 120)
      cdr.write_type_e(data.getDoorTraversals());else
          throw new RuntimeException("door_traversals field exceeds the maximum length: %d > %d".formatted(data.getDoorTraversals().size(), 120));

      if(data.getBuildingExplorations().size() <= 120)
      cdr.write_type_e(data.getBuildingExplorations());else
          throw new RuntimeException("building_explorations field exceeds the maximum length: %d > %d".formatted(data.getBuildingExplorations().size(), 120));

      if(data.getNeckActions().size() <= 120)
      cdr.write_type_e(data.getNeckActions());else
          throw new RuntimeException("neck_actions field exceeds the maximum length: %d > %d".formatted(data.getNeckActions().size(), 120));

      if(data.getSpineActions().size() <= 120)
      cdr.write_type_e(data.getSpineActions());else
          throw new RuntimeException("spine_actions field exceeds the maximum length: %d > %d".formatted(data.getSpineActions().size(), 120));

      if(data.getWalkActions().size() <= 120)
      cdr.write_type_e(data.getWalkActions());else
          throw new RuntimeException("walk_actions field exceeds the maximum length: %d > %d".formatted(data.getWalkActions().size(), 120));

      if(data.getEzGripperActions().size() <= 120)
      cdr.write_type_e(data.getEzGripperActions());else
          throw new RuntimeException("ez_gripper_actions field exceeds the maximum length: %d > %d".formatted(data.getEzGripperActions().size(), 120));

      if(data.getAbilityHandActions().size() <= 120)
      cdr.write_type_e(data.getAbilityHandActions());else
          throw new RuntimeException("ability_hand_actions field exceeds the maximum length: %d > %d".formatted(data.getAbilityHandActions().size(), 120));

      if(data.getArmActions().size() <= 120)
      cdr.write_type_e(data.getArmActions());else
          throw new RuntimeException("arm_actions field exceeds the maximum length: %d > %d".formatted(data.getArmActions().size(), 120));

      if(data.getHandWrenchActions().size() <= 120)
      cdr.write_type_e(data.getHandWrenchActions());else
          throw new RuntimeException("hand_wrench_actions field exceeds the maximum length: %d > %d".formatted(data.getHandWrenchActions().size(), 120));

      if(data.getScrewPrimitiveActions().size() <= 120)
      cdr.write_type_e(data.getScrewPrimitiveActions());else
          throw new RuntimeException("screw_primitive_actions field exceeds the maximum length: %d > %d".formatted(data.getScrewPrimitiveActions().size(), 120));

      if(data.getPelvisActions().size() <= 120)
      cdr.write_type_e(data.getPelvisActions());else
          throw new RuntimeException("pelvis_actions field exceeds the maximum length: %d > %d".formatted(data.getPelvisActions().size(), 120));

      if(data.getWaitActions().size() <= 120)
      cdr.write_type_e(data.getWaitActions());else
          throw new RuntimeException("wait_actions field exceeds the maximum length: %d > %d".formatted(data.getWaitActions().size(), 120));

      if(data.getLegActions().size() <= 120)
      cdr.write_type_e(data.getLegActions());else
          throw new RuntimeException("leg_actions field exceeds the maximum length: %d > %d".formatted(data.getLegActions().size(), 120));

   }

   public static void read(behavior_msgs.msg.dds.BehaviorTreeStateMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_12());
      	
      data.setNextId(cdr.read_type_4());
      	
      ihmc_common_msgs.msg.dds.LatestModificationMessagePubSubType.read(data.getLatestModificationToRootReference(), cdr);	
      ihmc_common_msgs.msg.dds.LatestModificationMessagePubSubType.read(data.getLatestModificationToData(), cdr);	
      cdr.read_type_e(data.getBehaviorTreeTypes());	
      cdr.read_type_e(data.getBehaviorTreeIndices());	
      cdr.read_type_e(data.getPartialDataNodes());	
      cdr.read_type_e(data.getRootNodes());	
      cdr.read_type_e(data.getBasicNodes());	
      cdr.read_type_e(data.getActionSequences());	
      cdr.read_type_e(data.getFallbackNodes());	
      cdr.read_type_e(data.getConditionNodes());	
      cdr.read_type_e(data.getGotoNodes());	
      cdr.read_type_e(data.getCheckpointNodes());	
      cdr.read_type_e(data.getSceneActions());	
      cdr.read_type_e(data.getAi2rNodes());	
      cdr.read_type_e(data.getDoorTraversals());	
      cdr.read_type_e(data.getBuildingExplorations());	
      cdr.read_type_e(data.getNeckActions());	
      cdr.read_type_e(data.getSpineActions());	
      cdr.read_type_e(data.getWalkActions());	
      cdr.read_type_e(data.getEzGripperActions());	
      cdr.read_type_e(data.getAbilityHandActions());	
      cdr.read_type_e(data.getArmActions());	
      cdr.read_type_e(data.getHandWrenchActions());	
      cdr.read_type_e(data.getScrewPrimitiveActions());	
      cdr.read_type_e(data.getPelvisActions());	
      cdr.read_type_e(data.getWaitActions());	
      cdr.read_type_e(data.getLegActions());	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.BehaviorTreeStateMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_12("sequence_id", data.getSequenceId());
      ser.write_type_4("next_id", data.getNextId());
      ser.write_type_a("latest_modification_to_root_reference", new ihmc_common_msgs.msg.dds.LatestModificationMessagePubSubType(), data.getLatestModificationToRootReference());

      ser.write_type_a("latest_modification_to_data", new ihmc_common_msgs.msg.dds.LatestModificationMessagePubSubType(), data.getLatestModificationToData());

      ser.write_type_e("behavior_tree_types", data.getBehaviorTreeTypes());
      ser.write_type_e("behavior_tree_indices", data.getBehaviorTreeIndices());
      ser.write_type_e("partial_data_nodes", data.getPartialDataNodes());
      ser.write_type_e("root_nodes", data.getRootNodes());
      ser.write_type_e("basic_nodes", data.getBasicNodes());
      ser.write_type_e("action_sequences", data.getActionSequences());
      ser.write_type_e("fallback_nodes", data.getFallbackNodes());
      ser.write_type_e("condition_nodes", data.getConditionNodes());
      ser.write_type_e("goto_nodes", data.getGotoNodes());
      ser.write_type_e("checkpoint_nodes", data.getCheckpointNodes());
      ser.write_type_e("scene_actions", data.getSceneActions());
      ser.write_type_e("ai2r_nodes", data.getAi2rNodes());
      ser.write_type_e("door_traversals", data.getDoorTraversals());
      ser.write_type_e("building_explorations", data.getBuildingExplorations());
      ser.write_type_e("neck_actions", data.getNeckActions());
      ser.write_type_e("spine_actions", data.getSpineActions());
      ser.write_type_e("walk_actions", data.getWalkActions());
      ser.write_type_e("ez_gripper_actions", data.getEzGripperActions());
      ser.write_type_e("ability_hand_actions", data.getAbilityHandActions());
      ser.write_type_e("arm_actions", data.getArmActions());
      ser.write_type_e("hand_wrench_actions", data.getHandWrenchActions());
      ser.write_type_e("screw_primitive_actions", data.getScrewPrimitiveActions());
      ser.write_type_e("pelvis_actions", data.getPelvisActions());
      ser.write_type_e("wait_actions", data.getWaitActions());
      ser.write_type_e("leg_actions", data.getLegActions());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.BehaviorTreeStateMessage data)
   {
      data.setSequenceId(ser.read_type_12("sequence_id"));
      data.setNextId(ser.read_type_4("next_id"));
      ser.read_type_a("latest_modification_to_root_reference", new ihmc_common_msgs.msg.dds.LatestModificationMessagePubSubType(), data.getLatestModificationToRootReference());

      ser.read_type_a("latest_modification_to_data", new ihmc_common_msgs.msg.dds.LatestModificationMessagePubSubType(), data.getLatestModificationToData());

      ser.read_type_e("behavior_tree_types", data.getBehaviorTreeTypes());
      ser.read_type_e("behavior_tree_indices", data.getBehaviorTreeIndices());
      ser.read_type_e("partial_data_nodes", data.getPartialDataNodes());
      ser.read_type_e("root_nodes", data.getRootNodes());
      ser.read_type_e("basic_nodes", data.getBasicNodes());
      ser.read_type_e("action_sequences", data.getActionSequences());
      ser.read_type_e("fallback_nodes", data.getFallbackNodes());
      ser.read_type_e("condition_nodes", data.getConditionNodes());
      ser.read_type_e("goto_nodes", data.getGotoNodes());
      ser.read_type_e("checkpoint_nodes", data.getCheckpointNodes());
      ser.read_type_e("scene_actions", data.getSceneActions());
      ser.read_type_e("ai2r_nodes", data.getAi2rNodes());
      ser.read_type_e("door_traversals", data.getDoorTraversals());
      ser.read_type_e("building_explorations", data.getBuildingExplorations());
      ser.read_type_e("neck_actions", data.getNeckActions());
      ser.read_type_e("spine_actions", data.getSpineActions());
      ser.read_type_e("walk_actions", data.getWalkActions());
      ser.read_type_e("ez_gripper_actions", data.getEzGripperActions());
      ser.read_type_e("ability_hand_actions", data.getAbilityHandActions());
      ser.read_type_e("arm_actions", data.getArmActions());
      ser.read_type_e("hand_wrench_actions", data.getHandWrenchActions());
      ser.read_type_e("screw_primitive_actions", data.getScrewPrimitiveActions());
      ser.read_type_e("pelvis_actions", data.getPelvisActions());
      ser.read_type_e("wait_actions", data.getWaitActions());
      ser.read_type_e("leg_actions", data.getLegActions());
   }

   public static void staticCopy(behavior_msgs.msg.dds.BehaviorTreeStateMessage src, behavior_msgs.msg.dds.BehaviorTreeStateMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.BehaviorTreeStateMessage createData()
   {
      return new behavior_msgs.msg.dds.BehaviorTreeStateMessage();
   }
   @Override
   public int getTypeSize()
   {
      return us.ihmc.idl.CDR.getTypeSize(getMaxCdrSerializedSize());
   }

   @Override
   public java.lang.String getName()
   {
      return name;
   }
   
   public void serialize(behavior_msgs.msg.dds.BehaviorTreeStateMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.BehaviorTreeStateMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.BehaviorTreeStateMessage src, behavior_msgs.msg.dds.BehaviorTreeStateMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public BehaviorTreeStateMessagePubSubType newInstance()
   {
      return new BehaviorTreeStateMessagePubSubType();
   }
}
