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
   		return "c0afee9109ac3ff2ca3bf5c9f24aaec083030cdfd0f2feff5dfed43d05f5403d";
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += ihmc_common_msgs.msg.dds.ConfirmableRequestMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (1000 * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (1000 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 200; ++i0)
      {
          current_alignment += behavior_msgs.msg.dds.BasicNodeStateMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 200; ++i0)
      {
          current_alignment += behavior_msgs.msg.dds.ActionSequenceStateMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 200; ++i0)
      {
          current_alignment += behavior_msgs.msg.dds.ArmJointAnglesActionStateMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 200; ++i0)
      {
          current_alignment += behavior_msgs.msg.dds.ChestOrientationActionStateMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 200; ++i0)
      {
          current_alignment += behavior_msgs.msg.dds.FootstepPlanActionStateMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 200; ++i0)
      {
          current_alignment += behavior_msgs.msg.dds.SakeHandCommandActionStateMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 200; ++i0)
      {
          current_alignment += behavior_msgs.msg.dds.HandPoseActionStateMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 200; ++i0)
      {
          current_alignment += behavior_msgs.msg.dds.HandWrenchActionStateMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 200; ++i0)
      {
          current_alignment += behavior_msgs.msg.dds.PelvisHeightPitchActionStateMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 200; ++i0)
      {
          current_alignment += behavior_msgs.msg.dds.WaitDurationActionStateMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 200; ++i0)
      {
          current_alignment += behavior_msgs.msg.dds.WalkActionStateMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.BehaviorTreeStateMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.BehaviorTreeStateMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += ihmc_common_msgs.msg.dds.ConfirmableRequestMessagePubSubType.getCdrSerializedSize(data.getConfirmableRequest(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getBehaviorTreeTypes().size() * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getBehaviorTreeIndices().size() * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getBasicNodes().size(); ++i0)
      {
          current_alignment += behavior_msgs.msg.dds.BasicNodeStateMessagePubSubType.getCdrSerializedSize(data.getBasicNodes().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getActionSequences().size(); ++i0)
      {
          current_alignment += behavior_msgs.msg.dds.ActionSequenceStateMessagePubSubType.getCdrSerializedSize(data.getActionSequences().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getArmJointAnglesActions().size(); ++i0)
      {
          current_alignment += behavior_msgs.msg.dds.ArmJointAnglesActionStateMessagePubSubType.getCdrSerializedSize(data.getArmJointAnglesActions().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getChestOrientationActions().size(); ++i0)
      {
          current_alignment += behavior_msgs.msg.dds.ChestOrientationActionStateMessagePubSubType.getCdrSerializedSize(data.getChestOrientationActions().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getFootstepPlanActions().size(); ++i0)
      {
          current_alignment += behavior_msgs.msg.dds.FootstepPlanActionStateMessagePubSubType.getCdrSerializedSize(data.getFootstepPlanActions().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getSakeHandCommandActions().size(); ++i0)
      {
          current_alignment += behavior_msgs.msg.dds.SakeHandCommandActionStateMessagePubSubType.getCdrSerializedSize(data.getSakeHandCommandActions().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getHandPoseActions().size(); ++i0)
      {
          current_alignment += behavior_msgs.msg.dds.HandPoseActionStateMessagePubSubType.getCdrSerializedSize(data.getHandPoseActions().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getHandWrenchActions().size(); ++i0)
      {
          current_alignment += behavior_msgs.msg.dds.HandWrenchActionStateMessagePubSubType.getCdrSerializedSize(data.getHandWrenchActions().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getPelvisHeightActions().size(); ++i0)
      {
          current_alignment += behavior_msgs.msg.dds.PelvisHeightPitchActionStateMessagePubSubType.getCdrSerializedSize(data.getPelvisHeightActions().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getWaitDurationActions().size(); ++i0)
      {
          current_alignment += behavior_msgs.msg.dds.WaitDurationActionStateMessagePubSubType.getCdrSerializedSize(data.getWaitDurationActions().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getWalkActions().size(); ++i0)
      {
          current_alignment += behavior_msgs.msg.dds.WalkActionStateMessagePubSubType.getCdrSerializedSize(data.getWalkActions().get(i0), current_alignment);}


      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.BehaviorTreeStateMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getNextId());

      ihmc_common_msgs.msg.dds.ConfirmableRequestMessagePubSubType.write(data.getConfirmableRequest(), cdr);
      if(data.getBehaviorTreeTypes().size() <= 1000)
      cdr.write_type_e(data.getBehaviorTreeTypes());else
          throw new RuntimeException("behavior_tree_types field exceeds the maximum length");

      if(data.getBehaviorTreeIndices().size() <= 1000)
      cdr.write_type_e(data.getBehaviorTreeIndices());else
          throw new RuntimeException("behavior_tree_indices field exceeds the maximum length");

      if(data.getBasicNodes().size() <= 200)
      cdr.write_type_e(data.getBasicNodes());else
          throw new RuntimeException("basic_nodes field exceeds the maximum length");

      if(data.getActionSequences().size() <= 200)
      cdr.write_type_e(data.getActionSequences());else
          throw new RuntimeException("action_sequences field exceeds the maximum length");

      if(data.getArmJointAnglesActions().size() <= 200)
      cdr.write_type_e(data.getArmJointAnglesActions());else
          throw new RuntimeException("arm_joint_angles_actions field exceeds the maximum length");

      if(data.getChestOrientationActions().size() <= 200)
      cdr.write_type_e(data.getChestOrientationActions());else
          throw new RuntimeException("chest_orientation_actions field exceeds the maximum length");

      if(data.getFootstepPlanActions().size() <= 200)
      cdr.write_type_e(data.getFootstepPlanActions());else
          throw new RuntimeException("footstep_plan_actions field exceeds the maximum length");

      if(data.getSakeHandCommandActions().size() <= 200)
      cdr.write_type_e(data.getSakeHandCommandActions());else
          throw new RuntimeException("sake_hand_command_actions field exceeds the maximum length");

      if(data.getHandPoseActions().size() <= 200)
      cdr.write_type_e(data.getHandPoseActions());else
          throw new RuntimeException("hand_pose_actions field exceeds the maximum length");

      if(data.getHandWrenchActions().size() <= 200)
      cdr.write_type_e(data.getHandWrenchActions());else
          throw new RuntimeException("hand_wrench_actions field exceeds the maximum length");

      if(data.getPelvisHeightActions().size() <= 200)
      cdr.write_type_e(data.getPelvisHeightActions());else
          throw new RuntimeException("pelvis_height_actions field exceeds the maximum length");

      if(data.getWaitDurationActions().size() <= 200)
      cdr.write_type_e(data.getWaitDurationActions());else
          throw new RuntimeException("wait_duration_actions field exceeds the maximum length");

      if(data.getWalkActions().size() <= 200)
      cdr.write_type_e(data.getWalkActions());else
          throw new RuntimeException("walk_actions field exceeds the maximum length");

   }

   public static void read(behavior_msgs.msg.dds.BehaviorTreeStateMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setNextId(cdr.read_type_4());
      	
      ihmc_common_msgs.msg.dds.ConfirmableRequestMessagePubSubType.read(data.getConfirmableRequest(), cdr);	
      cdr.read_type_e(data.getBehaviorTreeTypes());	
      cdr.read_type_e(data.getBehaviorTreeIndices());	
      cdr.read_type_e(data.getBasicNodes());	
      cdr.read_type_e(data.getActionSequences());	
      cdr.read_type_e(data.getArmJointAnglesActions());	
      cdr.read_type_e(data.getChestOrientationActions());	
      cdr.read_type_e(data.getFootstepPlanActions());	
      cdr.read_type_e(data.getSakeHandCommandActions());	
      cdr.read_type_e(data.getHandPoseActions());	
      cdr.read_type_e(data.getHandWrenchActions());	
      cdr.read_type_e(data.getPelvisHeightActions());	
      cdr.read_type_e(data.getWaitDurationActions());	
      cdr.read_type_e(data.getWalkActions());	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.BehaviorTreeStateMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("next_id", data.getNextId());
      ser.write_type_a("confirmable_request", new ihmc_common_msgs.msg.dds.ConfirmableRequestMessagePubSubType(), data.getConfirmableRequest());

      ser.write_type_e("behavior_tree_types", data.getBehaviorTreeTypes());
      ser.write_type_e("behavior_tree_indices", data.getBehaviorTreeIndices());
      ser.write_type_e("basic_nodes", data.getBasicNodes());
      ser.write_type_e("action_sequences", data.getActionSequences());
      ser.write_type_e("arm_joint_angles_actions", data.getArmJointAnglesActions());
      ser.write_type_e("chest_orientation_actions", data.getChestOrientationActions());
      ser.write_type_e("footstep_plan_actions", data.getFootstepPlanActions());
      ser.write_type_e("sake_hand_command_actions", data.getSakeHandCommandActions());
      ser.write_type_e("hand_pose_actions", data.getHandPoseActions());
      ser.write_type_e("hand_wrench_actions", data.getHandWrenchActions());
      ser.write_type_e("pelvis_height_actions", data.getPelvisHeightActions());
      ser.write_type_e("wait_duration_actions", data.getWaitDurationActions());
      ser.write_type_e("walk_actions", data.getWalkActions());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.BehaviorTreeStateMessage data)
   {
      data.setNextId(ser.read_type_4("next_id"));
      ser.read_type_a("confirmable_request", new ihmc_common_msgs.msg.dds.ConfirmableRequestMessagePubSubType(), data.getConfirmableRequest());

      ser.read_type_e("behavior_tree_types", data.getBehaviorTreeTypes());
      ser.read_type_e("behavior_tree_indices", data.getBehaviorTreeIndices());
      ser.read_type_e("basic_nodes", data.getBasicNodes());
      ser.read_type_e("action_sequences", data.getActionSequences());
      ser.read_type_e("arm_joint_angles_actions", data.getArmJointAnglesActions());
      ser.read_type_e("chest_orientation_actions", data.getChestOrientationActions());
      ser.read_type_e("footstep_plan_actions", data.getFootstepPlanActions());
      ser.read_type_e("sake_hand_command_actions", data.getSakeHandCommandActions());
      ser.read_type_e("hand_pose_actions", data.getHandPoseActions());
      ser.read_type_e("hand_wrench_actions", data.getHandWrenchActions());
      ser.read_type_e("pelvis_height_actions", data.getPelvisHeightActions());
      ser.read_type_e("wait_duration_actions", data.getWaitDurationActions());
      ser.read_type_e("walk_actions", data.getWalkActions());
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
