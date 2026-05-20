package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "AI2RNodeDefinitionMessage" defined in "AI2RNodeDefinitionMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from AI2RNodeDefinitionMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit AI2RNodeDefinitionMessage_.idl instead.
*
*/
public class AI2RNodeDefinitionMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.AI2RNodeDefinitionMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::AI2RNodeDefinitionMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "566162022a0b6b0f5854e54523e2ee0a44c81d32602fab6bbf652b7bdfdfc35a";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.AI2RNodeDefinitionMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.AI2RNodeDefinitionMessage data) throws java.io.IOException
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

      current_alignment += behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.AI2RNodeDefinitionMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.AI2RNodeDefinitionMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessagePubSubType.getCdrSerializedSize(data.getDefinition(), current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.getCdrSerializedSize(data.getRightHandTransformToChest(), current_alignment);

      current_alignment += controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.getCdrSerializedSize(data.getLeftHandTransformToChest(), current_alignment);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.getCdrSerializedSize(data.getPelvisTransformToParent(), current_alignment);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.AI2RNodeDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessagePubSubType.write(data.getDefinition(), cdr);
      cdr.write_type_7(data.getRandomizeGoToAction());

      cdr.write_type_7(data.getRandomizeWholeBodyAction());

      cdr.write_type_2(data.getNumberOfRandomizations());

      cdr.write_type_2(data.getWholeBodyRandomizationRequestId());

      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.write(data.getRightHandTransformToChest(), cdr);
      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.write(data.getLeftHandTransformToChest(), cdr);
      cdr.write_type_6(data.getSpinePitchDegrees());

      cdr.write_type_6(data.getSpineRollDegrees());

      cdr.write_type_6(data.getSpineYawDegrees());

      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.write(data.getPelvisTransformToParent(), cdr);
      cdr.write_type_6(data.getRightHandTrajectoryDuration());

      cdr.write_type_6(data.getLeftHandTrajectoryDuration());

      cdr.write_type_6(data.getSpineTrajectoryDuration());

      cdr.write_type_6(data.getPelvisTrajectoryDuration());

      cdr.write_type_2(data.getLeftHandMask());

      cdr.write_type_2(data.getRightHandMask());

      cdr.write_type_2(data.getSpineMask());

      cdr.write_type_2(data.getPelvisMask());

      cdr.write_type_6(data.getProbabilityOneEnabled());

      cdr.write_type_6(data.getProbabilityTwoEnabled());

      cdr.write_type_6(data.getProbabilityThreeEnabled());

      cdr.write_type_6(data.getProbabilityFourEnabled());

   }

   public static void read(behavior_msgs.msg.dds.AI2RNodeDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessagePubSubType.read(data.getDefinition(), cdr);	
      data.setRandomizeGoToAction(cdr.read_type_7());
      	
      data.setRandomizeWholeBodyAction(cdr.read_type_7());
      	
      data.setNumberOfRandomizations(cdr.read_type_2());
      	
      data.setWholeBodyRandomizationRequestId(cdr.read_type_2());
      	
      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.read(data.getRightHandTransformToChest(), cdr);	
      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.read(data.getLeftHandTransformToChest(), cdr);	
      data.setSpinePitchDegrees(cdr.read_type_6());
      	
      data.setSpineRollDegrees(cdr.read_type_6());
      	
      data.setSpineYawDegrees(cdr.read_type_6());
      	
      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.read(data.getPelvisTransformToParent(), cdr);	
      data.setRightHandTrajectoryDuration(cdr.read_type_6());
      	
      data.setLeftHandTrajectoryDuration(cdr.read_type_6());
      	
      data.setSpineTrajectoryDuration(cdr.read_type_6());
      	
      data.setPelvisTrajectoryDuration(cdr.read_type_6());
      	
      data.setLeftHandMask(cdr.read_type_2());
      	
      data.setRightHandMask(cdr.read_type_2());
      	
      data.setSpineMask(cdr.read_type_2());
      	
      data.setPelvisMask(cdr.read_type_2());
      	
      data.setProbabilityOneEnabled(cdr.read_type_6());
      	
      data.setProbabilityTwoEnabled(cdr.read_type_6());
      	
      data.setProbabilityThreeEnabled(cdr.read_type_6());
      	
      data.setProbabilityFourEnabled(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.AI2RNodeDefinitionMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("definition", new behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessagePubSubType(), data.getDefinition());

      ser.write_type_7("randomize_go_to_action", data.getRandomizeGoToAction());
      ser.write_type_7("randomize_whole_body_action", data.getRandomizeWholeBodyAction());
      ser.write_type_2("number_of_randomizations", data.getNumberOfRandomizations());
      ser.write_type_2("whole_body_randomization_request_id", data.getWholeBodyRandomizationRequestId());
      ser.write_type_a("right_hand_transform_to_chest", new controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType(), data.getRightHandTransformToChest());

      ser.write_type_a("left_hand_transform_to_chest", new controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType(), data.getLeftHandTransformToChest());

      ser.write_type_6("spine_pitch_degrees", data.getSpinePitchDegrees());
      ser.write_type_6("spine_roll_degrees", data.getSpineRollDegrees());
      ser.write_type_6("spine_yaw_degrees", data.getSpineYawDegrees());
      ser.write_type_a("pelvis_transform_to_parent", new controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType(), data.getPelvisTransformToParent());

      ser.write_type_6("right_hand_trajectory_duration", data.getRightHandTrajectoryDuration());
      ser.write_type_6("left_hand_trajectory_duration", data.getLeftHandTrajectoryDuration());
      ser.write_type_6("spine_trajectory_duration", data.getSpineTrajectoryDuration());
      ser.write_type_6("pelvis_trajectory_duration", data.getPelvisTrajectoryDuration());
      ser.write_type_2("left_hand_mask", data.getLeftHandMask());
      ser.write_type_2("right_hand_mask", data.getRightHandMask());
      ser.write_type_2("spine_mask", data.getSpineMask());
      ser.write_type_2("pelvis_mask", data.getPelvisMask());
      ser.write_type_6("probability_one_enabled", data.getProbabilityOneEnabled());
      ser.write_type_6("probability_two_enabled", data.getProbabilityTwoEnabled());
      ser.write_type_6("probability_three_enabled", data.getProbabilityThreeEnabled());
      ser.write_type_6("probability_four_enabled", data.getProbabilityFourEnabled());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.AI2RNodeDefinitionMessage data)
   {
      ser.read_type_a("definition", new behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessagePubSubType(), data.getDefinition());

      data.setRandomizeGoToAction(ser.read_type_7("randomize_go_to_action"));
      data.setRandomizeWholeBodyAction(ser.read_type_7("randomize_whole_body_action"));
      data.setNumberOfRandomizations(ser.read_type_2("number_of_randomizations"));
      data.setWholeBodyRandomizationRequestId(ser.read_type_2("whole_body_randomization_request_id"));
      ser.read_type_a("right_hand_transform_to_chest", new controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType(), data.getRightHandTransformToChest());

      ser.read_type_a("left_hand_transform_to_chest", new controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType(), data.getLeftHandTransformToChest());

      data.setSpinePitchDegrees(ser.read_type_6("spine_pitch_degrees"));
      data.setSpineRollDegrees(ser.read_type_6("spine_roll_degrees"));
      data.setSpineYawDegrees(ser.read_type_6("spine_yaw_degrees"));
      ser.read_type_a("pelvis_transform_to_parent", new controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType(), data.getPelvisTransformToParent());

      data.setRightHandTrajectoryDuration(ser.read_type_6("right_hand_trajectory_duration"));
      data.setLeftHandTrajectoryDuration(ser.read_type_6("left_hand_trajectory_duration"));
      data.setSpineTrajectoryDuration(ser.read_type_6("spine_trajectory_duration"));
      data.setPelvisTrajectoryDuration(ser.read_type_6("pelvis_trajectory_duration"));
      data.setLeftHandMask(ser.read_type_2("left_hand_mask"));
      data.setRightHandMask(ser.read_type_2("right_hand_mask"));
      data.setSpineMask(ser.read_type_2("spine_mask"));
      data.setPelvisMask(ser.read_type_2("pelvis_mask"));
      data.setProbabilityOneEnabled(ser.read_type_6("probability_one_enabled"));
      data.setProbabilityTwoEnabled(ser.read_type_6("probability_two_enabled"));
      data.setProbabilityThreeEnabled(ser.read_type_6("probability_three_enabled"));
      data.setProbabilityFourEnabled(ser.read_type_6("probability_four_enabled"));
   }

   public static void staticCopy(behavior_msgs.msg.dds.AI2RNodeDefinitionMessage src, behavior_msgs.msg.dds.AI2RNodeDefinitionMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.AI2RNodeDefinitionMessage createData()
   {
      return new behavior_msgs.msg.dds.AI2RNodeDefinitionMessage();
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
   
   public void serialize(behavior_msgs.msg.dds.AI2RNodeDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.AI2RNodeDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.AI2RNodeDefinitionMessage src, behavior_msgs.msg.dds.AI2RNodeDefinitionMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public AI2RNodeDefinitionMessagePubSubType newInstance()
   {
      return new AI2RNodeDefinitionMessagePubSubType();
   }
}
