package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "ConditionNodeDefinitionMessage" defined in "ConditionNodeDefinitionMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from ConditionNodeDefinitionMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit ConditionNodeDefinitionMessage_.idl instead.
*
*/
public class ConditionNodeDefinitionMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.ConditionNodeDefinitionMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::ConditionNodeDefinitionMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "5d7b0485e865119702ccc343d269d3e670361a286056a68d8bba9f9c3e3349fc";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.ConditionNodeDefinitionMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.ConditionNodeDefinitionMessage data) throws java.io.IOException
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

      current_alignment += behavior_msgs.msg.dds.LeafNodeDefinitionMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 10000 + 1;
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 10000 + 1;
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 10000 + 1;
      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.ConditionNodeDefinitionMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.ConditionNodeDefinitionMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += behavior_msgs.msg.dds.LeafNodeDefinitionMessagePubSubType.getCdrSerializedSize(data.getDefinition(), current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getSystem().length() + 1;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getPrompt().length() + 1;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getResponseMatcher().length() + 1;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getObjectFrameName().length() + 1;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getReferenceFrameName().length() + 1;

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.ConditionNodeDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.LeafNodeDefinitionMessagePubSubType.write(data.getDefinition(), cdr);
      cdr.write_type_9(data.getType());

      cdr.write_type_4(data.getCountTo());

      cdr.write_type_7(data.getResetContextEachRun());

      cdr.write_type_7(data.getInjectBehaviorState());

      cdr.write_type_7(data.getInjectEnvironmentState());

      cdr.write_type_7(data.getMatchIsSuccess());

      if(data.getSystem().length() <= 10000)
      cdr.write_type_d(data.getSystem());else
          throw new RuntimeException("system field exceeds the maximum length: %d > %d".formatted(data.getSystem().length(), 10000));

      if(data.getPrompt().length() <= 10000)
      cdr.write_type_d(data.getPrompt());else
          throw new RuntimeException("prompt field exceeds the maximum length: %d > %d".formatted(data.getPrompt().length(), 10000));

      if(data.getResponseMatcher().length() <= 10000)
      cdr.write_type_d(data.getResponseMatcher());else
          throw new RuntimeException("response_matcher field exceeds the maximum length: %d > %d".formatted(data.getResponseMatcher().length(), 10000));

      cdr.write_type_9(data.getDistanceType());

      if(data.getObjectFrameName().length() <= 255)
      cdr.write_type_d(data.getObjectFrameName());else
          throw new RuntimeException("object_frame_name field exceeds the maximum length: %d > %d".formatted(data.getObjectFrameName().length(), 255));

      if(data.getReferenceFrameName().length() <= 255)
      cdr.write_type_d(data.getReferenceFrameName());else
          throw new RuntimeException("reference_frame_name field exceeds the maximum length: %d > %d".formatted(data.getReferenceFrameName().length(), 255));

      cdr.write_type_6(data.getDistanceToObject());

      cdr.write_type_6(data.getEvaluationTime());

      cdr.write_type_7(data.getManageMissingFrameInternally());

   }

   public static void read(behavior_msgs.msg.dds.ConditionNodeDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.LeafNodeDefinitionMessagePubSubType.read(data.getDefinition(), cdr);	
      data.setType(cdr.read_type_9());
      	
      data.setCountTo(cdr.read_type_4());
      	
      data.setResetContextEachRun(cdr.read_type_7());
      	
      data.setInjectBehaviorState(cdr.read_type_7());
      	
      data.setInjectEnvironmentState(cdr.read_type_7());
      	
      data.setMatchIsSuccess(cdr.read_type_7());
      	
      cdr.read_type_d(data.getSystem());	
      cdr.read_type_d(data.getPrompt());	
      cdr.read_type_d(data.getResponseMatcher());	
      data.setDistanceType(cdr.read_type_9());
      	
      cdr.read_type_d(data.getObjectFrameName());	
      cdr.read_type_d(data.getReferenceFrameName());	
      data.setDistanceToObject(cdr.read_type_6());
      	
      data.setEvaluationTime(cdr.read_type_6());
      	
      data.setManageMissingFrameInternally(cdr.read_type_7());
      	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.ConditionNodeDefinitionMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("definition", new behavior_msgs.msg.dds.LeafNodeDefinitionMessagePubSubType(), data.getDefinition());

      ser.write_type_9("type", data.getType());
      ser.write_type_4("count_to", data.getCountTo());
      ser.write_type_7("reset_context_each_run", data.getResetContextEachRun());
      ser.write_type_7("inject_behavior_state", data.getInjectBehaviorState());
      ser.write_type_7("inject_environment_state", data.getInjectEnvironmentState());
      ser.write_type_7("match_is_success", data.getMatchIsSuccess());
      ser.write_type_d("system", data.getSystem());
      ser.write_type_d("prompt", data.getPrompt());
      ser.write_type_d("response_matcher", data.getResponseMatcher());
      ser.write_type_9("distance_type", data.getDistanceType());
      ser.write_type_d("object_frame_name", data.getObjectFrameName());
      ser.write_type_d("reference_frame_name", data.getReferenceFrameName());
      ser.write_type_6("distance_to_object", data.getDistanceToObject());
      ser.write_type_6("evaluation_time", data.getEvaluationTime());
      ser.write_type_7("manage_missing_frame_internally", data.getManageMissingFrameInternally());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.ConditionNodeDefinitionMessage data)
   {
      ser.read_type_a("definition", new behavior_msgs.msg.dds.LeafNodeDefinitionMessagePubSubType(), data.getDefinition());

      data.setType(ser.read_type_9("type"));
      data.setCountTo(ser.read_type_4("count_to"));
      data.setResetContextEachRun(ser.read_type_7("reset_context_each_run"));
      data.setInjectBehaviorState(ser.read_type_7("inject_behavior_state"));
      data.setInjectEnvironmentState(ser.read_type_7("inject_environment_state"));
      data.setMatchIsSuccess(ser.read_type_7("match_is_success"));
      ser.read_type_d("system", data.getSystem());
      ser.read_type_d("prompt", data.getPrompt());
      ser.read_type_d("response_matcher", data.getResponseMatcher());
      data.setDistanceType(ser.read_type_9("distance_type"));
      ser.read_type_d("object_frame_name", data.getObjectFrameName());
      ser.read_type_d("reference_frame_name", data.getReferenceFrameName());
      data.setDistanceToObject(ser.read_type_6("distance_to_object"));
      data.setEvaluationTime(ser.read_type_6("evaluation_time"));
      data.setManageMissingFrameInternally(ser.read_type_7("manage_missing_frame_internally"));
   }

   public static void staticCopy(behavior_msgs.msg.dds.ConditionNodeDefinitionMessage src, behavior_msgs.msg.dds.ConditionNodeDefinitionMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.ConditionNodeDefinitionMessage createData()
   {
      return new behavior_msgs.msg.dds.ConditionNodeDefinitionMessage();
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
   
   public void serialize(behavior_msgs.msg.dds.ConditionNodeDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.ConditionNodeDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.ConditionNodeDefinitionMessage src, behavior_msgs.msg.dds.ConditionNodeDefinitionMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public ConditionNodeDefinitionMessagePubSubType newInstance()
   {
      return new ConditionNodeDefinitionMessagePubSubType();
   }
}
