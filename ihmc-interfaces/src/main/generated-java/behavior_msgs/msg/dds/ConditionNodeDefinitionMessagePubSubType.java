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
   		return "6b4baa424299a8564f6cc1157784c15a47c0b181699afdb95cf3f8fa55221a96";
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 10000 + 1;
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 10000 + 1;

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


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getSystem().length() + 1;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getPrompt().length() + 1;


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

      if(data.getSystem().length() <= 10000)
      cdr.write_type_d(data.getSystem());else
          throw new RuntimeException("system field exceeds the maximum length: %d > %d".formatted(data.getSystem().length(), 10000));

      if(data.getPrompt().length() <= 10000)
      cdr.write_type_d(data.getPrompt());else
          throw new RuntimeException("prompt field exceeds the maximum length: %d > %d".formatted(data.getPrompt().length(), 10000));

   }

   public static void read(behavior_msgs.msg.dds.ConditionNodeDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.LeafNodeDefinitionMessagePubSubType.read(data.getDefinition(), cdr);	
      data.setType(cdr.read_type_9());
      	
      data.setCountTo(cdr.read_type_4());
      	
      data.setResetContextEachRun(cdr.read_type_7());
      	
      data.setInjectBehaviorState(cdr.read_type_7());
      	
      data.setInjectEnvironmentState(cdr.read_type_7());
      	
      cdr.read_type_d(data.getSystem());	
      cdr.read_type_d(data.getPrompt());	

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
      ser.write_type_d("system", data.getSystem());
      ser.write_type_d("prompt", data.getPrompt());
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
      ser.read_type_d("system", data.getSystem());
      ser.read_type_d("prompt", data.getPrompt());
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
