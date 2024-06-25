package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "ActionSequenceStateMessage" defined in "ActionSequenceStateMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from ActionSequenceStateMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit ActionSequenceStateMessage_.idl instead.
*
*/
public class ActionSequenceStateMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.ActionSequenceStateMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::ActionSequenceStateMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "96e6f4f8dfc5913fea89d1ef14c53553a77fbb86f9ea49d8318199b05c70a6a5";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.ActionSequenceStateMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.ActionSequenceStateMessage data) throws java.io.IOException
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

      current_alignment += behavior_msgs.msg.dds.BehaviorTreeNodeStateMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += behavior_msgs.msg.dds.ActionSequenceDefinitionMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.ActionSequenceStateMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.ActionSequenceStateMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += behavior_msgs.msg.dds.BehaviorTreeNodeStateMessagePubSubType.getCdrSerializedSize(data.getState(), current_alignment);

      current_alignment += behavior_msgs.msg.dds.ActionSequenceDefinitionMessagePubSubType.getCdrSerializedSize(data.getDefinition(), current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getNextActionRejectionTooltip().length() + 1;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.ActionSequenceStateMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.BehaviorTreeNodeStateMessagePubSubType.write(data.getState(), cdr);
      behavior_msgs.msg.dds.ActionSequenceDefinitionMessagePubSubType.write(data.getDefinition(), cdr);
      cdr.write_type_7(data.getAutomaticExecution());

      cdr.write_type_3(data.getExecutionNextIndex());

      if(data.getNextActionRejectionTooltip().length() <= 255)
      cdr.write_type_d(data.getNextActionRejectionTooltip());else
          throw new RuntimeException("next_action_rejection_tooltip field exceeds the maximum length");

      cdr.write_type_7(data.getManualExecutionRequested());

      cdr.write_type_7(data.getConcurrencyEnabled());

   }

   public static void read(behavior_msgs.msg.dds.ActionSequenceStateMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.BehaviorTreeNodeStateMessagePubSubType.read(data.getState(), cdr);	
      behavior_msgs.msg.dds.ActionSequenceDefinitionMessagePubSubType.read(data.getDefinition(), cdr);	
      data.setAutomaticExecution(cdr.read_type_7());
      	
      data.setExecutionNextIndex(cdr.read_type_3());
      	
      cdr.read_type_d(data.getNextActionRejectionTooltip());	
      data.setManualExecutionRequested(cdr.read_type_7());
      	
      data.setConcurrencyEnabled(cdr.read_type_7());
      	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.ActionSequenceStateMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("state", new behavior_msgs.msg.dds.BehaviorTreeNodeStateMessagePubSubType(), data.getState());

      ser.write_type_a("definition", new behavior_msgs.msg.dds.ActionSequenceDefinitionMessagePubSubType(), data.getDefinition());

      ser.write_type_7("automatic_execution", data.getAutomaticExecution());
      ser.write_type_3("execution_next_index", data.getExecutionNextIndex());
      ser.write_type_d("next_action_rejection_tooltip", data.getNextActionRejectionTooltip());
      ser.write_type_7("manual_execution_requested", data.getManualExecutionRequested());
      ser.write_type_7("concurrency_enabled", data.getConcurrencyEnabled());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.ActionSequenceStateMessage data)
   {
      ser.read_type_a("state", new behavior_msgs.msg.dds.BehaviorTreeNodeStateMessagePubSubType(), data.getState());

      ser.read_type_a("definition", new behavior_msgs.msg.dds.ActionSequenceDefinitionMessagePubSubType(), data.getDefinition());

      data.setAutomaticExecution(ser.read_type_7("automatic_execution"));
      data.setExecutionNextIndex(ser.read_type_3("execution_next_index"));
      ser.read_type_d("next_action_rejection_tooltip", data.getNextActionRejectionTooltip());
      data.setManualExecutionRequested(ser.read_type_7("manual_execution_requested"));
      data.setConcurrencyEnabled(ser.read_type_7("concurrency_enabled"));
   }

   public static void staticCopy(behavior_msgs.msg.dds.ActionSequenceStateMessage src, behavior_msgs.msg.dds.ActionSequenceStateMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.ActionSequenceStateMessage createData()
   {
      return new behavior_msgs.msg.dds.ActionSequenceStateMessage();
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
   
   public void serialize(behavior_msgs.msg.dds.ActionSequenceStateMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.ActionSequenceStateMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.ActionSequenceStateMessage src, behavior_msgs.msg.dds.ActionSequenceStateMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public ActionSequenceStateMessagePubSubType newInstance()
   {
      return new ActionSequenceStateMessagePubSubType();
   }
}
