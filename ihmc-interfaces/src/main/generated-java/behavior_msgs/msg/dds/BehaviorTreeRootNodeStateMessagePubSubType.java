package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "BehaviorTreeRootNodeStateMessage" defined in "BehaviorTreeRootNodeStateMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from BehaviorTreeRootNodeStateMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit BehaviorTreeRootNodeStateMessage_.idl instead.
*
*/
public class BehaviorTreeRootNodeStateMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.BehaviorTreeRootNodeStateMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::BehaviorTreeRootNodeStateMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "7c3af5f3655cf2664cf15fb46508f1ca7f43426db8c3b4705ef6a7de5951fa13";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.BehaviorTreeRootNodeStateMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.BehaviorTreeRootNodeStateMessage data) throws java.io.IOException
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

      current_alignment += behavior_msgs.msg.dds.BehaviorTreeRootNodeDefinitionMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.BehaviorTreeRootNodeStateMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.BehaviorTreeRootNodeStateMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += behavior_msgs.msg.dds.BehaviorTreeNodeStateMessagePubSubType.getCdrSerializedSize(data.getState(), current_alignment);

      current_alignment += behavior_msgs.msg.dds.BehaviorTreeRootNodeDefinitionMessagePubSubType.getCdrSerializedSize(data.getDefinition(), current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getNextActionRejectionTooltip().length() + 1;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.BehaviorTreeRootNodeStateMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.BehaviorTreeNodeStateMessagePubSubType.write(data.getState(), cdr);
      behavior_msgs.msg.dds.BehaviorTreeRootNodeDefinitionMessagePubSubType.write(data.getDefinition(), cdr);
      cdr.write_type_7(data.getAutomaticExecution());

      cdr.write_type_3(data.getExecutionNextIndex());

      if(data.getNextActionRejectionTooltip().length() <= 255)
      cdr.write_type_d(data.getNextActionRejectionTooltip());else
          throw new RuntimeException("next_action_rejection_tooltip field exceeds the maximum length");

      cdr.write_type_7(data.getManualExecutionRequested());

      cdr.write_type_7(data.getConcurrencyEnabled());

   }

   public static void read(behavior_msgs.msg.dds.BehaviorTreeRootNodeStateMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.BehaviorTreeNodeStateMessagePubSubType.read(data.getState(), cdr);	
      behavior_msgs.msg.dds.BehaviorTreeRootNodeDefinitionMessagePubSubType.read(data.getDefinition(), cdr);	
      data.setAutomaticExecution(cdr.read_type_7());
      	
      data.setExecutionNextIndex(cdr.read_type_3());
      	
      cdr.read_type_d(data.getNextActionRejectionTooltip());	
      data.setManualExecutionRequested(cdr.read_type_7());
      	
      data.setConcurrencyEnabled(cdr.read_type_7());
      	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.BehaviorTreeRootNodeStateMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("state", new behavior_msgs.msg.dds.BehaviorTreeNodeStateMessagePubSubType(), data.getState());

      ser.write_type_a("definition", new behavior_msgs.msg.dds.BehaviorTreeRootNodeDefinitionMessagePubSubType(), data.getDefinition());

      ser.write_type_7("automatic_execution", data.getAutomaticExecution());
      ser.write_type_3("execution_next_index", data.getExecutionNextIndex());
      ser.write_type_d("next_action_rejection_tooltip", data.getNextActionRejectionTooltip());
      ser.write_type_7("manual_execution_requested", data.getManualExecutionRequested());
      ser.write_type_7("concurrency_enabled", data.getConcurrencyEnabled());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.BehaviorTreeRootNodeStateMessage data)
   {
      ser.read_type_a("state", new behavior_msgs.msg.dds.BehaviorTreeNodeStateMessagePubSubType(), data.getState());

      ser.read_type_a("definition", new behavior_msgs.msg.dds.BehaviorTreeRootNodeDefinitionMessagePubSubType(), data.getDefinition());

      data.setAutomaticExecution(ser.read_type_7("automatic_execution"));
      data.setExecutionNextIndex(ser.read_type_3("execution_next_index"));
      ser.read_type_d("next_action_rejection_tooltip", data.getNextActionRejectionTooltip());
      data.setManualExecutionRequested(ser.read_type_7("manual_execution_requested"));
      data.setConcurrencyEnabled(ser.read_type_7("concurrency_enabled"));
   }

   public static void staticCopy(behavior_msgs.msg.dds.BehaviorTreeRootNodeStateMessage src, behavior_msgs.msg.dds.BehaviorTreeRootNodeStateMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.BehaviorTreeRootNodeStateMessage createData()
   {
      return new behavior_msgs.msg.dds.BehaviorTreeRootNodeStateMessage();
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
   
   public void serialize(behavior_msgs.msg.dds.BehaviorTreeRootNodeStateMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.BehaviorTreeRootNodeStateMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.BehaviorTreeRootNodeStateMessage src, behavior_msgs.msg.dds.BehaviorTreeRootNodeStateMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public BehaviorTreeRootNodeStateMessagePubSubType newInstance()
   {
      return new BehaviorTreeRootNodeStateMessagePubSubType();
   }
}
