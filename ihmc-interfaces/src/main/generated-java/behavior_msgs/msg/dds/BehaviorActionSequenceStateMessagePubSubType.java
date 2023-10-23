package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "BehaviorActionSequenceStateMessage" defined in "BehaviorActionSequenceStateMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from BehaviorActionSequenceStateMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit BehaviorActionSequenceStateMessage_.idl instead.
*
*/
public class BehaviorActionSequenceStateMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.BehaviorActionSequenceStateMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::BehaviorActionSequenceStateMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "1546540266011b55269b68a2abe83ba4fe79e614444e96c2068cdf8ccd648b7b";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.BehaviorActionSequenceStateMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.BehaviorActionSequenceStateMessage data) throws java.io.IOException
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

      current_alignment += behavior_msgs.msg.dds.BehaviorActionSequenceDefinitionMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.BehaviorActionSequenceStateMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.BehaviorActionSequenceStateMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += behavior_msgs.msg.dds.BehaviorTreeNodeStateMessagePubSubType.getCdrSerializedSize(data.getState(), current_alignment);

      current_alignment += behavior_msgs.msg.dds.BehaviorActionSequenceDefinitionMessagePubSubType.getCdrSerializedSize(data.getDefinition(), current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);



      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.BehaviorActionSequenceStateMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.BehaviorTreeNodeStateMessagePubSubType.write(data.getState(), cdr);
      behavior_msgs.msg.dds.BehaviorActionSequenceDefinitionMessagePubSubType.write(data.getDefinition(), cdr);
      cdr.write_type_7(data.getAutomaticExecution());

      cdr.write_type_3(data.getExecutionNextIndex());

   }

   public static void read(behavior_msgs.msg.dds.BehaviorActionSequenceStateMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.BehaviorTreeNodeStateMessagePubSubType.read(data.getState(), cdr);	
      behavior_msgs.msg.dds.BehaviorActionSequenceDefinitionMessagePubSubType.read(data.getDefinition(), cdr);	
      data.setAutomaticExecution(cdr.read_type_7());
      	
      data.setExecutionNextIndex(cdr.read_type_3());
      	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.BehaviorActionSequenceStateMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("state", new behavior_msgs.msg.dds.BehaviorTreeNodeStateMessagePubSubType(), data.getState());

      ser.write_type_a("definition", new behavior_msgs.msg.dds.BehaviorActionSequenceDefinitionMessagePubSubType(), data.getDefinition());

      ser.write_type_7("automatic_execution", data.getAutomaticExecution());
      ser.write_type_3("execution_next_index", data.getExecutionNextIndex());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.BehaviorActionSequenceStateMessage data)
   {
      ser.read_type_a("state", new behavior_msgs.msg.dds.BehaviorTreeNodeStateMessagePubSubType(), data.getState());

      ser.read_type_a("definition", new behavior_msgs.msg.dds.BehaviorActionSequenceDefinitionMessagePubSubType(), data.getDefinition());

      data.setAutomaticExecution(ser.read_type_7("automatic_execution"));
      data.setExecutionNextIndex(ser.read_type_3("execution_next_index"));
   }

   public static void staticCopy(behavior_msgs.msg.dds.BehaviorActionSequenceStateMessage src, behavior_msgs.msg.dds.BehaviorActionSequenceStateMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.BehaviorActionSequenceStateMessage createData()
   {
      return new behavior_msgs.msg.dds.BehaviorActionSequenceStateMessage();
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
   
   public void serialize(behavior_msgs.msg.dds.BehaviorActionSequenceStateMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.BehaviorActionSequenceStateMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.BehaviorActionSequenceStateMessage src, behavior_msgs.msg.dds.BehaviorActionSequenceStateMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public BehaviorActionSequenceStateMessagePubSubType newInstance()
   {
      return new BehaviorActionSequenceStateMessagePubSubType();
   }
}
