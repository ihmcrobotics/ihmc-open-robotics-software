package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "BehaviorActionStateMessage" defined in "BehaviorActionStateMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from BehaviorActionStateMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit BehaviorActionStateMessage_.idl instead.
*
*/
public class BehaviorActionStateMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.BehaviorActionStateMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::BehaviorActionStateMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "6bce6bbe6b27e8527841b0c258597653bda2584a5dc909c4254ea52f4f5fa6c9";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.BehaviorActionStateMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.BehaviorActionStateMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.BehaviorActionStateMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.BehaviorActionStateMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += behavior_msgs.msg.dds.BehaviorTreeNodeStateMessagePubSubType.getCdrSerializedSize(data.getNodeState(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.BehaviorActionStateMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.BehaviorTreeNodeStateMessagePubSubType.write(data.getNodeState(), cdr);
      cdr.write_type_4(data.getId());

      cdr.write_type_2(data.getActionIndex());

      cdr.write_type_7(data.getIsNextForExecution());

      cdr.write_type_7(data.getIsToBeExecutedConcurrently());

   }

   public static void read(behavior_msgs.msg.dds.BehaviorActionStateMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.BehaviorTreeNodeStateMessagePubSubType.read(data.getNodeState(), cdr);	
      data.setId(cdr.read_type_4());
      	
      data.setActionIndex(cdr.read_type_2());
      	
      data.setIsNextForExecution(cdr.read_type_7());
      	
      data.setIsToBeExecutedConcurrently(cdr.read_type_7());
      	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.BehaviorActionStateMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("node_state", new behavior_msgs.msg.dds.BehaviorTreeNodeStateMessagePubSubType(), data.getNodeState());

      ser.write_type_4("id", data.getId());
      ser.write_type_2("action_index", data.getActionIndex());
      ser.write_type_7("is_next_for_execution", data.getIsNextForExecution());
      ser.write_type_7("is_to_be_executed_concurrently", data.getIsToBeExecutedConcurrently());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.BehaviorActionStateMessage data)
   {
      ser.read_type_a("node_state", new behavior_msgs.msg.dds.BehaviorTreeNodeStateMessagePubSubType(), data.getNodeState());

      data.setId(ser.read_type_4("id"));
      data.setActionIndex(ser.read_type_2("action_index"));
      data.setIsNextForExecution(ser.read_type_7("is_next_for_execution"));
      data.setIsToBeExecutedConcurrently(ser.read_type_7("is_to_be_executed_concurrently"));
   }

   public static void staticCopy(behavior_msgs.msg.dds.BehaviorActionStateMessage src, behavior_msgs.msg.dds.BehaviorActionStateMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.BehaviorActionStateMessage createData()
   {
      return new behavior_msgs.msg.dds.BehaviorActionStateMessage();
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
   
   public void serialize(behavior_msgs.msg.dds.BehaviorActionStateMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.BehaviorActionStateMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.BehaviorActionStateMessage src, behavior_msgs.msg.dds.BehaviorActionStateMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public BehaviorActionStateMessagePubSubType newInstance()
   {
      return new BehaviorActionStateMessagePubSubType();
   }
}
