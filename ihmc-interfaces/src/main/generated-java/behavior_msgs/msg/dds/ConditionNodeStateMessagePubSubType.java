package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "ConditionNodeStateMessage" defined in "ConditionNodeStateMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from ConditionNodeStateMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit ConditionNodeStateMessage_.idl instead.
*
*/
public class ConditionNodeStateMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.ConditionNodeStateMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::ConditionNodeStateMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "1adcebcc2246ed74767816afd3a958e7341f68dac6f48ed4d0791976acb98964";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.ConditionNodeStateMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.ConditionNodeStateMessage data) throws java.io.IOException
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

      current_alignment += behavior_msgs.msg.dds.LeafNodeStateMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += behavior_msgs.msg.dds.ConditionNodeDefinitionMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.ConditionNodeStateMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.ConditionNodeStateMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += behavior_msgs.msg.dds.LeafNodeStateMessagePubSubType.getCdrSerializedSize(data.getState(), current_alignment);

      current_alignment += behavior_msgs.msg.dds.ConditionNodeDefinitionMessagePubSubType.getCdrSerializedSize(data.getDefinition(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.ConditionNodeStateMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.LeafNodeStateMessagePubSubType.write(data.getState(), cdr);
      behavior_msgs.msg.dds.ConditionNodeDefinitionMessagePubSubType.write(data.getDefinition(), cdr);
      cdr.write_type_4(data.getCount());

      cdr.write_type_7(data.getRequestResetContext());

      cdr.write_type_6(data.getCurrentDistance());

   }

   public static void read(behavior_msgs.msg.dds.ConditionNodeStateMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.LeafNodeStateMessagePubSubType.read(data.getState(), cdr);	
      behavior_msgs.msg.dds.ConditionNodeDefinitionMessagePubSubType.read(data.getDefinition(), cdr);	
      data.setCount(cdr.read_type_4());
      	
      data.setRequestResetContext(cdr.read_type_7());
      	
      data.setCurrentDistance(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.ConditionNodeStateMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("state", new behavior_msgs.msg.dds.LeafNodeStateMessagePubSubType(), data.getState());

      ser.write_type_a("definition", new behavior_msgs.msg.dds.ConditionNodeDefinitionMessagePubSubType(), data.getDefinition());

      ser.write_type_4("count", data.getCount());
      ser.write_type_7("request_reset_context", data.getRequestResetContext());
      ser.write_type_6("current_distance", data.getCurrentDistance());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.ConditionNodeStateMessage data)
   {
      ser.read_type_a("state", new behavior_msgs.msg.dds.LeafNodeStateMessagePubSubType(), data.getState());

      ser.read_type_a("definition", new behavior_msgs.msg.dds.ConditionNodeDefinitionMessagePubSubType(), data.getDefinition());

      data.setCount(ser.read_type_4("count"));
      data.setRequestResetContext(ser.read_type_7("request_reset_context"));
      data.setCurrentDistance(ser.read_type_6("current_distance"));
   }

   public static void staticCopy(behavior_msgs.msg.dds.ConditionNodeStateMessage src, behavior_msgs.msg.dds.ConditionNodeStateMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.ConditionNodeStateMessage createData()
   {
      return new behavior_msgs.msg.dds.ConditionNodeStateMessage();
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
   
   public void serialize(behavior_msgs.msg.dds.ConditionNodeStateMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.ConditionNodeStateMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.ConditionNodeStateMessage src, behavior_msgs.msg.dds.ConditionNodeStateMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public ConditionNodeStateMessagePubSubType newInstance()
   {
      return new ConditionNodeStateMessagePubSubType();
   }
}
