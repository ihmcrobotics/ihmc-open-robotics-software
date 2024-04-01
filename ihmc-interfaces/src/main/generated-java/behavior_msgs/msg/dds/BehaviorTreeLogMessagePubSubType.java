package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "BehaviorTreeLogMessage" defined in "BehaviorTreeLogMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from BehaviorTreeLogMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit BehaviorTreeLogMessage_.idl instead.
*
*/
public class BehaviorTreeLogMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.BehaviorTreeLogMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::BehaviorTreeLogMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "3ae07b14debcf097d1151bba2ec109c9643c969d561c807d5744efd5eac193fa";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.BehaviorTreeLogMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.BehaviorTreeLogMessage data) throws java.io.IOException
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

      current_alignment += ihmc_common_msgs.msg.dds.InstantMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);

      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.BehaviorTreeLogMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.BehaviorTreeLogMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += ihmc_common_msgs.msg.dds.InstantMessagePubSubType.getCdrSerializedSize(data.getInstant(), current_alignment);

      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);


      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getMessage().length() + 1;


      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.BehaviorTreeLogMessage data, us.ihmc.idl.CDR cdr)
   {
      ihmc_common_msgs.msg.dds.InstantMessagePubSubType.write(data.getInstant(), cdr);
      cdr.write_type_3(data.getLogLevel());

      cdr.write_type_3(data.getSourceNodeId());

      if(data.getMessage().length() <= 255)
      cdr.write_type_d(data.getMessage());else
          throw new RuntimeException("message field exceeds the maximum length");

   }

   public static void read(behavior_msgs.msg.dds.BehaviorTreeLogMessage data, us.ihmc.idl.CDR cdr)
   {
      ihmc_common_msgs.msg.dds.InstantMessagePubSubType.read(data.getInstant(), cdr);	
      data.setLogLevel(cdr.read_type_3());
      	
      data.setSourceNodeId(cdr.read_type_3());
      	
      cdr.read_type_d(data.getMessage());	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.BehaviorTreeLogMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("instant", new ihmc_common_msgs.msg.dds.InstantMessagePubSubType(), data.getInstant());

      ser.write_type_3("log_level", data.getLogLevel());
      ser.write_type_3("source_node_id", data.getSourceNodeId());
      ser.write_type_d("message", data.getMessage());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.BehaviorTreeLogMessage data)
   {
      ser.read_type_a("instant", new ihmc_common_msgs.msg.dds.InstantMessagePubSubType(), data.getInstant());

      data.setLogLevel(ser.read_type_3("log_level"));
      data.setSourceNodeId(ser.read_type_3("source_node_id"));
      ser.read_type_d("message", data.getMessage());
   }

   public static void staticCopy(behavior_msgs.msg.dds.BehaviorTreeLogMessage src, behavior_msgs.msg.dds.BehaviorTreeLogMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.BehaviorTreeLogMessage createData()
   {
      return new behavior_msgs.msg.dds.BehaviorTreeLogMessage();
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
   
   public void serialize(behavior_msgs.msg.dds.BehaviorTreeLogMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.BehaviorTreeLogMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.BehaviorTreeLogMessage src, behavior_msgs.msg.dds.BehaviorTreeLogMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public BehaviorTreeLogMessagePubSubType newInstance()
   {
      return new BehaviorTreeLogMessagePubSubType();
   }
}
