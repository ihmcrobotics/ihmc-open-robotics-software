package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "ActionsExecutionStatusMessage" defined in "ActionsExecutionStatusMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from ActionsExecutionStatusMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit ActionsExecutionStatusMessage_.idl instead.
*
*/
public class ActionsExecutionStatusMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.ActionsExecutionStatusMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::ActionsExecutionStatusMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "c383d3dbf6836a1f7fd35e874692cd28663a6787927eec2e6b45ff6d687252fd";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.ActionsExecutionStatusMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.ActionsExecutionStatusMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 200; ++i0)
      {
          current_alignment += behavior_msgs.msg.dds.ActionExecutionStatusMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.ActionsExecutionStatusMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.ActionsExecutionStatusMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getActionStatusList().size(); ++i0)
      {
          current_alignment += behavior_msgs.msg.dds.ActionExecutionStatusMessagePubSubType.getCdrSerializedSize(data.getActionStatusList().get(i0), current_alignment);}

      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.ActionsExecutionStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      if(data.getActionStatusList().size() <= 200)
      cdr.write_type_e(data.getActionStatusList());else
          throw new RuntimeException("action_status_list field exceeds the maximum length");

   }

   public static void read(behavior_msgs.msg.dds.ActionsExecutionStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.read_type_e(data.getActionStatusList());	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.ActionsExecutionStatusMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_e("action_status_list", data.getActionStatusList());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.ActionsExecutionStatusMessage data)
   {
      ser.read_type_e("action_status_list", data.getActionStatusList());
   }

   public static void staticCopy(behavior_msgs.msg.dds.ActionsExecutionStatusMessage src, behavior_msgs.msg.dds.ActionsExecutionStatusMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.ActionsExecutionStatusMessage createData()
   {
      return new behavior_msgs.msg.dds.ActionsExecutionStatusMessage();
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
   
   public void serialize(behavior_msgs.msg.dds.ActionsExecutionStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.ActionsExecutionStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.ActionsExecutionStatusMessage src, behavior_msgs.msg.dds.ActionsExecutionStatusMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public ActionsExecutionStatusMessagePubSubType newInstance()
   {
      return new ActionsExecutionStatusMessagePubSubType();
   }
}
