package toolbox_msgs.msg.dds;

/**
* 
* Topic data type of the struct "ToolboxStateMessage" defined in "ToolboxStateMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from ToolboxStateMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit ToolboxStateMessage_.idl instead.
*
*/
public class ToolboxStateMessagePubSubType implements us.ihmc.pubsub.TopicDataType<toolbox_msgs.msg.dds.ToolboxStateMessage>
{
   public static final java.lang.String name = "toolbox_msgs::msg::dds_::ToolboxStateMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "2ef84ac5bb57bd393272385767b8c1c6a6a48d6de3938c44b73bd096edcb1855";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(toolbox_msgs.msg.dds.ToolboxStateMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, toolbox_msgs.msg.dds.ToolboxStateMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.ToolboxStateMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.ToolboxStateMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(toolbox_msgs.msg.dds.ToolboxStateMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_9(data.getRequestedToolboxState());

      cdr.write_type_7(data.getRequestLogging());

   }

   public static void read(toolbox_msgs.msg.dds.ToolboxStateMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setRequestedToolboxState(cdr.read_type_9());
      	
      data.setRequestLogging(cdr.read_type_7());
      	

   }

   @Override
   public final void serialize(toolbox_msgs.msg.dds.ToolboxStateMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_9("requested_toolbox_state", data.getRequestedToolboxState());
      ser.write_type_7("request_logging", data.getRequestLogging());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, toolbox_msgs.msg.dds.ToolboxStateMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setRequestedToolboxState(ser.read_type_9("requested_toolbox_state"));
      data.setRequestLogging(ser.read_type_7("request_logging"));
   }

   public static void staticCopy(toolbox_msgs.msg.dds.ToolboxStateMessage src, toolbox_msgs.msg.dds.ToolboxStateMessage dest)
   {
      dest.set(src);
   }

   @Override
   public toolbox_msgs.msg.dds.ToolboxStateMessage createData()
   {
      return new toolbox_msgs.msg.dds.ToolboxStateMessage();
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
   
   public void serialize(toolbox_msgs.msg.dds.ToolboxStateMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(toolbox_msgs.msg.dds.ToolboxStateMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(toolbox_msgs.msg.dds.ToolboxStateMessage src, toolbox_msgs.msg.dds.ToolboxStateMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public ToolboxStateMessagePubSubType newInstance()
   {
      return new ToolboxStateMessagePubSubType();
   }
}
