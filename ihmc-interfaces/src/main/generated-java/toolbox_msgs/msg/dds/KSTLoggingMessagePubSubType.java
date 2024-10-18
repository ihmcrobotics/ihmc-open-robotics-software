package toolbox_msgs.msg.dds;

/**
* 
* Topic data type of the struct "KSTLoggingMessage" defined in "KSTLoggingMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from KSTLoggingMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit KSTLoggingMessage_.idl instead.
*
*/
public class KSTLoggingMessagePubSubType implements us.ihmc.pubsub.TopicDataType<toolbox_msgs.msg.dds.KSTLoggingMessage>
{
   public static final java.lang.String name = "toolbox_msgs::msg::dds_::KSTLoggingMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "b7ee9699164c427259de3221a02942b05a5af7ddebc40fd55e6cc39615433b21";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(toolbox_msgs.msg.dds.KSTLoggingMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, toolbox_msgs.msg.dds.KSTLoggingMessage data) throws java.io.IOException
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

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.KSTLoggingMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.KSTLoggingMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(toolbox_msgs.msg.dds.KSTLoggingMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_7(data.getStartLogging());

   }

   public static void read(toolbox_msgs.msg.dds.KSTLoggingMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setStartLogging(cdr.read_type_7());
      	

   }

   @Override
   public final void serialize(toolbox_msgs.msg.dds.KSTLoggingMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_7("start_logging", data.getStartLogging());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, toolbox_msgs.msg.dds.KSTLoggingMessage data)
   {
      data.setStartLogging(ser.read_type_7("start_logging"));   }

   public static void staticCopy(toolbox_msgs.msg.dds.KSTLoggingMessage src, toolbox_msgs.msg.dds.KSTLoggingMessage dest)
   {
      dest.set(src);
   }

   @Override
   public toolbox_msgs.msg.dds.KSTLoggingMessage createData()
   {
      return new toolbox_msgs.msg.dds.KSTLoggingMessage();
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
   
   public void serialize(toolbox_msgs.msg.dds.KSTLoggingMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(toolbox_msgs.msg.dds.KSTLoggingMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(toolbox_msgs.msg.dds.KSTLoggingMessage src, toolbox_msgs.msg.dds.KSTLoggingMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public KSTLoggingMessagePubSubType newInstance()
   {
      return new KSTLoggingMessagePubSubType();
   }
}
