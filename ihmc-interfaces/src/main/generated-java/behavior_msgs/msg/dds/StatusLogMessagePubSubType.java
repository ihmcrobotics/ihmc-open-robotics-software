package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "StatusLogMessage" defined in "StatusLogMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from StatusLogMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit StatusLogMessage_.idl instead.
*
*/
public class StatusLogMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.StatusLogMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::StatusLogMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "9871e3859fdc9f7c85aa1b7d32161a7555a28fb312ad15de7b954440ed22fbd8";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.StatusLogMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.StatusLogMessage data) throws java.io.IOException
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

      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (100 * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.StatusLogMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.StatusLogMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getLogMessage().size() * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.StatusLogMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_3(data.getLogLevel());

      if(data.getLogMessage().size() <= 100)
      cdr.write_type_e(data.getLogMessage());else
          throw new RuntimeException("log_message field exceeds the maximum length");

   }

   public static void read(behavior_msgs.msg.dds.StatusLogMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setLogLevel(cdr.read_type_3());
      	
      cdr.read_type_e(data.getLogMessage());	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.StatusLogMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_3("log_level", data.getLogLevel());
      ser.write_type_e("log_message", data.getLogMessage());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.StatusLogMessage data)
   {
      data.setLogLevel(ser.read_type_3("log_level"));
      ser.read_type_e("log_message", data.getLogMessage());
   }

   public static void staticCopy(behavior_msgs.msg.dds.StatusLogMessage src, behavior_msgs.msg.dds.StatusLogMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.StatusLogMessage createData()
   {
      return new behavior_msgs.msg.dds.StatusLogMessage();
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
   
   public void serialize(behavior_msgs.msg.dds.StatusLogMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.StatusLogMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.StatusLogMessage src, behavior_msgs.msg.dds.StatusLogMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public StatusLogMessagePubSubType newInstance()
   {
      return new StatusLogMessagePubSubType();
   }
}
