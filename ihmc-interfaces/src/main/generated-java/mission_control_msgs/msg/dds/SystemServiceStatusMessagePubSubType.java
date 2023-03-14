package mission_control_msgs.msg.dds;

/**
* 
* Topic data type of the struct "SystemServiceStatusMessage" defined in "SystemServiceStatusMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from SystemServiceStatusMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit SystemServiceStatusMessage_.idl instead.
*
*/
public class SystemServiceStatusMessagePubSubType implements us.ihmc.pubsub.TopicDataType<mission_control_msgs.msg.dds.SystemServiceStatusMessage>
{
   public static final java.lang.String name = "mission_control_msgs::msg::dds_::SystemServiceStatusMessage_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(mission_control_msgs.msg.dds.SystemServiceStatusMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, mission_control_msgs.msg.dds.SystemServiceStatusMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 100; ++i0)
      {
        current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      }

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(mission_control_msgs.msg.dds.SystemServiceStatusMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(mission_control_msgs.msg.dds.SystemServiceStatusMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getServiceName().length() + 1;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getStatus().length() + 1;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getLogLines().size(); ++i0)
      {
          current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getLogLines().get(i0).length() + 1;
      }

      return current_alignment - initial_alignment;
   }

   public static void write(mission_control_msgs.msg.dds.SystemServiceStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      if(data.getServiceName().length() <= 255)
      cdr.write_type_d(data.getServiceName());else
          throw new RuntimeException("service_name field exceeds the maximum length");

      if(data.getStatus().length() <= 255)
      cdr.write_type_d(data.getStatus());else
          throw new RuntimeException("status field exceeds the maximum length");

      cdr.write_type_2(data.getLogLineCount());

      if(data.getLogLines().size() <= 100)
      cdr.write_type_e(data.getLogLines());else
          throw new RuntimeException("log_lines field exceeds the maximum length");

   }

   public static void read(mission_control_msgs.msg.dds.SystemServiceStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.read_type_d(data.getServiceName());	
      cdr.read_type_d(data.getStatus());	
      data.setLogLineCount(cdr.read_type_2());
      	
      cdr.read_type_e(data.getLogLines());	

   }

   @Override
   public final void serialize(mission_control_msgs.msg.dds.SystemServiceStatusMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_d("service_name", data.getServiceName());
      ser.write_type_d("status", data.getStatus());
      ser.write_type_2("log_line_count", data.getLogLineCount());
      ser.write_type_e("log_lines", data.getLogLines());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, mission_control_msgs.msg.dds.SystemServiceStatusMessage data)
   {
      ser.read_type_d("service_name", data.getServiceName());
      ser.read_type_d("status", data.getStatus());
      data.setLogLineCount(ser.read_type_2("log_line_count"));
      ser.read_type_e("log_lines", data.getLogLines());
   }

   public static void staticCopy(mission_control_msgs.msg.dds.SystemServiceStatusMessage src, mission_control_msgs.msg.dds.SystemServiceStatusMessage dest)
   {
      dest.set(src);
   }

   @Override
   public mission_control_msgs.msg.dds.SystemServiceStatusMessage createData()
   {
      return new mission_control_msgs.msg.dds.SystemServiceStatusMessage();
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
   
   public void serialize(mission_control_msgs.msg.dds.SystemServiceStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(mission_control_msgs.msg.dds.SystemServiceStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(mission_control_msgs.msg.dds.SystemServiceStatusMessage src, mission_control_msgs.msg.dds.SystemServiceStatusMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public SystemServiceStatusMessagePubSubType newInstance()
   {
      return new SystemServiceStatusMessagePubSubType();
   }
}
