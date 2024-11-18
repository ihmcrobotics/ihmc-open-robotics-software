package system_monitor_msgs.msg.dds;

/**
* 
* Topic data type of the struct "SystemServiceStatusMessage" defined in "SystemServiceStatusMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from SystemServiceStatusMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit SystemServiceStatusMessage_.idl instead.
*
*/
public class SystemServiceStatusMessagePubSubType implements us.ihmc.pubsub.TopicDataType<system_monitor_msgs.msg.dds.SystemServiceStatusMessage>
{
   public static final java.lang.String name = "system_monitor_msgs::msg::dds_::SystemServiceStatusMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "741aa1c62507792b5d19de089b2efd2fc1f9b7e24537d4a5f0764d734a2b58e6";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(system_monitor_msgs.msg.dds.SystemServiceStatusMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, system_monitor_msgs.msg.dds.SystemServiceStatusMessage data) throws java.io.IOException
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
      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (25000000 * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(system_monitor_msgs.msg.dds.SystemServiceStatusMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(system_monitor_msgs.msg.dds.SystemServiceStatusMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getServiceName().length() + 1;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getStatus().length() + 1;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getLogData().size() * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(system_monitor_msgs.msg.dds.SystemServiceStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      if(data.getServiceName().length() <= 255)
      cdr.write_type_d(data.getServiceName());else
          throw new RuntimeException("service_name field exceeds the maximum length");

      if(data.getStatus().length() <= 255)
      cdr.write_type_d(data.getStatus());else
          throw new RuntimeException("status field exceeds the maximum length");

      cdr.write_type_7(data.getRefresh());

      if(data.getLogData().size() <= 25000000)
      cdr.write_type_e(data.getLogData());else
          throw new RuntimeException("log_data field exceeds the maximum length");

   }

   public static void read(system_monitor_msgs.msg.dds.SystemServiceStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.read_type_d(data.getServiceName());	
      cdr.read_type_d(data.getStatus());	
      data.setRefresh(cdr.read_type_7());
      	
      cdr.read_type_e(data.getLogData());	

   }

   @Override
   public final void serialize(system_monitor_msgs.msg.dds.SystemServiceStatusMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_d("service_name", data.getServiceName());
      ser.write_type_d("status", data.getStatus());
      ser.write_type_7("refresh", data.getRefresh());
      ser.write_type_e("log_data", data.getLogData());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, system_monitor_msgs.msg.dds.SystemServiceStatusMessage data)
   {
      ser.read_type_d("service_name", data.getServiceName());
      ser.read_type_d("status", data.getStatus());
      data.setRefresh(ser.read_type_7("refresh"));
      ser.read_type_e("log_data", data.getLogData());
   }

   public static void staticCopy(system_monitor_msgs.msg.dds.SystemServiceStatusMessage src, system_monitor_msgs.msg.dds.SystemServiceStatusMessage dest)
   {
      dest.set(src);
   }

   @Override
   public system_monitor_msgs.msg.dds.SystemServiceStatusMessage createData()
   {
      return new system_monitor_msgs.msg.dds.SystemServiceStatusMessage();
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
   
   public void serialize(system_monitor_msgs.msg.dds.SystemServiceStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(system_monitor_msgs.msg.dds.SystemServiceStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(system_monitor_msgs.msg.dds.SystemServiceStatusMessage src, system_monitor_msgs.msg.dds.SystemServiceStatusMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public SystemServiceStatusMessagePubSubType newInstance()
   {
      return new SystemServiceStatusMessagePubSubType();
   }
}
