package mission_control_msgs.msg.dds;

/**
* 
* Topic data type of the struct "SystemResourceUsageMessage" defined in "SystemResourceUsageMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from SystemResourceUsageMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit SystemResourceUsageMessage_.idl instead.
*
*/
public class SystemResourceUsageMessagePubSubType implements us.ihmc.pubsub.TopicDataType<mission_control_msgs.msg.dds.SystemResourceUsageMessage>
{
   public static final java.lang.String name = "mission_control_msgs::msg::dds_::SystemResourceUsageMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "d5bd186e539b2e030b5496a82970d103813afba170e801d328b5d9b39bf56ea9";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(mission_control_msgs.msg.dds.SystemResourceUsageMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, mission_control_msgs.msg.dds.SystemResourceUsageMessage data) throws java.io.IOException
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
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (100 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (100 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 100; ++i0)
      {
        current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      }
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (100 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (100 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 100; ++i0)
      {
        current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      }
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (100 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (100 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (100 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (100 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(mission_control_msgs.msg.dds.SystemResourceUsageMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(mission_control_msgs.msg.dds.SystemResourceUsageMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getUptime().length() + 1;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getCpuUsages().size() * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getCpuTemps().size() * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getIfaceNames().size(); ++i0)
      {
          current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getIfaceNames().get(i0).length() + 1;
      }
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getIfaceRxKbps().size() * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getIfaceTxKbps().size() * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getNvidiaGpuModels().size(); ++i0)
      {
          current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getNvidiaGpuModels().get(i0).length() + 1;
      }
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getNvidiaGpuMemoryUsed().size() * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getNvidiaGpuMemoryTotal().size() * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getNvidiaGpuUtilization().size() * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getNvidiaGpuTemps().size() * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      return current_alignment - initial_alignment;
   }

   public static void write(mission_control_msgs.msg.dds.SystemResourceUsageMessage data, us.ihmc.idl.CDR cdr)
   {
      if(data.getUptime().length() <= 255)
      cdr.write_type_d(data.getUptime());else
          throw new RuntimeException("uptime field exceeds the maximum length");

      cdr.write_type_5(data.getMemoryUsed());

      cdr.write_type_5(data.getMemoryTotal());

      cdr.write_type_2(data.getCpuCount());

      if(data.getCpuUsages().size() <= 100)
      cdr.write_type_e(data.getCpuUsages());else
          throw new RuntimeException("cpu_usages field exceeds the maximum length");

      if(data.getCpuTemps().size() <= 100)
      cdr.write_type_e(data.getCpuTemps());else
          throw new RuntimeException("cpu_temps field exceeds the maximum length");

      cdr.write_type_2(data.getIfaceCount());

      if(data.getIfaceNames().size() <= 100)
      cdr.write_type_e(data.getIfaceNames());else
          throw new RuntimeException("iface_names field exceeds the maximum length");

      if(data.getIfaceRxKbps().size() <= 100)
      cdr.write_type_e(data.getIfaceRxKbps());else
          throw new RuntimeException("iface_rx_kbps field exceeds the maximum length");

      if(data.getIfaceTxKbps().size() <= 100)
      cdr.write_type_e(data.getIfaceTxKbps());else
          throw new RuntimeException("iface_tx_kbps field exceeds the maximum length");

      cdr.write_type_2(data.getNvidiaGpuCount());

      if(data.getNvidiaGpuModels().size() <= 100)
      cdr.write_type_e(data.getNvidiaGpuModels());else
          throw new RuntimeException("nvidia_gpu_models field exceeds the maximum length");

      if(data.getNvidiaGpuMemoryUsed().size() <= 100)
      cdr.write_type_e(data.getNvidiaGpuMemoryUsed());else
          throw new RuntimeException("nvidia_gpu_memory_used field exceeds the maximum length");

      if(data.getNvidiaGpuMemoryTotal().size() <= 100)
      cdr.write_type_e(data.getNvidiaGpuMemoryTotal());else
          throw new RuntimeException("nvidia_gpu_memory_total field exceeds the maximum length");

      if(data.getNvidiaGpuUtilization().size() <= 100)
      cdr.write_type_e(data.getNvidiaGpuUtilization());else
          throw new RuntimeException("nvidia_gpu_utilization field exceeds the maximum length");

      if(data.getNvidiaGpuTemps().size() <= 100)
      cdr.write_type_e(data.getNvidiaGpuTemps());else
          throw new RuntimeException("nvidia_gpu_temps field exceeds the maximum length");

   }

   public static void read(mission_control_msgs.msg.dds.SystemResourceUsageMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.read_type_d(data.getUptime());	
      data.setMemoryUsed(cdr.read_type_5());
      	
      data.setMemoryTotal(cdr.read_type_5());
      	
      data.setCpuCount(cdr.read_type_2());
      	
      cdr.read_type_e(data.getCpuUsages());	
      cdr.read_type_e(data.getCpuTemps());	
      data.setIfaceCount(cdr.read_type_2());
      	
      cdr.read_type_e(data.getIfaceNames());	
      cdr.read_type_e(data.getIfaceRxKbps());	
      cdr.read_type_e(data.getIfaceTxKbps());	
      data.setNvidiaGpuCount(cdr.read_type_2());
      	
      cdr.read_type_e(data.getNvidiaGpuModels());	
      cdr.read_type_e(data.getNvidiaGpuMemoryUsed());	
      cdr.read_type_e(data.getNvidiaGpuMemoryTotal());	
      cdr.read_type_e(data.getNvidiaGpuUtilization());	
      cdr.read_type_e(data.getNvidiaGpuTemps());	

   }

   @Override
   public final void serialize(mission_control_msgs.msg.dds.SystemResourceUsageMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_d("uptime", data.getUptime());
      ser.write_type_5("memory_used", data.getMemoryUsed());
      ser.write_type_5("memory_total", data.getMemoryTotal());
      ser.write_type_2("cpu_count", data.getCpuCount());
      ser.write_type_e("cpu_usages", data.getCpuUsages());
      ser.write_type_e("cpu_temps", data.getCpuTemps());
      ser.write_type_2("iface_count", data.getIfaceCount());
      ser.write_type_e("iface_names", data.getIfaceNames());
      ser.write_type_e("iface_rx_kbps", data.getIfaceRxKbps());
      ser.write_type_e("iface_tx_kbps", data.getIfaceTxKbps());
      ser.write_type_2("nvidia_gpu_count", data.getNvidiaGpuCount());
      ser.write_type_e("nvidia_gpu_models", data.getNvidiaGpuModels());
      ser.write_type_e("nvidia_gpu_memory_used", data.getNvidiaGpuMemoryUsed());
      ser.write_type_e("nvidia_gpu_memory_total", data.getNvidiaGpuMemoryTotal());
      ser.write_type_e("nvidia_gpu_utilization", data.getNvidiaGpuUtilization());
      ser.write_type_e("nvidia_gpu_temps", data.getNvidiaGpuTemps());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, mission_control_msgs.msg.dds.SystemResourceUsageMessage data)
   {
      ser.read_type_d("uptime", data.getUptime());
      data.setMemoryUsed(ser.read_type_5("memory_used"));
      data.setMemoryTotal(ser.read_type_5("memory_total"));
      data.setCpuCount(ser.read_type_2("cpu_count"));
      ser.read_type_e("cpu_usages", data.getCpuUsages());
      ser.read_type_e("cpu_temps", data.getCpuTemps());
      data.setIfaceCount(ser.read_type_2("iface_count"));
      ser.read_type_e("iface_names", data.getIfaceNames());
      ser.read_type_e("iface_rx_kbps", data.getIfaceRxKbps());
      ser.read_type_e("iface_tx_kbps", data.getIfaceTxKbps());
      data.setNvidiaGpuCount(ser.read_type_2("nvidia_gpu_count"));
      ser.read_type_e("nvidia_gpu_models", data.getNvidiaGpuModels());
      ser.read_type_e("nvidia_gpu_memory_used", data.getNvidiaGpuMemoryUsed());
      ser.read_type_e("nvidia_gpu_memory_total", data.getNvidiaGpuMemoryTotal());
      ser.read_type_e("nvidia_gpu_utilization", data.getNvidiaGpuUtilization());
      ser.read_type_e("nvidia_gpu_temps", data.getNvidiaGpuTemps());
   }

   public static void staticCopy(mission_control_msgs.msg.dds.SystemResourceUsageMessage src, mission_control_msgs.msg.dds.SystemResourceUsageMessage dest)
   {
      dest.set(src);
   }

   @Override
   public mission_control_msgs.msg.dds.SystemResourceUsageMessage createData()
   {
      return new mission_control_msgs.msg.dds.SystemResourceUsageMessage();
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
   
   public void serialize(mission_control_msgs.msg.dds.SystemResourceUsageMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(mission_control_msgs.msg.dds.SystemResourceUsageMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(mission_control_msgs.msg.dds.SystemResourceUsageMessage src, mission_control_msgs.msg.dds.SystemResourceUsageMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public SystemResourceUsageMessagePubSubType newInstance()
   {
      return new SystemResourceUsageMessagePubSubType();
   }
}
