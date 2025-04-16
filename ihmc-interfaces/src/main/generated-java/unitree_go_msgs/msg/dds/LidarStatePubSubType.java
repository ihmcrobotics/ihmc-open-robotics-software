package unitree_go_msgs.msg.dds;

/**
* 
* Topic data type of the struct "LidarState" defined in "LidarState_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from LidarState_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit LidarState_.idl instead.
*
*/
public class LidarStatePubSubType implements us.ihmc.pubsub.TopicDataType<unitree_go_msgs.msg.dds.LidarState>
{
   public static final java.lang.String name = "unitree_go_msgs::msg::dds_::LidarState_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "bcd4e6dfb4f9182d93087ad905a65fc6d88e14092b3e475a629090a3caa18d0c";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(unitree_go_msgs.msg.dds.LidarState data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, unitree_go_msgs.msg.dds.LidarState data) throws java.io.IOException
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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += ((3) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(unitree_go_msgs.msg.dds.LidarState data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(unitree_go_msgs.msg.dds.LidarState data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getFirmwareVersion().length() + 1;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getSoftwareVersion().length() + 1;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getSdkVersion().length() + 1;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += ((3) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      return current_alignment - initial_alignment;
   }

   public static void write(unitree_go_msgs.msg.dds.LidarState data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_6(data.getStamp());

      if(data.getFirmwareVersion().length() <= 255)
      cdr.write_type_d(data.getFirmwareVersion());else
          throw new RuntimeException("firmware_version field exceeds the maximum length: %d > %d".formatted(data.getFirmwareVersion().length(), 255));

      if(data.getSoftwareVersion().length() <= 255)
      cdr.write_type_d(data.getSoftwareVersion());else
          throw new RuntimeException("software_version field exceeds the maximum length: %d > %d".formatted(data.getSoftwareVersion().length(), 255));

      if(data.getSdkVersion().length() <= 255)
      cdr.write_type_d(data.getSdkVersion());else
          throw new RuntimeException("sdk_version field exceeds the maximum length: %d > %d".formatted(data.getSdkVersion().length(), 255));

      cdr.write_type_5(data.getSysRotationSpeed());

      cdr.write_type_5(data.getComRotationSpeed());

      cdr.write_type_9(data.getErrorState());

      cdr.write_type_5(data.getCloudFrequency());

      cdr.write_type_5(data.getCloudPacketLossRate());

      cdr.write_type_4(data.getCloudSize());

      cdr.write_type_4(data.getCloudScanNum());

      cdr.write_type_5(data.getImuFrequency());

      cdr.write_type_5(data.getImuPacketLossRate());

      for(int i0 = 0; i0 < data.getImuRpy().length; ++i0)
      {
        	cdr.write_type_5(data.getImuRpy()[i0]);	
      }

      cdr.write_type_6(data.getSerialRecvStamp());

      cdr.write_type_4(data.getSerialBufferSize());

      cdr.write_type_4(data.getSerialBufferRead());

   }

   public static void read(unitree_go_msgs.msg.dds.LidarState data, us.ihmc.idl.CDR cdr)
   {
      data.setStamp(cdr.read_type_6());
      	
      cdr.read_type_d(data.getFirmwareVersion());	
      cdr.read_type_d(data.getSoftwareVersion());	
      cdr.read_type_d(data.getSdkVersion());	
      data.setSysRotationSpeed(cdr.read_type_5());
      	
      data.setComRotationSpeed(cdr.read_type_5());
      	
      data.setErrorState(cdr.read_type_9());
      	
      data.setCloudFrequency(cdr.read_type_5());
      	
      data.setCloudPacketLossRate(cdr.read_type_5());
      	
      data.setCloudSize(cdr.read_type_4());
      	
      data.setCloudScanNum(cdr.read_type_4());
      	
      data.setImuFrequency(cdr.read_type_5());
      	
      data.setImuPacketLossRate(cdr.read_type_5());
      	
      for(int i0 = 0; i0 < data.getImuRpy().length; ++i0)
      {
        	data.getImuRpy()[i0] = cdr.read_type_5();
        	
      }
      	
      data.setSerialRecvStamp(cdr.read_type_6());
      	
      data.setSerialBufferSize(cdr.read_type_4());
      	
      data.setSerialBufferRead(cdr.read_type_4());
      	

   }

   @Override
   public final void serialize(unitree_go_msgs.msg.dds.LidarState data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_6("stamp", data.getStamp());
      ser.write_type_d("firmware_version", data.getFirmwareVersion());
      ser.write_type_d("software_version", data.getSoftwareVersion());
      ser.write_type_d("sdk_version", data.getSdkVersion());
      ser.write_type_5("sys_rotation_speed", data.getSysRotationSpeed());
      ser.write_type_5("com_rotation_speed", data.getComRotationSpeed());
      ser.write_type_9("error_state", data.getErrorState());
      ser.write_type_5("cloud_frequency", data.getCloudFrequency());
      ser.write_type_5("cloud_packet_loss_rate", data.getCloudPacketLossRate());
      ser.write_type_4("cloud_size", data.getCloudSize());
      ser.write_type_4("cloud_scan_num", data.getCloudScanNum());
      ser.write_type_5("imu_frequency", data.getImuFrequency());
      ser.write_type_5("imu_packet_loss_rate", data.getImuPacketLossRate());
      ser.write_type_f("imu_rpy", data.getImuRpy());
      ser.write_type_6("serial_recv_stamp", data.getSerialRecvStamp());
      ser.write_type_4("serial_buffer_size", data.getSerialBufferSize());
      ser.write_type_4("serial_buffer_read", data.getSerialBufferRead());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, unitree_go_msgs.msg.dds.LidarState data)
   {
      data.setStamp(ser.read_type_6("stamp"));
      ser.read_type_d("firmware_version", data.getFirmwareVersion());
      ser.read_type_d("software_version", data.getSoftwareVersion());
      ser.read_type_d("sdk_version", data.getSdkVersion());
      data.setSysRotationSpeed(ser.read_type_5("sys_rotation_speed"));
      data.setComRotationSpeed(ser.read_type_5("com_rotation_speed"));
      data.setErrorState(ser.read_type_9("error_state"));
      data.setCloudFrequency(ser.read_type_5("cloud_frequency"));
      data.setCloudPacketLossRate(ser.read_type_5("cloud_packet_loss_rate"));
      data.setCloudSize(ser.read_type_4("cloud_size"));
      data.setCloudScanNum(ser.read_type_4("cloud_scan_num"));
      data.setImuFrequency(ser.read_type_5("imu_frequency"));
      data.setImuPacketLossRate(ser.read_type_5("imu_packet_loss_rate"));
      ser.read_type_f("imu_rpy", data.getImuRpy());
      data.setSerialRecvStamp(ser.read_type_6("serial_recv_stamp"));
      data.setSerialBufferSize(ser.read_type_4("serial_buffer_size"));
      data.setSerialBufferRead(ser.read_type_4("serial_buffer_read"));
   }

   public static void staticCopy(unitree_go_msgs.msg.dds.LidarState src, unitree_go_msgs.msg.dds.LidarState dest)
   {
      dest.set(src);
   }

   @Override
   public unitree_go_msgs.msg.dds.LidarState createData()
   {
      return new unitree_go_msgs.msg.dds.LidarState();
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
   
   public void serialize(unitree_go_msgs.msg.dds.LidarState data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(unitree_go_msgs.msg.dds.LidarState data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(unitree_go_msgs.msg.dds.LidarState src, unitree_go_msgs.msg.dds.LidarState dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public LidarStatePubSubType newInstance()
   {
      return new LidarStatePubSubType();
   }
}
