package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "LidarScanParametersMessage" defined in "LidarScanParametersMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from LidarScanParametersMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit LidarScanParametersMessage_.idl instead.
*
*/
public class LidarScanParametersMessagePubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.LidarScanParametersMessage>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::LidarScanParametersMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "0643ee583832c2786376b1a87e522ec5560aafe77400502b6f3034ababbaeb1e";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.LidarScanParametersMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.LidarScanParametersMessage data) throws java.io.IOException
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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.LidarScanParametersMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.LidarScanParametersMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.LidarScanParametersMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_11(data.getTimestamp());

      cdr.write_type_5(data.getSweepYawMax());

      cdr.write_type_5(data.getSweepYawMin());

      cdr.write_type_5(data.getHeightPitchMax());

      cdr.write_type_5(data.getHeightPitchMin());

      cdr.write_type_5(data.getTimeIncrement());

      cdr.write_type_5(data.getScanTime());

      cdr.write_type_5(data.getMinRange());

      cdr.write_type_5(data.getMaxRange());

      cdr.write_type_2(data.getPointsPerSweep());

      cdr.write_type_2(data.getScanHeight());

   }

   public static void read(perception_msgs.msg.dds.LidarScanParametersMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setTimestamp(cdr.read_type_11());
      	
      data.setSweepYawMax(cdr.read_type_5());
      	
      data.setSweepYawMin(cdr.read_type_5());
      	
      data.setHeightPitchMax(cdr.read_type_5());
      	
      data.setHeightPitchMin(cdr.read_type_5());
      	
      data.setTimeIncrement(cdr.read_type_5());
      	
      data.setScanTime(cdr.read_type_5());
      	
      data.setMinRange(cdr.read_type_5());
      	
      data.setMaxRange(cdr.read_type_5());
      	
      data.setPointsPerSweep(cdr.read_type_2());
      	
      data.setScanHeight(cdr.read_type_2());
      	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.LidarScanParametersMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_11("timestamp", data.getTimestamp());
      ser.write_type_5("sweep_yaw_max", data.getSweepYawMax());
      ser.write_type_5("sweep_yaw_min", data.getSweepYawMin());
      ser.write_type_5("height_pitch_max", data.getHeightPitchMax());
      ser.write_type_5("height_pitch_min", data.getHeightPitchMin());
      ser.write_type_5("time_increment", data.getTimeIncrement());
      ser.write_type_5("scan_time", data.getScanTime());
      ser.write_type_5("min_range", data.getMinRange());
      ser.write_type_5("max_range", data.getMaxRange());
      ser.write_type_2("points_per_sweep", data.getPointsPerSweep());
      ser.write_type_2("scan_height", data.getScanHeight());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.LidarScanParametersMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setTimestamp(ser.read_type_11("timestamp"));
      data.setSweepYawMax(ser.read_type_5("sweep_yaw_max"));
      data.setSweepYawMin(ser.read_type_5("sweep_yaw_min"));
      data.setHeightPitchMax(ser.read_type_5("height_pitch_max"));
      data.setHeightPitchMin(ser.read_type_5("height_pitch_min"));
      data.setTimeIncrement(ser.read_type_5("time_increment"));
      data.setScanTime(ser.read_type_5("scan_time"));
      data.setMinRange(ser.read_type_5("min_range"));
      data.setMaxRange(ser.read_type_5("max_range"));
      data.setPointsPerSweep(ser.read_type_2("points_per_sweep"));
      data.setScanHeight(ser.read_type_2("scan_height"));
   }

   public static void staticCopy(perception_msgs.msg.dds.LidarScanParametersMessage src, perception_msgs.msg.dds.LidarScanParametersMessage dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.LidarScanParametersMessage createData()
   {
      return new perception_msgs.msg.dds.LidarScanParametersMessage();
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
   
   public void serialize(perception_msgs.msg.dds.LidarScanParametersMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.LidarScanParametersMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.LidarScanParametersMessage src, perception_msgs.msg.dds.LidarScanParametersMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public LidarScanParametersMessagePubSubType newInstance()
   {
      return new LidarScanParametersMessagePubSubType();
   }
}
