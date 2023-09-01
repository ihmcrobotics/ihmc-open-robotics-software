package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "SimulatedLidarScanPacket" defined in "SimulatedLidarScanPacket_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from SimulatedLidarScanPacket_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit SimulatedLidarScanPacket_.idl instead.
*
*/
public class SimulatedLidarScanPacketPubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.SimulatedLidarScanPacket>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::SimulatedLidarScanPacket_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "17d8dc0fdc243da2b10928a0b732b44e0b8ca979975fd46f9442c62ed3418a7b";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.SimulatedLidarScanPacket data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.SimulatedLidarScanPacket data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (100 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += perception_msgs.msg.dds.LidarScanParametersMessagePubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.SimulatedLidarScanPacket data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.SimulatedLidarScanPacket data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getRanges().size() * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += perception_msgs.msg.dds.LidarScanParametersMessagePubSubType.getCdrSerializedSize(data.getLidarScanParameters(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.SimulatedLidarScanPacket data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      if(data.getRanges().size() <= 100)
      cdr.write_type_e(data.getRanges());else
          throw new RuntimeException("ranges field exceeds the maximum length");

      cdr.write_type_2(data.getSensorId());

      perception_msgs.msg.dds.LidarScanParametersMessagePubSubType.write(data.getLidarScanParameters(), cdr);
   }

   public static void read(perception_msgs.msg.dds.SimulatedLidarScanPacket data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      cdr.read_type_e(data.getRanges());	
      data.setSensorId(cdr.read_type_2());
      	
      perception_msgs.msg.dds.LidarScanParametersMessagePubSubType.read(data.getLidarScanParameters(), cdr);	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.SimulatedLidarScanPacket data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_e("ranges", data.getRanges());
      ser.write_type_2("sensor_id", data.getSensorId());
      ser.write_type_a("lidar_scan_parameters", new perception_msgs.msg.dds.LidarScanParametersMessagePubSubType(), data.getLidarScanParameters());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.SimulatedLidarScanPacket data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      ser.read_type_e("ranges", data.getRanges());
      data.setSensorId(ser.read_type_2("sensor_id"));
      ser.read_type_a("lidar_scan_parameters", new perception_msgs.msg.dds.LidarScanParametersMessagePubSubType(), data.getLidarScanParameters());

   }

   public static void staticCopy(perception_msgs.msg.dds.SimulatedLidarScanPacket src, perception_msgs.msg.dds.SimulatedLidarScanPacket dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.SimulatedLidarScanPacket createData()
   {
      return new perception_msgs.msg.dds.SimulatedLidarScanPacket();
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
   
   public void serialize(perception_msgs.msg.dds.SimulatedLidarScanPacket data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.SimulatedLidarScanPacket data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.SimulatedLidarScanPacket src, perception_msgs.msg.dds.SimulatedLidarScanPacket dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public SimulatedLidarScanPacketPubSubType newInstance()
   {
      return new SimulatedLidarScanPacketPubSubType();
   }
}
