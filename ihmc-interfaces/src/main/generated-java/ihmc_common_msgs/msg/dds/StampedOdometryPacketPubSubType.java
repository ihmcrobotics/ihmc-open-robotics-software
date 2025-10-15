package ihmc_common_msgs.msg.dds;

/**
* 
* Topic data type of the struct "StampedOdometryPacket" defined in "StampedOdometryPacket_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from StampedOdometryPacket_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit StampedOdometryPacket_.idl instead.
*
*/
public class StampedOdometryPacketPubSubType implements us.ihmc.pubsub.TopicDataType<ihmc_common_msgs.msg.dds.StampedOdometryPacket>
{
   public static final java.lang.String name = "ihmc_common_msgs::msg::dds_::StampedOdometryPacket_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "829a972552d8cc398f6b5069df918f3843f2b6c6486359a5befc78e0cdade8e5";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(ihmc_common_msgs.msg.dds.StampedOdometryPacket data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, ihmc_common_msgs.msg.dds.StampedOdometryPacket data) throws java.io.IOException
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

      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(ihmc_common_msgs.msg.dds.StampedOdometryPacket data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(ihmc_common_msgs.msg.dds.StampedOdometryPacket data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getCdrSerializedSize(data.getPose(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getCdrSerializedSize(data.getImuOrientation(), current_alignment);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(ihmc_common_msgs.msg.dds.StampedOdometryPacket data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_12(data.getSequenceId());

      geometry_msgs.msg.dds.PosePubSubType.write(data.getPose(), cdr);
      geometry_msgs.msg.dds.QuaternionPubSubType.write(data.getImuOrientation(), cdr);
      cdr.write_type_11(data.getTimestamp());

      cdr.write_type_6(data.getConfidenceFactor());

   }

   public static void read(ihmc_common_msgs.msg.dds.StampedOdometryPacket data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_12());
      	
      geometry_msgs.msg.dds.PosePubSubType.read(data.getPose(), cdr);	
      geometry_msgs.msg.dds.QuaternionPubSubType.read(data.getImuOrientation(), cdr);	
      data.setTimestamp(cdr.read_type_11());
      	
      data.setConfidenceFactor(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(ihmc_common_msgs.msg.dds.StampedOdometryPacket data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_12("sequence_id", data.getSequenceId());
      ser.write_type_a("pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getPose());

      ser.write_type_a("imu_orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getImuOrientation());

      ser.write_type_11("timestamp", data.getTimestamp());
      ser.write_type_6("confidence_factor", data.getConfidenceFactor());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, ihmc_common_msgs.msg.dds.StampedOdometryPacket data)
   {
      data.setSequenceId(ser.read_type_12("sequence_id"));
      ser.read_type_a("pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getPose());

      ser.read_type_a("imu_orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getImuOrientation());

      data.setTimestamp(ser.read_type_11("timestamp"));
      data.setConfidenceFactor(ser.read_type_6("confidence_factor"));
   }

   public static void staticCopy(ihmc_common_msgs.msg.dds.StampedOdometryPacket src, ihmc_common_msgs.msg.dds.StampedOdometryPacket dest)
   {
      dest.set(src);
   }

   @Override
   public ihmc_common_msgs.msg.dds.StampedOdometryPacket createData()
   {
      return new ihmc_common_msgs.msg.dds.StampedOdometryPacket();
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
   
   public void serialize(ihmc_common_msgs.msg.dds.StampedOdometryPacket data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(ihmc_common_msgs.msg.dds.StampedOdometryPacket data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(ihmc_common_msgs.msg.dds.StampedOdometryPacket src, ihmc_common_msgs.msg.dds.StampedOdometryPacket dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public StampedOdometryPacketPubSubType newInstance()
   {
      return new StampedOdometryPacketPubSubType();
   }
}
