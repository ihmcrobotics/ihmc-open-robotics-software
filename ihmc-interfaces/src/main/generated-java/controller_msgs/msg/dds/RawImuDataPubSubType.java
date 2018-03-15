package controller_msgs.msg.dds;

/**
 * Topic data type of the struct "RawImuData" defined in "RawImuData_.idl". Use this class to provide the TopicDataType to a Participant.
 *
 * This file was automatically generated from RawImuData_.idl by us.ihmc.idl.generator.IDLGenerator.
 * Do not update this file directly, edit RawImuData_.idl instead.
 */
public class RawImuDataPubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.RawImuData>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::RawImuData_";
   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   public RawImuDataPubSubType()
   {

   }

   public static int getMaxCdrSerializedSize()
   {
      return getMaxCdrSerializedSize(0);
   }

   public static int getMaxCdrSerializedSize(int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getMaxCdrSerializedSize(current_alignment);
      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getMaxCdrSerializedSize(current_alignment);

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.RawImuData data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.RawImuData data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getCdrSerializedSize(data.getImuRates(), current_alignment);
      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getCdrSerializedSize(data.getImuDeltas(), current_alignment);

      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.RawImuData data, us.ihmc.idl.CDR cdr)
   {

      cdr.write_type_11(data.getTimestamp());

      cdr.write_type_11(data.getPacketCount());

      geometry_msgs.msg.dds.Vector3PubSubType.write(data.getImuRates(), cdr);

      geometry_msgs.msg.dds.Vector3PubSubType.write(data.getImuDeltas(), cdr);
   }

   public static void read(controller_msgs.msg.dds.RawImuData data, us.ihmc.idl.CDR cdr)
   {

      data.setTimestamp(cdr.read_type_11());

      data.setPacketCount(cdr.read_type_11());

      geometry_msgs.msg.dds.Vector3PubSubType.read(data.getImuRates(), cdr);

      geometry_msgs.msg.dds.Vector3PubSubType.read(data.getImuDeltas(), cdr);
   }

   public static void staticCopy(controller_msgs.msg.dds.RawImuData src, controller_msgs.msg.dds.RawImuData dest)
   {
      dest.set(src);
   }

   @Override
   public void serialize(controller_msgs.msg.dds.RawImuData data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.RawImuData data) throws java.io.IOException
   {
      deserializeCDR.deserialize(serializedPayload);
      read(data, deserializeCDR);
      deserializeCDR.finishDeserialize();
   }

   @Override
   public final void serialize(controller_msgs.msg.dds.RawImuData data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_11("timestamp", data.getTimestamp());

      ser.write_type_11("packet_count", data.getPacketCount());

      ser.write_type_a("imu_rates", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getImuRates());

      ser.write_type_a("imu_deltas", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getImuDeltas());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.RawImuData data)
   {
      data.setTimestamp(ser.read_type_11("timestamp"));

      data.setPacketCount(ser.read_type_11("packet_count"));

      ser.read_type_a("imu_rates", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getImuRates());

      ser.read_type_a("imu_deltas", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getImuDeltas());
   }

   @Override
   public controller_msgs.msg.dds.RawImuData createData()
   {
      return new controller_msgs.msg.dds.RawImuData();
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

   public void serialize(controller_msgs.msg.dds.RawImuData data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.RawImuData data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }

   public void copy(controller_msgs.msg.dds.RawImuData src, controller_msgs.msg.dds.RawImuData dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public RawImuDataPubSubType newInstance()
   {
      return new RawImuDataPubSubType();
   }
}