package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "VideoFrameExtraData" defined in "VideoFrameExtraData_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from VideoFrameExtraData_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit VideoFrameExtraData_.idl instead.
*
*/
public class VideoFrameExtraDataPubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.VideoFrameExtraData>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::VideoFrameExtraData_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "b68a1d98d0667979755ded0487b25719ffe4e28fe337e39cf460819a08b397bd";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.VideoFrameExtraData data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.VideoFrameExtraData data) throws java.io.IOException
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

      current_alignment += ihmc_common_msgs.msg.dds.InstantMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.VideoFrameExtraData data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.VideoFrameExtraData data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += ihmc_common_msgs.msg.dds.InstantMessagePubSubType.getCdrSerializedSize(data.getAcquisitionTime(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getCdrSerializedSize(data.getSensorPose(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.VideoFrameExtraData data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceNumber());

      ihmc_common_msgs.msg.dds.InstantMessagePubSubType.write(data.getAcquisitionTime(), cdr);
      geometry_msgs.msg.dds.PosePubSubType.write(data.getSensorPose(), cdr);
   }

   public static void read(perception_msgs.msg.dds.VideoFrameExtraData data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceNumber(cdr.read_type_4());
      	
      ihmc_common_msgs.msg.dds.InstantMessagePubSubType.read(data.getAcquisitionTime(), cdr);	
      geometry_msgs.msg.dds.PosePubSubType.read(data.getSensorPose(), cdr);	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.VideoFrameExtraData data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_number", data.getSequenceNumber());
      ser.write_type_a("acquisition_time", new ihmc_common_msgs.msg.dds.InstantMessagePubSubType(), data.getAcquisitionTime());

      ser.write_type_a("sensor_pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getSensorPose());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.VideoFrameExtraData data)
   {
      data.setSequenceNumber(ser.read_type_4("sequence_number"));
      ser.read_type_a("acquisition_time", new ihmc_common_msgs.msg.dds.InstantMessagePubSubType(), data.getAcquisitionTime());

      ser.read_type_a("sensor_pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getSensorPose());

   }

   public static void staticCopy(perception_msgs.msg.dds.VideoFrameExtraData src, perception_msgs.msg.dds.VideoFrameExtraData dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.VideoFrameExtraData createData()
   {
      return new perception_msgs.msg.dds.VideoFrameExtraData();
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
   
   public void serialize(perception_msgs.msg.dds.VideoFrameExtraData data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.VideoFrameExtraData data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.VideoFrameExtraData src, perception_msgs.msg.dds.VideoFrameExtraData dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public VideoFrameExtraDataPubSubType newInstance()
   {
      return new VideoFrameExtraDataPubSubType();
   }
}
