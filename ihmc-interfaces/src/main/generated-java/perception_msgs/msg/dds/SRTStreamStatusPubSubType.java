package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "SRTStreamStatus" defined in "SRTStreamStatus_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from SRTStreamStatus_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit SRTStreamStatus_.idl instead.
*
*/
public class SRTStreamStatusPubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.SRTStreamStatus>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::SRTStreamStatus_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "fbe64bd6dd9310f28cb915592e0be7a9e9b834d2e978cf7e6fdbf749eff8ed62";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.SRTStreamStatus data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.SRTStreamStatus data) throws java.io.IOException
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
      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);

      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += perception_msgs.msg.dds.VideoFrameExtraDataPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.SRTStreamStatus data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.SRTStreamStatus data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getStreamerAddress().length() + 1;

      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);


      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += perception_msgs.msg.dds.VideoFrameExtraDataPubSubType.getCdrSerializedSize(data.getFrameExtraData(), current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.SRTStreamStatus data, us.ihmc.idl.CDR cdr)
   {
      if(data.getStreamerAddress().length() <= 255)
      cdr.write_type_d(data.getStreamerAddress());else
          throw new RuntimeException("streamer_address field exceeds the maximum length");

      cdr.write_type_3(data.getStreamerPort());

      cdr.write_type_7(data.getIsStreaming());

      cdr.write_type_5(data.getExpectedPublishFrequency());

      cdr.write_type_3(data.getImageWidth());

      cdr.write_type_3(data.getImageHeight());

      cdr.write_type_5(data.getFx());

      cdr.write_type_5(data.getFy());

      cdr.write_type_5(data.getCx());

      cdr.write_type_5(data.getCy());

      cdr.write_type_5(data.getDepthDiscretization());

      perception_msgs.msg.dds.VideoFrameExtraDataPubSubType.write(data.getFrameExtraData(), cdr);
      cdr.write_type_7(data.getContainsExtraData());

   }

   public static void read(perception_msgs.msg.dds.SRTStreamStatus data, us.ihmc.idl.CDR cdr)
   {
      cdr.read_type_d(data.getStreamerAddress());	
      data.setStreamerPort(cdr.read_type_3());
      	
      data.setIsStreaming(cdr.read_type_7());
      	
      data.setExpectedPublishFrequency(cdr.read_type_5());
      	
      data.setImageWidth(cdr.read_type_3());
      	
      data.setImageHeight(cdr.read_type_3());
      	
      data.setFx(cdr.read_type_5());
      	
      data.setFy(cdr.read_type_5());
      	
      data.setCx(cdr.read_type_5());
      	
      data.setCy(cdr.read_type_5());
      	
      data.setDepthDiscretization(cdr.read_type_5());
      	
      perception_msgs.msg.dds.VideoFrameExtraDataPubSubType.read(data.getFrameExtraData(), cdr);	
      data.setContainsExtraData(cdr.read_type_7());
      	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.SRTStreamStatus data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_d("streamer_address", data.getStreamerAddress());
      ser.write_type_3("streamer_port", data.getStreamerPort());
      ser.write_type_7("is_streaming", data.getIsStreaming());
      ser.write_type_5("expected_publish_frequency", data.getExpectedPublishFrequency());
      ser.write_type_3("image_width", data.getImageWidth());
      ser.write_type_3("image_height", data.getImageHeight());
      ser.write_type_5("fx", data.getFx());
      ser.write_type_5("fy", data.getFy());
      ser.write_type_5("cx", data.getCx());
      ser.write_type_5("cy", data.getCy());
      ser.write_type_5("depth_discretization", data.getDepthDiscretization());
      ser.write_type_a("frame_extra_data", new perception_msgs.msg.dds.VideoFrameExtraDataPubSubType(), data.getFrameExtraData());

      ser.write_type_7("contains_extra_data", data.getContainsExtraData());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.SRTStreamStatus data)
   {
      ser.read_type_d("streamer_address", data.getStreamerAddress());
      data.setStreamerPort(ser.read_type_3("streamer_port"));
      data.setIsStreaming(ser.read_type_7("is_streaming"));
      data.setExpectedPublishFrequency(ser.read_type_5("expected_publish_frequency"));
      data.setImageWidth(ser.read_type_3("image_width"));
      data.setImageHeight(ser.read_type_3("image_height"));
      data.setFx(ser.read_type_5("fx"));
      data.setFy(ser.read_type_5("fy"));
      data.setCx(ser.read_type_5("cx"));
      data.setCy(ser.read_type_5("cy"));
      data.setDepthDiscretization(ser.read_type_5("depth_discretization"));
      ser.read_type_a("frame_extra_data", new perception_msgs.msg.dds.VideoFrameExtraDataPubSubType(), data.getFrameExtraData());

      data.setContainsExtraData(ser.read_type_7("contains_extra_data"));
   }

   public static void staticCopy(perception_msgs.msg.dds.SRTStreamStatus src, perception_msgs.msg.dds.SRTStreamStatus dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.SRTStreamStatus createData()
   {
      return new perception_msgs.msg.dds.SRTStreamStatus();
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
   
   public void serialize(perception_msgs.msg.dds.SRTStreamStatus data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.SRTStreamStatus data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.SRTStreamStatus src, perception_msgs.msg.dds.SRTStreamStatus dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public SRTStreamStatusPubSubType newInstance()
   {
      return new SRTStreamStatusPubSubType();
   }
}
