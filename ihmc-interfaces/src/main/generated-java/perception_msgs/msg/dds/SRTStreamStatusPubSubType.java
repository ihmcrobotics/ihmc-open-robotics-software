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
   		return "f1c4661f39967ba831746964347dde0a6385011960bd8584a8ce88ca834e4afd";
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

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getMaxCdrSerializedSize(current_alignment);


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


      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getPosition(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getCdrSerializedSize(data.getOrientation(), current_alignment);


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

      geometry_msgs.msg.dds.PointPubSubType.write(data.getPosition(), cdr);
      geometry_msgs.msg.dds.QuaternionPubSubType.write(data.getOrientation(), cdr);
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
      	
      geometry_msgs.msg.dds.PointPubSubType.read(data.getPosition(), cdr);	
      geometry_msgs.msg.dds.QuaternionPubSubType.read(data.getOrientation(), cdr);	

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
      ser.write_type_a("position", new geometry_msgs.msg.dds.PointPubSubType(), data.getPosition());

      ser.write_type_a("orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getOrientation());

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
      ser.read_type_a("position", new geometry_msgs.msg.dds.PointPubSubType(), data.getPosition());

      ser.read_type_a("orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getOrientation());

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
