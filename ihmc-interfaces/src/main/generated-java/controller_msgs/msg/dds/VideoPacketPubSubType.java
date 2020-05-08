package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "VideoPacket" defined in "VideoPacket_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from VideoPacket_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit VideoPacket_.idl instead.
*
*/
public class VideoPacketPubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.VideoPacket>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::VideoPacket_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.VideoPacket data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.VideoPacket data) throws java.io.IOException
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


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (262144 * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);


      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getMaxCdrSerializedSize(current_alignment);


      current_alignment += controller_msgs.msg.dds.IntrinsicParametersMessagePubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.VideoPacket data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.VideoPacket data, int current_alignment)
   {
      int initial_alignment = current_alignment;


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getData().size() * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getPosition(), current_alignment);


      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getCdrSerializedSize(data.getOrientation(), current_alignment);


      current_alignment += controller_msgs.msg.dds.IntrinsicParametersMessagePubSubType.getCdrSerializedSize(data.getIntrinsicParameters(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.VideoPacket data, us.ihmc.idl.CDR cdr)
   {

      cdr.write_type_4(data.getSequenceId());


      cdr.write_type_9(data.getVideoSource());


      cdr.write_type_11(data.getTimestamp());


      if(data.getData().size() <= 262144)
      cdr.write_type_e(data.getData());else
          throw new RuntimeException("data field exceeds the maximum length");


      geometry_msgs.msg.dds.PointPubSubType.write(data.getPosition(), cdr);

      geometry_msgs.msg.dds.QuaternionPubSubType.write(data.getOrientation(), cdr);

      controller_msgs.msg.dds.IntrinsicParametersMessagePubSubType.write(data.getIntrinsicParameters(), cdr);
   }

   public static void read(controller_msgs.msg.dds.VideoPacket data, us.ihmc.idl.CDR cdr)
   {

      data.setSequenceId(cdr.read_type_4());
      	

      data.setVideoSource(cdr.read_type_9());
      	

      data.setTimestamp(cdr.read_type_11());
      	

      cdr.read_type_e(data.getData());	

      geometry_msgs.msg.dds.PointPubSubType.read(data.getPosition(), cdr);	

      geometry_msgs.msg.dds.QuaternionPubSubType.read(data.getOrientation(), cdr);	

      controller_msgs.msg.dds.IntrinsicParametersMessagePubSubType.read(data.getIntrinsicParameters(), cdr);	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.VideoPacket data, us.ihmc.idl.InterchangeSerializer ser)
   {

      ser.write_type_4("sequence_id", data.getSequenceId());

      ser.write_type_9("video_source", data.getVideoSource());

      ser.write_type_11("timestamp", data.getTimestamp());

      ser.write_type_e("data", data.getData());

      ser.write_type_a("position", new geometry_msgs.msg.dds.PointPubSubType(), data.getPosition());


      ser.write_type_a("orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getOrientation());


      ser.write_type_a("intrinsic_parameters", new controller_msgs.msg.dds.IntrinsicParametersMessagePubSubType(), data.getIntrinsicParameters());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.VideoPacket data)
   {

      data.setSequenceId(ser.read_type_4("sequence_id"));

      data.setVideoSource(ser.read_type_9("video_source"));

      data.setTimestamp(ser.read_type_11("timestamp"));

      ser.read_type_e("data", data.getData());

      ser.read_type_a("position", new geometry_msgs.msg.dds.PointPubSubType(), data.getPosition());


      ser.read_type_a("orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getOrientation());


      ser.read_type_a("intrinsic_parameters", new controller_msgs.msg.dds.IntrinsicParametersMessagePubSubType(), data.getIntrinsicParameters());

   }

   public static void staticCopy(controller_msgs.msg.dds.VideoPacket src, controller_msgs.msg.dds.VideoPacket dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.VideoPacket createData()
   {
      return new controller_msgs.msg.dds.VideoPacket();
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
   
   public void serialize(controller_msgs.msg.dds.VideoPacket data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.VideoPacket data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.VideoPacket src, controller_msgs.msg.dds.VideoPacket dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public VideoPacketPubSubType newInstance()
   {
      return new VideoPacketPubSubType();
   }
}
