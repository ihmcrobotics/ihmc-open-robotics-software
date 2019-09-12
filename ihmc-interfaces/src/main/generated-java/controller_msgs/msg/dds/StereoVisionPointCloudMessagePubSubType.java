package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "StereoVisionPointCloudMessage" defined in "StereoVisionPointCloudMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from StereoVisionPointCloudMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit StereoVisionPointCloudMessage_.idl instead.
*
*/
public class StereoVisionPointCloudMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.StereoVisionPointCloudMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::StereoVisionPointCloudMessage_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.StereoVisionPointCloudMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.StereoVisionPointCloudMessage data) throws java.io.IOException
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

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (600000 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (200000 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.StereoVisionPointCloudMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.StereoVisionPointCloudMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getSensorPosition(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getCdrSerializedSize(data.getSensorOrientation(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getPointCloud().size() * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getColors().size() * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.StereoVisionPointCloudMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_11(data.getTimestamp());

      geometry_msgs.msg.dds.PointPubSubType.write(data.getSensorPosition(), cdr);
      geometry_msgs.msg.dds.QuaternionPubSubType.write(data.getSensorOrientation(), cdr);
      if(data.getPointCloud().size() <= 600000)
      cdr.write_type_e(data.getPointCloud());else
          throw new RuntimeException("point_cloud field exceeds the maximum length");

      if(data.getColors().size() <= 200000)
      cdr.write_type_e(data.getColors());else
          throw new RuntimeException("colors field exceeds the maximum length");

   }

   public static void read(controller_msgs.msg.dds.StereoVisionPointCloudMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setTimestamp(cdr.read_type_11());
      	
      geometry_msgs.msg.dds.PointPubSubType.read(data.getSensorPosition(), cdr);	
      geometry_msgs.msg.dds.QuaternionPubSubType.read(data.getSensorOrientation(), cdr);	
      cdr.read_type_e(data.getPointCloud());	
      cdr.read_type_e(data.getColors());	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.StereoVisionPointCloudMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_11("timestamp", data.getTimestamp());
      ser.write_type_a("sensor_position", new geometry_msgs.msg.dds.PointPubSubType(), data.getSensorPosition());

      ser.write_type_a("sensor_orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getSensorOrientation());

      ser.write_type_e("point_cloud", data.getPointCloud());
      ser.write_type_e("colors", data.getColors());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.StereoVisionPointCloudMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setTimestamp(ser.read_type_11("timestamp"));
      ser.read_type_a("sensor_position", new geometry_msgs.msg.dds.PointPubSubType(), data.getSensorPosition());

      ser.read_type_a("sensor_orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getSensorOrientation());

      ser.read_type_e("point_cloud", data.getPointCloud());
      ser.read_type_e("colors", data.getColors());
   }

   public static void staticCopy(controller_msgs.msg.dds.StereoVisionPointCloudMessage src, controller_msgs.msg.dds.StereoVisionPointCloudMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.StereoVisionPointCloudMessage createData()
   {
      return new controller_msgs.msg.dds.StereoVisionPointCloudMessage();
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
   
   public void serialize(controller_msgs.msg.dds.StereoVisionPointCloudMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.StereoVisionPointCloudMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.StereoVisionPointCloudMessage src, controller_msgs.msg.dds.StereoVisionPointCloudMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public StereoVisionPointCloudMessagePubSubType newInstance()
   {
      return new StereoVisionPointCloudMessagePubSubType();
   }
}
