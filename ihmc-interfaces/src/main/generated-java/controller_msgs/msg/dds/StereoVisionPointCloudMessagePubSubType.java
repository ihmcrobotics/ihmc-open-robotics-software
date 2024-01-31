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
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "3e76445e4ac77629618271518702b598fc59e147ad9651760b5970d8284ae004";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

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

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (20000000 * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (7000000 * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


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

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getPointCloudCenter(), current_alignment);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getPointCloud().size() * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getColors().size() * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.StereoVisionPointCloudMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_11(data.getTimestamp());

      geometry_msgs.msg.dds.PointPubSubType.write(data.getSensorPosition(), cdr);
      geometry_msgs.msg.dds.QuaternionPubSubType.write(data.getSensorOrientation(), cdr);
      cdr.write_type_7(data.getIsDataLocalToSensor());

      cdr.write_type_6(data.getSensorPoseConfidence());

      cdr.write_type_6(data.getPointCloudConfidence());

      geometry_msgs.msg.dds.PointPubSubType.write(data.getPointCloudCenter(), cdr);
      cdr.write_type_6(data.getResolution());

      cdr.write_type_2(data.getNumberOfPoints());

      if(data.getPointCloud().size() <= 20000000)
      cdr.write_type_e(data.getPointCloud());else
          throw new RuntimeException("point_cloud field exceeds the maximum length");

      if(data.getColors().size() <= 7000000)
      cdr.write_type_e(data.getColors());else
          throw new RuntimeException("colors field exceeds the maximum length");

      cdr.write_type_7(data.getLz4Compressed());

   }

   public static void read(controller_msgs.msg.dds.StereoVisionPointCloudMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setTimestamp(cdr.read_type_11());
      	
      geometry_msgs.msg.dds.PointPubSubType.read(data.getSensorPosition(), cdr);	
      geometry_msgs.msg.dds.QuaternionPubSubType.read(data.getSensorOrientation(), cdr);	
      data.setIsDataLocalToSensor(cdr.read_type_7());
      	
      data.setSensorPoseConfidence(cdr.read_type_6());
      	
      data.setPointCloudConfidence(cdr.read_type_6());
      	
      geometry_msgs.msg.dds.PointPubSubType.read(data.getPointCloudCenter(), cdr);	
      data.setResolution(cdr.read_type_6());
      	
      data.setNumberOfPoints(cdr.read_type_2());
      	
      cdr.read_type_e(data.getPointCloud());	
      cdr.read_type_e(data.getColors());	
      data.setLz4Compressed(cdr.read_type_7());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.StereoVisionPointCloudMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_11("timestamp", data.getTimestamp());
      ser.write_type_a("sensor_position", new geometry_msgs.msg.dds.PointPubSubType(), data.getSensorPosition());

      ser.write_type_a("sensor_orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getSensorOrientation());

      ser.write_type_7("is_data_local_to_sensor", data.getIsDataLocalToSensor());
      ser.write_type_6("sensor_pose_confidence", data.getSensorPoseConfidence());
      ser.write_type_6("point_cloud_confidence", data.getPointCloudConfidence());
      ser.write_type_a("point_cloud_center", new geometry_msgs.msg.dds.PointPubSubType(), data.getPointCloudCenter());

      ser.write_type_6("resolution", data.getResolution());
      ser.write_type_2("number_of_points", data.getNumberOfPoints());
      ser.write_type_e("point_cloud", data.getPointCloud());
      ser.write_type_e("colors", data.getColors());
      ser.write_type_7("lz4_compressed", data.getLz4Compressed());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.StereoVisionPointCloudMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setTimestamp(ser.read_type_11("timestamp"));
      ser.read_type_a("sensor_position", new geometry_msgs.msg.dds.PointPubSubType(), data.getSensorPosition());

      ser.read_type_a("sensor_orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getSensorOrientation());

      data.setIsDataLocalToSensor(ser.read_type_7("is_data_local_to_sensor"));
      data.setSensorPoseConfidence(ser.read_type_6("sensor_pose_confidence"));
      data.setPointCloudConfidence(ser.read_type_6("point_cloud_confidence"));
      ser.read_type_a("point_cloud_center", new geometry_msgs.msg.dds.PointPubSubType(), data.getPointCloudCenter());

      data.setResolution(ser.read_type_6("resolution"));
      data.setNumberOfPoints(ser.read_type_2("number_of_points"));
      ser.read_type_e("point_cloud", data.getPointCloud());
      ser.read_type_e("colors", data.getColors());
      data.setLz4Compressed(ser.read_type_7("lz4_compressed"));
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
