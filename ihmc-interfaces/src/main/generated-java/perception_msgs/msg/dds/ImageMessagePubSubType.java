package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "ImageMessage" defined in "ImageMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from ImageMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit ImageMessage_.idl instead.
*
*/
public class ImageMessagePubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.ImageMessage>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::ImageMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "861742e3ba65a4001a48beb5dfe643935ac3c8bc1b1014cd07c2e7a9b324d057";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.ImageMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.ImageMessage data) throws java.io.IOException
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

      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);

      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (25000000 * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (128 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (128 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.ImageMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.ImageMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += ihmc_common_msgs.msg.dds.InstantMessagePubSubType.getCdrSerializedSize(data.getAcquisitionTime(), current_alignment);

      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);


      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getData().size() * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getPosition(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getCdrSerializedSize(data.getOrientation(), current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getOusterBeamAltitudeAngles().size() * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getOusterBeamAzimuthAngles().size() * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.ImageMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceNumber());

      ihmc_common_msgs.msg.dds.InstantMessagePubSubType.write(data.getAcquisitionTime(), cdr);
      cdr.write_type_3(data.getImageWidth());

      cdr.write_type_3(data.getImageHeight());

      cdr.write_type_5(data.getDepthDiscretization());

      if(data.getData().size() <= 25000000)
      cdr.write_type_e(data.getData());else
          throw new RuntimeException("data field exceeds the maximum length");

      cdr.write_type_9(data.getFormat());

      geometry_msgs.msg.dds.PointPubSubType.write(data.getPosition(), cdr);
      geometry_msgs.msg.dds.QuaternionPubSubType.write(data.getOrientation(), cdr);
      cdr.write_type_9(data.getCameraModel());

      cdr.write_type_5(data.getFocalLengthXPixels());

      cdr.write_type_5(data.getFocalLengthYPixels());

      cdr.write_type_5(data.getPrincipalPointXPixels());

      cdr.write_type_5(data.getPrincipalPointYPixels());

      if(data.getOusterBeamAltitudeAngles().size() <= 128)
      cdr.write_type_e(data.getOusterBeamAltitudeAngles());else
          throw new RuntimeException("ouster_beam_altitude_angles field exceeds the maximum length");

      if(data.getOusterBeamAzimuthAngles().size() <= 128)
      cdr.write_type_e(data.getOusterBeamAzimuthAngles());else
          throw new RuntimeException("ouster_beam_azimuth_angles field exceeds the maximum length");

   }

   public static void read(perception_msgs.msg.dds.ImageMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceNumber(cdr.read_type_4());
      	
      ihmc_common_msgs.msg.dds.InstantMessagePubSubType.read(data.getAcquisitionTime(), cdr);	
      data.setImageWidth(cdr.read_type_3());
      	
      data.setImageHeight(cdr.read_type_3());
      	
      data.setDepthDiscretization(cdr.read_type_5());
      	
      cdr.read_type_e(data.getData());	
      data.setFormat(cdr.read_type_9());
      	
      geometry_msgs.msg.dds.PointPubSubType.read(data.getPosition(), cdr);	
      geometry_msgs.msg.dds.QuaternionPubSubType.read(data.getOrientation(), cdr);	
      data.setCameraModel(cdr.read_type_9());
      	
      data.setFocalLengthXPixels(cdr.read_type_5());
      	
      data.setFocalLengthYPixels(cdr.read_type_5());
      	
      data.setPrincipalPointXPixels(cdr.read_type_5());
      	
      data.setPrincipalPointYPixels(cdr.read_type_5());
      	
      cdr.read_type_e(data.getOusterBeamAltitudeAngles());	
      cdr.read_type_e(data.getOusterBeamAzimuthAngles());	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.ImageMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_number", data.getSequenceNumber());
      ser.write_type_a("acquisition_time", new ihmc_common_msgs.msg.dds.InstantMessagePubSubType(), data.getAcquisitionTime());

      ser.write_type_3("image_width", data.getImageWidth());
      ser.write_type_3("image_height", data.getImageHeight());
      ser.write_type_5("depth_discretization", data.getDepthDiscretization());
      ser.write_type_e("data", data.getData());
      ser.write_type_9("format", data.getFormat());
      ser.write_type_a("position", new geometry_msgs.msg.dds.PointPubSubType(), data.getPosition());

      ser.write_type_a("orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getOrientation());

      ser.write_type_9("camera_model", data.getCameraModel());
      ser.write_type_5("focal_length_x_pixels", data.getFocalLengthXPixels());
      ser.write_type_5("focal_length_y_pixels", data.getFocalLengthYPixels());
      ser.write_type_5("principal_point_x_pixels", data.getPrincipalPointXPixels());
      ser.write_type_5("principal_point_y_pixels", data.getPrincipalPointYPixels());
      ser.write_type_e("ouster_beam_altitude_angles", data.getOusterBeamAltitudeAngles());
      ser.write_type_e("ouster_beam_azimuth_angles", data.getOusterBeamAzimuthAngles());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.ImageMessage data)
   {
      data.setSequenceNumber(ser.read_type_4("sequence_number"));
      ser.read_type_a("acquisition_time", new ihmc_common_msgs.msg.dds.InstantMessagePubSubType(), data.getAcquisitionTime());

      data.setImageWidth(ser.read_type_3("image_width"));
      data.setImageHeight(ser.read_type_3("image_height"));
      data.setDepthDiscretization(ser.read_type_5("depth_discretization"));
      ser.read_type_e("data", data.getData());
      data.setFormat(ser.read_type_9("format"));
      ser.read_type_a("position", new geometry_msgs.msg.dds.PointPubSubType(), data.getPosition());

      ser.read_type_a("orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getOrientation());

      data.setCameraModel(ser.read_type_9("camera_model"));
      data.setFocalLengthXPixels(ser.read_type_5("focal_length_x_pixels"));
      data.setFocalLengthYPixels(ser.read_type_5("focal_length_y_pixels"));
      data.setPrincipalPointXPixels(ser.read_type_5("principal_point_x_pixels"));
      data.setPrincipalPointYPixels(ser.read_type_5("principal_point_y_pixels"));
      ser.read_type_e("ouster_beam_altitude_angles", data.getOusterBeamAltitudeAngles());
      ser.read_type_e("ouster_beam_azimuth_angles", data.getOusterBeamAzimuthAngles());
   }

   public static void staticCopy(perception_msgs.msg.dds.ImageMessage src, perception_msgs.msg.dds.ImageMessage dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.ImageMessage createData()
   {
      return new perception_msgs.msg.dds.ImageMessage();
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
   
   public void serialize(perception_msgs.msg.dds.ImageMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.ImageMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.ImageMessage src, perception_msgs.msg.dds.ImageMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public ImageMessagePubSubType newInstance()
   {
      return new ImageMessagePubSubType();
   }
}
