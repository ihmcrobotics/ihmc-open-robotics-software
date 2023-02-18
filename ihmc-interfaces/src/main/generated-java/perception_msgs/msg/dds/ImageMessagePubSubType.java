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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (25000000 * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


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
      current_alignment += (data.getData().size() * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);


      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getPosition(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getCdrSerializedSize(data.getOrientation(), current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.ImageMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceNumber());

      ihmc_common_msgs.msg.dds.InstantMessagePubSubType.write(data.getAcquisitionTime(), cdr);
      cdr.write_type_3(data.getImageWidth());

      cdr.write_type_3(data.getImageHeight());

      if(data.getData().size() <= 25000000)
      cdr.write_type_e(data.getData());else
          throw new RuntimeException("data field exceeds the maximum length");

      cdr.write_type_3(data.getFormat());

      geometry_msgs.msg.dds.PointPubSubType.write(data.getPosition(), cdr);
      geometry_msgs.msg.dds.QuaternionPubSubType.write(data.getOrientation(), cdr);
      cdr.write_type_7(data.getIsPinholeCameraModel());

      cdr.write_type_7(data.getIsEquidistantFisheyeCameraModel());

      cdr.write_type_7(data.getIsOusterCameraModel());

      cdr.write_type_5(data.getFocalLengthXPixels());

      cdr.write_type_5(data.getFocalLengthYPixels());

      cdr.write_type_5(data.getPrincipalPointXPixels());

      cdr.write_type_5(data.getPrincipalPointYPixels());

      cdr.write_type_5(data.getDepthDiscretization());

      cdr.write_type_5(data.getOusterVerticalFieldOfView());

      cdr.write_type_5(data.getOusterHorizontalFieldOfView());

   }

   public static void read(perception_msgs.msg.dds.ImageMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceNumber(cdr.read_type_4());
      	
      ihmc_common_msgs.msg.dds.InstantMessagePubSubType.read(data.getAcquisitionTime(), cdr);	
      data.setImageWidth(cdr.read_type_3());
      	
      data.setImageHeight(cdr.read_type_3());
      	
      cdr.read_type_e(data.getData());	
      data.setFormat(cdr.read_type_3());
      	
      geometry_msgs.msg.dds.PointPubSubType.read(data.getPosition(), cdr);	
      geometry_msgs.msg.dds.QuaternionPubSubType.read(data.getOrientation(), cdr);	
      data.setIsPinholeCameraModel(cdr.read_type_7());
      	
      data.setIsEquidistantFisheyeCameraModel(cdr.read_type_7());
      	
      data.setIsOusterCameraModel(cdr.read_type_7());
      	
      data.setFocalLengthXPixels(cdr.read_type_5());
      	
      data.setFocalLengthYPixels(cdr.read_type_5());
      	
      data.setPrincipalPointXPixels(cdr.read_type_5());
      	
      data.setPrincipalPointYPixels(cdr.read_type_5());
      	
      data.setDepthDiscretization(cdr.read_type_5());
      	
      data.setOusterVerticalFieldOfView(cdr.read_type_5());
      	
      data.setOusterHorizontalFieldOfView(cdr.read_type_5());
      	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.ImageMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_number", data.getSequenceNumber());
      ser.write_type_a("acquisition_time", new ihmc_common_msgs.msg.dds.InstantMessagePubSubType(), data.getAcquisitionTime());

      ser.write_type_3("image_width", data.getImageWidth());
      ser.write_type_3("image_height", data.getImageHeight());
      ser.write_type_e("data", data.getData());
      ser.write_type_3("format", data.getFormat());
      ser.write_type_a("position", new geometry_msgs.msg.dds.PointPubSubType(), data.getPosition());

      ser.write_type_a("orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getOrientation());

      ser.write_type_7("is_pinhole_camera_model", data.getIsPinholeCameraModel());
      ser.write_type_7("is_equidistant_fisheye_camera_model", data.getIsEquidistantFisheyeCameraModel());
      ser.write_type_7("is_ouster_camera_model", data.getIsOusterCameraModel());
      ser.write_type_5("focal_length_x_pixels", data.getFocalLengthXPixels());
      ser.write_type_5("focal_length_y_pixels", data.getFocalLengthYPixels());
      ser.write_type_5("principal_point_x_pixels", data.getPrincipalPointXPixels());
      ser.write_type_5("principal_point_y_pixels", data.getPrincipalPointYPixels());
      ser.write_type_5("depth_discretization", data.getDepthDiscretization());
      ser.write_type_5("ouster_vertical_field_of_view", data.getOusterVerticalFieldOfView());
      ser.write_type_5("ouster_horizontal_field_of_view", data.getOusterHorizontalFieldOfView());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.ImageMessage data)
   {
      data.setSequenceNumber(ser.read_type_4("sequence_number"));
      ser.read_type_a("acquisition_time", new ihmc_common_msgs.msg.dds.InstantMessagePubSubType(), data.getAcquisitionTime());

      data.setImageWidth(ser.read_type_3("image_width"));
      data.setImageHeight(ser.read_type_3("image_height"));
      ser.read_type_e("data", data.getData());
      data.setFormat(ser.read_type_3("format"));
      ser.read_type_a("position", new geometry_msgs.msg.dds.PointPubSubType(), data.getPosition());

      ser.read_type_a("orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getOrientation());

      data.setIsPinholeCameraModel(ser.read_type_7("is_pinhole_camera_model"));
      data.setIsEquidistantFisheyeCameraModel(ser.read_type_7("is_equidistant_fisheye_camera_model"));
      data.setIsOusterCameraModel(ser.read_type_7("is_ouster_camera_model"));
      data.setFocalLengthXPixels(ser.read_type_5("focal_length_x_pixels"));
      data.setFocalLengthYPixels(ser.read_type_5("focal_length_y_pixels"));
      data.setPrincipalPointXPixels(ser.read_type_5("principal_point_x_pixels"));
      data.setPrincipalPointYPixels(ser.read_type_5("principal_point_y_pixels"));
      data.setDepthDiscretization(ser.read_type_5("depth_discretization"));
      data.setOusterVerticalFieldOfView(ser.read_type_5("ouster_vertical_field_of_view"));
      data.setOusterHorizontalFieldOfView(ser.read_type_5("ouster_horizontal_field_of_view"));
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
