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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);

      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (25000000 * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getMaxCdrSerializedSize(current_alignment);


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


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);


      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getData().size() * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);


      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getPosition(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getCdrSerializedSize(data.getOrientation(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.ImageMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceNumber());

      cdr.write_type_4(data.getAcquisitionTimeSecondsSinceEpoch());

      cdr.write_type_4(data.getAcquisitionTimeAdditionalNanos());

      cdr.write_type_3(data.getImageWidth());

      cdr.write_type_3(data.getImageHeight());

      if(data.getData().size() <= 25000000)
      cdr.write_type_e(data.getData());else
          throw new RuntimeException("data field exceeds the maximum length");

      cdr.write_type_3(data.getFormat());

      geometry_msgs.msg.dds.PointPubSubType.write(data.getPosition(), cdr);
      geometry_msgs.msg.dds.QuaternionPubSubType.write(data.getOrientation(), cdr);
   }

   public static void read(perception_msgs.msg.dds.ImageMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceNumber(cdr.read_type_4());
      	
      data.setAcquisitionTimeSecondsSinceEpoch(cdr.read_type_4());
      	
      data.setAcquisitionTimeAdditionalNanos(cdr.read_type_4());
      	
      data.setImageWidth(cdr.read_type_3());
      	
      data.setImageHeight(cdr.read_type_3());
      	
      cdr.read_type_e(data.getData());	
      data.setFormat(cdr.read_type_3());
      	
      geometry_msgs.msg.dds.PointPubSubType.read(data.getPosition(), cdr);	
      geometry_msgs.msg.dds.QuaternionPubSubType.read(data.getOrientation(), cdr);	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.ImageMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_number", data.getSequenceNumber());
      ser.write_type_4("acquisition_time_seconds_since_epoch", data.getAcquisitionTimeSecondsSinceEpoch());
      ser.write_type_4("acquisition_time_additional_nanos", data.getAcquisitionTimeAdditionalNanos());
      ser.write_type_3("image_width", data.getImageWidth());
      ser.write_type_3("image_height", data.getImageHeight());
      ser.write_type_e("data", data.getData());
      ser.write_type_3("format", data.getFormat());
      ser.write_type_a("position", new geometry_msgs.msg.dds.PointPubSubType(), data.getPosition());

      ser.write_type_a("orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getOrientation());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.ImageMessage data)
   {
      data.setSequenceNumber(ser.read_type_4("sequence_number"));
      data.setAcquisitionTimeSecondsSinceEpoch(ser.read_type_4("acquisition_time_seconds_since_epoch"));
      data.setAcquisitionTimeAdditionalNanos(ser.read_type_4("acquisition_time_additional_nanos"));
      data.setImageWidth(ser.read_type_3("image_width"));
      data.setImageHeight(ser.read_type_3("image_height"));
      ser.read_type_e("data", data.getData());
      data.setFormat(ser.read_type_3("format"));
      ser.read_type_a("position", new geometry_msgs.msg.dds.PointPubSubType(), data.getPosition());

      ser.read_type_a("orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getOrientation());

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
