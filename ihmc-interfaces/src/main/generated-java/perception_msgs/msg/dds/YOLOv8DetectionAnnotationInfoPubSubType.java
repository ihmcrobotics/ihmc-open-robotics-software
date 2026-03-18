package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "YOLOv8DetectionAnnotationInfo" defined in "YOLOv8DetectionAnnotationInfo_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from YOLOv8DetectionAnnotationInfo_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit YOLOv8DetectionAnnotationInfo_.idl instead.
*
*/
public class YOLOv8DetectionAnnotationInfoPubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.YOLOv8DetectionAnnotationInfo>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::YOLOv8DetectionAnnotationInfo_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "b7109ae98afbd4d43e97fa8938340c5b2c930b0aef77211be88f52ec129605d2";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.YOLOv8DetectionAnnotationInfo data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.YOLOv8DetectionAnnotationInfo data) throws java.io.IOException
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

      current_alignment += ihmc_common_msgs.msg.dds.InstantMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      current_alignment += vision_msgs.msg.dds.BoundingBox2DPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 100; ++i0)
      {
          current_alignment += perception_msgs.msg.dds.Point2DArrayPubSubType.getMaxCdrSerializedSize(current_alignment);}

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.YOLOv8DetectionAnnotationInfo data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.YOLOv8DetectionAnnotationInfo data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += ihmc_common_msgs.msg.dds.InstantMessagePubSubType.getCdrSerializedSize(data.getDetectionInstant(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getObjectClass().length() + 1;

      current_alignment += vision_msgs.msg.dds.BoundingBox2DPubSubType.getCdrSerializedSize(data.getBoundingBox(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getMaskPolygons().size(); ++i0)
      {
          current_alignment += perception_msgs.msg.dds.Point2DArrayPubSubType.getCdrSerializedSize(data.getMaskPolygons().get(i0), current_alignment);}


      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.YOLOv8DetectionAnnotationInfo data, us.ihmc.idl.CDR cdr)
   {
      ihmc_common_msgs.msg.dds.InstantMessagePubSubType.write(data.getDetectionInstant(), cdr);
      if(data.getObjectClass().length() <= 255)
      cdr.write_type_d(data.getObjectClass());else
          throw new RuntimeException("object_class field exceeds the maximum length: %d > %d".formatted(data.getObjectClass().length(), 255));

      vision_msgs.msg.dds.BoundingBox2DPubSubType.write(data.getBoundingBox(), cdr);
      if(data.getMaskPolygons().size() <= 100)
      cdr.write_type_e(data.getMaskPolygons());else
          throw new RuntimeException("mask_polygons field exceeds the maximum length: %d > %d".formatted(data.getMaskPolygons().size(), 100));

   }

   public static void read(perception_msgs.msg.dds.YOLOv8DetectionAnnotationInfo data, us.ihmc.idl.CDR cdr)
   {
      ihmc_common_msgs.msg.dds.InstantMessagePubSubType.read(data.getDetectionInstant(), cdr);	
      cdr.read_type_d(data.getObjectClass());	
      vision_msgs.msg.dds.BoundingBox2DPubSubType.read(data.getBoundingBox(), cdr);	
      cdr.read_type_e(data.getMaskPolygons());	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.YOLOv8DetectionAnnotationInfo data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("detection_instant", new ihmc_common_msgs.msg.dds.InstantMessagePubSubType(), data.getDetectionInstant());

      ser.write_type_d("object_class", data.getObjectClass());
      ser.write_type_a("bounding_box", new vision_msgs.msg.dds.BoundingBox2DPubSubType(), data.getBoundingBox());

      ser.write_type_e("mask_polygons", data.getMaskPolygons());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.YOLOv8DetectionAnnotationInfo data)
   {
      ser.read_type_a("detection_instant", new ihmc_common_msgs.msg.dds.InstantMessagePubSubType(), data.getDetectionInstant());

      ser.read_type_d("object_class", data.getObjectClass());
      ser.read_type_a("bounding_box", new vision_msgs.msg.dds.BoundingBox2DPubSubType(), data.getBoundingBox());

      ser.read_type_e("mask_polygons", data.getMaskPolygons());
   }

   public static void staticCopy(perception_msgs.msg.dds.YOLOv8DetectionAnnotationInfo src, perception_msgs.msg.dds.YOLOv8DetectionAnnotationInfo dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.YOLOv8DetectionAnnotationInfo createData()
   {
      return new perception_msgs.msg.dds.YOLOv8DetectionAnnotationInfo();
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
   
   public void serialize(perception_msgs.msg.dds.YOLOv8DetectionAnnotationInfo data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.YOLOv8DetectionAnnotationInfo data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.YOLOv8DetectionAnnotationInfo src, perception_msgs.msg.dds.YOLOv8DetectionAnnotationInfo dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public YOLOv8DetectionAnnotationInfoPubSubType newInstance()
   {
      return new YOLOv8DetectionAnnotationInfoPubSubType();
   }
}
