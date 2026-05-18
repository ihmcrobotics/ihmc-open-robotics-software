package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "YOLOv8AnnotationInfoMessage" defined in "YOLOv8AnnotationInfoMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from YOLOv8AnnotationInfoMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit YOLOv8AnnotationInfoMessage_.idl instead.
*
*/
public class YOLOv8AnnotationInfoMessagePubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.YOLOv8AnnotationInfoMessage>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::YOLOv8AnnotationInfoMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "d4bc03cc433ac47231e6a4b32709f75161898593ebac06174b7b614449338f7e";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.YOLOv8AnnotationInfoMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.YOLOv8AnnotationInfoMessage data) throws java.io.IOException
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
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += vision_msgs.msg.dds.BoundingBox2DPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += perception_msgs.msg.dds.Float32MultiArrayHackPubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.YOLOv8AnnotationInfoMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.YOLOv8AnnotationInfoMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getObjectClass().length() + 1;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += vision_msgs.msg.dds.BoundingBox2DPubSubType.getCdrSerializedSize(data.getBoundingBox(), current_alignment);

      current_alignment += perception_msgs.msg.dds.Float32MultiArrayHackPubSubType.getCdrSerializedSize(data.getMaskPolygons(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.YOLOv8AnnotationInfoMessage data, us.ihmc.idl.CDR cdr)
   {
      if(data.getObjectClass().length() <= 255)
      cdr.write_type_d(data.getObjectClass());else
          throw new RuntimeException("object_class field exceeds the maximum length: %d > %d".formatted(data.getObjectClass().length(), 255));

      cdr.write_type_5(data.getConfidence());

      vision_msgs.msg.dds.BoundingBox2DPubSubType.write(data.getBoundingBox(), cdr);
      perception_msgs.msg.dds.Float32MultiArrayHackPubSubType.write(data.getMaskPolygons(), cdr);
   }

   public static void read(perception_msgs.msg.dds.YOLOv8AnnotationInfoMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.read_type_d(data.getObjectClass());	
      data.setConfidence(cdr.read_type_5());
      	
      vision_msgs.msg.dds.BoundingBox2DPubSubType.read(data.getBoundingBox(), cdr);	
      perception_msgs.msg.dds.Float32MultiArrayHackPubSubType.read(data.getMaskPolygons(), cdr);	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.YOLOv8AnnotationInfoMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_d("object_class", data.getObjectClass());
      ser.write_type_5("confidence", data.getConfidence());
      ser.write_type_a("bounding_box", new vision_msgs.msg.dds.BoundingBox2DPubSubType(), data.getBoundingBox());

      ser.write_type_a("mask_polygons", new perception_msgs.msg.dds.Float32MultiArrayHackPubSubType(), data.getMaskPolygons());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.YOLOv8AnnotationInfoMessage data)
   {
      ser.read_type_d("object_class", data.getObjectClass());
      data.setConfidence(ser.read_type_5("confidence"));
      ser.read_type_a("bounding_box", new vision_msgs.msg.dds.BoundingBox2DPubSubType(), data.getBoundingBox());

      ser.read_type_a("mask_polygons", new perception_msgs.msg.dds.Float32MultiArrayHackPubSubType(), data.getMaskPolygons());

   }

   public static void staticCopy(perception_msgs.msg.dds.YOLOv8AnnotationInfoMessage src, perception_msgs.msg.dds.YOLOv8AnnotationInfoMessage dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.YOLOv8AnnotationInfoMessage createData()
   {
      return new perception_msgs.msg.dds.YOLOv8AnnotationInfoMessage();
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
   
   public void serialize(perception_msgs.msg.dds.YOLOv8AnnotationInfoMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.YOLOv8AnnotationInfoMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.YOLOv8AnnotationInfoMessage src, perception_msgs.msg.dds.YOLOv8AnnotationInfoMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public YOLOv8AnnotationInfoMessagePubSubType newInstance()
   {
      return new YOLOv8AnnotationInfoMessagePubSubType();
   }
}
