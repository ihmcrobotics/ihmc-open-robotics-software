package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "YOLOv8ResultAnnotationInfo" defined in "YOLOv8ResultAnnotationInfo_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from YOLOv8ResultAnnotationInfo_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit YOLOv8ResultAnnotationInfo_.idl instead.
*
*/
public class YOLOv8ResultAnnotationInfoPubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.YOLOv8ResultAnnotationInfo>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::YOLOv8ResultAnnotationInfo_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "eb6eb5cbb2644b64e7aebfba1881e33bc5735840fddc845f5ec5cad4648678e4";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.YOLOv8ResultAnnotationInfo data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.YOLOv8ResultAnnotationInfo data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 100; ++i0)
      {
          current_alignment += perception_msgs.msg.dds.YOLOv8DetectionAnnotationInfoPubSubType.getMaxCdrSerializedSize(current_alignment);}
      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.YOLOv8ResultAnnotationInfo data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.YOLOv8ResultAnnotationInfo data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getDetectionAnnotationInfos().size(); ++i0)
      {
          current_alignment += perception_msgs.msg.dds.YOLOv8DetectionAnnotationInfoPubSubType.getCdrSerializedSize(data.getDetectionAnnotationInfos().get(i0), current_alignment);}

      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.YOLOv8ResultAnnotationInfo data, us.ihmc.idl.CDR cdr)
   {
      if(data.getDetectionAnnotationInfos().size() <= 100)
      cdr.write_type_e(data.getDetectionAnnotationInfos());else
          throw new RuntimeException("detection_annotation_infos field exceeds the maximum length: %d > %d".formatted(data.getDetectionAnnotationInfos().size(), 100));

   }

   public static void read(perception_msgs.msg.dds.YOLOv8ResultAnnotationInfo data, us.ihmc.idl.CDR cdr)
   {
      cdr.read_type_e(data.getDetectionAnnotationInfos());	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.YOLOv8ResultAnnotationInfo data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_e("detection_annotation_infos", data.getDetectionAnnotationInfos());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.YOLOv8ResultAnnotationInfo data)
   {
      ser.read_type_e("detection_annotation_infos", data.getDetectionAnnotationInfos());
   }

   public static void staticCopy(perception_msgs.msg.dds.YOLOv8ResultAnnotationInfo src, perception_msgs.msg.dds.YOLOv8ResultAnnotationInfo dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.YOLOv8ResultAnnotationInfo createData()
   {
      return new perception_msgs.msg.dds.YOLOv8ResultAnnotationInfo();
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
   
   public void serialize(perception_msgs.msg.dds.YOLOv8ResultAnnotationInfo data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.YOLOv8ResultAnnotationInfo data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.YOLOv8ResultAnnotationInfo src, perception_msgs.msg.dds.YOLOv8ResultAnnotationInfo dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public YOLOv8ResultAnnotationInfoPubSubType newInstance()
   {
      return new YOLOv8ResultAnnotationInfoPubSubType();
   }
}
