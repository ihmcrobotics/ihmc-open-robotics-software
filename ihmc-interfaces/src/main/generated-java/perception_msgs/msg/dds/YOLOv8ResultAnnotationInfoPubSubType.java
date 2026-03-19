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
   		return "0f5ffa7e3c40c371ff397a9a0f207048b2650eec58261c8dad73fd302efdbf75";
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

      current_alignment += ihmc_common_msgs.msg.dds.InstantMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 100; ++i0)
      {
          current_alignment += perception_msgs.msg.dds.YOLOv8AnnotationRecordMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.YOLOv8ResultAnnotationInfo data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.YOLOv8ResultAnnotationInfo data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += ihmc_common_msgs.msg.dds.InstantMessagePubSubType.getCdrSerializedSize(data.getDetectionInstant(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getAnnotationRecords().size(); ++i0)
      {
          current_alignment += perception_msgs.msg.dds.YOLOv8AnnotationRecordMessagePubSubType.getCdrSerializedSize(data.getAnnotationRecords().get(i0), current_alignment);}


      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.YOLOv8ResultAnnotationInfo data, us.ihmc.idl.CDR cdr)
   {
      ihmc_common_msgs.msg.dds.InstantMessagePubSubType.write(data.getDetectionInstant(), cdr);
      if(data.getAnnotationRecords().size() <= 100)
      cdr.write_type_e(data.getAnnotationRecords());else
          throw new RuntimeException("annotation_records field exceeds the maximum length: %d > %d".formatted(data.getAnnotationRecords().size(), 100));

   }

   public static void read(perception_msgs.msg.dds.YOLOv8ResultAnnotationInfo data, us.ihmc.idl.CDR cdr)
   {
      ihmc_common_msgs.msg.dds.InstantMessagePubSubType.read(data.getDetectionInstant(), cdr);	
      cdr.read_type_e(data.getAnnotationRecords());	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.YOLOv8ResultAnnotationInfo data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("detection_instant", new ihmc_common_msgs.msg.dds.InstantMessagePubSubType(), data.getDetectionInstant());

      ser.write_type_e("annotation_records", data.getAnnotationRecords());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.YOLOv8ResultAnnotationInfo data)
   {
      ser.read_type_a("detection_instant", new ihmc_common_msgs.msg.dds.InstantMessagePubSubType(), data.getDetectionInstant());

      ser.read_type_e("annotation_records", data.getAnnotationRecords());
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
