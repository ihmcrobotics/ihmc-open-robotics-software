package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "YOLOv8AnnotationInfoList" defined in "YOLOv8AnnotationInfoList_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from YOLOv8AnnotationInfoList_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit YOLOv8AnnotationInfoList_.idl instead.
*
*/
public class YOLOv8AnnotationInfoListPubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.YOLOv8AnnotationInfoList>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::YOLOv8AnnotationInfoList_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "220f3cf61f0b9417fc04aaef0765971f1a7cf934ff3d6c0160170af3859295cd";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.YOLOv8AnnotationInfoList data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.YOLOv8AnnotationInfoList data) throws java.io.IOException
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
          current_alignment += perception_msgs.msg.dds.YOLOv8AnnotationInfoMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.YOLOv8AnnotationInfoList data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.YOLOv8AnnotationInfoList data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += ihmc_common_msgs.msg.dds.InstantMessagePubSubType.getCdrSerializedSize(data.getDetectionInstant(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getAnnotationInfos().size(); ++i0)
      {
          current_alignment += perception_msgs.msg.dds.YOLOv8AnnotationInfoMessagePubSubType.getCdrSerializedSize(data.getAnnotationInfos().get(i0), current_alignment);}


      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.YOLOv8AnnotationInfoList data, us.ihmc.idl.CDR cdr)
   {
      ihmc_common_msgs.msg.dds.InstantMessagePubSubType.write(data.getDetectionInstant(), cdr);
      if(data.getAnnotationInfos().size() <= 100)
      cdr.write_type_e(data.getAnnotationInfos());else
          throw new RuntimeException("annotation_infos field exceeds the maximum length: %d > %d".formatted(data.getAnnotationInfos().size(), 100));

   }

   public static void read(perception_msgs.msg.dds.YOLOv8AnnotationInfoList data, us.ihmc.idl.CDR cdr)
   {
      ihmc_common_msgs.msg.dds.InstantMessagePubSubType.read(data.getDetectionInstant(), cdr);	
      cdr.read_type_e(data.getAnnotationInfos());	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.YOLOv8AnnotationInfoList data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("detection_instant", new ihmc_common_msgs.msg.dds.InstantMessagePubSubType(), data.getDetectionInstant());

      ser.write_type_e("annotation_infos", data.getAnnotationInfos());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.YOLOv8AnnotationInfoList data)
   {
      ser.read_type_a("detection_instant", new ihmc_common_msgs.msg.dds.InstantMessagePubSubType(), data.getDetectionInstant());

      ser.read_type_e("annotation_infos", data.getAnnotationInfos());
   }

   public static void staticCopy(perception_msgs.msg.dds.YOLOv8AnnotationInfoList src, perception_msgs.msg.dds.YOLOv8AnnotationInfoList dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.YOLOv8AnnotationInfoList createData()
   {
      return new perception_msgs.msg.dds.YOLOv8AnnotationInfoList();
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
   
   public void serialize(perception_msgs.msg.dds.YOLOv8AnnotationInfoList data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.YOLOv8AnnotationInfoList data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.YOLOv8AnnotationInfoList src, perception_msgs.msg.dds.YOLOv8AnnotationInfoList dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public YOLOv8AnnotationInfoListPubSubType newInstance()
   {
      return new YOLOv8AnnotationInfoListPubSubType();
   }
}
