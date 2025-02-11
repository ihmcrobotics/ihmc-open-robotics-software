package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "YOLOv8ModelInfo" defined in "YOLOv8ModelInfo_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from YOLOv8ModelInfo_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit YOLOv8ModelInfo_.idl instead.
*
*/
public class YOLOv8ModelInfoPubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.YOLOv8ModelInfo>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::YOLOv8ModelInfo_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "c2a03b2b0310e9d9fd08d81d4a1e5281af81a647880a6ea54255f3e4af2f2842";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.YOLOv8ModelInfo data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.YOLOv8ModelInfo data) throws java.io.IOException
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
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 96; ++i0)
      {
        current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      }

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.YOLOv8ModelInfo data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.YOLOv8ModelInfo data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getModelName().length() + 1;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getDetectableObjectClasses().size(); ++i0)
      {
          current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getDetectableObjectClasses().get(i0).length() + 1;
      }

      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.YOLOv8ModelInfo data, us.ihmc.idl.CDR cdr)
   {
      if(data.getModelName().length() <= 255)
      cdr.write_type_d(data.getModelName());else
          throw new RuntimeException("model_name field exceeds the maximum length");

      if(data.getDetectableObjectClasses().size() <= 96)
      cdr.write_type_e(data.getDetectableObjectClasses());else
          throw new RuntimeException("detectable_object_classes field exceeds the maximum length");

   }

   public static void read(perception_msgs.msg.dds.YOLOv8ModelInfo data, us.ihmc.idl.CDR cdr)
   {
      cdr.read_type_d(data.getModelName());	
      cdr.read_type_e(data.getDetectableObjectClasses());	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.YOLOv8ModelInfo data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_d("model_name", data.getModelName());
      ser.write_type_e("detectable_object_classes", data.getDetectableObjectClasses());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.YOLOv8ModelInfo data)
   {
      ser.read_type_d("model_name", data.getModelName());
      ser.read_type_e("detectable_object_classes", data.getDetectableObjectClasses());
   }

   public static void staticCopy(perception_msgs.msg.dds.YOLOv8ModelInfo src, perception_msgs.msg.dds.YOLOv8ModelInfo dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.YOLOv8ModelInfo createData()
   {
      return new perception_msgs.msg.dds.YOLOv8ModelInfo();
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
   
   public void serialize(perception_msgs.msg.dds.YOLOv8ModelInfo data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.YOLOv8ModelInfo data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.YOLOv8ModelInfo src, perception_msgs.msg.dds.YOLOv8ModelInfo dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public YOLOv8ModelInfoPubSubType newInstance()
   {
      return new YOLOv8ModelInfoPubSubType();
   }
}
