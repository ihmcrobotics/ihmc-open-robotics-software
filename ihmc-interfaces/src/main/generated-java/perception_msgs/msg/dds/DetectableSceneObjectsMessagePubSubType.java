package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "DetectableSceneObjectsMessage" defined in "DetectableSceneObjectsMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from DetectableSceneObjectsMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit DetectableSceneObjectsMessage_.idl instead.
*
*/
public class DetectableSceneObjectsMessagePubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.DetectableSceneObjectsMessage>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::DetectableSceneObjectsMessage_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.DetectableSceneObjectsMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.DetectableSceneObjectsMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 5000; ++i0)
      {
          current_alignment += perception_msgs.msg.dds.DetectableSceneObjectMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.DetectableSceneObjectsMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.DetectableSceneObjectsMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getDetectableSceneObjects().size(); ++i0)
      {
          current_alignment += perception_msgs.msg.dds.DetectableSceneObjectMessagePubSubType.getCdrSerializedSize(data.getDetectableSceneObjects().get(i0), current_alignment);}

      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.DetectableSceneObjectsMessage data, us.ihmc.idl.CDR cdr)
   {
      if(data.getDetectableSceneObjects().size() <= 5000)
      cdr.write_type_e(data.getDetectableSceneObjects());else
          throw new RuntimeException("detectable_scene_objects field exceeds the maximum length");

   }

   public static void read(perception_msgs.msg.dds.DetectableSceneObjectsMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.read_type_e(data.getDetectableSceneObjects());	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.DetectableSceneObjectsMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_e("detectable_scene_objects", data.getDetectableSceneObjects());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.DetectableSceneObjectsMessage data)
   {
      ser.read_type_e("detectable_scene_objects", data.getDetectableSceneObjects());
   }

   public static void staticCopy(perception_msgs.msg.dds.DetectableSceneObjectsMessage src, perception_msgs.msg.dds.DetectableSceneObjectsMessage dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.DetectableSceneObjectsMessage createData()
   {
      return new perception_msgs.msg.dds.DetectableSceneObjectsMessage();
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
   
   public void serialize(perception_msgs.msg.dds.DetectableSceneObjectsMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.DetectableSceneObjectsMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.DetectableSceneObjectsMessage src, perception_msgs.msg.dds.DetectableSceneObjectsMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public DetectableSceneObjectsMessagePubSubType newInstance()
   {
      return new DetectableSceneObjectsMessagePubSubType();
   }
}
