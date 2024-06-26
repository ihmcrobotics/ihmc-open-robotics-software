package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "DetectableSceneNodeMessage" defined in "DetectableSceneNodeMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from DetectableSceneNodeMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit DetectableSceneNodeMessage_.idl instead.
*
*/
public class DetectableSceneNodeMessagePubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.DetectableSceneNodeMessage>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::DetectableSceneNodeMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "03acf6926dcb6bf75295c524e2fa997c0f981160e3ba5f03681b1d0488b7cd2f";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.DetectableSceneNodeMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.DetectableSceneNodeMessage data) throws java.io.IOException
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

      current_alignment += perception_msgs.msg.dds.SceneNodeMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.DetectableSceneNodeMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.DetectableSceneNodeMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += perception_msgs.msg.dds.SceneNodeMessagePubSubType.getCdrSerializedSize(data.getSceneNode(), current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.DetectableSceneNodeMessage data, us.ihmc.idl.CDR cdr)
   {
      perception_msgs.msg.dds.SceneNodeMessagePubSubType.write(data.getSceneNode(), cdr);
      cdr.write_type_7(data.getCurrentlyDetected());

   }

   public static void read(perception_msgs.msg.dds.DetectableSceneNodeMessage data, us.ihmc.idl.CDR cdr)
   {
      perception_msgs.msg.dds.SceneNodeMessagePubSubType.read(data.getSceneNode(), cdr);	
      data.setCurrentlyDetected(cdr.read_type_7());
      	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.DetectableSceneNodeMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("scene_node", new perception_msgs.msg.dds.SceneNodeMessagePubSubType(), data.getSceneNode());

      ser.write_type_7("currently_detected", data.getCurrentlyDetected());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.DetectableSceneNodeMessage data)
   {
      ser.read_type_a("scene_node", new perception_msgs.msg.dds.SceneNodeMessagePubSubType(), data.getSceneNode());

      data.setCurrentlyDetected(ser.read_type_7("currently_detected"));
   }

   public static void staticCopy(perception_msgs.msg.dds.DetectableSceneNodeMessage src, perception_msgs.msg.dds.DetectableSceneNodeMessage dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.DetectableSceneNodeMessage createData()
   {
      return new perception_msgs.msg.dds.DetectableSceneNodeMessage();
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
   
   public void serialize(perception_msgs.msg.dds.DetectableSceneNodeMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.DetectableSceneNodeMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.DetectableSceneNodeMessage src, perception_msgs.msg.dds.DetectableSceneNodeMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public DetectableSceneNodeMessagePubSubType newInstance()
   {
      return new DetectableSceneNodeMessagePubSubType();
   }
}
