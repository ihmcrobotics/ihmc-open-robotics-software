package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "CouchNodeMessage" defined in "CouchNodeMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from CouchNodeMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit CouchNodeMessage_.idl instead.
*
*/
public class CouchNodeMessagePubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.CouchNodeMessage>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::CouchNodeMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "c4d44c24ebc80267adc2b5531a13ec34559d1a272b9ea7eafcbc67080fcf91a1";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.CouchNodeMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.CouchNodeMessage data) throws java.io.IOException
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

      current_alignment += perception_msgs.msg.dds.DetectableSceneNodeMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.TransformPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.TransformPubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.CouchNodeMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.CouchNodeMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += perception_msgs.msg.dds.DetectableSceneNodeMessagePubSubType.getCdrSerializedSize(data.getDetectableSceneNode(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.TransformPubSubType.getCdrSerializedSize(data.getCouchCentroidToWorldTransform(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.TransformPubSubType.getCdrSerializedSize(data.getPillowToWorldTransform(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.CouchNodeMessage data, us.ihmc.idl.CDR cdr)
   {
      perception_msgs.msg.dds.DetectableSceneNodeMessagePubSubType.write(data.getDetectableSceneNode(), cdr);
      geometry_msgs.msg.dds.TransformPubSubType.write(data.getCouchCentroidToWorldTransform(), cdr);
      geometry_msgs.msg.dds.TransformPubSubType.write(data.getPillowToWorldTransform(), cdr);
   }

   public static void read(perception_msgs.msg.dds.CouchNodeMessage data, us.ihmc.idl.CDR cdr)
   {
      perception_msgs.msg.dds.DetectableSceneNodeMessagePubSubType.read(data.getDetectableSceneNode(), cdr);	
      geometry_msgs.msg.dds.TransformPubSubType.read(data.getCouchCentroidToWorldTransform(), cdr);	
      geometry_msgs.msg.dds.TransformPubSubType.read(data.getPillowToWorldTransform(), cdr);	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.CouchNodeMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("detectable_scene_node", new perception_msgs.msg.dds.DetectableSceneNodeMessagePubSubType(), data.getDetectableSceneNode());

      ser.write_type_a("couch_centroid_to_world_transform", new geometry_msgs.msg.dds.TransformPubSubType(), data.getCouchCentroidToWorldTransform());

      ser.write_type_a("pillow_to_world_transform", new geometry_msgs.msg.dds.TransformPubSubType(), data.getPillowToWorldTransform());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.CouchNodeMessage data)
   {
      ser.read_type_a("detectable_scene_node", new perception_msgs.msg.dds.DetectableSceneNodeMessagePubSubType(), data.getDetectableSceneNode());

      ser.read_type_a("couch_centroid_to_world_transform", new geometry_msgs.msg.dds.TransformPubSubType(), data.getCouchCentroidToWorldTransform());

      ser.read_type_a("pillow_to_world_transform", new geometry_msgs.msg.dds.TransformPubSubType(), data.getPillowToWorldTransform());

   }

   public static void staticCopy(perception_msgs.msg.dds.CouchNodeMessage src, perception_msgs.msg.dds.CouchNodeMessage dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.CouchNodeMessage createData()
   {
      return new perception_msgs.msg.dds.CouchNodeMessage();
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
   
   public void serialize(perception_msgs.msg.dds.CouchNodeMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.CouchNodeMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.CouchNodeMessage src, perception_msgs.msg.dds.CouchNodeMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public CouchNodeMessagePubSubType newInstance()
   {
      return new CouchNodeMessagePubSubType();
   }
}
