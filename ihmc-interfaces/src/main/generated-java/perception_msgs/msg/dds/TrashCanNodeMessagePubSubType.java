package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "TrashCanNodeMessage" defined in "TrashCanNodeMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from TrashCanNodeMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit TrashCanNodeMessage_.idl instead.
*
*/
public class TrashCanNodeMessagePubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.TrashCanNodeMessage>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::TrashCanNodeMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "7ee42811b3b00f967b991119d183c1278c0d81b811d6d9e5db7ec799b88d966e";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.TrashCanNodeMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.TrashCanNodeMessage data) throws java.io.IOException
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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.TrashCanNodeMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.TrashCanNodeMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += perception_msgs.msg.dds.DetectableSceneNodeMessagePubSubType.getCdrSerializedSize(data.getDetectableSceneNode(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.TransformPubSubType.getCdrSerializedSize(data.getTrashCanToWorldTransform(), current_alignment);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.TrashCanNodeMessage data, us.ihmc.idl.CDR cdr)
   {
      perception_msgs.msg.dds.DetectableSceneNodeMessagePubSubType.write(data.getDetectableSceneNode(), cdr);
      geometry_msgs.msg.dds.TransformPubSubType.write(data.getTrashCanToWorldTransform(), cdr);
      cdr.write_type_6(data.getTrashCanYaw());

   }

   public static void read(perception_msgs.msg.dds.TrashCanNodeMessage data, us.ihmc.idl.CDR cdr)
   {
      perception_msgs.msg.dds.DetectableSceneNodeMessagePubSubType.read(data.getDetectableSceneNode(), cdr);	
      geometry_msgs.msg.dds.TransformPubSubType.read(data.getTrashCanToWorldTransform(), cdr);	
      data.setTrashCanYaw(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.TrashCanNodeMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("detectable_scene_node", new perception_msgs.msg.dds.DetectableSceneNodeMessagePubSubType(), data.getDetectableSceneNode());

      ser.write_type_a("trash_can_to_world_transform", new geometry_msgs.msg.dds.TransformPubSubType(), data.getTrashCanToWorldTransform());

      ser.write_type_6("trash_can_yaw", data.getTrashCanYaw());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.TrashCanNodeMessage data)
   {
      ser.read_type_a("detectable_scene_node", new perception_msgs.msg.dds.DetectableSceneNodeMessagePubSubType(), data.getDetectableSceneNode());

      ser.read_type_a("trash_can_to_world_transform", new geometry_msgs.msg.dds.TransformPubSubType(), data.getTrashCanToWorldTransform());

      data.setTrashCanYaw(ser.read_type_6("trash_can_yaw"));
   }

   public static void staticCopy(perception_msgs.msg.dds.TrashCanNodeMessage src, perception_msgs.msg.dds.TrashCanNodeMessage dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.TrashCanNodeMessage createData()
   {
      return new perception_msgs.msg.dds.TrashCanNodeMessage();
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
   
   public void serialize(perception_msgs.msg.dds.TrashCanNodeMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.TrashCanNodeMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.TrashCanNodeMessage src, perception_msgs.msg.dds.TrashCanNodeMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public TrashCanNodeMessagePubSubType newInstance()
   {
      return new TrashCanNodeMessagePubSubType();
   }
}
