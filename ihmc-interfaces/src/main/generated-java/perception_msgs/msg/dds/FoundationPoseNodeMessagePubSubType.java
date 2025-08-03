package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "FoundationPoseNodeMessage" defined in "FoundationPoseNodeMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from FoundationPoseNodeMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit FoundationPoseNodeMessage_.idl instead.
*
*/
public class FoundationPoseNodeMessagePubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.FoundationPoseNodeMessage>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::FoundationPoseNodeMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "fcf892ee98ed33d8688966f12c52bd5e8dae6140a99250fb41bd6a90b08d9fff";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.FoundationPoseNodeMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.FoundationPoseNodeMessage data) throws java.io.IOException
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

      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.FoundationPoseNodeMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.FoundationPoseNodeMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += perception_msgs.msg.dds.DetectableSceneNodeMessagePubSubType.getCdrSerializedSize(data.getDetectableSceneNode(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getCdrSerializedSize(data.getObjectPose(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.FoundationPoseNodeMessage data, us.ihmc.idl.CDR cdr)
   {
      perception_msgs.msg.dds.DetectableSceneNodeMessagePubSubType.write(data.getDetectableSceneNode(), cdr);
      geometry_msgs.msg.dds.PosePubSubType.write(data.getObjectPose(), cdr);
   }

   public static void read(perception_msgs.msg.dds.FoundationPoseNodeMessage data, us.ihmc.idl.CDR cdr)
   {
      perception_msgs.msg.dds.DetectableSceneNodeMessagePubSubType.read(data.getDetectableSceneNode(), cdr);	
      geometry_msgs.msg.dds.PosePubSubType.read(data.getObjectPose(), cdr);	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.FoundationPoseNodeMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("detectable_scene_node", new perception_msgs.msg.dds.DetectableSceneNodeMessagePubSubType(), data.getDetectableSceneNode());

      ser.write_type_a("object_pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getObjectPose());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.FoundationPoseNodeMessage data)
   {
      ser.read_type_a("detectable_scene_node", new perception_msgs.msg.dds.DetectableSceneNodeMessagePubSubType(), data.getDetectableSceneNode());

      ser.read_type_a("object_pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getObjectPose());

   }

   public static void staticCopy(perception_msgs.msg.dds.FoundationPoseNodeMessage src, perception_msgs.msg.dds.FoundationPoseNodeMessage dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.FoundationPoseNodeMessage createData()
   {
      return new perception_msgs.msg.dds.FoundationPoseNodeMessage();
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
   
   public void serialize(perception_msgs.msg.dds.FoundationPoseNodeMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.FoundationPoseNodeMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.FoundationPoseNodeMessage src, perception_msgs.msg.dds.FoundationPoseNodeMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public FoundationPoseNodeMessagePubSubType newInstance()
   {
      return new FoundationPoseNodeMessagePubSubType();
   }
}
