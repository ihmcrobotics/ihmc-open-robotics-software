package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "PrimitiveRigidBodySceneNodeMessage" defined in "PrimitiveRigidBodySceneNodeMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from PrimitiveRigidBodySceneNodeMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit PrimitiveRigidBodySceneNodeMessage_.idl instead.
*
*/
public class PrimitiveRigidBodySceneNodeMessagePubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.PrimitiveRigidBodySceneNodeMessage>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::PrimitiveRigidBodySceneNodeMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "0bd8cbdb1c493de0ed9a6a15a5e9401e169008dc5e8900958cee3fe7d5d57463";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.PrimitiveRigidBodySceneNodeMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.PrimitiveRigidBodySceneNodeMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.PrimitiveRigidBodySceneNodeMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.PrimitiveRigidBodySceneNodeMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += perception_msgs.msg.dds.SceneNodeMessagePubSubType.getCdrSerializedSize(data.getSceneNode(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.getCdrSerializedSize(data.getInitialTransformToParent(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getShape().length() + 1;


      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.PrimitiveRigidBodySceneNodeMessage data, us.ihmc.idl.CDR cdr)
   {
      perception_msgs.msg.dds.SceneNodeMessagePubSubType.write(data.getSceneNode(), cdr);
      cdr.write_type_4(data.getInitialParentId());

      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.write(data.getInitialTransformToParent(), cdr);
      if(data.getShape().length() <= 255)
      cdr.write_type_d(data.getShape());else
          throw new RuntimeException("shape field exceeds the maximum length");

   }

   public static void read(perception_msgs.msg.dds.PrimitiveRigidBodySceneNodeMessage data, us.ihmc.idl.CDR cdr)
   {
      perception_msgs.msg.dds.SceneNodeMessagePubSubType.read(data.getSceneNode(), cdr);	
      data.setInitialParentId(cdr.read_type_4());
      	
      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.read(data.getInitialTransformToParent(), cdr);	
      cdr.read_type_d(data.getShape());	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.PrimitiveRigidBodySceneNodeMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("scene_node", new perception_msgs.msg.dds.SceneNodeMessagePubSubType(), data.getSceneNode());

      ser.write_type_4("initial_parent_id", data.getInitialParentId());
      ser.write_type_a("initial_transform_to_parent", new controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType(), data.getInitialTransformToParent());

      ser.write_type_d("shape", data.getShape());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.PrimitiveRigidBodySceneNodeMessage data)
   {
      ser.read_type_a("scene_node", new perception_msgs.msg.dds.SceneNodeMessagePubSubType(), data.getSceneNode());

      data.setInitialParentId(ser.read_type_4("initial_parent_id"));
      ser.read_type_a("initial_transform_to_parent", new controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType(), data.getInitialTransformToParent());

      ser.read_type_d("shape", data.getShape());
   }

   public static void staticCopy(perception_msgs.msg.dds.PrimitiveRigidBodySceneNodeMessage src, perception_msgs.msg.dds.PrimitiveRigidBodySceneNodeMessage dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.PrimitiveRigidBodySceneNodeMessage createData()
   {
      return new perception_msgs.msg.dds.PrimitiveRigidBodySceneNodeMessage();
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
   
   public void serialize(perception_msgs.msg.dds.PrimitiveRigidBodySceneNodeMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.PrimitiveRigidBodySceneNodeMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.PrimitiveRigidBodySceneNodeMessage src, perception_msgs.msg.dds.PrimitiveRigidBodySceneNodeMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public PrimitiveRigidBodySceneNodeMessagePubSubType newInstance()
   {
      return new PrimitiveRigidBodySceneNodeMessagePubSubType();
   }
}
