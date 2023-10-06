package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "ReshapableRigidBodySceneNodeMessage" defined in "ReshapableRigidBodySceneNodeMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from ReshapableRigidBodySceneNodeMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit ReshapableRigidBodySceneNodeMessage_.idl instead.
*
*/
public class ReshapableRigidBodySceneNodeMessagePubSubType implements us.ihmc.pubsub.TopicDataType<PrimitiveRigidBodySceneNodeMessage>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::ReshapableRigidBodySceneNodeMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "9fd92d5fc6a49d40d03d648dc89ce2b59ebbde322e21cbc01fd5dd4c8284f89c";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(PrimitiveRigidBodySceneNodeMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, PrimitiveRigidBodySceneNodeMessage data) throws java.io.IOException
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


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(PrimitiveRigidBodySceneNodeMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(PrimitiveRigidBodySceneNodeMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += perception_msgs.msg.dds.SceneNodeMessagePubSubType.getCdrSerializedSize(data.getSceneNode(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.getCdrSerializedSize(data.getInitialTransformToParent(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(PrimitiveRigidBodySceneNodeMessage data, us.ihmc.idl.CDR cdr)
   {
      perception_msgs.msg.dds.SceneNodeMessagePubSubType.write(data.getSceneNode(), cdr);
      cdr.write_type_4(data.getInitialParentId());

      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.write(data.getInitialTransformToParent(), cdr);
   }

   public static void read(PrimitiveRigidBodySceneNodeMessage data, us.ihmc.idl.CDR cdr)
   {
      perception_msgs.msg.dds.SceneNodeMessagePubSubType.read(data.getSceneNode(), cdr);	
      data.setInitialParentId(cdr.read_type_4());
      	
      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.read(data.getInitialTransformToParent(), cdr);	

   }

   @Override
   public final void serialize(PrimitiveRigidBodySceneNodeMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("scene_node", new perception_msgs.msg.dds.SceneNodeMessagePubSubType(), data.getSceneNode());

      ser.write_type_4("initial_parent_id", data.getInitialParentId());
      ser.write_type_a("initial_transform_to_parent", new controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType(), data.getInitialTransformToParent());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, PrimitiveRigidBodySceneNodeMessage data)
   {
      ser.read_type_a("scene_node", new perception_msgs.msg.dds.SceneNodeMessagePubSubType(), data.getSceneNode());

      data.setInitialParentId(ser.read_type_4("initial_parent_id"));
      ser.read_type_a("initial_transform_to_parent", new controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType(), data.getInitialTransformToParent());

   }

   public static void staticCopy(PrimitiveRigidBodySceneNodeMessage src, PrimitiveRigidBodySceneNodeMessage dest)
   {
      dest.set(src);
   }

   @Override
   public PrimitiveRigidBodySceneNodeMessage createData()
   {
      return new PrimitiveRigidBodySceneNodeMessage();
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
   
   public void serialize(PrimitiveRigidBodySceneNodeMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(PrimitiveRigidBodySceneNodeMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(PrimitiveRigidBodySceneNodeMessage src, PrimitiveRigidBodySceneNodeMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public ReshapableRigidBodySceneNodeMessagePubSubType newInstance()
   {
      return new ReshapableRigidBodySceneNodeMessagePubSubType();
   }
}
