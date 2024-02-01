package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "PredefinedRigidBodySceneNodeMessage" defined in "PredefinedRigidBodySceneNodeMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from PredefinedRigidBodySceneNodeMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit PredefinedRigidBodySceneNodeMessage_.idl instead.
*
*/
public class PredefinedRigidBodySceneNodeMessagePubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.PredefinedRigidBodySceneNodeMessage>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::PredefinedRigidBodySceneNodeMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "bed84d14b59c1e056ce9d3bfccce12a592f1d07ade7b1228230d01359f6b452f";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.PredefinedRigidBodySceneNodeMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.PredefinedRigidBodySceneNodeMessage data) throws java.io.IOException
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
      current_alignment += controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.PredefinedRigidBodySceneNodeMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.PredefinedRigidBodySceneNodeMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += perception_msgs.msg.dds.SceneNodeMessagePubSubType.getCdrSerializedSize(data.getSceneNode(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.getCdrSerializedSize(data.getInitialTransformToParent(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getVisualModelFilePath().length() + 1;

      current_alignment += controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.getCdrSerializedSize(data.getVisualTransformToParent(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.PredefinedRigidBodySceneNodeMessage data, us.ihmc.idl.CDR cdr)
   {
      perception_msgs.msg.dds.SceneNodeMessagePubSubType.write(data.getSceneNode(), cdr);
      cdr.write_type_4(data.getInitialParentId());

      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.write(data.getInitialTransformToParent(), cdr);
      if(data.getVisualModelFilePath().length() <= 255)
      cdr.write_type_d(data.getVisualModelFilePath());else
          throw new RuntimeException("visual_model_file_path field exceeds the maximum length");

      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.write(data.getVisualTransformToParent(), cdr);
   }

   public static void read(perception_msgs.msg.dds.PredefinedRigidBodySceneNodeMessage data, us.ihmc.idl.CDR cdr)
   {
      perception_msgs.msg.dds.SceneNodeMessagePubSubType.read(data.getSceneNode(), cdr);	
      data.setInitialParentId(cdr.read_type_4());
      	
      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.read(data.getInitialTransformToParent(), cdr);	
      cdr.read_type_d(data.getVisualModelFilePath());	
      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.read(data.getVisualTransformToParent(), cdr);	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.PredefinedRigidBodySceneNodeMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("scene_node", new perception_msgs.msg.dds.SceneNodeMessagePubSubType(), data.getSceneNode());

      ser.write_type_4("initial_parent_id", data.getInitialParentId());
      ser.write_type_a("initial_transform_to_parent", new controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType(), data.getInitialTransformToParent());

      ser.write_type_d("visual_model_file_path", data.getVisualModelFilePath());
      ser.write_type_a("visual_transform_to_parent", new controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType(), data.getVisualTransformToParent());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.PredefinedRigidBodySceneNodeMessage data)
   {
      ser.read_type_a("scene_node", new perception_msgs.msg.dds.SceneNodeMessagePubSubType(), data.getSceneNode());

      data.setInitialParentId(ser.read_type_4("initial_parent_id"));
      ser.read_type_a("initial_transform_to_parent", new controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType(), data.getInitialTransformToParent());

      ser.read_type_d("visual_model_file_path", data.getVisualModelFilePath());
      ser.read_type_a("visual_transform_to_parent", new controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType(), data.getVisualTransformToParent());

   }

   public static void staticCopy(perception_msgs.msg.dds.PredefinedRigidBodySceneNodeMessage src, perception_msgs.msg.dds.PredefinedRigidBodySceneNodeMessage dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.PredefinedRigidBodySceneNodeMessage createData()
   {
      return new perception_msgs.msg.dds.PredefinedRigidBodySceneNodeMessage();
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
   
   public void serialize(perception_msgs.msg.dds.PredefinedRigidBodySceneNodeMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.PredefinedRigidBodySceneNodeMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.PredefinedRigidBodySceneNodeMessage src, perception_msgs.msg.dds.PredefinedRigidBodySceneNodeMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public PredefinedRigidBodySceneNodeMessagePubSubType newInstance()
   {
      return new PredefinedRigidBodySceneNodeMessagePubSubType();
   }
}
