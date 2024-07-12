package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "SceneGraphMessage" defined in "SceneGraphMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from SceneGraphMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit SceneGraphMessage_.idl instead.
*
*/
public class SceneGraphMessagePubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.SceneGraphMessage>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::SceneGraphMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "a9decdcb9c27d11b7d0d7350fb5e3eeb58729241281a790109926e4f008bebcd";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.SceneGraphMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.SceneGraphMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (1000 * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (1000 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 200; ++i0)
      {
          current_alignment += perception_msgs.msg.dds.SceneNodeMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 200; ++i0)
      {
          current_alignment += perception_msgs.msg.dds.DetectableSceneNodeMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 200; ++i0)
      {
          current_alignment += perception_msgs.msg.dds.PredefinedRigidBodySceneNodeMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 200; ++i0)
      {
          current_alignment += perception_msgs.msg.dds.ArUcoMarkerNodeMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 200; ++i0)
      {
          current_alignment += perception_msgs.msg.dds.CenterposeNodeMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 200; ++i0)
      {
          current_alignment += perception_msgs.msg.dds.StaticRelativeSceneNodeMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 200; ++i0)
      {
          current_alignment += perception_msgs.msg.dds.PrimitiveRigidBodySceneNodeMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 200; ++i0)
      {
          current_alignment += perception_msgs.msg.dds.YOLOv8NodeMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 200; ++i0)
      {
          current_alignment += perception_msgs.msg.dds.DoorNodeMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 200; ++i0)
      {
          current_alignment += perception_msgs.msg.dds.TrashCanNodeMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.SceneGraphMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.SceneGraphMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getSceneTreeTypes().size() * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getSceneTreeIndices().size() * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getSceneNodes().size(); ++i0)
      {
          current_alignment += perception_msgs.msg.dds.SceneNodeMessagePubSubType.getCdrSerializedSize(data.getSceneNodes().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getDetectableSceneNodes().size(); ++i0)
      {
          current_alignment += perception_msgs.msg.dds.DetectableSceneNodeMessagePubSubType.getCdrSerializedSize(data.getDetectableSceneNodes().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getPredefinedRigidBodySceneNodes().size(); ++i0)
      {
          current_alignment += perception_msgs.msg.dds.PredefinedRigidBodySceneNodeMessagePubSubType.getCdrSerializedSize(data.getPredefinedRigidBodySceneNodes().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getArucoMarkerSceneNodes().size(); ++i0)
      {
          current_alignment += perception_msgs.msg.dds.ArUcoMarkerNodeMessagePubSubType.getCdrSerializedSize(data.getArucoMarkerSceneNodes().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getCenterposeSceneNodes().size(); ++i0)
      {
          current_alignment += perception_msgs.msg.dds.CenterposeNodeMessagePubSubType.getCdrSerializedSize(data.getCenterposeSceneNodes().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getStaticRelativeSceneNodes().size(); ++i0)
      {
          current_alignment += perception_msgs.msg.dds.StaticRelativeSceneNodeMessagePubSubType.getCdrSerializedSize(data.getStaticRelativeSceneNodes().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getPrimitiveRigidBodySceneNodes().size(); ++i0)
      {
          current_alignment += perception_msgs.msg.dds.PrimitiveRigidBodySceneNodeMessagePubSubType.getCdrSerializedSize(data.getPrimitiveRigidBodySceneNodes().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getYoloSceneNodes().size(); ++i0)
      {
          current_alignment += perception_msgs.msg.dds.YOLOv8NodeMessagePubSubType.getCdrSerializedSize(data.getYoloSceneNodes().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getDoorSceneNodes().size(); ++i0)
      {
          current_alignment += perception_msgs.msg.dds.DoorNodeMessagePubSubType.getCdrSerializedSize(data.getDoorSceneNodes().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getTrashCanNodes().size(); ++i0)
      {
          current_alignment += perception_msgs.msg.dds.TrashCanNodeMessagePubSubType.getCdrSerializedSize(data.getTrashCanNodes().get(i0), current_alignment);}


      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.SceneGraphMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_4(data.getNextId());

      if(data.getSceneTreeTypes().size() <= 1000)
      cdr.write_type_e(data.getSceneTreeTypes());else
          throw new RuntimeException("scene_tree_types field exceeds the maximum length");

      if(data.getSceneTreeIndices().size() <= 1000)
      cdr.write_type_e(data.getSceneTreeIndices());else
          throw new RuntimeException("scene_tree_indices field exceeds the maximum length");

      if(data.getSceneNodes().size() <= 200)
      cdr.write_type_e(data.getSceneNodes());else
          throw new RuntimeException("scene_nodes field exceeds the maximum length");

      if(data.getDetectableSceneNodes().size() <= 200)
      cdr.write_type_e(data.getDetectableSceneNodes());else
          throw new RuntimeException("detectable_scene_nodes field exceeds the maximum length");

      if(data.getPredefinedRigidBodySceneNodes().size() <= 200)
      cdr.write_type_e(data.getPredefinedRigidBodySceneNodes());else
          throw new RuntimeException("predefined_rigid_body_scene_nodes field exceeds the maximum length");

      if(data.getArucoMarkerSceneNodes().size() <= 200)
      cdr.write_type_e(data.getArucoMarkerSceneNodes());else
          throw new RuntimeException("aruco_marker_scene_nodes field exceeds the maximum length");

      if(data.getCenterposeSceneNodes().size() <= 200)
      cdr.write_type_e(data.getCenterposeSceneNodes());else
          throw new RuntimeException("centerpose_scene_nodes field exceeds the maximum length");

      if(data.getStaticRelativeSceneNodes().size() <= 200)
      cdr.write_type_e(data.getStaticRelativeSceneNodes());else
          throw new RuntimeException("static_relative_scene_nodes field exceeds the maximum length");

      if(data.getPrimitiveRigidBodySceneNodes().size() <= 200)
      cdr.write_type_e(data.getPrimitiveRigidBodySceneNodes());else
          throw new RuntimeException("primitive_rigid_body_scene_nodes field exceeds the maximum length");

      if(data.getYoloSceneNodes().size() <= 200)
      cdr.write_type_e(data.getYoloSceneNodes());else
          throw new RuntimeException("yolo_scene_nodes field exceeds the maximum length");

      if(data.getDoorSceneNodes().size() <= 200)
      cdr.write_type_e(data.getDoorSceneNodes());else
          throw new RuntimeException("door_scene_nodes field exceeds the maximum length");

      if(data.getTrashCanNodes().size() <= 200)
      cdr.write_type_e(data.getTrashCanNodes());else
          throw new RuntimeException("trash_can_nodes field exceeds the maximum length");

   }

   public static void read(perception_msgs.msg.dds.SceneGraphMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setNextId(cdr.read_type_4());
      	
      cdr.read_type_e(data.getSceneTreeTypes());	
      cdr.read_type_e(data.getSceneTreeIndices());	
      cdr.read_type_e(data.getSceneNodes());	
      cdr.read_type_e(data.getDetectableSceneNodes());	
      cdr.read_type_e(data.getPredefinedRigidBodySceneNodes());	
      cdr.read_type_e(data.getArucoMarkerSceneNodes());	
      cdr.read_type_e(data.getCenterposeSceneNodes());	
      cdr.read_type_e(data.getStaticRelativeSceneNodes());	
      cdr.read_type_e(data.getPrimitiveRigidBodySceneNodes());	
      cdr.read_type_e(data.getYoloSceneNodes());	
      cdr.read_type_e(data.getDoorSceneNodes());	
      cdr.read_type_e(data.getTrashCanNodes());	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.SceneGraphMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_4("next_id", data.getNextId());
      ser.write_type_e("scene_tree_types", data.getSceneTreeTypes());
      ser.write_type_e("scene_tree_indices", data.getSceneTreeIndices());
      ser.write_type_e("scene_nodes", data.getSceneNodes());
      ser.write_type_e("detectable_scene_nodes", data.getDetectableSceneNodes());
      ser.write_type_e("predefined_rigid_body_scene_nodes", data.getPredefinedRigidBodySceneNodes());
      ser.write_type_e("aruco_marker_scene_nodes", data.getArucoMarkerSceneNodes());
      ser.write_type_e("centerpose_scene_nodes", data.getCenterposeSceneNodes());
      ser.write_type_e("static_relative_scene_nodes", data.getStaticRelativeSceneNodes());
      ser.write_type_e("primitive_rigid_body_scene_nodes", data.getPrimitiveRigidBodySceneNodes());
      ser.write_type_e("yolo_scene_nodes", data.getYoloSceneNodes());
      ser.write_type_e("door_scene_nodes", data.getDoorSceneNodes());
      ser.write_type_e("trash_can_nodes", data.getTrashCanNodes());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.SceneGraphMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setNextId(ser.read_type_4("next_id"));
      ser.read_type_e("scene_tree_types", data.getSceneTreeTypes());
      ser.read_type_e("scene_tree_indices", data.getSceneTreeIndices());
      ser.read_type_e("scene_nodes", data.getSceneNodes());
      ser.read_type_e("detectable_scene_nodes", data.getDetectableSceneNodes());
      ser.read_type_e("predefined_rigid_body_scene_nodes", data.getPredefinedRigidBodySceneNodes());
      ser.read_type_e("aruco_marker_scene_nodes", data.getArucoMarkerSceneNodes());
      ser.read_type_e("centerpose_scene_nodes", data.getCenterposeSceneNodes());
      ser.read_type_e("static_relative_scene_nodes", data.getStaticRelativeSceneNodes());
      ser.read_type_e("primitive_rigid_body_scene_nodes", data.getPrimitiveRigidBodySceneNodes());
      ser.read_type_e("yolo_scene_nodes", data.getYoloSceneNodes());
      ser.read_type_e("door_scene_nodes", data.getDoorSceneNodes());
      ser.read_type_e("trash_can_nodes", data.getTrashCanNodes());
   }

   public static void staticCopy(perception_msgs.msg.dds.SceneGraphMessage src, perception_msgs.msg.dds.SceneGraphMessage dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.SceneGraphMessage createData()
   {
      return new perception_msgs.msg.dds.SceneGraphMessage();
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
   
   public void serialize(perception_msgs.msg.dds.SceneGraphMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.SceneGraphMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.SceneGraphMessage src, perception_msgs.msg.dds.SceneGraphMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public SceneGraphMessagePubSubType newInstance()
   {
      return new SceneGraphMessagePubSubType();
   }
}
