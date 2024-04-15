package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "DoorNodeMessage" defined in "DoorNodeMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from DoorNodeMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit DoorNodeMessage_.idl instead.
*
*/
public class DoorNodeMessagePubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.DoorNodeMessage>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::DoorNodeMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "c444fbe9984229c8f7cdf296caaf1d80d84e708ce1a5ed3f54355e066486d8f2";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.DoorNodeMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.DoorNodeMessage data) throws java.io.IOException
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

      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.TransformPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += perception_msgs.msg.dds.PlanarRegionMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 5000; ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.Point32PubSubType.getMaxCdrSerializedSize(current_alignment);}

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.DoorNodeMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.DoorNodeMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += perception_msgs.msg.dds.SceneNodeMessagePubSubType.getCdrSerializedSize(data.getSceneNode(), current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getCdrSerializedSize(data.getObjectPose(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.TransformPubSubType.getCdrSerializedSize(data.getVisualTransformToObjectPose(), current_alignment);

      current_alignment += perception_msgs.msg.dds.PlanarRegionMessagePubSubType.getCdrSerializedSize(data.getDoorPlanarRegion(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getPresumedDoorPointCloud().size(); ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.Point32PubSubType.getCdrSerializedSize(data.getPresumedDoorPointCloud().get(i0), current_alignment);}


      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.DoorNodeMessage data, us.ihmc.idl.CDR cdr)
   {
      perception_msgs.msg.dds.SceneNodeMessagePubSubType.write(data.getSceneNode(), cdr);
      cdr.write_type_9(data.getDoorHardwareType());

      geometry_msgs.msg.dds.PosePubSubType.write(data.getObjectPose(), cdr);
      geometry_msgs.msg.dds.TransformPubSubType.write(data.getVisualTransformToObjectPose(), cdr);
      perception_msgs.msg.dds.PlanarRegionMessagePubSubType.write(data.getDoorPlanarRegion(), cdr);
      if(data.getPresumedDoorPointCloud().size() <= 5000)
      cdr.write_type_e(data.getPresumedDoorPointCloud());else
          throw new RuntimeException("presumed_door_point_cloud field exceeds the maximum length");

   }

   public static void read(perception_msgs.msg.dds.DoorNodeMessage data, us.ihmc.idl.CDR cdr)
   {
      perception_msgs.msg.dds.SceneNodeMessagePubSubType.read(data.getSceneNode(), cdr);	
      data.setDoorHardwareType(cdr.read_type_9());
      	
      geometry_msgs.msg.dds.PosePubSubType.read(data.getObjectPose(), cdr);	
      geometry_msgs.msg.dds.TransformPubSubType.read(data.getVisualTransformToObjectPose(), cdr);	
      perception_msgs.msg.dds.PlanarRegionMessagePubSubType.read(data.getDoorPlanarRegion(), cdr);	
      cdr.read_type_e(data.getPresumedDoorPointCloud());	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.DoorNodeMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("scene_node", new perception_msgs.msg.dds.SceneNodeMessagePubSubType(), data.getSceneNode());

      ser.write_type_9("door_hardware_type", data.getDoorHardwareType());
      ser.write_type_a("object_pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getObjectPose());

      ser.write_type_a("visual_transform_to_object_pose", new geometry_msgs.msg.dds.TransformPubSubType(), data.getVisualTransformToObjectPose());

      ser.write_type_a("door_planar_region", new perception_msgs.msg.dds.PlanarRegionMessagePubSubType(), data.getDoorPlanarRegion());

      ser.write_type_e("presumed_door_point_cloud", data.getPresumedDoorPointCloud());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.DoorNodeMessage data)
   {
      ser.read_type_a("scene_node", new perception_msgs.msg.dds.SceneNodeMessagePubSubType(), data.getSceneNode());

      data.setDoorHardwareType(ser.read_type_9("door_hardware_type"));
      ser.read_type_a("object_pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getObjectPose());

      ser.read_type_a("visual_transform_to_object_pose", new geometry_msgs.msg.dds.TransformPubSubType(), data.getVisualTransformToObjectPose());

      ser.read_type_a("door_planar_region", new perception_msgs.msg.dds.PlanarRegionMessagePubSubType(), data.getDoorPlanarRegion());

      ser.read_type_e("presumed_door_point_cloud", data.getPresumedDoorPointCloud());
   }

   public static void staticCopy(perception_msgs.msg.dds.DoorNodeMessage src, perception_msgs.msg.dds.DoorNodeMessage dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.DoorNodeMessage createData()
   {
      return new perception_msgs.msg.dds.DoorNodeMessage();
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
   
   public void serialize(perception_msgs.msg.dds.DoorNodeMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.DoorNodeMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.DoorNodeMessage src, perception_msgs.msg.dds.DoorNodeMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public DoorNodeMessagePubSubType newInstance()
   {
      return new DoorNodeMessagePubSubType();
   }
}
