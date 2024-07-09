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
   		return "5f69c4e84fe0f033113501b12b50589b9c4336e4940f773a2aacf217a7dae785";
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

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += perception_msgs.msg.dds.PlanarRegionMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


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


      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getOpeningMechanismPoint(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getCdrSerializedSize(data.getOpeningMechanismPose(), current_alignment);

      current_alignment += perception_msgs.msg.dds.PlanarRegionMessagePubSubType.getCdrSerializedSize(data.getDoorPlanarRegion(), current_alignment);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.DoorNodeMessage data, us.ihmc.idl.CDR cdr)
   {
      perception_msgs.msg.dds.SceneNodeMessagePubSubType.write(data.getSceneNode(), cdr);
      cdr.write_type_9(data.getOpeningMechanismType());

      geometry_msgs.msg.dds.PointPubSubType.write(data.getOpeningMechanismPoint(), cdr);
      geometry_msgs.msg.dds.PosePubSubType.write(data.getOpeningMechanismPose(), cdr);
      perception_msgs.msg.dds.PlanarRegionMessagePubSubType.write(data.getDoorPlanarRegion(), cdr);
      cdr.write_type_12(data.getDoorPlanarRegionUpdateTimeMillis());

   }

   public static void read(perception_msgs.msg.dds.DoorNodeMessage data, us.ihmc.idl.CDR cdr)
   {
      perception_msgs.msg.dds.SceneNodeMessagePubSubType.read(data.getSceneNode(), cdr);	
      data.setOpeningMechanismType(cdr.read_type_9());
      	
      geometry_msgs.msg.dds.PointPubSubType.read(data.getOpeningMechanismPoint(), cdr);	
      geometry_msgs.msg.dds.PosePubSubType.read(data.getOpeningMechanismPose(), cdr);	
      perception_msgs.msg.dds.PlanarRegionMessagePubSubType.read(data.getDoorPlanarRegion(), cdr);	
      data.setDoorPlanarRegionUpdateTimeMillis(cdr.read_type_12());
      	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.DoorNodeMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("scene_node", new perception_msgs.msg.dds.SceneNodeMessagePubSubType(), data.getSceneNode());

      ser.write_type_9("opening_mechanism_type", data.getOpeningMechanismType());
      ser.write_type_a("opening_mechanism_point", new geometry_msgs.msg.dds.PointPubSubType(), data.getOpeningMechanismPoint());

      ser.write_type_a("opening_mechanism_pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getOpeningMechanismPose());

      ser.write_type_a("door_planar_region", new perception_msgs.msg.dds.PlanarRegionMessagePubSubType(), data.getDoorPlanarRegion());

      ser.write_type_12("door_planar_region_update_time_millis", data.getDoorPlanarRegionUpdateTimeMillis());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.DoorNodeMessage data)
   {
      ser.read_type_a("scene_node", new perception_msgs.msg.dds.SceneNodeMessagePubSubType(), data.getSceneNode());

      data.setOpeningMechanismType(ser.read_type_9("opening_mechanism_type"));
      ser.read_type_a("opening_mechanism_point", new geometry_msgs.msg.dds.PointPubSubType(), data.getOpeningMechanismPoint());

      ser.read_type_a("opening_mechanism_pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getOpeningMechanismPose());

      ser.read_type_a("door_planar_region", new perception_msgs.msg.dds.PlanarRegionMessagePubSubType(), data.getDoorPlanarRegion());

      data.setDoorPlanarRegionUpdateTimeMillis(ser.read_type_12("door_planar_region_update_time_millis"));
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
