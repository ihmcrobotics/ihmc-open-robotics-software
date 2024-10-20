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
   		return "cd0b30608bf0e139277eb0e741529a1376a7f6d1c87f9fe3031878474197d3f2";
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

      current_alignment += perception_msgs.msg.dds.DetectableSceneNodeMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.TransformPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += perception_msgs.msg.dds.DoorPanelMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 100; ++i0)
      {
          current_alignment += perception_msgs.msg.dds.DoorOpeningMechanismMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.DoorNodeMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.DoorNodeMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += perception_msgs.msg.dds.DetectableSceneNodeMessagePubSubType.getCdrSerializedSize(data.getDetectableSceneNode(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.TransformPubSubType.getCdrSerializedSize(data.getDoorCornerTransformToWorld(), current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += perception_msgs.msg.dds.DoorPanelMessagePubSubType.getCdrSerializedSize(data.getDoorPanel(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getOpeningMechanisms().size(); ++i0)
      {
          current_alignment += perception_msgs.msg.dds.DoorOpeningMechanismMessagePubSubType.getCdrSerializedSize(data.getOpeningMechanisms().get(i0), current_alignment);}


      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.DoorNodeMessage data, us.ihmc.idl.CDR cdr)
   {
      perception_msgs.msg.dds.DetectableSceneNodeMessagePubSubType.write(data.getDetectableSceneNode(), cdr);
      geometry_msgs.msg.dds.TransformPubSubType.write(data.getDoorCornerTransformToWorld(), cdr);
      cdr.write_type_7(data.getPoseLocked());

      perception_msgs.msg.dds.DoorPanelMessagePubSubType.write(data.getDoorPanel(), cdr);
      if(data.getOpeningMechanisms().size() <= 100)
      cdr.write_type_e(data.getOpeningMechanisms());else
          throw new RuntimeException("opening_mechanisms field exceeds the maximum length");

   }

   public static void read(perception_msgs.msg.dds.DoorNodeMessage data, us.ihmc.idl.CDR cdr)
   {
      perception_msgs.msg.dds.DetectableSceneNodeMessagePubSubType.read(data.getDetectableSceneNode(), cdr);	
      geometry_msgs.msg.dds.TransformPubSubType.read(data.getDoorCornerTransformToWorld(), cdr);	
      data.setPoseLocked(cdr.read_type_7());
      	
      perception_msgs.msg.dds.DoorPanelMessagePubSubType.read(data.getDoorPanel(), cdr);	
      cdr.read_type_e(data.getOpeningMechanisms());	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.DoorNodeMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("detectable_scene_node", new perception_msgs.msg.dds.DetectableSceneNodeMessagePubSubType(), data.getDetectableSceneNode());

      ser.write_type_a("door_corner_transform_to_world", new geometry_msgs.msg.dds.TransformPubSubType(), data.getDoorCornerTransformToWorld());

      ser.write_type_7("pose_locked", data.getPoseLocked());
      ser.write_type_a("door_panel", new perception_msgs.msg.dds.DoorPanelMessagePubSubType(), data.getDoorPanel());

      ser.write_type_e("opening_mechanisms", data.getOpeningMechanisms());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.DoorNodeMessage data)
   {
      ser.read_type_a("detectable_scene_node", new perception_msgs.msg.dds.DetectableSceneNodeMessagePubSubType(), data.getDetectableSceneNode());

      ser.read_type_a("door_corner_transform_to_world", new geometry_msgs.msg.dds.TransformPubSubType(), data.getDoorCornerTransformToWorld());

      data.setPoseLocked(ser.read_type_7("pose_locked"));
      ser.read_type_a("door_panel", new perception_msgs.msg.dds.DoorPanelMessagePubSubType(), data.getDoorPanel());

      ser.read_type_e("opening_mechanisms", data.getOpeningMechanisms());
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
