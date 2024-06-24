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
   		return "143704435859b97632e49738992159a17dc3127ba2906ecc0b082beb56f04cf1";
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

      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getMaxCdrSerializedSize(current_alignment);

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

      current_alignment += perception_msgs.msg.dds.SceneNodeMessagePubSubType.getCdrSerializedSize(data.getSceneNode(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getCdrSerializedSize(data.getDoorFramePose(), current_alignment);

      current_alignment += perception_msgs.msg.dds.DoorPanelMessagePubSubType.getCdrSerializedSize(data.getDoorPanel(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getOpeningMechanisms().size(); ++i0)
      {
          current_alignment += perception_msgs.msg.dds.DoorOpeningMechanismMessagePubSubType.getCdrSerializedSize(data.getOpeningMechanisms().get(i0), current_alignment);}


      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.DoorNodeMessage data, us.ihmc.idl.CDR cdr)
   {
      perception_msgs.msg.dds.SceneNodeMessagePubSubType.write(data.getSceneNode(), cdr);
      geometry_msgs.msg.dds.PosePubSubType.write(data.getDoorFramePose(), cdr);
      perception_msgs.msg.dds.DoorPanelMessagePubSubType.write(data.getDoorPanel(), cdr);
      if(data.getOpeningMechanisms().size() <= 100)
      cdr.write_type_e(data.getOpeningMechanisms());else
          throw new RuntimeException("opening_mechanisms field exceeds the maximum length");

   }

   public static void read(perception_msgs.msg.dds.DoorNodeMessage data, us.ihmc.idl.CDR cdr)
   {
      perception_msgs.msg.dds.SceneNodeMessagePubSubType.read(data.getSceneNode(), cdr);	
      geometry_msgs.msg.dds.PosePubSubType.read(data.getDoorFramePose(), cdr);	
      perception_msgs.msg.dds.DoorPanelMessagePubSubType.read(data.getDoorPanel(), cdr);	
      cdr.read_type_e(data.getOpeningMechanisms());	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.DoorNodeMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("scene_node", new perception_msgs.msg.dds.SceneNodeMessagePubSubType(), data.getSceneNode());

      ser.write_type_a("door_frame_pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getDoorFramePose());

      ser.write_type_a("door_panel", new perception_msgs.msg.dds.DoorPanelMessagePubSubType(), data.getDoorPanel());

      ser.write_type_e("opening_mechanisms", data.getOpeningMechanisms());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.DoorNodeMessage data)
   {
      ser.read_type_a("scene_node", new perception_msgs.msg.dds.SceneNodeMessagePubSubType(), data.getSceneNode());

      ser.read_type_a("door_frame_pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getDoorFramePose());

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
