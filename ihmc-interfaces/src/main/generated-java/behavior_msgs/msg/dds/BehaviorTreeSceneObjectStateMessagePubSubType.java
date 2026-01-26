package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "BehaviorTreeSceneObjectStateMessage" defined in "BehaviorTreeSceneObjectStateMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from BehaviorTreeSceneObjectStateMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit BehaviorTreeSceneObjectStateMessage_.idl instead.
*
*/
public class BehaviorTreeSceneObjectStateMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.BehaviorTreeSceneObjectStateMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::BehaviorTreeSceneObjectStateMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "439162368efea2551346704e22eb0b9b9cb5526216b078ac3716c22d208bab57";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.BehaviorTreeSceneObjectStateMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.BehaviorTreeSceneObjectStateMessage data) throws java.io.IOException
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

      current_alignment += ihmc_common_msgs.msg.dds.LatestModificationMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += behavior_msgs.msg.dds.BehaviorTreeSceneObjectDefinitionMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += behavior_msgs.msg.dds.PersistentDetectionStatusMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += behavior_msgs.msg.dds.PersistentDetectionStatusMessagePubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.BehaviorTreeSceneObjectStateMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.BehaviorTreeSceneObjectStateMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += ihmc_common_msgs.msg.dds.LatestModificationMessagePubSubType.getCdrSerializedSize(data.getLatestModificationToData(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += behavior_msgs.msg.dds.BehaviorTreeSceneObjectDefinitionMessagePubSubType.getCdrSerializedSize(data.getDefinition(), current_alignment);

      current_alignment += behavior_msgs.msg.dds.PersistentDetectionStatusMessagePubSubType.getCdrSerializedSize(data.getPersistentDetection(), current_alignment);

      current_alignment += controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.getCdrSerializedSize(data.getTransformToWorld(), current_alignment);

      current_alignment += behavior_msgs.msg.dds.PersistentDetectionStatusMessagePubSubType.getCdrSerializedSize(data.getDoorPanelDetection(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.BehaviorTreeSceneObjectStateMessage data, us.ihmc.idl.CDR cdr)
   {
      ihmc_common_msgs.msg.dds.LatestModificationMessagePubSubType.write(data.getLatestModificationToData(), cdr);
      cdr.write_type_4(data.getId());

      behavior_msgs.msg.dds.BehaviorTreeSceneObjectDefinitionMessagePubSubType.write(data.getDefinition(), cdr);
      behavior_msgs.msg.dds.PersistentDetectionStatusMessagePubSubType.write(data.getPersistentDetection(), cdr);
      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.write(data.getTransformToWorld(), cdr);
      behavior_msgs.msg.dds.PersistentDetectionStatusMessagePubSubType.write(data.getDoorPanelDetection(), cdr);
   }

   public static void read(behavior_msgs.msg.dds.BehaviorTreeSceneObjectStateMessage data, us.ihmc.idl.CDR cdr)
   {
      ihmc_common_msgs.msg.dds.LatestModificationMessagePubSubType.read(data.getLatestModificationToData(), cdr);	
      data.setId(cdr.read_type_4());
      	
      behavior_msgs.msg.dds.BehaviorTreeSceneObjectDefinitionMessagePubSubType.read(data.getDefinition(), cdr);	
      behavior_msgs.msg.dds.PersistentDetectionStatusMessagePubSubType.read(data.getPersistentDetection(), cdr);	
      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.read(data.getTransformToWorld(), cdr);	
      behavior_msgs.msg.dds.PersistentDetectionStatusMessagePubSubType.read(data.getDoorPanelDetection(), cdr);	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.BehaviorTreeSceneObjectStateMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("latest_modification_to_data", new ihmc_common_msgs.msg.dds.LatestModificationMessagePubSubType(), data.getLatestModificationToData());

      ser.write_type_4("id", data.getId());
      ser.write_type_a("definition", new behavior_msgs.msg.dds.BehaviorTreeSceneObjectDefinitionMessagePubSubType(), data.getDefinition());

      ser.write_type_a("persistent_detection", new behavior_msgs.msg.dds.PersistentDetectionStatusMessagePubSubType(), data.getPersistentDetection());

      ser.write_type_a("transform_to_world", new controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType(), data.getTransformToWorld());

      ser.write_type_a("door_panel_detection", new behavior_msgs.msg.dds.PersistentDetectionStatusMessagePubSubType(), data.getDoorPanelDetection());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.BehaviorTreeSceneObjectStateMessage data)
   {
      ser.read_type_a("latest_modification_to_data", new ihmc_common_msgs.msg.dds.LatestModificationMessagePubSubType(), data.getLatestModificationToData());

      data.setId(ser.read_type_4("id"));
      ser.read_type_a("definition", new behavior_msgs.msg.dds.BehaviorTreeSceneObjectDefinitionMessagePubSubType(), data.getDefinition());

      ser.read_type_a("persistent_detection", new behavior_msgs.msg.dds.PersistentDetectionStatusMessagePubSubType(), data.getPersistentDetection());

      ser.read_type_a("transform_to_world", new controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType(), data.getTransformToWorld());

      ser.read_type_a("door_panel_detection", new behavior_msgs.msg.dds.PersistentDetectionStatusMessagePubSubType(), data.getDoorPanelDetection());

   }

   public static void staticCopy(behavior_msgs.msg.dds.BehaviorTreeSceneObjectStateMessage src, behavior_msgs.msg.dds.BehaviorTreeSceneObjectStateMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.BehaviorTreeSceneObjectStateMessage createData()
   {
      return new behavior_msgs.msg.dds.BehaviorTreeSceneObjectStateMessage();
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
   
   public void serialize(behavior_msgs.msg.dds.BehaviorTreeSceneObjectStateMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.BehaviorTreeSceneObjectStateMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.BehaviorTreeSceneObjectStateMessage src, behavior_msgs.msg.dds.BehaviorTreeSceneObjectStateMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public BehaviorTreeSceneObjectStateMessagePubSubType newInstance()
   {
      return new BehaviorTreeSceneObjectStateMessagePubSubType();
   }
}
