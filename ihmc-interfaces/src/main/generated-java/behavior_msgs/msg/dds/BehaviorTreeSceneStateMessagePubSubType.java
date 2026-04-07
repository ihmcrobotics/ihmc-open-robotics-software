package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "BehaviorTreeSceneStateMessage" defined in "BehaviorTreeSceneStateMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from BehaviorTreeSceneStateMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit BehaviorTreeSceneStateMessage_.idl instead.
*
*/
public class BehaviorTreeSceneStateMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.BehaviorTreeSceneStateMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::BehaviorTreeSceneStateMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "50ae65ee8785b9e995d7db36c93307b7e5e78d30fd6e4a876f77f5327bbb250f";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.BehaviorTreeSceneStateMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.BehaviorTreeSceneStateMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 100; ++i0)
      {
          current_alignment += behavior_msgs.msg.dds.BehaviorTreeSceneObjectStateMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 100; ++i0)
      {
          current_alignment += behavior_msgs.msg.dds.PersistentDetectionStatusMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.BehaviorTreeSceneStateMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.BehaviorTreeSceneStateMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += ihmc_common_msgs.msg.dds.LatestModificationMessagePubSubType.getCdrSerializedSize(data.getLatestModificationToList(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getObjects().size(); ++i0)
      {
          current_alignment += behavior_msgs.msg.dds.BehaviorTreeSceneObjectStateMessagePubSubType.getCdrSerializedSize(data.getObjects().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getPersistentDetections().size(); ++i0)
      {
          current_alignment += behavior_msgs.msg.dds.PersistentDetectionStatusMessagePubSubType.getCdrSerializedSize(data.getPersistentDetections().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.BehaviorTreeSceneStateMessage data, us.ihmc.idl.CDR cdr)
   {
      ihmc_common_msgs.msg.dds.LatestModificationMessagePubSubType.write(data.getLatestModificationToList(), cdr);
      if(data.getObjects().size() <= 100)
      cdr.write_type_e(data.getObjects());else
          throw new RuntimeException("objects field exceeds the maximum length: %d > %d".formatted(data.getObjects().size(), 100));

      if(data.getPersistentDetections().size() <= 100)
      cdr.write_type_e(data.getPersistentDetections());else
          throw new RuntimeException("persistent_detections field exceeds the maximum length: %d > %d".formatted(data.getPersistentDetections().size(), 100));

      cdr.write_type_5(data.getPoseFilterAlpha());

      cdr.write_type_5(data.getAcceptanceConfidence());

      cdr.write_type_5(data.getStabilityFrequency());

      cdr.write_type_5(data.getHistoryDuration());

   }

   public static void read(behavior_msgs.msg.dds.BehaviorTreeSceneStateMessage data, us.ihmc.idl.CDR cdr)
   {
      ihmc_common_msgs.msg.dds.LatestModificationMessagePubSubType.read(data.getLatestModificationToList(), cdr);	
      cdr.read_type_e(data.getObjects());	
      cdr.read_type_e(data.getPersistentDetections());	
      data.setPoseFilterAlpha(cdr.read_type_5());
      	
      data.setAcceptanceConfidence(cdr.read_type_5());
      	
      data.setStabilityFrequency(cdr.read_type_5());
      	
      data.setHistoryDuration(cdr.read_type_5());
      	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.BehaviorTreeSceneStateMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("latest_modification_to_list", new ihmc_common_msgs.msg.dds.LatestModificationMessagePubSubType(), data.getLatestModificationToList());

      ser.write_type_e("objects", data.getObjects());
      ser.write_type_e("persistent_detections", data.getPersistentDetections());
      ser.write_type_5("pose_filter_alpha", data.getPoseFilterAlpha());
      ser.write_type_5("acceptance_confidence", data.getAcceptanceConfidence());
      ser.write_type_5("stability_frequency", data.getStabilityFrequency());
      ser.write_type_5("history_duration", data.getHistoryDuration());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.BehaviorTreeSceneStateMessage data)
   {
      ser.read_type_a("latest_modification_to_list", new ihmc_common_msgs.msg.dds.LatestModificationMessagePubSubType(), data.getLatestModificationToList());

      ser.read_type_e("objects", data.getObjects());
      ser.read_type_e("persistent_detections", data.getPersistentDetections());
      data.setPoseFilterAlpha(ser.read_type_5("pose_filter_alpha"));
      data.setAcceptanceConfidence(ser.read_type_5("acceptance_confidence"));
      data.setStabilityFrequency(ser.read_type_5("stability_frequency"));
      data.setHistoryDuration(ser.read_type_5("history_duration"));
   }

   public static void staticCopy(behavior_msgs.msg.dds.BehaviorTreeSceneStateMessage src, behavior_msgs.msg.dds.BehaviorTreeSceneStateMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.BehaviorTreeSceneStateMessage createData()
   {
      return new behavior_msgs.msg.dds.BehaviorTreeSceneStateMessage();
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
   
   public void serialize(behavior_msgs.msg.dds.BehaviorTreeSceneStateMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.BehaviorTreeSceneStateMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.BehaviorTreeSceneStateMessage src, behavior_msgs.msg.dds.BehaviorTreeSceneStateMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public BehaviorTreeSceneStateMessagePubSubType newInstance()
   {
      return new BehaviorTreeSceneStateMessagePubSubType();
   }
}
