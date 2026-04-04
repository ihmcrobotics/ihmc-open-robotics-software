package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "AI2RCommandMessage" defined in "AI2RCommandMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from AI2RCommandMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit AI2RCommandMessage_.idl instead.
*
*/
public class AI2RCommandMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.AI2RCommandMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::AI2RCommandMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "b7210c729d2b938d8703fd14c21e7f9b428feb472c3d9f56834ae98bf2e86193";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.AI2RCommandMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.AI2RCommandMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += behavior_msgs.msg.dds.AI2RHandPoseAdaptationMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += behavior_msgs.msg.dds.AI2RNavigationMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += behavior_msgs.msg.dds.AI2RReceiveObjectMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += behavior_msgs.msg.dds.AI2RScanMessagePubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.AI2RCommandMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.AI2RCommandMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getBehaviorToExecute().length() + 1;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += behavior_msgs.msg.dds.AI2RHandPoseAdaptationMessagePubSubType.getCdrSerializedSize(data.getHandPoseAdaptation(), current_alignment);

      current_alignment += behavior_msgs.msg.dds.AI2RNavigationMessagePubSubType.getCdrSerializedSize(data.getNavigation(), current_alignment);

      current_alignment += behavior_msgs.msg.dds.AI2RReceiveObjectMessagePubSubType.getCdrSerializedSize(data.getReceiveObject(), current_alignment);

      current_alignment += behavior_msgs.msg.dds.AI2RScanMessagePubSubType.getCdrSerializedSize(data.getScan(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.AI2RCommandMessage data, us.ihmc.idl.CDR cdr)
   {
      if(data.getBehaviorToExecute().length() <= 255)
      cdr.write_type_d(data.getBehaviorToExecute());else
          throw new RuntimeException("behavior_to_execute field exceeds the maximum length: %d > %d".formatted(data.getBehaviorToExecute().length(), 255));

      cdr.write_type_7(data.getAdaptingBehavior());

      behavior_msgs.msg.dds.AI2RHandPoseAdaptationMessagePubSubType.write(data.getHandPoseAdaptation(), cdr);
      behavior_msgs.msg.dds.AI2RNavigationMessagePubSubType.write(data.getNavigation(), cdr);
      behavior_msgs.msg.dds.AI2RReceiveObjectMessagePubSubType.write(data.getReceiveObject(), cdr);
      behavior_msgs.msg.dds.AI2RScanMessagePubSubType.write(data.getScan(), cdr);
   }

   public static void read(behavior_msgs.msg.dds.AI2RCommandMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.read_type_d(data.getBehaviorToExecute());	
      data.setAdaptingBehavior(cdr.read_type_7());
      	
      behavior_msgs.msg.dds.AI2RHandPoseAdaptationMessagePubSubType.read(data.getHandPoseAdaptation(), cdr);	
      behavior_msgs.msg.dds.AI2RNavigationMessagePubSubType.read(data.getNavigation(), cdr);	
      behavior_msgs.msg.dds.AI2RReceiveObjectMessagePubSubType.read(data.getReceiveObject(), cdr);	
      behavior_msgs.msg.dds.AI2RScanMessagePubSubType.read(data.getScan(), cdr);	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.AI2RCommandMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_d("behavior_to_execute", data.getBehaviorToExecute());
      ser.write_type_7("adapting_behavior", data.getAdaptingBehavior());
      ser.write_type_a("hand_pose_adaptation", new behavior_msgs.msg.dds.AI2RHandPoseAdaptationMessagePubSubType(), data.getHandPoseAdaptation());

      ser.write_type_a("navigation", new behavior_msgs.msg.dds.AI2RNavigationMessagePubSubType(), data.getNavigation());

      ser.write_type_a("receive_object", new behavior_msgs.msg.dds.AI2RReceiveObjectMessagePubSubType(), data.getReceiveObject());

      ser.write_type_a("scan", new behavior_msgs.msg.dds.AI2RScanMessagePubSubType(), data.getScan());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.AI2RCommandMessage data)
   {
      ser.read_type_d("behavior_to_execute", data.getBehaviorToExecute());
      data.setAdaptingBehavior(ser.read_type_7("adapting_behavior"));
      ser.read_type_a("hand_pose_adaptation", new behavior_msgs.msg.dds.AI2RHandPoseAdaptationMessagePubSubType(), data.getHandPoseAdaptation());

      ser.read_type_a("navigation", new behavior_msgs.msg.dds.AI2RNavigationMessagePubSubType(), data.getNavigation());

      ser.read_type_a("receive_object", new behavior_msgs.msg.dds.AI2RReceiveObjectMessagePubSubType(), data.getReceiveObject());

      ser.read_type_a("scan", new behavior_msgs.msg.dds.AI2RScanMessagePubSubType(), data.getScan());

   }

   public static void staticCopy(behavior_msgs.msg.dds.AI2RCommandMessage src, behavior_msgs.msg.dds.AI2RCommandMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.AI2RCommandMessage createData()
   {
      return new behavior_msgs.msg.dds.AI2RCommandMessage();
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
   
   public void serialize(behavior_msgs.msg.dds.AI2RCommandMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.AI2RCommandMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.AI2RCommandMessage src, behavior_msgs.msg.dds.AI2RCommandMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public AI2RCommandMessagePubSubType newInstance()
   {
      return new AI2RCommandMessagePubSubType();
   }
}
