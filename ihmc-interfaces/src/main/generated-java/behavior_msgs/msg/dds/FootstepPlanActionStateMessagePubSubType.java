package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "FootstepPlanActionStateMessage" defined in "FootstepPlanActionStateMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from FootstepPlanActionStateMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit FootstepPlanActionStateMessage_.idl instead.
*
*/
public class FootstepPlanActionStateMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.FootstepPlanActionStateMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::FootstepPlanActionStateMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "0cec7d0777a3220d60c681d5932bca9a0b0ad003f09c311857c97df405da9be2";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.FootstepPlanActionStateMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.FootstepPlanActionStateMessage data) throws java.io.IOException
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

      current_alignment += behavior_msgs.msg.dds.BehaviorActionStateMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += behavior_msgs.msg.dds.FootstepPlanActionDefinitionMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 50; ++i0)
      {
          current_alignment += behavior_msgs.msg.dds.FootstepPlanActionFootstepStateMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.FootstepPlanActionStateMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.FootstepPlanActionStateMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += behavior_msgs.msg.dds.BehaviorActionStateMessagePubSubType.getCdrSerializedSize(data.getActionState(), current_alignment);

      current_alignment += behavior_msgs.msg.dds.FootstepPlanActionDefinitionMessagePubSubType.getCdrSerializedSize(data.getDefinition(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getFootsteps().size(); ++i0)
      {
          current_alignment += behavior_msgs.msg.dds.FootstepPlanActionFootstepStateMessagePubSubType.getCdrSerializedSize(data.getFootsteps().get(i0), current_alignment);}


      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.FootstepPlanActionStateMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.BehaviorActionStateMessagePubSubType.write(data.getActionState(), cdr);
      behavior_msgs.msg.dds.FootstepPlanActionDefinitionMessagePubSubType.write(data.getDefinition(), cdr);
      if(data.getFootsteps().size() <= 50)
      cdr.write_type_e(data.getFootsteps());else
          throw new RuntimeException("footsteps field exceeds the maximum length");

   }

   public static void read(behavior_msgs.msg.dds.FootstepPlanActionStateMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.BehaviorActionStateMessagePubSubType.read(data.getActionState(), cdr);	
      behavior_msgs.msg.dds.FootstepPlanActionDefinitionMessagePubSubType.read(data.getDefinition(), cdr);	
      cdr.read_type_e(data.getFootsteps());	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.FootstepPlanActionStateMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("action_state", new behavior_msgs.msg.dds.BehaviorActionStateMessagePubSubType(), data.getActionState());

      ser.write_type_a("definition", new behavior_msgs.msg.dds.FootstepPlanActionDefinitionMessagePubSubType(), data.getDefinition());

      ser.write_type_e("footsteps", data.getFootsteps());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.FootstepPlanActionStateMessage data)
   {
      ser.read_type_a("action_state", new behavior_msgs.msg.dds.BehaviorActionStateMessagePubSubType(), data.getActionState());

      ser.read_type_a("definition", new behavior_msgs.msg.dds.FootstepPlanActionDefinitionMessagePubSubType(), data.getDefinition());

      ser.read_type_e("footsteps", data.getFootsteps());
   }

   public static void staticCopy(behavior_msgs.msg.dds.FootstepPlanActionStateMessage src, behavior_msgs.msg.dds.FootstepPlanActionStateMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.FootstepPlanActionStateMessage createData()
   {
      return new behavior_msgs.msg.dds.FootstepPlanActionStateMessage();
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
   
   public void serialize(behavior_msgs.msg.dds.FootstepPlanActionStateMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.FootstepPlanActionStateMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.FootstepPlanActionStateMessage src, behavior_msgs.msg.dds.FootstepPlanActionStateMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public FootstepPlanActionStateMessagePubSubType newInstance()
   {
      return new FootstepPlanActionStateMessagePubSubType();
   }
}
