package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "FootstepPlanActionFootstepStateMessage" defined in "FootstepPlanActionFootstepStateMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from FootstepPlanActionFootstepStateMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit FootstepPlanActionFootstepStateMessage_.idl instead.
*
*/
public class FootstepPlanActionFootstepStateMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.FootstepPlanActionFootstepStateMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::FootstepPlanActionFootstepStateMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "c1054031a2f6d8b8f6bfd40839662d152694e6206552d15a45bbdfbc63df771f";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.FootstepPlanActionFootstepStateMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.FootstepPlanActionFootstepStateMessage data) throws java.io.IOException
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

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.FootstepPlanActionFootstepStateMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.FootstepPlanActionFootstepStateMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.FootstepPlanActionFootstepStateMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_7(data.getUnusedPlaceholderField());

   }

   public static void read(behavior_msgs.msg.dds.FootstepPlanActionFootstepStateMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setUnusedPlaceholderField(cdr.read_type_7());
      	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.FootstepPlanActionFootstepStateMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_7("unused_placeholder_field", data.getUnusedPlaceholderField());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.FootstepPlanActionFootstepStateMessage data)
   {
      data.setUnusedPlaceholderField(ser.read_type_7("unused_placeholder_field"));   }

   public static void staticCopy(behavior_msgs.msg.dds.FootstepPlanActionFootstepStateMessage src, behavior_msgs.msg.dds.FootstepPlanActionFootstepStateMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.FootstepPlanActionFootstepStateMessage createData()
   {
      return new behavior_msgs.msg.dds.FootstepPlanActionFootstepStateMessage();
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
   
   public void serialize(behavior_msgs.msg.dds.FootstepPlanActionFootstepStateMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.FootstepPlanActionFootstepStateMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.FootstepPlanActionFootstepStateMessage src, behavior_msgs.msg.dds.FootstepPlanActionFootstepStateMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public FootstepPlanActionFootstepStateMessagePubSubType newInstance()
   {
      return new FootstepPlanActionFootstepStateMessagePubSubType();
   }
}
