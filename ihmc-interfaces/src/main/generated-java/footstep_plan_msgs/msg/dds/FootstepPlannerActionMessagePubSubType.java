package footstep_plan_msgs.msg.dds;

/**
* 
* Topic data type of the struct "FootstepPlannerActionMessage" defined in "FootstepPlannerActionMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from FootstepPlannerActionMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit FootstepPlannerActionMessage_.idl instead.
*
*/
public class FootstepPlannerActionMessagePubSubType implements us.ihmc.pubsub.TopicDataType<footstep_plan_msgs.msg.dds.FootstepPlannerActionMessage>
{
   public static final java.lang.String name = "footstep_plan_msgs::msg::dds_::FootstepPlannerActionMessage_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(footstep_plan_msgs.msg.dds.FootstepPlannerActionMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, footstep_plan_msgs.msg.dds.FootstepPlannerActionMessage data) throws java.io.IOException
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

   public final static int getCdrSerializedSize(footstep_plan_msgs.msg.dds.FootstepPlannerActionMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(footstep_plan_msgs.msg.dds.FootstepPlannerActionMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(footstep_plan_msgs.msg.dds.FootstepPlannerActionMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_9(data.getRequestedAction());

   }

   public static void read(footstep_plan_msgs.msg.dds.FootstepPlannerActionMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setRequestedAction(cdr.read_type_9());
      	

   }

   @Override
   public final void serialize(footstep_plan_msgs.msg.dds.FootstepPlannerActionMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_9("requested_action", data.getRequestedAction());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, footstep_plan_msgs.msg.dds.FootstepPlannerActionMessage data)
   {
      data.setRequestedAction(ser.read_type_9("requested_action"));   }

   public static void staticCopy(footstep_plan_msgs.msg.dds.FootstepPlannerActionMessage src, footstep_plan_msgs.msg.dds.FootstepPlannerActionMessage dest)
   {
      dest.set(src);
   }

   @Override
   public footstep_plan_msgs.msg.dds.FootstepPlannerActionMessage createData()
   {
      return new footstep_plan_msgs.msg.dds.FootstepPlannerActionMessage();
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
   
   public void serialize(footstep_plan_msgs.msg.dds.FootstepPlannerActionMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(footstep_plan_msgs.msg.dds.FootstepPlannerActionMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(footstep_plan_msgs.msg.dds.FootstepPlannerActionMessage src, footstep_plan_msgs.msg.dds.FootstepPlannerActionMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public FootstepPlannerActionMessagePubSubType newInstance()
   {
      return new FootstepPlannerActionMessagePubSubType();
   }
}
