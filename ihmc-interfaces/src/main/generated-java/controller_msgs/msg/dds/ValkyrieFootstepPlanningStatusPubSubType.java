package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "ValkyrieFootstepPlanningStatus" defined in "ValkyrieFootstepPlanningStatus_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from ValkyrieFootstepPlanningStatus_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit ValkyrieFootstepPlanningStatus_.idl instead.
*
*/
public class ValkyrieFootstepPlanningStatusPubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.ValkyrieFootstepPlanningStatus>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::ValkyrieFootstepPlanningStatus_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.ValkyrieFootstepPlanningStatus data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.ValkyrieFootstepPlanningStatus data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += controller_msgs.msg.dds.FootstepDataListMessagePubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.ValkyrieFootstepPlanningStatus data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.ValkyrieFootstepPlanningStatus data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += controller_msgs.msg.dds.FootstepDataListMessagePubSubType.getCdrSerializedSize(data.getFootstepDataList(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.ValkyrieFootstepPlanningStatus data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_2(data.getPlanId());

      cdr.write_type_9(data.getPlannerStatus());

      controller_msgs.msg.dds.FootstepDataListMessagePubSubType.write(data.getFootstepDataList(), cdr);
   }

   public static void read(controller_msgs.msg.dds.ValkyrieFootstepPlanningStatus data, us.ihmc.idl.CDR cdr)
   {
      data.setPlanId(cdr.read_type_2());
      	
      data.setPlannerStatus(cdr.read_type_9());
      	
      controller_msgs.msg.dds.FootstepDataListMessagePubSubType.read(data.getFootstepDataList(), cdr);	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.ValkyrieFootstepPlanningStatus data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_2("plan_id", data.getPlanId());
      ser.write_type_9("planner_status", data.getPlannerStatus());
      ser.write_type_a("footstep_data_list", new controller_msgs.msg.dds.FootstepDataListMessagePubSubType(), data.getFootstepDataList());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.ValkyrieFootstepPlanningStatus data)
   {
      data.setPlanId(ser.read_type_2("plan_id"));
      data.setPlannerStatus(ser.read_type_9("planner_status"));
      ser.read_type_a("footstep_data_list", new controller_msgs.msg.dds.FootstepDataListMessagePubSubType(), data.getFootstepDataList());

   }

   public static void staticCopy(controller_msgs.msg.dds.ValkyrieFootstepPlanningStatus src, controller_msgs.msg.dds.ValkyrieFootstepPlanningStatus dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.ValkyrieFootstepPlanningStatus createData()
   {
      return new controller_msgs.msg.dds.ValkyrieFootstepPlanningStatus();
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
   
   public void serialize(controller_msgs.msg.dds.ValkyrieFootstepPlanningStatus data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.ValkyrieFootstepPlanningStatus data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.ValkyrieFootstepPlanningStatus src, controller_msgs.msg.dds.ValkyrieFootstepPlanningStatus dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public ValkyrieFootstepPlanningStatusPubSubType newInstance()
   {
      return new ValkyrieFootstepPlanningStatusPubSubType();
   }
}
