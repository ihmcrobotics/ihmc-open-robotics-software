package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "ValkyrieFootstepPlanningResult" defined in "ValkyrieFootstepPlanningResult_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from ValkyrieFootstepPlanningResult_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit ValkyrieFootstepPlanningResult_.idl instead.
*
*/
public class ValkyrieFootstepPlanningResultPubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.ValkyrieFootstepPlanningResult>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::ValkyrieFootstepPlanningResult_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.ValkyrieFootstepPlanningResult data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.ValkyrieFootstepPlanningResult data) throws java.io.IOException
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

      current_alignment += controller_msgs.msg.dds.FootstepDataListMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += controller_msgs.msg.dds.PlanarRegionsListMessagePubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.ValkyrieFootstepPlanningResult data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.ValkyrieFootstepPlanningResult data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += controller_msgs.msg.dds.FootstepDataListMessagePubSubType.getCdrSerializedSize(data.getFootstepDataList(), current_alignment);

      current_alignment += controller_msgs.msg.dds.PlanarRegionsListMessagePubSubType.getCdrSerializedSize(data.getPlanarRegionsList(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.ValkyrieFootstepPlanningResult data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_2(data.getPlanId());

      controller_msgs.msg.dds.FootstepDataListMessagePubSubType.write(data.getFootstepDataList(), cdr);
      controller_msgs.msg.dds.PlanarRegionsListMessagePubSubType.write(data.getPlanarRegionsList(), cdr);
   }

   public static void read(controller_msgs.msg.dds.ValkyrieFootstepPlanningResult data, us.ihmc.idl.CDR cdr)
   {
      data.setPlanId(cdr.read_type_2());
      	
      controller_msgs.msg.dds.FootstepDataListMessagePubSubType.read(data.getFootstepDataList(), cdr);	
      controller_msgs.msg.dds.PlanarRegionsListMessagePubSubType.read(data.getPlanarRegionsList(), cdr);	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.ValkyrieFootstepPlanningResult data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_2("plan_id", data.getPlanId());
      ser.write_type_a("footstep_data_list", new controller_msgs.msg.dds.FootstepDataListMessagePubSubType(), data.getFootstepDataList());

      ser.write_type_a("planar_regions_list", new controller_msgs.msg.dds.PlanarRegionsListMessagePubSubType(), data.getPlanarRegionsList());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.ValkyrieFootstepPlanningResult data)
   {
      data.setPlanId(ser.read_type_2("plan_id"));
      ser.read_type_a("footstep_data_list", new controller_msgs.msg.dds.FootstepDataListMessagePubSubType(), data.getFootstepDataList());

      ser.read_type_a("planar_regions_list", new controller_msgs.msg.dds.PlanarRegionsListMessagePubSubType(), data.getPlanarRegionsList());

   }

   public static void staticCopy(controller_msgs.msg.dds.ValkyrieFootstepPlanningResult src, controller_msgs.msg.dds.ValkyrieFootstepPlanningResult dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.ValkyrieFootstepPlanningResult createData()
   {
      return new controller_msgs.msg.dds.ValkyrieFootstepPlanningResult();
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
   
   public void serialize(controller_msgs.msg.dds.ValkyrieFootstepPlanningResult data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.ValkyrieFootstepPlanningResult data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.ValkyrieFootstepPlanningResult src, controller_msgs.msg.dds.ValkyrieFootstepPlanningResult dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public ValkyrieFootstepPlanningResultPubSubType newInstance()
   {
      return new ValkyrieFootstepPlanningResultPubSubType();
   }
}
