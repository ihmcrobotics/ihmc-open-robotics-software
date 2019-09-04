package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "FootstepPlanningToolboxOutputStatus" defined in "FootstepPlanningToolboxOutputStatus_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from FootstepPlanningToolboxOutputStatus_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit FootstepPlanningToolboxOutputStatus_.idl instead.
*
*/
public class FootstepPlanningToolboxOutputStatusPubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.FootstepPlanningToolboxOutputStatus>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::FootstepPlanningToolboxOutputStatus_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.FootstepPlanningToolboxOutputStatus data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.FootstepPlanningToolboxOutputStatus data) throws java.io.IOException
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

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += controller_msgs.msg.dds.PlanarRegionsListMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 100; ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PosePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += controller_msgs.msg.dds.FootstepPlanningStatisticsPubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.FootstepPlanningToolboxOutputStatus data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.FootstepPlanningToolboxOutputStatus data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += controller_msgs.msg.dds.FootstepDataListMessagePubSubType.getCdrSerializedSize(data.getFootstepDataList(), current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += controller_msgs.msg.dds.PlanarRegionsListMessagePubSubType.getCdrSerializedSize(data.getPlanarRegionsList(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getBodyPath().size(); ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PosePubSubType.getCdrSerializedSize(data.getBodyPath().get(i0), current_alignment);}

      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getCdrSerializedSize(data.getLowLevelPlannerGoal(), current_alignment);

      current_alignment += controller_msgs.msg.dds.FootstepPlanningStatisticsPubSubType.getCdrSerializedSize(data.getFootstepPlanningStatistics(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.FootstepPlanningToolboxOutputStatus data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      controller_msgs.msg.dds.FootstepDataListMessagePubSubType.write(data.getFootstepDataList(), cdr);
      cdr.write_type_9(data.getFootstepPlanningResult());

      cdr.write_type_2(data.getPlanId());

      controller_msgs.msg.dds.PlanarRegionsListMessagePubSubType.write(data.getPlanarRegionsList(), cdr);
      if(data.getBodyPath().size() <= 100)
      cdr.write_type_e(data.getBodyPath());else
          throw new RuntimeException("body_path field exceeds the maximum length");

      geometry_msgs.msg.dds.PosePubSubType.write(data.getLowLevelPlannerGoal(), cdr);
      controller_msgs.msg.dds.FootstepPlanningStatisticsPubSubType.write(data.getFootstepPlanningStatistics(), cdr);
   }

   public static void read(controller_msgs.msg.dds.FootstepPlanningToolboxOutputStatus data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      controller_msgs.msg.dds.FootstepDataListMessagePubSubType.read(data.getFootstepDataList(), cdr);	
      data.setFootstepPlanningResult(cdr.read_type_9());
      	
      data.setPlanId(cdr.read_type_2());
      	
      controller_msgs.msg.dds.PlanarRegionsListMessagePubSubType.read(data.getPlanarRegionsList(), cdr);	
      cdr.read_type_e(data.getBodyPath());	
      geometry_msgs.msg.dds.PosePubSubType.read(data.getLowLevelPlannerGoal(), cdr);	
      controller_msgs.msg.dds.FootstepPlanningStatisticsPubSubType.read(data.getFootstepPlanningStatistics(), cdr);	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.FootstepPlanningToolboxOutputStatus data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_a("footstep_data_list", new controller_msgs.msg.dds.FootstepDataListMessagePubSubType(), data.getFootstepDataList());

      ser.write_type_9("footstep_planning_result", data.getFootstepPlanningResult());
      ser.write_type_2("plan_id", data.getPlanId());
      ser.write_type_a("planar_regions_list", new controller_msgs.msg.dds.PlanarRegionsListMessagePubSubType(), data.getPlanarRegionsList());

      ser.write_type_e("body_path", data.getBodyPath());
      ser.write_type_a("low_level_planner_goal", new geometry_msgs.msg.dds.PosePubSubType(), data.getLowLevelPlannerGoal());

      ser.write_type_a("footstep_planning_statistics", new controller_msgs.msg.dds.FootstepPlanningStatisticsPubSubType(), data.getFootstepPlanningStatistics());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.FootstepPlanningToolboxOutputStatus data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      ser.read_type_a("footstep_data_list", new controller_msgs.msg.dds.FootstepDataListMessagePubSubType(), data.getFootstepDataList());

      data.setFootstepPlanningResult(ser.read_type_9("footstep_planning_result"));
      data.setPlanId(ser.read_type_2("plan_id"));
      ser.read_type_a("planar_regions_list", new controller_msgs.msg.dds.PlanarRegionsListMessagePubSubType(), data.getPlanarRegionsList());

      ser.read_type_e("body_path", data.getBodyPath());
      ser.read_type_a("low_level_planner_goal", new geometry_msgs.msg.dds.PosePubSubType(), data.getLowLevelPlannerGoal());

      ser.read_type_a("footstep_planning_statistics", new controller_msgs.msg.dds.FootstepPlanningStatisticsPubSubType(), data.getFootstepPlanningStatistics());

   }

   public static void staticCopy(controller_msgs.msg.dds.FootstepPlanningToolboxOutputStatus src, controller_msgs.msg.dds.FootstepPlanningToolboxOutputStatus dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.FootstepPlanningToolboxOutputStatus createData()
   {
      return new controller_msgs.msg.dds.FootstepPlanningToolboxOutputStatus();
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
   
   public void serialize(controller_msgs.msg.dds.FootstepPlanningToolboxOutputStatus data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.FootstepPlanningToolboxOutputStatus data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.FootstepPlanningToolboxOutputStatus src, controller_msgs.msg.dds.FootstepPlanningToolboxOutputStatus dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public FootstepPlanningToolboxOutputStatusPubSubType newInstance()
   {
      return new FootstepPlanningToolboxOutputStatusPubSubType();
   }
}
