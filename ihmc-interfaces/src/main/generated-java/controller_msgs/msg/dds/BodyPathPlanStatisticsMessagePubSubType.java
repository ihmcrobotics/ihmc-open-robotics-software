package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "BodyPathPlanStatisticsMessage" defined in "BodyPathPlanStatisticsMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from BodyPathPlanStatisticsMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit BodyPathPlanStatisticsMessage_.idl instead.
*
*/
public class BodyPathPlanStatisticsMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.BodyPathPlanStatisticsMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::BodyPathPlanStatisticsMessage_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.BodyPathPlanStatisticsMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.BodyPathPlanStatisticsMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 25; ++i0)
      {
          current_alignment += controller_msgs.msg.dds.VisibilityMapWithNavigableRegionMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += controller_msgs.msg.dds.VisibilityMapMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += controller_msgs.msg.dds.VisibilityMapMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += controller_msgs.msg.dds.VisibilityMapMessagePubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.BodyPathPlanStatisticsMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.BodyPathPlanStatisticsMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getNavigableRegions().size(); ++i0)
      {
          current_alignment += controller_msgs.msg.dds.VisibilityMapWithNavigableRegionMessagePubSubType.getCdrSerializedSize(data.getNavigableRegions().get(i0), current_alignment);}

      current_alignment += controller_msgs.msg.dds.VisibilityMapMessagePubSubType.getCdrSerializedSize(data.getInterRegionsMap(), current_alignment);

      current_alignment += controller_msgs.msg.dds.VisibilityMapMessagePubSubType.getCdrSerializedSize(data.getStartVisibilityMap(), current_alignment);

      current_alignment += controller_msgs.msg.dds.VisibilityMapMessagePubSubType.getCdrSerializedSize(data.getGoalVisibilityMap(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.BodyPathPlanStatisticsMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_2(data.getPlanId());

      if(data.getNavigableRegions().size() <= 25)
      cdr.write_type_e(data.getNavigableRegions());else
          throw new RuntimeException("navigable_regions field exceeds the maximum length");

      controller_msgs.msg.dds.VisibilityMapMessagePubSubType.write(data.getInterRegionsMap(), cdr);
      controller_msgs.msg.dds.VisibilityMapMessagePubSubType.write(data.getStartVisibilityMap(), cdr);
      controller_msgs.msg.dds.VisibilityMapMessagePubSubType.write(data.getGoalVisibilityMap(), cdr);
   }

   public static void read(controller_msgs.msg.dds.BodyPathPlanStatisticsMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setPlanId(cdr.read_type_2());
      	
      cdr.read_type_e(data.getNavigableRegions());	
      controller_msgs.msg.dds.VisibilityMapMessagePubSubType.read(data.getInterRegionsMap(), cdr);	
      controller_msgs.msg.dds.VisibilityMapMessagePubSubType.read(data.getStartVisibilityMap(), cdr);	
      controller_msgs.msg.dds.VisibilityMapMessagePubSubType.read(data.getGoalVisibilityMap(), cdr);	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.BodyPathPlanStatisticsMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_2("plan_id", data.getPlanId());
      ser.write_type_e("navigable_regions", data.getNavigableRegions());
      ser.write_type_a("inter_regions_map", new controller_msgs.msg.dds.VisibilityMapMessagePubSubType(), data.getInterRegionsMap());

      ser.write_type_a("start_visibility_map", new controller_msgs.msg.dds.VisibilityMapMessagePubSubType(), data.getStartVisibilityMap());

      ser.write_type_a("goal_visibility_map", new controller_msgs.msg.dds.VisibilityMapMessagePubSubType(), data.getGoalVisibilityMap());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.BodyPathPlanStatisticsMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setPlanId(ser.read_type_2("plan_id"));
      ser.read_type_e("navigable_regions", data.getNavigableRegions());
      ser.read_type_a("inter_regions_map", new controller_msgs.msg.dds.VisibilityMapMessagePubSubType(), data.getInterRegionsMap());

      ser.read_type_a("start_visibility_map", new controller_msgs.msg.dds.VisibilityMapMessagePubSubType(), data.getStartVisibilityMap());

      ser.read_type_a("goal_visibility_map", new controller_msgs.msg.dds.VisibilityMapMessagePubSubType(), data.getGoalVisibilityMap());

   }

   public static void staticCopy(controller_msgs.msg.dds.BodyPathPlanStatisticsMessage src, controller_msgs.msg.dds.BodyPathPlanStatisticsMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.BodyPathPlanStatisticsMessage createData()
   {
      return new controller_msgs.msg.dds.BodyPathPlanStatisticsMessage();
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
   
   public void serialize(controller_msgs.msg.dds.BodyPathPlanStatisticsMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.BodyPathPlanStatisticsMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.BodyPathPlanStatisticsMessage src, controller_msgs.msg.dds.BodyPathPlanStatisticsMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public BodyPathPlanStatisticsMessagePubSubType newInstance()
   {
      return new BodyPathPlanStatisticsMessagePubSubType();
   }
}
