package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "FootstepPathPlanPacket" defined in "FootstepPathPlanPacket_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from FootstepPathPlanPacket_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit FootstepPathPlanPacket_.idl instead.
*
*/
public class FootstepPathPlanPacketPubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.FootstepPathPlanPacket>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::FootstepPathPlanPacket_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.FootstepPathPlanPacket data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.FootstepPathPlanPacket data) throws java.io.IOException
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

      current_alignment += controller_msgs.msg.dds.FootstepDataMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 100; ++i0)
      {
          current_alignment += controller_msgs.msg.dds.FootstepDataMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 100; ++i0)
      {
          current_alignment += controller_msgs.msg.dds.FootstepDataMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (100 * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.FootstepPathPlanPacket data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.FootstepPathPlanPacket data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += controller_msgs.msg.dds.FootstepDataMessagePubSubType.getCdrSerializedSize(data.getStart(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getOriginalGoals().size(); ++i0)
      {
          current_alignment += controller_msgs.msg.dds.FootstepDataMessagePubSubType.getCdrSerializedSize(data.getOriginalGoals().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getPathPlan().size(); ++i0)
      {
          current_alignment += controller_msgs.msg.dds.FootstepDataMessagePubSubType.getCdrSerializedSize(data.getPathPlan().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getFootstepUnknown().size() * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.FootstepPathPlanPacket data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_7(data.getGoalsValid());

      controller_msgs.msg.dds.FootstepDataMessagePubSubType.write(data.getStart(), cdr);
      if(data.getOriginalGoals().size() <= 100)
      cdr.write_type_e(data.getOriginalGoals());else
          throw new RuntimeException("original_goals field exceeds the maximum length");

      if(data.getPathPlan().size() <= 100)
      cdr.write_type_e(data.getPathPlan());else
          throw new RuntimeException("path_plan field exceeds the maximum length");

      if(data.getFootstepUnknown().size() <= 100)
      cdr.write_type_e(data.getFootstepUnknown());else
          throw new RuntimeException("footstep_unknown field exceeds the maximum length");

      cdr.write_type_6(data.getSubOptimality());

      cdr.write_type_6(data.getPathCost());

   }

   public static void read(controller_msgs.msg.dds.FootstepPathPlanPacket data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setGoalsValid(cdr.read_type_7());
      	
      controller_msgs.msg.dds.FootstepDataMessagePubSubType.read(data.getStart(), cdr);	
      cdr.read_type_e(data.getOriginalGoals());	
      cdr.read_type_e(data.getPathPlan());	
      cdr.read_type_e(data.getFootstepUnknown());	
      data.setSubOptimality(cdr.read_type_6());
      	
      data.setPathCost(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.FootstepPathPlanPacket data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_7("goals_valid", data.getGoalsValid());
      ser.write_type_a("start", new controller_msgs.msg.dds.FootstepDataMessagePubSubType(), data.getStart());

      ser.write_type_e("original_goals", data.getOriginalGoals());
      ser.write_type_e("path_plan", data.getPathPlan());
      ser.write_type_e("footstep_unknown", data.getFootstepUnknown());
      ser.write_type_6("sub_optimality", data.getSubOptimality());
      ser.write_type_6("path_cost", data.getPathCost());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.FootstepPathPlanPacket data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setGoalsValid(ser.read_type_7("goals_valid"));
      ser.read_type_a("start", new controller_msgs.msg.dds.FootstepDataMessagePubSubType(), data.getStart());

      ser.read_type_e("original_goals", data.getOriginalGoals());
      ser.read_type_e("path_plan", data.getPathPlan());
      ser.read_type_e("footstep_unknown", data.getFootstepUnknown());
      data.setSubOptimality(ser.read_type_6("sub_optimality"));
      data.setPathCost(ser.read_type_6("path_cost"));
   }

   public static void staticCopy(controller_msgs.msg.dds.FootstepPathPlanPacket src, controller_msgs.msg.dds.FootstepPathPlanPacket dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.FootstepPathPlanPacket createData()
   {
      return new controller_msgs.msg.dds.FootstepPathPlanPacket();
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
   
   public void serialize(controller_msgs.msg.dds.FootstepPathPlanPacket data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.FootstepPathPlanPacket data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.FootstepPathPlanPacket src, controller_msgs.msg.dds.FootstepPathPlanPacket dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public FootstepPathPlanPacketPubSubType newInstance()
   {
      return new FootstepPathPlanPacketPubSubType();
   }
}
