package controller_msgs.msg.dds;

/**
 * 
 * Topic data type of the struct "FootstepPlanRequestPacket" defined in
 * "FootstepPlanRequestPacket_.idl". Use this class to provide the TopicDataType to a Participant.
 *
 * This file was automatically generated from FootstepPlanRequestPacket_.idl by
 * us.ihmc.idl.generator.IDLGenerator. Do not update this file directly, edit
 * FootstepPlanRequestPacket_.idl instead.
 *
 */
public class FootstepPlanRequestPacketPubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.FootstepPlanRequestPacket>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::FootstepPlanRequestPacket_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.FootstepPlanRequestPacket data, us.ihmc.pubsub.common.SerializedPayload serializedPayload)
         throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.FootstepPlanRequestPacket data)
         throws java.io.IOException
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

      current_alignment += controller_msgs.msg.dds.FootstepDataMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for (int i0 = 0; i0 < 100; ++i0)
      {
         current_alignment += controller_msgs.msg.dds.FootstepDataMessagePubSubType.getMaxCdrSerializedSize(current_alignment);
      }
      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.FootstepPlanRequestPacket data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.FootstepPlanRequestPacket data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += controller_msgs.msg.dds.FootstepDataMessagePubSubType.getCdrSerializedSize(data.getStartFootstep(), current_alignment);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for (int i0 = 0; i0 < data.getGoals().size(); ++i0)
      {
         current_alignment += controller_msgs.msg.dds.FootstepDataMessagePubSubType.getCdrSerializedSize(data.getGoals().get(i0), current_alignment);
      }

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.FootstepPlanRequestPacket data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      controller_msgs.msg.dds.FootstepDataMessagePubSubType.write(data.getStartFootstep(), cdr);
      cdr.write_type_6(data.getThetaStart());

      cdr.write_type_6(data.getMaxSubOptimality());

      if (data.getGoals().size() <= 100)
         cdr.write_type_e(data.getGoals());
      else
         throw new RuntimeException("goals field exceeds the maximum length");

      cdr.write_type_9(data.getFootstepPlanRequestType());

   }

   public static void read(controller_msgs.msg.dds.FootstepPlanRequestPacket data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());

      controller_msgs.msg.dds.FootstepDataMessagePubSubType.read(data.getStartFootstep(), cdr);
      data.setThetaStart(cdr.read_type_6());

      data.setMaxSubOptimality(cdr.read_type_6());

      cdr.read_type_e(data.getGoals());
      data.setFootstepPlanRequestType(cdr.read_type_9());

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.FootstepPlanRequestPacket data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_a("start_footstep", new controller_msgs.msg.dds.FootstepDataMessagePubSubType(), data.getStartFootstep());

      ser.write_type_6("theta_start", data.getThetaStart());
      ser.write_type_6("max_sub_optimality", data.getMaxSubOptimality());
      ser.write_type_e("goals", data.getGoals());
      ser.write_type_9("footstep_plan_request_type", data.getFootstepPlanRequestType());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.FootstepPlanRequestPacket data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      ser.read_type_a("start_footstep", new controller_msgs.msg.dds.FootstepDataMessagePubSubType(), data.getStartFootstep());

      data.setThetaStart(ser.read_type_6("theta_start"));
      data.setMaxSubOptimality(ser.read_type_6("max_sub_optimality"));
      ser.read_type_e("goals", data.getGoals());
      data.setFootstepPlanRequestType(ser.read_type_9("footstep_plan_request_type"));
   }

   public static void staticCopy(controller_msgs.msg.dds.FootstepPlanRequestPacket src, controller_msgs.msg.dds.FootstepPlanRequestPacket dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.FootstepPlanRequestPacket createData()
   {
      return new controller_msgs.msg.dds.FootstepPlanRequestPacket();
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

   public void serialize(controller_msgs.msg.dds.FootstepPlanRequestPacket data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.FootstepPlanRequestPacket data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }

   public void copy(controller_msgs.msg.dds.FootstepPlanRequestPacket src, controller_msgs.msg.dds.FootstepPlanRequestPacket dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public FootstepPlanRequestPacketPubSubType newInstance()
   {
      return new FootstepPlanRequestPacketPubSubType();
   }
}
