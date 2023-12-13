package quadruped_msgs.msg.dds;

/**
* 
* Topic data type of the struct "QuadrupedXGaitSettingsPacket" defined in "QuadrupedXGaitSettingsPacket_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from QuadrupedXGaitSettingsPacket_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit QuadrupedXGaitSettingsPacket_.idl instead.
*
*/
public class QuadrupedXGaitSettingsPacketPubSubType implements us.ihmc.pubsub.TopicDataType<quadruped_msgs.msg.dds.QuadrupedXGaitSettingsPacket>
{
   public static final java.lang.String name = "quadruped_msgs::msg::dds_::QuadrupedXGaitSettingsPacket_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "090c8f92734182b12dcc63f9395305656b25370d1ef1e8ee27781cee5caf5234";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(quadruped_msgs.msg.dds.QuadrupedXGaitSettingsPacket data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, quadruped_msgs.msg.dds.QuadrupedXGaitSettingsPacket data) throws java.io.IOException
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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += quadruped_msgs.msg.dds.QuadrupedGaitTimingsPacketPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += quadruped_msgs.msg.dds.QuadrupedGaitTimingsPacketPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += quadruped_msgs.msg.dds.QuadrupedGaitTimingsPacketPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += quadruped_msgs.msg.dds.QuadrupedGaitTimingsPacketPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += quadruped_msgs.msg.dds.QuadrupedGaitTimingsPacketPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += quadruped_msgs.msg.dds.QuadrupedGaitTimingsPacketPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += quadruped_msgs.msg.dds.QuadrupedGaitTimingsPacketPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += quadruped_msgs.msg.dds.QuadrupedGaitTimingsPacketPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += quadruped_msgs.msg.dds.QuadrupedGaitTimingsPacketPubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(quadruped_msgs.msg.dds.QuadrupedXGaitSettingsPacket data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(quadruped_msgs.msg.dds.QuadrupedXGaitSettingsPacket data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += quadruped_msgs.msg.dds.QuadrupedGaitTimingsPacketPubSubType.getCdrSerializedSize(data.getPaceSlowSettingsPacket(), current_alignment);

      current_alignment += quadruped_msgs.msg.dds.QuadrupedGaitTimingsPacketPubSubType.getCdrSerializedSize(data.getPaceMediumSettingsPacket(), current_alignment);

      current_alignment += quadruped_msgs.msg.dds.QuadrupedGaitTimingsPacketPubSubType.getCdrSerializedSize(data.getPaceFastSettingsPacket(), current_alignment);

      current_alignment += quadruped_msgs.msg.dds.QuadrupedGaitTimingsPacketPubSubType.getCdrSerializedSize(data.getAmbleSlowSettingsPacket(), current_alignment);

      current_alignment += quadruped_msgs.msg.dds.QuadrupedGaitTimingsPacketPubSubType.getCdrSerializedSize(data.getAmbleMediumSettingsPacket(), current_alignment);

      current_alignment += quadruped_msgs.msg.dds.QuadrupedGaitTimingsPacketPubSubType.getCdrSerializedSize(data.getAmbleFastSettingsPacket(), current_alignment);

      current_alignment += quadruped_msgs.msg.dds.QuadrupedGaitTimingsPacketPubSubType.getCdrSerializedSize(data.getTrotSlowSettingsPacket(), current_alignment);

      current_alignment += quadruped_msgs.msg.dds.QuadrupedGaitTimingsPacketPubSubType.getCdrSerializedSize(data.getTrotMediumSettingsPacket(), current_alignment);

      current_alignment += quadruped_msgs.msg.dds.QuadrupedGaitTimingsPacketPubSubType.getCdrSerializedSize(data.getTrotFastSettingsPacket(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(quadruped_msgs.msg.dds.QuadrupedXGaitSettingsPacket data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_6(data.getEndPhaseShift());

      cdr.write_type_6(data.getStanceLength());

      cdr.write_type_6(data.getStanceWidth());

      cdr.write_type_6(data.getStepGroundClearance());

      cdr.write_type_6(data.getMaxHorizontalSpeedFraction());

      cdr.write_type_6(data.getMaxYawSpeedFraction());

      cdr.write_type_9(data.getQuadrupedSpeed());

      quadruped_msgs.msg.dds.QuadrupedGaitTimingsPacketPubSubType.write(data.getPaceSlowSettingsPacket(), cdr);
      quadruped_msgs.msg.dds.QuadrupedGaitTimingsPacketPubSubType.write(data.getPaceMediumSettingsPacket(), cdr);
      quadruped_msgs.msg.dds.QuadrupedGaitTimingsPacketPubSubType.write(data.getPaceFastSettingsPacket(), cdr);
      quadruped_msgs.msg.dds.QuadrupedGaitTimingsPacketPubSubType.write(data.getAmbleSlowSettingsPacket(), cdr);
      quadruped_msgs.msg.dds.QuadrupedGaitTimingsPacketPubSubType.write(data.getAmbleMediumSettingsPacket(), cdr);
      quadruped_msgs.msg.dds.QuadrupedGaitTimingsPacketPubSubType.write(data.getAmbleFastSettingsPacket(), cdr);
      quadruped_msgs.msg.dds.QuadrupedGaitTimingsPacketPubSubType.write(data.getTrotSlowSettingsPacket(), cdr);
      quadruped_msgs.msg.dds.QuadrupedGaitTimingsPacketPubSubType.write(data.getTrotMediumSettingsPacket(), cdr);
      quadruped_msgs.msg.dds.QuadrupedGaitTimingsPacketPubSubType.write(data.getTrotFastSettingsPacket(), cdr);
   }

   public static void read(quadruped_msgs.msg.dds.QuadrupedXGaitSettingsPacket data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setEndPhaseShift(cdr.read_type_6());
      	
      data.setStanceLength(cdr.read_type_6());
      	
      data.setStanceWidth(cdr.read_type_6());
      	
      data.setStepGroundClearance(cdr.read_type_6());
      	
      data.setMaxHorizontalSpeedFraction(cdr.read_type_6());
      	
      data.setMaxYawSpeedFraction(cdr.read_type_6());
      	
      data.setQuadrupedSpeed(cdr.read_type_9());
      	
      quadruped_msgs.msg.dds.QuadrupedGaitTimingsPacketPubSubType.read(data.getPaceSlowSettingsPacket(), cdr);	
      quadruped_msgs.msg.dds.QuadrupedGaitTimingsPacketPubSubType.read(data.getPaceMediumSettingsPacket(), cdr);	
      quadruped_msgs.msg.dds.QuadrupedGaitTimingsPacketPubSubType.read(data.getPaceFastSettingsPacket(), cdr);	
      quadruped_msgs.msg.dds.QuadrupedGaitTimingsPacketPubSubType.read(data.getAmbleSlowSettingsPacket(), cdr);	
      quadruped_msgs.msg.dds.QuadrupedGaitTimingsPacketPubSubType.read(data.getAmbleMediumSettingsPacket(), cdr);	
      quadruped_msgs.msg.dds.QuadrupedGaitTimingsPacketPubSubType.read(data.getAmbleFastSettingsPacket(), cdr);	
      quadruped_msgs.msg.dds.QuadrupedGaitTimingsPacketPubSubType.read(data.getTrotSlowSettingsPacket(), cdr);	
      quadruped_msgs.msg.dds.QuadrupedGaitTimingsPacketPubSubType.read(data.getTrotMediumSettingsPacket(), cdr);	
      quadruped_msgs.msg.dds.QuadrupedGaitTimingsPacketPubSubType.read(data.getTrotFastSettingsPacket(), cdr);	

   }

   @Override
   public final void serialize(quadruped_msgs.msg.dds.QuadrupedXGaitSettingsPacket data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_6("end_phase_shift", data.getEndPhaseShift());
      ser.write_type_6("stance_length", data.getStanceLength());
      ser.write_type_6("stance_width", data.getStanceWidth());
      ser.write_type_6("step_ground_clearance", data.getStepGroundClearance());
      ser.write_type_6("max_horizontal_speed_fraction", data.getMaxHorizontalSpeedFraction());
      ser.write_type_6("max_yaw_speed_fraction", data.getMaxYawSpeedFraction());
      ser.write_type_9("quadruped_speed", data.getQuadrupedSpeed());
      ser.write_type_a("pace_slow_settings_packet", new quadruped_msgs.msg.dds.QuadrupedGaitTimingsPacketPubSubType(), data.getPaceSlowSettingsPacket());

      ser.write_type_a("pace_medium_settings_packet", new quadruped_msgs.msg.dds.QuadrupedGaitTimingsPacketPubSubType(), data.getPaceMediumSettingsPacket());

      ser.write_type_a("pace_fast_settings_packet", new quadruped_msgs.msg.dds.QuadrupedGaitTimingsPacketPubSubType(), data.getPaceFastSettingsPacket());

      ser.write_type_a("amble_slow_settings_packet", new quadruped_msgs.msg.dds.QuadrupedGaitTimingsPacketPubSubType(), data.getAmbleSlowSettingsPacket());

      ser.write_type_a("amble_medium_settings_packet", new quadruped_msgs.msg.dds.QuadrupedGaitTimingsPacketPubSubType(), data.getAmbleMediumSettingsPacket());

      ser.write_type_a("amble_fast_settings_packet", new quadruped_msgs.msg.dds.QuadrupedGaitTimingsPacketPubSubType(), data.getAmbleFastSettingsPacket());

      ser.write_type_a("trot_slow_settings_packet", new quadruped_msgs.msg.dds.QuadrupedGaitTimingsPacketPubSubType(), data.getTrotSlowSettingsPacket());

      ser.write_type_a("trot_medium_settings_packet", new quadruped_msgs.msg.dds.QuadrupedGaitTimingsPacketPubSubType(), data.getTrotMediumSettingsPacket());

      ser.write_type_a("trot_fast_settings_packet", new quadruped_msgs.msg.dds.QuadrupedGaitTimingsPacketPubSubType(), data.getTrotFastSettingsPacket());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, quadruped_msgs.msg.dds.QuadrupedXGaitSettingsPacket data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setEndPhaseShift(ser.read_type_6("end_phase_shift"));
      data.setStanceLength(ser.read_type_6("stance_length"));
      data.setStanceWidth(ser.read_type_6("stance_width"));
      data.setStepGroundClearance(ser.read_type_6("step_ground_clearance"));
      data.setMaxHorizontalSpeedFraction(ser.read_type_6("max_horizontal_speed_fraction"));
      data.setMaxYawSpeedFraction(ser.read_type_6("max_yaw_speed_fraction"));
      data.setQuadrupedSpeed(ser.read_type_9("quadruped_speed"));
      ser.read_type_a("pace_slow_settings_packet", new quadruped_msgs.msg.dds.QuadrupedGaitTimingsPacketPubSubType(), data.getPaceSlowSettingsPacket());

      ser.read_type_a("pace_medium_settings_packet", new quadruped_msgs.msg.dds.QuadrupedGaitTimingsPacketPubSubType(), data.getPaceMediumSettingsPacket());

      ser.read_type_a("pace_fast_settings_packet", new quadruped_msgs.msg.dds.QuadrupedGaitTimingsPacketPubSubType(), data.getPaceFastSettingsPacket());

      ser.read_type_a("amble_slow_settings_packet", new quadruped_msgs.msg.dds.QuadrupedGaitTimingsPacketPubSubType(), data.getAmbleSlowSettingsPacket());

      ser.read_type_a("amble_medium_settings_packet", new quadruped_msgs.msg.dds.QuadrupedGaitTimingsPacketPubSubType(), data.getAmbleMediumSettingsPacket());

      ser.read_type_a("amble_fast_settings_packet", new quadruped_msgs.msg.dds.QuadrupedGaitTimingsPacketPubSubType(), data.getAmbleFastSettingsPacket());

      ser.read_type_a("trot_slow_settings_packet", new quadruped_msgs.msg.dds.QuadrupedGaitTimingsPacketPubSubType(), data.getTrotSlowSettingsPacket());

      ser.read_type_a("trot_medium_settings_packet", new quadruped_msgs.msg.dds.QuadrupedGaitTimingsPacketPubSubType(), data.getTrotMediumSettingsPacket());

      ser.read_type_a("trot_fast_settings_packet", new quadruped_msgs.msg.dds.QuadrupedGaitTimingsPacketPubSubType(), data.getTrotFastSettingsPacket());

   }

   public static void staticCopy(quadruped_msgs.msg.dds.QuadrupedXGaitSettingsPacket src, quadruped_msgs.msg.dds.QuadrupedXGaitSettingsPacket dest)
   {
      dest.set(src);
   }

   @Override
   public quadruped_msgs.msg.dds.QuadrupedXGaitSettingsPacket createData()
   {
      return new quadruped_msgs.msg.dds.QuadrupedXGaitSettingsPacket();
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
   
   public void serialize(quadruped_msgs.msg.dds.QuadrupedXGaitSettingsPacket data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(quadruped_msgs.msg.dds.QuadrupedXGaitSettingsPacket data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(quadruped_msgs.msg.dds.QuadrupedXGaitSettingsPacket src, quadruped_msgs.msg.dds.QuadrupedXGaitSettingsPacket dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public QuadrupedXGaitSettingsPacketPubSubType newInstance()
   {
      return new QuadrupedXGaitSettingsPacketPubSubType();
   }
}
