package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "QuadrupedXGaitSettingsPacket" defined in "QuadrupedXGaitSettingsPacket_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from QuadrupedXGaitSettingsPacket_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit QuadrupedXGaitSettingsPacket_.idl instead.
*
*/
public class QuadrupedXGaitSettingsPacketPubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.QuadrupedXGaitSettingsPacket>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::QuadrupedXGaitSettingsPacket_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.QuadrupedXGaitSettingsPacket data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.QuadrupedXGaitSettingsPacket data) throws java.io.IOException
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


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.QuadrupedXGaitSettingsPacket data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.QuadrupedXGaitSettingsPacket data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.QuadrupedXGaitSettingsPacket data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_6(data.getStanceLength());

      cdr.write_type_6(data.getStanceWidth());

      cdr.write_type_6(data.getStepGroundClearance());

      cdr.write_type_6(data.getStepDuration());

      cdr.write_type_6(data.getEndDoubleSupportDuration());

      cdr.write_type_6(data.getEndPhaseShift());

   }

   public static void read(controller_msgs.msg.dds.QuadrupedXGaitSettingsPacket data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setStanceLength(cdr.read_type_6());
      	
      data.setStanceWidth(cdr.read_type_6());
      	
      data.setStepGroundClearance(cdr.read_type_6());
      	
      data.setStepDuration(cdr.read_type_6());
      	
      data.setEndDoubleSupportDuration(cdr.read_type_6());
      	
      data.setEndPhaseShift(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.QuadrupedXGaitSettingsPacket data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_6("stance_length", data.getStanceLength());
      ser.write_type_6("stance_width", data.getStanceWidth());
      ser.write_type_6("step_ground_clearance", data.getStepGroundClearance());
      ser.write_type_6("step_duration", data.getStepDuration());
      ser.write_type_6("end_double_support_duration", data.getEndDoubleSupportDuration());
      ser.write_type_6("end_phase_shift", data.getEndPhaseShift());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.QuadrupedXGaitSettingsPacket data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setStanceLength(ser.read_type_6("stance_length"));
      data.setStanceWidth(ser.read_type_6("stance_width"));
      data.setStepGroundClearance(ser.read_type_6("step_ground_clearance"));
      data.setStepDuration(ser.read_type_6("step_duration"));
      data.setEndDoubleSupportDuration(ser.read_type_6("end_double_support_duration"));
      data.setEndPhaseShift(ser.read_type_6("end_phase_shift"));
   }

   public static void staticCopy(controller_msgs.msg.dds.QuadrupedXGaitSettingsPacket src, controller_msgs.msg.dds.QuadrupedXGaitSettingsPacket dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.QuadrupedXGaitSettingsPacket createData()
   {
      return new controller_msgs.msg.dds.QuadrupedXGaitSettingsPacket();
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
   
   public void serialize(controller_msgs.msg.dds.QuadrupedXGaitSettingsPacket data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.QuadrupedXGaitSettingsPacket data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.QuadrupedXGaitSettingsPacket src, controller_msgs.msg.dds.QuadrupedXGaitSettingsPacket dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public QuadrupedXGaitSettingsPacketPubSubType newInstance()
   {
      return new QuadrupedXGaitSettingsPacketPubSubType();
   }
}
