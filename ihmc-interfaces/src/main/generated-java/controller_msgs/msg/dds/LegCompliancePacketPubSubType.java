package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "LegCompliancePacket" defined in "LegCompliancePacket_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from LegCompliancePacket_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit LegCompliancePacket_.idl instead.
*
*/
public class LegCompliancePacketPubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.LegCompliancePacket>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::LegCompliancePacket_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "284a196eb06c22c2872f0d4f5ba911c2ec134e7192d43288f21f236b5e9acbfc";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.LegCompliancePacket data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.LegCompliancePacket data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (100 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.LegCompliancePacket data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.LegCompliancePacket data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getMaxVelocityDeltas().size() * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.LegCompliancePacket data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      if(data.getMaxVelocityDeltas().size() <= 100)
      cdr.write_type_e(data.getMaxVelocityDeltas());else
          throw new RuntimeException("max_velocity_deltas field exceeds the maximum length");

      cdr.write_type_9(data.getRobotSide());

   }

   public static void read(controller_msgs.msg.dds.LegCompliancePacket data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      cdr.read_type_e(data.getMaxVelocityDeltas());	
      data.setRobotSide(cdr.read_type_9());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.LegCompliancePacket data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_e("max_velocity_deltas", data.getMaxVelocityDeltas());
      ser.write_type_9("robot_side", data.getRobotSide());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.LegCompliancePacket data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      ser.read_type_e("max_velocity_deltas", data.getMaxVelocityDeltas());
      data.setRobotSide(ser.read_type_9("robot_side"));
   }

   public static void staticCopy(controller_msgs.msg.dds.LegCompliancePacket src, controller_msgs.msg.dds.LegCompliancePacket dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.LegCompliancePacket createData()
   {
      return new controller_msgs.msg.dds.LegCompliancePacket();
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
   
   public void serialize(controller_msgs.msg.dds.LegCompliancePacket data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.LegCompliancePacket data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.LegCompliancePacket src, controller_msgs.msg.dds.LegCompliancePacket dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public LegCompliancePacketPubSubType newInstance()
   {
      return new LegCompliancePacketPubSubType();
   }
}
