package exoskeleton_msgs.msg.dds;

/**
* 
* Topic data type of the struct "ExoStepDataMessage" defined in "ExoStepDataMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from ExoStepDataMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit ExoStepDataMessage_.idl instead.
*
*/
public class ExoStepDataMessagePubSubType implements us.ihmc.pubsub.TopicDataType<exoskeleton_msgs.msg.dds.ExoStepDataMessage>
{
   public static final java.lang.String name = "exoskeleton_msgs::msg::dds_::ExoStepDataMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "c13922017e674a2dd4f2207ce1388ed6ea8bd9932387c98ee32aad5d07e505fb";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(exoskeleton_msgs.msg.dds.ExoStepDataMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, exoskeleton_msgs.msg.dds.ExoStepDataMessage data) throws java.io.IOException
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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(exoskeleton_msgs.msg.dds.ExoStepDataMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(exoskeleton_msgs.msg.dds.ExoStepDataMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(exoskeleton_msgs.msg.dds.ExoStepDataMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_9(data.getRobotSide());

      cdr.write_type_6(data.getStepLength());

      cdr.write_type_6(data.getStepHeight());

      cdr.write_type_6(data.getStepPitch());

      cdr.write_type_6(data.getSwingHeight());

      cdr.write_type_6(data.getSwingDuration());

      cdr.write_type_6(data.getTransferDuration());

   }

   public static void read(exoskeleton_msgs.msg.dds.ExoStepDataMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setRobotSide(cdr.read_type_9());
      	
      data.setStepLength(cdr.read_type_6());
      	
      data.setStepHeight(cdr.read_type_6());
      	
      data.setStepPitch(cdr.read_type_6());
      	
      data.setSwingHeight(cdr.read_type_6());
      	
      data.setSwingDuration(cdr.read_type_6());
      	
      data.setTransferDuration(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(exoskeleton_msgs.msg.dds.ExoStepDataMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_9("robot_side", data.getRobotSide());
      ser.write_type_6("step_length", data.getStepLength());
      ser.write_type_6("step_height", data.getStepHeight());
      ser.write_type_6("step_pitch", data.getStepPitch());
      ser.write_type_6("swing_height", data.getSwingHeight());
      ser.write_type_6("swing_duration", data.getSwingDuration());
      ser.write_type_6("transfer_duration", data.getTransferDuration());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, exoskeleton_msgs.msg.dds.ExoStepDataMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setRobotSide(ser.read_type_9("robot_side"));
      data.setStepLength(ser.read_type_6("step_length"));
      data.setStepHeight(ser.read_type_6("step_height"));
      data.setStepPitch(ser.read_type_6("step_pitch"));
      data.setSwingHeight(ser.read_type_6("swing_height"));
      data.setSwingDuration(ser.read_type_6("swing_duration"));
      data.setTransferDuration(ser.read_type_6("transfer_duration"));
   }

   public static void staticCopy(exoskeleton_msgs.msg.dds.ExoStepDataMessage src, exoskeleton_msgs.msg.dds.ExoStepDataMessage dest)
   {
      dest.set(src);
   }

   @Override
   public exoskeleton_msgs.msg.dds.ExoStepDataMessage createData()
   {
      return new exoskeleton_msgs.msg.dds.ExoStepDataMessage();
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
   
   public void serialize(exoskeleton_msgs.msg.dds.ExoStepDataMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(exoskeleton_msgs.msg.dds.ExoStepDataMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(exoskeleton_msgs.msg.dds.ExoStepDataMessage src, exoskeleton_msgs.msg.dds.ExoStepDataMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public ExoStepDataMessagePubSubType newInstance()
   {
      return new ExoStepDataMessagePubSubType();
   }
}
