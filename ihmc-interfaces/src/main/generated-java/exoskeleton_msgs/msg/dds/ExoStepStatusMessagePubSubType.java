package exoskeleton_msgs.msg.dds;

/**
* 
* Topic data type of the struct "ExoStepStatusMessage" defined in "ExoStepStatusMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from ExoStepStatusMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit ExoStepStatusMessage_.idl instead.
*
*/
public class ExoStepStatusMessagePubSubType implements us.ihmc.pubsub.TopicDataType<exoskeleton_msgs.msg.dds.ExoStepStatusMessage>
{
   public static final java.lang.String name = "exoskeleton_msgs::msg::dds_::ExoStepStatusMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "7ec73e400d94ffad616f0b1a7bcc73a1f28700475cc42a75a558e2d868f6f00c";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(exoskeleton_msgs.msg.dds.ExoStepStatusMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, exoskeleton_msgs.msg.dds.ExoStepStatusMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(exoskeleton_msgs.msg.dds.ExoStepStatusMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(exoskeleton_msgs.msg.dds.ExoStepStatusMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(exoskeleton_msgs.msg.dds.ExoStepStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_9(data.getFootstepStatus());

      cdr.write_type_2(data.getFootstepIndex());

      cdr.write_type_9(data.getRobotSide());

      cdr.write_type_6(data.getDesiredStepLength());

      cdr.write_type_6(data.getDesiredStepHeight());

      cdr.write_type_6(data.getDesiredStepPitch());

   }

   public static void read(exoskeleton_msgs.msg.dds.ExoStepStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setFootstepStatus(cdr.read_type_9());
      	
      data.setFootstepIndex(cdr.read_type_2());
      	
      data.setRobotSide(cdr.read_type_9());
      	
      data.setDesiredStepLength(cdr.read_type_6());
      	
      data.setDesiredStepHeight(cdr.read_type_6());
      	
      data.setDesiredStepPitch(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(exoskeleton_msgs.msg.dds.ExoStepStatusMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_9("footstep_status", data.getFootstepStatus());
      ser.write_type_2("footstep_index", data.getFootstepIndex());
      ser.write_type_9("robot_side", data.getRobotSide());
      ser.write_type_6("desired_step_length", data.getDesiredStepLength());
      ser.write_type_6("desired_step_height", data.getDesiredStepHeight());
      ser.write_type_6("desired_step_pitch", data.getDesiredStepPitch());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, exoskeleton_msgs.msg.dds.ExoStepStatusMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setFootstepStatus(ser.read_type_9("footstep_status"));
      data.setFootstepIndex(ser.read_type_2("footstep_index"));
      data.setRobotSide(ser.read_type_9("robot_side"));
      data.setDesiredStepLength(ser.read_type_6("desired_step_length"));
      data.setDesiredStepHeight(ser.read_type_6("desired_step_height"));
      data.setDesiredStepPitch(ser.read_type_6("desired_step_pitch"));
   }

   public static void staticCopy(exoskeleton_msgs.msg.dds.ExoStepStatusMessage src, exoskeleton_msgs.msg.dds.ExoStepStatusMessage dest)
   {
      dest.set(src);
   }

   @Override
   public exoskeleton_msgs.msg.dds.ExoStepStatusMessage createData()
   {
      return new exoskeleton_msgs.msg.dds.ExoStepStatusMessage();
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
   
   public void serialize(exoskeleton_msgs.msg.dds.ExoStepStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(exoskeleton_msgs.msg.dds.ExoStepStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(exoskeleton_msgs.msg.dds.ExoStepStatusMessage src, exoskeleton_msgs.msg.dds.ExoStepStatusMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public ExoStepStatusMessagePubSubType newInstance()
   {
      return new ExoStepStatusMessagePubSubType();
   }
}
