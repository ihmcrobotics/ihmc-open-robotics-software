package exoskeleton_msgs.msg.dds;

/**
* 
* Topic data type of the struct "PilotInterfacePacket" defined in "PilotInterfacePacket_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from PilotInterfacePacket_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit PilotInterfacePacket_.idl instead.
*
*/
public class PilotInterfacePacketPubSubType implements us.ihmc.pubsub.TopicDataType<exoskeleton_msgs.msg.dds.PilotInterfacePacket>
{
   public static final java.lang.String name = "exoskeleton_msgs::msg::dds_::PilotInterfacePacket_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "afd3a7159f820393b19eac5f8f15f18d01224eb93a247a2a24f28e93865b7b46";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(exoskeleton_msgs.msg.dds.PilotInterfacePacket data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, exoskeleton_msgs.msg.dds.PilotInterfacePacket data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(exoskeleton_msgs.msg.dds.PilotInterfacePacket data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(exoskeleton_msgs.msg.dds.PilotInterfacePacket data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      return current_alignment - initial_alignment;
   }

   public static void write(exoskeleton_msgs.msg.dds.PilotInterfacePacket data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_2(data.getBehaviourState());

      cdr.write_type_2(data.getRequestedBehaviorState());

      cdr.write_type_2(data.getDesiredStepType());

      cdr.write_type_2(data.getDesiredStepLengthType());

      cdr.write_type_2(data.getDesiredStepStairsType());

      cdr.write_type_7(data.getDesiredStepContinousWalk());

      cdr.write_type_2(data.getDesiredStepsToTake());

      cdr.write_type_7(data.getExecuteBehavior());

      cdr.write_type_2(data.getDesiredSlopeStepType());

      cdr.write_type_2(data.getCurrentPilotState());

   }

   public static void read(exoskeleton_msgs.msg.dds.PilotInterfacePacket data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setBehaviourState(cdr.read_type_2());
      	
      data.setRequestedBehaviorState(cdr.read_type_2());
      	
      data.setDesiredStepType(cdr.read_type_2());
      	
      data.setDesiredStepLengthType(cdr.read_type_2());
      	
      data.setDesiredStepStairsType(cdr.read_type_2());
      	
      data.setDesiredStepContinousWalk(cdr.read_type_7());
      	
      data.setDesiredStepsToTake(cdr.read_type_2());
      	
      data.setExecuteBehavior(cdr.read_type_7());
      	
      data.setDesiredSlopeStepType(cdr.read_type_2());
      	
      data.setCurrentPilotState(cdr.read_type_2());
      	

   }

   @Override
   public final void serialize(exoskeleton_msgs.msg.dds.PilotInterfacePacket data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_2("behaviour_state", data.getBehaviourState());
      ser.write_type_2("requested_behavior_state", data.getRequestedBehaviorState());
      ser.write_type_2("desired_step_type", data.getDesiredStepType());
      ser.write_type_2("desired_step_length_type", data.getDesiredStepLengthType());
      ser.write_type_2("desired_step_stairs_type", data.getDesiredStepStairsType());
      ser.write_type_7("desired_step_continous_walk", data.getDesiredStepContinousWalk());
      ser.write_type_2("desired_steps_to_take", data.getDesiredStepsToTake());
      ser.write_type_7("execute_behavior", data.getExecuteBehavior());
      ser.write_type_2("desired_slope_step_type", data.getDesiredSlopeStepType());
      ser.write_type_2("current_pilot_state", data.getCurrentPilotState());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, exoskeleton_msgs.msg.dds.PilotInterfacePacket data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setBehaviourState(ser.read_type_2("behaviour_state"));
      data.setRequestedBehaviorState(ser.read_type_2("requested_behavior_state"));
      data.setDesiredStepType(ser.read_type_2("desired_step_type"));
      data.setDesiredStepLengthType(ser.read_type_2("desired_step_length_type"));
      data.setDesiredStepStairsType(ser.read_type_2("desired_step_stairs_type"));
      data.setDesiredStepContinousWalk(ser.read_type_7("desired_step_continous_walk"));
      data.setDesiredStepsToTake(ser.read_type_2("desired_steps_to_take"));
      data.setExecuteBehavior(ser.read_type_7("execute_behavior"));
      data.setDesiredSlopeStepType(ser.read_type_2("desired_slope_step_type"));
      data.setCurrentPilotState(ser.read_type_2("current_pilot_state"));
   }

   public static void staticCopy(exoskeleton_msgs.msg.dds.PilotInterfacePacket src, exoskeleton_msgs.msg.dds.PilotInterfacePacket dest)
   {
      dest.set(src);
   }

   @Override
   public exoskeleton_msgs.msg.dds.PilotInterfacePacket createData()
   {
      return new exoskeleton_msgs.msg.dds.PilotInterfacePacket();
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
   
   public void serialize(exoskeleton_msgs.msg.dds.PilotInterfacePacket data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(exoskeleton_msgs.msg.dds.PilotInterfacePacket data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(exoskeleton_msgs.msg.dds.PilotInterfacePacket src, exoskeleton_msgs.msg.dds.PilotInterfacePacket dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public PilotInterfacePacketPubSubType newInstance()
   {
      return new PilotInterfacePacketPubSubType();
   }
}
