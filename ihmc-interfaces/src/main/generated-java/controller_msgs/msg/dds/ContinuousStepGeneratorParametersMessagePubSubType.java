package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "ContinuousStepGeneratorParametersMessage" defined in "ContinuousStepGeneratorParametersMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from ContinuousStepGeneratorParametersMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit ContinuousStepGeneratorParametersMessage_.idl instead.
*
*/
public class ContinuousStepGeneratorParametersMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.ContinuousStepGeneratorParametersMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::ContinuousStepGeneratorParametersMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "e080b3ef45ee83b61fcdbb9c4457172fba07903e003f0a850072e47cc08b09f1";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.ContinuousStepGeneratorParametersMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.ContinuousStepGeneratorParametersMessage data) throws java.io.IOException
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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.ContinuousStepGeneratorParametersMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.ContinuousStepGeneratorParametersMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.ContinuousStepGeneratorParametersMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_12(data.getSequenceId());

      cdr.write_type_2(data.getNumberOfFootstepsToPlan());

      cdr.write_type_2(data.getNumberOfFixedFootsteps());

      cdr.write_type_6(data.getSwingHeight());

      cdr.write_type_6(data.getSwingDuration());

      cdr.write_type_6(data.getTransferDuration());

      cdr.write_type_6(data.getMaxStepLengthForwards());

      cdr.write_type_6(data.getMaxStepLengthBackwards());

      cdr.write_type_6(data.getDefaultStepWidth());

      cdr.write_type_6(data.getMinStepWidth());

      cdr.write_type_6(data.getMaxStepWidth());

      cdr.write_type_6(data.getTurnMaxAngleInward());

      cdr.write_type_6(data.getTurnMaxAngleOutward());

      cdr.write_type_7(data.getStepsAreAdjustable());

      cdr.write_type_7(data.getSnapToHeightmap());

      cdr.write_type_7(data.getAccountForGroundDrift());

   }

   public static void read(controller_msgs.msg.dds.ContinuousStepGeneratorParametersMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_12());
      	
      data.setNumberOfFootstepsToPlan(cdr.read_type_2());
      	
      data.setNumberOfFixedFootsteps(cdr.read_type_2());
      	
      data.setSwingHeight(cdr.read_type_6());
      	
      data.setSwingDuration(cdr.read_type_6());
      	
      data.setTransferDuration(cdr.read_type_6());
      	
      data.setMaxStepLengthForwards(cdr.read_type_6());
      	
      data.setMaxStepLengthBackwards(cdr.read_type_6());
      	
      data.setDefaultStepWidth(cdr.read_type_6());
      	
      data.setMinStepWidth(cdr.read_type_6());
      	
      data.setMaxStepWidth(cdr.read_type_6());
      	
      data.setTurnMaxAngleInward(cdr.read_type_6());
      	
      data.setTurnMaxAngleOutward(cdr.read_type_6());
      	
      data.setStepsAreAdjustable(cdr.read_type_7());
      	
      data.setSnapToHeightmap(cdr.read_type_7());
      	
      data.setAccountForGroundDrift(cdr.read_type_7());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.ContinuousStepGeneratorParametersMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_12("sequence_id", data.getSequenceId());
      ser.write_type_2("number_of_footsteps_to_plan", data.getNumberOfFootstepsToPlan());
      ser.write_type_2("number_of_fixed_footsteps", data.getNumberOfFixedFootsteps());
      ser.write_type_6("swing_height", data.getSwingHeight());
      ser.write_type_6("swing_duration", data.getSwingDuration());
      ser.write_type_6("transfer_duration", data.getTransferDuration());
      ser.write_type_6("max_step_length_forwards", data.getMaxStepLengthForwards());
      ser.write_type_6("max_step_length_backwards", data.getMaxStepLengthBackwards());
      ser.write_type_6("default_step_width", data.getDefaultStepWidth());
      ser.write_type_6("min_step_width", data.getMinStepWidth());
      ser.write_type_6("max_step_width", data.getMaxStepWidth());
      ser.write_type_6("turn_max_angle_inward", data.getTurnMaxAngleInward());
      ser.write_type_6("turn_max_angle_outward", data.getTurnMaxAngleOutward());
      ser.write_type_7("steps_are_adjustable", data.getStepsAreAdjustable());
      ser.write_type_7("snap_to_heightmap", data.getSnapToHeightmap());
      ser.write_type_7("account_for_ground_drift", data.getAccountForGroundDrift());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.ContinuousStepGeneratorParametersMessage data)
   {
      data.setSequenceId(ser.read_type_12("sequence_id"));
      data.setNumberOfFootstepsToPlan(ser.read_type_2("number_of_footsteps_to_plan"));
      data.setNumberOfFixedFootsteps(ser.read_type_2("number_of_fixed_footsteps"));
      data.setSwingHeight(ser.read_type_6("swing_height"));
      data.setSwingDuration(ser.read_type_6("swing_duration"));
      data.setTransferDuration(ser.read_type_6("transfer_duration"));
      data.setMaxStepLengthForwards(ser.read_type_6("max_step_length_forwards"));
      data.setMaxStepLengthBackwards(ser.read_type_6("max_step_length_backwards"));
      data.setDefaultStepWidth(ser.read_type_6("default_step_width"));
      data.setMinStepWidth(ser.read_type_6("min_step_width"));
      data.setMaxStepWidth(ser.read_type_6("max_step_width"));
      data.setTurnMaxAngleInward(ser.read_type_6("turn_max_angle_inward"));
      data.setTurnMaxAngleOutward(ser.read_type_6("turn_max_angle_outward"));
      data.setStepsAreAdjustable(ser.read_type_7("steps_are_adjustable"));
      data.setSnapToHeightmap(ser.read_type_7("snap_to_heightmap"));
      data.setAccountForGroundDrift(ser.read_type_7("account_for_ground_drift"));
   }

   public static void staticCopy(controller_msgs.msg.dds.ContinuousStepGeneratorParametersMessage src, controller_msgs.msg.dds.ContinuousStepGeneratorParametersMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.ContinuousStepGeneratorParametersMessage createData()
   {
      return new controller_msgs.msg.dds.ContinuousStepGeneratorParametersMessage();
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
   
   public void serialize(controller_msgs.msg.dds.ContinuousStepGeneratorParametersMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.ContinuousStepGeneratorParametersMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.ContinuousStepGeneratorParametersMessage src, controller_msgs.msg.dds.ContinuousStepGeneratorParametersMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public ContinuousStepGeneratorParametersMessagePubSubType newInstance()
   {
      return new ContinuousStepGeneratorParametersMessagePubSubType();
   }
}
