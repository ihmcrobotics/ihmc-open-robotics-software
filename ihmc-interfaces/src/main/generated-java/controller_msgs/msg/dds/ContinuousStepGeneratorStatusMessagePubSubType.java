package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "ContinuousStepGeneratorStatusMessage" defined in "ContinuousStepGeneratorStatusMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from ContinuousStepGeneratorStatusMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit ContinuousStepGeneratorStatusMessage_.idl instead.
*
*/
public class ContinuousStepGeneratorStatusMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.ContinuousStepGeneratorStatusMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::ContinuousStepGeneratorStatusMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "866695640f461c37882c4b65734666d23122e0a9a91b74884f16fb16a84ac076";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.ContinuousStepGeneratorStatusMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.ContinuousStepGeneratorStatusMessage data) throws java.io.IOException
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

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.ContinuousStepGeneratorStatusMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.ContinuousStepGeneratorStatusMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


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


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.ContinuousStepGeneratorStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_7(data.getIsWalking());

      cdr.write_type_7(data.getIsInUnitVelocities());

      cdr.write_type_6(data.getCurrentForwardVelocity());

      cdr.write_type_6(data.getCurrentLateralVelocity());

      cdr.write_type_6(data.getCurrentTurnVelocity());

      cdr.write_type_6(data.getCurrentSwingHeight());

      cdr.write_type_6(data.getCurrentSwingDuration());

      cdr.write_type_6(data.getCurrentTransferDuration());

      cdr.write_type_6(data.getCurrentMaxStepLengthForwards());

      cdr.write_type_6(data.getCurrentMaxStepLengthBackwards());

      cdr.write_type_6(data.getCurrentMaxStepWidth());

      cdr.write_type_6(data.getCurrentMinStepWidth());

      cdr.write_type_6(data.getCurrentDefaultStepWidth());

      cdr.write_type_6(data.getCurrentTurnMaxAngleInward());

      cdr.write_type_6(data.getCurrentTurnMaxAngleOutward());

      cdr.write_type_7(data.getAreStepsAdjustable());

      cdr.write_type_7(data.getSnappingToHeightmap());

      cdr.write_type_7(data.getAccountingForGroundDrift());

   }

   public static void read(controller_msgs.msg.dds.ContinuousStepGeneratorStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setIsWalking(cdr.read_type_7());
      	
      data.setIsInUnitVelocities(cdr.read_type_7());
      	
      data.setCurrentForwardVelocity(cdr.read_type_6());
      	
      data.setCurrentLateralVelocity(cdr.read_type_6());
      	
      data.setCurrentTurnVelocity(cdr.read_type_6());
      	
      data.setCurrentSwingHeight(cdr.read_type_6());
      	
      data.setCurrentSwingDuration(cdr.read_type_6());
      	
      data.setCurrentTransferDuration(cdr.read_type_6());
      	
      data.setCurrentMaxStepLengthForwards(cdr.read_type_6());
      	
      data.setCurrentMaxStepLengthBackwards(cdr.read_type_6());
      	
      data.setCurrentMaxStepWidth(cdr.read_type_6());
      	
      data.setCurrentMinStepWidth(cdr.read_type_6());
      	
      data.setCurrentDefaultStepWidth(cdr.read_type_6());
      	
      data.setCurrentTurnMaxAngleInward(cdr.read_type_6());
      	
      data.setCurrentTurnMaxAngleOutward(cdr.read_type_6());
      	
      data.setAreStepsAdjustable(cdr.read_type_7());
      	
      data.setSnappingToHeightmap(cdr.read_type_7());
      	
      data.setAccountingForGroundDrift(cdr.read_type_7());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.ContinuousStepGeneratorStatusMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_7("is_walking", data.getIsWalking());
      ser.write_type_7("is_in_unit_velocities", data.getIsInUnitVelocities());
      ser.write_type_6("current_forward_velocity", data.getCurrentForwardVelocity());
      ser.write_type_6("current_lateral_velocity", data.getCurrentLateralVelocity());
      ser.write_type_6("current_turn_velocity", data.getCurrentTurnVelocity());
      ser.write_type_6("current_swing_height", data.getCurrentSwingHeight());
      ser.write_type_6("current_swing_duration", data.getCurrentSwingDuration());
      ser.write_type_6("current_transfer_duration", data.getCurrentTransferDuration());
      ser.write_type_6("current_max_step_length_forwards", data.getCurrentMaxStepLengthForwards());
      ser.write_type_6("current_max_step_length_backwards", data.getCurrentMaxStepLengthBackwards());
      ser.write_type_6("current_max_step_width", data.getCurrentMaxStepWidth());
      ser.write_type_6("current_min_step_width", data.getCurrentMinStepWidth());
      ser.write_type_6("current_default_step_width", data.getCurrentDefaultStepWidth());
      ser.write_type_6("current_turn_max_angle_inward", data.getCurrentTurnMaxAngleInward());
      ser.write_type_6("current_turn_max_angle_outward", data.getCurrentTurnMaxAngleOutward());
      ser.write_type_7("are_steps_adjustable", data.getAreStepsAdjustable());
      ser.write_type_7("snapping_to_heightmap", data.getSnappingToHeightmap());
      ser.write_type_7("accounting_for_ground_drift", data.getAccountingForGroundDrift());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.ContinuousStepGeneratorStatusMessage data)
   {
      data.setIsWalking(ser.read_type_7("is_walking"));
      data.setIsInUnitVelocities(ser.read_type_7("is_in_unit_velocities"));
      data.setCurrentForwardVelocity(ser.read_type_6("current_forward_velocity"));
      data.setCurrentLateralVelocity(ser.read_type_6("current_lateral_velocity"));
      data.setCurrentTurnVelocity(ser.read_type_6("current_turn_velocity"));
      data.setCurrentSwingHeight(ser.read_type_6("current_swing_height"));
      data.setCurrentSwingDuration(ser.read_type_6("current_swing_duration"));
      data.setCurrentTransferDuration(ser.read_type_6("current_transfer_duration"));
      data.setCurrentMaxStepLengthForwards(ser.read_type_6("current_max_step_length_forwards"));
      data.setCurrentMaxStepLengthBackwards(ser.read_type_6("current_max_step_length_backwards"));
      data.setCurrentMaxStepWidth(ser.read_type_6("current_max_step_width"));
      data.setCurrentMinStepWidth(ser.read_type_6("current_min_step_width"));
      data.setCurrentDefaultStepWidth(ser.read_type_6("current_default_step_width"));
      data.setCurrentTurnMaxAngleInward(ser.read_type_6("current_turn_max_angle_inward"));
      data.setCurrentTurnMaxAngleOutward(ser.read_type_6("current_turn_max_angle_outward"));
      data.setAreStepsAdjustable(ser.read_type_7("are_steps_adjustable"));
      data.setSnappingToHeightmap(ser.read_type_7("snapping_to_heightmap"));
      data.setAccountingForGroundDrift(ser.read_type_7("accounting_for_ground_drift"));
   }

   public static void staticCopy(controller_msgs.msg.dds.ContinuousStepGeneratorStatusMessage src, controller_msgs.msg.dds.ContinuousStepGeneratorStatusMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.ContinuousStepGeneratorStatusMessage createData()
   {
      return new controller_msgs.msg.dds.ContinuousStepGeneratorStatusMessage();
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
   
   public void serialize(controller_msgs.msg.dds.ContinuousStepGeneratorStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.ContinuousStepGeneratorStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.ContinuousStepGeneratorStatusMessage src, controller_msgs.msg.dds.ContinuousStepGeneratorStatusMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public ContinuousStepGeneratorStatusMessagePubSubType newInstance()
   {
      return new ContinuousStepGeneratorStatusMessagePubSubType();
   }
}
