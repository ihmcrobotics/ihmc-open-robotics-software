package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "ContinuousHikingCommandMessage" defined in "ContinuousHikingCommandMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from ContinuousHikingCommandMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit ContinuousHikingCommandMessage_.idl instead.
*
*/
public class ContinuousHikingCommandMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.ContinuousHikingCommandMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::ContinuousHikingCommandMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "16a67ca29bda8379d5278ae1f141a59cf18a01a63fe6d9d13557e347e9445765";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.ContinuousHikingCommandMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.ContinuousHikingCommandMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.ContinuousHikingCommandMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.ContinuousHikingCommandMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.ContinuousHikingCommandMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_7(data.getEnableContinuousHiking());

      cdr.write_type_4(data.getStepsBeforeSafetyStop());

      cdr.write_type_7(data.getWalkForwards());

      cdr.write_type_7(data.getSquareUpToGoal());

      cdr.write_type_7(data.getUseAstarFootstepPlanner());

      cdr.write_type_7(data.getUseMonteCarloFootstepPlanner());

      cdr.write_type_7(data.getUsePreviousPlanAsReference());

      cdr.write_type_7(data.getUseMonteCarloPlanAsReference());

      cdr.write_type_7(data.getUseJoystickController());

      cdr.write_type_6(data.getForwardValue());

      cdr.write_type_7(data.getWalkBackwards());

      cdr.write_type_6(data.getLateralValue());

      cdr.write_type_6(data.getTurningValue());

   }

   public static void read(behavior_msgs.msg.dds.ContinuousHikingCommandMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setEnableContinuousHiking(cdr.read_type_7());
      	
      data.setStepsBeforeSafetyStop(cdr.read_type_4());
      	
      data.setWalkForwards(cdr.read_type_7());
      	
      data.setSquareUpToGoal(cdr.read_type_7());
      	
      data.setUseAstarFootstepPlanner(cdr.read_type_7());
      	
      data.setUseMonteCarloFootstepPlanner(cdr.read_type_7());
      	
      data.setUsePreviousPlanAsReference(cdr.read_type_7());
      	
      data.setUseMonteCarloPlanAsReference(cdr.read_type_7());
      	
      data.setUseJoystickController(cdr.read_type_7());
      	
      data.setForwardValue(cdr.read_type_6());
      	
      data.setWalkBackwards(cdr.read_type_7());
      	
      data.setLateralValue(cdr.read_type_6());
      	
      data.setTurningValue(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.ContinuousHikingCommandMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_7("enable_continuous_hiking", data.getEnableContinuousHiking());
      ser.write_type_4("steps_before_safety_stop", data.getStepsBeforeSafetyStop());
      ser.write_type_7("walk_forwards", data.getWalkForwards());
      ser.write_type_7("square_up_to_goal", data.getSquareUpToGoal());
      ser.write_type_7("use_astar_footstep_planner", data.getUseAstarFootstepPlanner());
      ser.write_type_7("use_monte_carlo_footstep_planner", data.getUseMonteCarloFootstepPlanner());
      ser.write_type_7("use_previous_plan_as_reference", data.getUsePreviousPlanAsReference());
      ser.write_type_7("use_monte_carlo_plan_as_reference", data.getUseMonteCarloPlanAsReference());
      ser.write_type_7("use_joystick_controller", data.getUseJoystickController());
      ser.write_type_6("forward_value", data.getForwardValue());
      ser.write_type_7("walk_backwards", data.getWalkBackwards());
      ser.write_type_6("lateral_value", data.getLateralValue());
      ser.write_type_6("turning_value", data.getTurningValue());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.ContinuousHikingCommandMessage data)
   {
      data.setEnableContinuousHiking(ser.read_type_7("enable_continuous_hiking"));
      data.setStepsBeforeSafetyStop(ser.read_type_4("steps_before_safety_stop"));
      data.setWalkForwards(ser.read_type_7("walk_forwards"));
      data.setSquareUpToGoal(ser.read_type_7("square_up_to_goal"));
      data.setUseAstarFootstepPlanner(ser.read_type_7("use_astar_footstep_planner"));
      data.setUseMonteCarloFootstepPlanner(ser.read_type_7("use_monte_carlo_footstep_planner"));
      data.setUsePreviousPlanAsReference(ser.read_type_7("use_previous_plan_as_reference"));
      data.setUseMonteCarloPlanAsReference(ser.read_type_7("use_monte_carlo_plan_as_reference"));
      data.setUseJoystickController(ser.read_type_7("use_joystick_controller"));
      data.setForwardValue(ser.read_type_6("forward_value"));
      data.setWalkBackwards(ser.read_type_7("walk_backwards"));
      data.setLateralValue(ser.read_type_6("lateral_value"));
      data.setTurningValue(ser.read_type_6("turning_value"));
   }

   public static void staticCopy(behavior_msgs.msg.dds.ContinuousHikingCommandMessage src, behavior_msgs.msg.dds.ContinuousHikingCommandMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.ContinuousHikingCommandMessage createData()
   {
      return new behavior_msgs.msg.dds.ContinuousHikingCommandMessage();
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
   
   public void serialize(behavior_msgs.msg.dds.ContinuousHikingCommandMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.ContinuousHikingCommandMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.ContinuousHikingCommandMessage src, behavior_msgs.msg.dds.ContinuousHikingCommandMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public ContinuousHikingCommandMessagePubSubType newInstance()
   {
      return new ContinuousHikingCommandMessagePubSubType();
   }
}
