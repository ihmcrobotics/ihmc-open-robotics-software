package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "AbilityHandActionDefinitionMessage" defined in "AbilityHandActionDefinitionMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from AbilityHandActionDefinitionMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit AbilityHandActionDefinitionMessage_.idl instead.
*
*/
public class AbilityHandActionDefinitionMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.AbilityHandActionDefinitionMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::AbilityHandActionDefinitionMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "f86152b3120b24a068a687036eb2afab4280995d8acebb9e89028ea7b1ec6610";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.AbilityHandActionDefinitionMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.AbilityHandActionDefinitionMessage data) throws java.io.IOException
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

      current_alignment += behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += ((6) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += ((6) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.AbilityHandActionDefinitionMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.AbilityHandActionDefinitionMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType.getCdrSerializedSize(data.getDefinition(), current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += ((6) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += ((6) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.AbilityHandActionDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType.write(data.getDefinition(), cdr);
      cdr.write_type_9(data.getRobotSide());

      cdr.write_type_9(data.getControlMode());

      cdr.write_type_9(data.getGrip());

      for(int i0 = 0; i0 < data.getGoalPositions().length; ++i0)
      {
        	cdr.write_type_5(data.getGoalPositions()[i0]);	
      }

      for(int i0 = 0; i0 < data.getGoalVelocities().length; ++i0)
      {
        	cdr.write_type_5(data.getGoalVelocities()[i0]);	
      }

      cdr.write_type_9(data.getSuccessCriteria());

      cdr.write_type_5(data.getEachJointPositionTolerance());

      cdr.write_type_5(data.getSufficientCumulativeJointMovement());

      cdr.write_type_7(data.getEnableWiggleOnFailure());

      cdr.write_type_5(data.getTimeToWiggle());

      cdr.write_type_5(data.getUltimateTimeout());

   }

   public static void read(behavior_msgs.msg.dds.AbilityHandActionDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType.read(data.getDefinition(), cdr);	
      data.setRobotSide(cdr.read_type_9());
      	
      data.setControlMode(cdr.read_type_9());
      	
      data.setGrip(cdr.read_type_9());
      	
      for(int i0 = 0; i0 < data.getGoalPositions().length; ++i0)
      {
        	data.getGoalPositions()[i0] = cdr.read_type_5();
        	
      }
      	
      for(int i0 = 0; i0 < data.getGoalVelocities().length; ++i0)
      {
        	data.getGoalVelocities()[i0] = cdr.read_type_5();
        	
      }
      	
      data.setSuccessCriteria(cdr.read_type_9());
      	
      data.setEachJointPositionTolerance(cdr.read_type_5());
      	
      data.setSufficientCumulativeJointMovement(cdr.read_type_5());
      	
      data.setEnableWiggleOnFailure(cdr.read_type_7());
      	
      data.setTimeToWiggle(cdr.read_type_5());
      	
      data.setUltimateTimeout(cdr.read_type_5());
      	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.AbilityHandActionDefinitionMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("definition", new behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType(), data.getDefinition());

      ser.write_type_9("robot_side", data.getRobotSide());
      ser.write_type_9("control_mode", data.getControlMode());
      ser.write_type_9("grip", data.getGrip());
      ser.write_type_f("goal_positions", data.getGoalPositions());
      ser.write_type_f("goal_velocities", data.getGoalVelocities());
      ser.write_type_9("success_criteria", data.getSuccessCriteria());
      ser.write_type_5("each_joint_position_tolerance", data.getEachJointPositionTolerance());
      ser.write_type_5("sufficient_cumulative_joint_movement", data.getSufficientCumulativeJointMovement());
      ser.write_type_7("enable_wiggle_on_failure", data.getEnableWiggleOnFailure());
      ser.write_type_5("time_to_wiggle", data.getTimeToWiggle());
      ser.write_type_5("ultimate_timeout", data.getUltimateTimeout());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.AbilityHandActionDefinitionMessage data)
   {
      ser.read_type_a("definition", new behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType(), data.getDefinition());

      data.setRobotSide(ser.read_type_9("robot_side"));
      data.setControlMode(ser.read_type_9("control_mode"));
      data.setGrip(ser.read_type_9("grip"));
      ser.read_type_f("goal_positions", data.getGoalPositions());
      ser.read_type_f("goal_velocities", data.getGoalVelocities());
      data.setSuccessCriteria(ser.read_type_9("success_criteria"));
      data.setEachJointPositionTolerance(ser.read_type_5("each_joint_position_tolerance"));
      data.setSufficientCumulativeJointMovement(ser.read_type_5("sufficient_cumulative_joint_movement"));
      data.setEnableWiggleOnFailure(ser.read_type_7("enable_wiggle_on_failure"));
      data.setTimeToWiggle(ser.read_type_5("time_to_wiggle"));
      data.setUltimateTimeout(ser.read_type_5("ultimate_timeout"));
   }

   public static void staticCopy(behavior_msgs.msg.dds.AbilityHandActionDefinitionMessage src, behavior_msgs.msg.dds.AbilityHandActionDefinitionMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.AbilityHandActionDefinitionMessage createData()
   {
      return new behavior_msgs.msg.dds.AbilityHandActionDefinitionMessage();
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
   
   public void serialize(behavior_msgs.msg.dds.AbilityHandActionDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.AbilityHandActionDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.AbilityHandActionDefinitionMessage src, behavior_msgs.msg.dds.AbilityHandActionDefinitionMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public AbilityHandActionDefinitionMessagePubSubType newInstance()
   {
      return new AbilityHandActionDefinitionMessagePubSubType();
   }
}
