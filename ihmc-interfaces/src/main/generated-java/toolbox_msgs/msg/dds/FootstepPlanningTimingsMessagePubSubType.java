package toolbox_msgs.msg.dds;

/**
* 
* Topic data type of the struct "FootstepPlanningTimingsMessage" defined in "FootstepPlanningTimingsMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from FootstepPlanningTimingsMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit FootstepPlanningTimingsMessage_.idl instead.
*
*/
public class FootstepPlanningTimingsMessagePubSubType implements us.ihmc.pubsub.TopicDataType<toolbox_msgs.msg.dds.FootstepPlanningTimingsMessage>
{
   public static final java.lang.String name = "toolbox_msgs::msg::dds_::FootstepPlanningTimingsMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "feb88f1e93d3f50315ab69bbf6566ceab6145c00dcc29d709cffbd23adc974e0";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(toolbox_msgs.msg.dds.FootstepPlanningTimingsMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, toolbox_msgs.msg.dds.FootstepPlanningTimingsMessage data) throws java.io.IOException
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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.FootstepPlanningTimingsMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.FootstepPlanningTimingsMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(toolbox_msgs.msg.dds.FootstepPlanningTimingsMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_6(data.getTotalElapsedSeconds());

      cdr.write_type_6(data.getTimeBeforePlanningSeconds());

      cdr.write_type_6(data.getTimePlanningBodyPathSeconds());

      cdr.write_type_6(data.getTimePlanningStepsSeconds());

      cdr.write_type_11(data.getPathPlanningIterations());

      cdr.write_type_11(data.getStepPlanningIterations());

   }

   public static void read(toolbox_msgs.msg.dds.FootstepPlanningTimingsMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setTotalElapsedSeconds(cdr.read_type_6());
      	
      data.setTimeBeforePlanningSeconds(cdr.read_type_6());
      	
      data.setTimePlanningBodyPathSeconds(cdr.read_type_6());
      	
      data.setTimePlanningStepsSeconds(cdr.read_type_6());
      	
      data.setPathPlanningIterations(cdr.read_type_11());
      	
      data.setStepPlanningIterations(cdr.read_type_11());
      	

   }

   @Override
   public final void serialize(toolbox_msgs.msg.dds.FootstepPlanningTimingsMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_6("total_elapsed_seconds", data.getTotalElapsedSeconds());
      ser.write_type_6("time_before_planning_seconds", data.getTimeBeforePlanningSeconds());
      ser.write_type_6("time_planning_body_path_seconds", data.getTimePlanningBodyPathSeconds());
      ser.write_type_6("time_planning_steps_seconds", data.getTimePlanningStepsSeconds());
      ser.write_type_11("path_planning_iterations", data.getPathPlanningIterations());
      ser.write_type_11("step_planning_iterations", data.getStepPlanningIterations());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, toolbox_msgs.msg.dds.FootstepPlanningTimingsMessage data)
   {
      data.setTotalElapsedSeconds(ser.read_type_6("total_elapsed_seconds"));
      data.setTimeBeforePlanningSeconds(ser.read_type_6("time_before_planning_seconds"));
      data.setTimePlanningBodyPathSeconds(ser.read_type_6("time_planning_body_path_seconds"));
      data.setTimePlanningStepsSeconds(ser.read_type_6("time_planning_steps_seconds"));
      data.setPathPlanningIterations(ser.read_type_11("path_planning_iterations"));
      data.setStepPlanningIterations(ser.read_type_11("step_planning_iterations"));
   }

   public static void staticCopy(toolbox_msgs.msg.dds.FootstepPlanningTimingsMessage src, toolbox_msgs.msg.dds.FootstepPlanningTimingsMessage dest)
   {
      dest.set(src);
   }

   @Override
   public toolbox_msgs.msg.dds.FootstepPlanningTimingsMessage createData()
   {
      return new toolbox_msgs.msg.dds.FootstepPlanningTimingsMessage();
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
   
   public void serialize(toolbox_msgs.msg.dds.FootstepPlanningTimingsMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(toolbox_msgs.msg.dds.FootstepPlanningTimingsMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(toolbox_msgs.msg.dds.FootstepPlanningTimingsMessage src, toolbox_msgs.msg.dds.FootstepPlanningTimingsMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public FootstepPlanningTimingsMessagePubSubType newInstance()
   {
      return new FootstepPlanningTimingsMessagePubSubType();
   }
}
