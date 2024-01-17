package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "ContinuousWalkingStatusMessage" defined in "ContinuousWalkingStatusMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from ContinuousWalkingStatusMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit ContinuousWalkingStatusMessage_.idl instead.
*
*/
public class ContinuousWalkingStatusMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.ContinuousWalkingStatusMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::ContinuousWalkingStatusMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "f05891e202312a90bd2e05c60ac1ddc59760072ba6c01beb2409d38a4feb6af6";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.ContinuousWalkingStatusMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.ContinuousWalkingStatusMessage data) throws java.io.IOException
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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);

      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.ContinuousWalkingStatusMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.ContinuousWalkingStatusMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);


      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);



      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.ContinuousWalkingStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_7(data.getLeftStanceToPlanFrom());

      cdr.write_type_6(data.getMonteCarloPlanningTime());

      cdr.write_type_6(data.getAStarPlanningTime());

      cdr.write_type_3(data.getAStarNumSteps());

      cdr.write_type_3(data.getMonteCarloNumSteps());

   }

   public static void read(behavior_msgs.msg.dds.ContinuousWalkingStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setLeftStanceToPlanFrom(cdr.read_type_7());
      	
      data.setMonteCarloPlanningTime(cdr.read_type_6());
      	
      data.setAStarPlanningTime(cdr.read_type_6());
      	
      data.setAStarNumSteps(cdr.read_type_3());
      	
      data.setMonteCarloNumSteps(cdr.read_type_3());
      	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.ContinuousWalkingStatusMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_7("left_stance_to_plan_from", data.getLeftStanceToPlanFrom());
      ser.write_type_6("monte_carlo_planning_time", data.getMonteCarloPlanningTime());
      ser.write_type_6("a_star_planning_time", data.getAStarPlanningTime());
      ser.write_type_3("a_star_num_steps", data.getAStarNumSteps());
      ser.write_type_3("monte_carlo_num_steps", data.getMonteCarloNumSteps());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.ContinuousWalkingStatusMessage data)
   {
      data.setLeftStanceToPlanFrom(ser.read_type_7("left_stance_to_plan_from"));
      data.setMonteCarloPlanningTime(ser.read_type_6("monte_carlo_planning_time"));
      data.setAStarPlanningTime(ser.read_type_6("a_star_planning_time"));
      data.setAStarNumSteps(ser.read_type_3("a_star_num_steps"));
      data.setMonteCarloNumSteps(ser.read_type_3("monte_carlo_num_steps"));
   }

   public static void staticCopy(behavior_msgs.msg.dds.ContinuousWalkingStatusMessage src, behavior_msgs.msg.dds.ContinuousWalkingStatusMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.ContinuousWalkingStatusMessage createData()
   {
      return new behavior_msgs.msg.dds.ContinuousWalkingStatusMessage();
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
   
   public void serialize(behavior_msgs.msg.dds.ContinuousWalkingStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.ContinuousWalkingStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.ContinuousWalkingStatusMessage src, behavior_msgs.msg.dds.ContinuousWalkingStatusMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public ContinuousWalkingStatusMessagePubSubType newInstance()
   {
      return new ContinuousWalkingStatusMessagePubSubType();
   }
}
