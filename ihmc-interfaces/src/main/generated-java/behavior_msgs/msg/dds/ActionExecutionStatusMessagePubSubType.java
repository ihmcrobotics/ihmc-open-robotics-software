package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "ActionExecutionStatusMessage" defined in "ActionExecutionStatusMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from ActionExecutionStatusMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit ActionExecutionStatusMessage_.idl instead.
*
*/
public class ActionExecutionStatusMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.ActionExecutionStatusMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::ActionExecutionStatusMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "8cd233c86781f79a8a9e841d1e6a36c72bfc906d611745593b7b4867ce039a49";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.ActionExecutionStatusMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.ActionExecutionStatusMessage data) throws java.io.IOException
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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);

      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.ActionExecutionStatusMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.ActionExecutionStatusMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);


      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.ActionExecutionStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_2(data.getActionIndex());

      cdr.write_type_6(data.getNominalExecutionDuration());

      cdr.write_type_6(data.getElapsedExecutionTime());

      cdr.write_type_3(data.getTotalNumberOfFootsteps());

      cdr.write_type_3(data.getNumberOfIncompleteFootsteps());

      cdr.write_type_6(data.getTranslationCurrentDistanceToGoal());

      cdr.write_type_6(data.getTranslationStartDistanceToGoal());

      cdr.write_type_6(data.getOrientationCurrentDistanceToGoal());

      cdr.write_type_6(data.getOrientationStartDistanceToGoal());

   }

   public static void read(behavior_msgs.msg.dds.ActionExecutionStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setActionIndex(cdr.read_type_2());
      	
      data.setNominalExecutionDuration(cdr.read_type_6());
      	
      data.setElapsedExecutionTime(cdr.read_type_6());
      	
      data.setTotalNumberOfFootsteps(cdr.read_type_3());
      	
      data.setNumberOfIncompleteFootsteps(cdr.read_type_3());
      	
      data.setTranslationCurrentDistanceToGoal(cdr.read_type_6());
      	
      data.setTranslationStartDistanceToGoal(cdr.read_type_6());
      	
      data.setOrientationCurrentDistanceToGoal(cdr.read_type_6());
      	
      data.setOrientationStartDistanceToGoal(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.ActionExecutionStatusMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_2("action_index", data.getActionIndex());
      ser.write_type_6("nominal_execution_duration", data.getNominalExecutionDuration());
      ser.write_type_6("elapsed_execution_time", data.getElapsedExecutionTime());
      ser.write_type_3("total_number_of_footsteps", data.getTotalNumberOfFootsteps());
      ser.write_type_3("number_of_incomplete_footsteps", data.getNumberOfIncompleteFootsteps());
      ser.write_type_6("translation_current_distance_to_goal", data.getTranslationCurrentDistanceToGoal());
      ser.write_type_6("translation_start_distance_to_goal", data.getTranslationStartDistanceToGoal());
      ser.write_type_6("orientation_current_distance_to_goal", data.getOrientationCurrentDistanceToGoal());
      ser.write_type_6("orientation_start_distance_to_goal", data.getOrientationStartDistanceToGoal());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.ActionExecutionStatusMessage data)
   {
      data.setActionIndex(ser.read_type_2("action_index"));
      data.setNominalExecutionDuration(ser.read_type_6("nominal_execution_duration"));
      data.setElapsedExecutionTime(ser.read_type_6("elapsed_execution_time"));
      data.setTotalNumberOfFootsteps(ser.read_type_3("total_number_of_footsteps"));
      data.setNumberOfIncompleteFootsteps(ser.read_type_3("number_of_incomplete_footsteps"));
      data.setTranslationCurrentDistanceToGoal(ser.read_type_6("translation_current_distance_to_goal"));
      data.setTranslationStartDistanceToGoal(ser.read_type_6("translation_start_distance_to_goal"));
      data.setOrientationCurrentDistanceToGoal(ser.read_type_6("orientation_current_distance_to_goal"));
      data.setOrientationStartDistanceToGoal(ser.read_type_6("orientation_start_distance_to_goal"));
   }

   public static void staticCopy(behavior_msgs.msg.dds.ActionExecutionStatusMessage src, behavior_msgs.msg.dds.ActionExecutionStatusMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.ActionExecutionStatusMessage createData()
   {
      return new behavior_msgs.msg.dds.ActionExecutionStatusMessage();
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
   
   public void serialize(behavior_msgs.msg.dds.ActionExecutionStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.ActionExecutionStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.ActionExecutionStatusMessage src, behavior_msgs.msg.dds.ActionExecutionStatusMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public ActionExecutionStatusMessagePubSubType newInstance()
   {
      return new ActionExecutionStatusMessagePubSubType();
   }
}
