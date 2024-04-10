package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "KickDoorApproachPlanDefinitionMessage" defined in "KickDoorApproachPlanDefinitionMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from KickDoorApproachPlanDefinitionMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit KickDoorApproachPlanDefinitionMessage_.idl instead.
*
*/
public class KickDoorApproachPlanDefinitionMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.KickDoorApproachPlanDefinitionMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::KickDoorApproachPlanDefinitionMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "6cd05c5ccc35db9123d75e869e60679411dfaf2259ac9cbad039b64e1655ea08";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.KickDoorApproachPlanDefinitionMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.KickDoorApproachPlanDefinitionMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.KickDoorApproachPlanDefinitionMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.KickDoorApproachPlanDefinitionMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType.getCdrSerializedSize(data.getDefinition(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getParentFrameName().length() + 1;

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.KickDoorApproachPlanDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType.write(data.getDefinition(), cdr);
      if(data.getParentFrameName().length() <= 255)
      cdr.write_type_d(data.getParentFrameName());else
          throw new RuntimeException("parent_frame_name field exceeds the maximum length");

      cdr.write_type_6(data.getSwingDuration());

      cdr.write_type_6(data.getTransferDuration());

      cdr.write_type_2(data.getExecutionMode());

      cdr.write_type_9(data.getRobotSide());

      cdr.write_type_6(data.getKickImpulse());

      cdr.write_type_6(data.getKickTargetDistance());

      cdr.write_type_6(data.getHorizontalDistanceFromHandle());

      cdr.write_type_6(data.getStanceFootWidth());

      cdr.write_type_6(data.getPrekickWeightDistribution());

   }

   public static void read(behavior_msgs.msg.dds.KickDoorApproachPlanDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType.read(data.getDefinition(), cdr);	
      cdr.read_type_d(data.getParentFrameName());	
      data.setSwingDuration(cdr.read_type_6());
      	
      data.setTransferDuration(cdr.read_type_6());
      	
      data.setExecutionMode(cdr.read_type_2());
      	
      data.setRobotSide(cdr.read_type_9());
      	
      data.setKickImpulse(cdr.read_type_6());
      	
      data.setKickTargetDistance(cdr.read_type_6());
      	
      data.setHorizontalDistanceFromHandle(cdr.read_type_6());
      	
      data.setStanceFootWidth(cdr.read_type_6());
      	
      data.setPrekickWeightDistribution(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.KickDoorApproachPlanDefinitionMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("definition", new behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType(), data.getDefinition());

      ser.write_type_d("parent_frame_name", data.getParentFrameName());
      ser.write_type_6("swing_duration", data.getSwingDuration());
      ser.write_type_6("transfer_duration", data.getTransferDuration());
      ser.write_type_2("execution_mode", data.getExecutionMode());
      ser.write_type_9("robot_side", data.getRobotSide());
      ser.write_type_6("kick_impulse", data.getKickImpulse());
      ser.write_type_6("kick_target_distance", data.getKickTargetDistance());
      ser.write_type_6("horizontal_distance_from_handle", data.getHorizontalDistanceFromHandle());
      ser.write_type_6("stance_foot_width", data.getStanceFootWidth());
      ser.write_type_6("prekick_weight_distribution", data.getPrekickWeightDistribution());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.KickDoorApproachPlanDefinitionMessage data)
   {
      ser.read_type_a("definition", new behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType(), data.getDefinition());

      ser.read_type_d("parent_frame_name", data.getParentFrameName());
      data.setSwingDuration(ser.read_type_6("swing_duration"));
      data.setTransferDuration(ser.read_type_6("transfer_duration"));
      data.setExecutionMode(ser.read_type_2("execution_mode"));
      data.setRobotSide(ser.read_type_9("robot_side"));
      data.setKickImpulse(ser.read_type_6("kick_impulse"));
      data.setKickTargetDistance(ser.read_type_6("kick_target_distance"));
      data.setHorizontalDistanceFromHandle(ser.read_type_6("horizontal_distance_from_handle"));
      data.setStanceFootWidth(ser.read_type_6("stance_foot_width"));
      data.setPrekickWeightDistribution(ser.read_type_6("prekick_weight_distribution"));
   }

   public static void staticCopy(behavior_msgs.msg.dds.KickDoorApproachPlanDefinitionMessage src, behavior_msgs.msg.dds.KickDoorApproachPlanDefinitionMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.KickDoorApproachPlanDefinitionMessage createData()
   {
      return new behavior_msgs.msg.dds.KickDoorApproachPlanDefinitionMessage();
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
   
   public void serialize(behavior_msgs.msg.dds.KickDoorApproachPlanDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.KickDoorApproachPlanDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.KickDoorApproachPlanDefinitionMessage src, behavior_msgs.msg.dds.KickDoorApproachPlanDefinitionMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public KickDoorApproachPlanDefinitionMessagePubSubType newInstance()
   {
      return new KickDoorApproachPlanDefinitionMessagePubSubType();
   }
}
