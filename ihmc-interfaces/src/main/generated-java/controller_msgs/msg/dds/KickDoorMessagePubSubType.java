package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "KickDoorMessage" defined in "KickDoorMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from KickDoorMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit KickDoorMessage_.idl instead.
*
*/
public class KickDoorMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.KickDoorMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::KickDoorMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "b1357ce9e113f26ceb2843a82d4f242f1fc1bb1d286d3b3469528444c5669609";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.KickDoorMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.KickDoorMessage data) throws java.io.IOException
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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.KickDoorMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.KickDoorMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.KickDoorMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_9(data.getRobotSide());

      cdr.write_type_6(data.getKickHeight());

      cdr.write_type_6(data.getKickImpulse());

      cdr.write_type_6(data.getKickTargetDistance());

      cdr.write_type_7(data.getTriggerKickRequest());

      cdr.write_type_6(data.getPrekickWeightDistribution());

   }

   public static void read(controller_msgs.msg.dds.KickDoorMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setRobotSide(cdr.read_type_9());
      	
      data.setKickHeight(cdr.read_type_6());
      	
      data.setKickImpulse(cdr.read_type_6());
      	
      data.setKickTargetDistance(cdr.read_type_6());
      	
      data.setTriggerKickRequest(cdr.read_type_7());
      	
      data.setPrekickWeightDistribution(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.KickDoorMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_9("robot_side", data.getRobotSide());
      ser.write_type_6("kick_height", data.getKickHeight());
      ser.write_type_6("kick_impulse", data.getKickImpulse());
      ser.write_type_6("kick_target_distance", data.getKickTargetDistance());
      ser.write_type_7("trigger_kick_request", data.getTriggerKickRequest());
      ser.write_type_6("prekick_weight_distribution", data.getPrekickWeightDistribution());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.KickDoorMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setRobotSide(ser.read_type_9("robot_side"));
      data.setKickHeight(ser.read_type_6("kick_height"));
      data.setKickImpulse(ser.read_type_6("kick_impulse"));
      data.setKickTargetDistance(ser.read_type_6("kick_target_distance"));
      data.setTriggerKickRequest(ser.read_type_7("trigger_kick_request"));
      data.setPrekickWeightDistribution(ser.read_type_6("prekick_weight_distribution"));
   }

   public static void staticCopy(controller_msgs.msg.dds.KickDoorMessage src, controller_msgs.msg.dds.KickDoorMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.KickDoorMessage createData()
   {
      return new controller_msgs.msg.dds.KickDoorMessage();
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
   
   public void serialize(controller_msgs.msg.dds.KickDoorMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.KickDoorMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.KickDoorMessage src, controller_msgs.msg.dds.KickDoorMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public KickDoorMessagePubSubType newInstance()
   {
      return new KickDoorMessagePubSubType();
   }
}
