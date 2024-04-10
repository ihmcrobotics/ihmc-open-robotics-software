package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "KickDoorActionDefinitionMessage" defined in "KickDoorActionDefinitionMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from KickDoorActionDefinitionMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit KickDoorActionDefinitionMessage_.idl instead.
*
*/
public class KickDoorActionDefinitionMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.KickDoorActionDefinitionMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::KickDoorActionDefinitionMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "366578c3b0a680658f3e37f139cf007285c9fe8e78e6de2c5c5c9107e5f3dbf7";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.KickDoorActionDefinitionMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.KickDoorActionDefinitionMessage data) throws java.io.IOException
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
      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.KickDoorActionDefinitionMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.KickDoorActionDefinitionMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType.getCdrSerializedSize(data.getDefinition(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getParentFrameName().length() + 1;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.KickDoorActionDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType.write(data.getDefinition(), cdr);
      if(data.getParentFrameName().length() <= 255)
      cdr.write_type_d(data.getParentFrameName());else
          throw new RuntimeException("parent_frame_name field exceeds the maximum length");

      cdr.write_type_9(data.getRobotSide());

      cdr.write_type_6(data.getKickHeight());

      cdr.write_type_6(data.getKickImpulse());

      cdr.write_type_6(data.getKickTargetDistance());

      cdr.write_type_6(data.getPrekickWeightDistribution());

   }

   public static void read(behavior_msgs.msg.dds.KickDoorActionDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType.read(data.getDefinition(), cdr);	
      cdr.read_type_d(data.getParentFrameName());	
      data.setRobotSide(cdr.read_type_9());
      	
      data.setKickHeight(cdr.read_type_6());
      	
      data.setKickImpulse(cdr.read_type_6());
      	
      data.setKickTargetDistance(cdr.read_type_6());
      	
      data.setPrekickWeightDistribution(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.KickDoorActionDefinitionMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("definition", new behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType(), data.getDefinition());

      ser.write_type_d("parent_frame_name", data.getParentFrameName());
      ser.write_type_9("robot_side", data.getRobotSide());
      ser.write_type_6("kick_height", data.getKickHeight());
      ser.write_type_6("kick_impulse", data.getKickImpulse());
      ser.write_type_6("kick_target_distance", data.getKickTargetDistance());
      ser.write_type_6("prekick_weight_distribution", data.getPrekickWeightDistribution());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.KickDoorActionDefinitionMessage data)
   {
      ser.read_type_a("definition", new behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType(), data.getDefinition());

      ser.read_type_d("parent_frame_name", data.getParentFrameName());
      data.setRobotSide(ser.read_type_9("robot_side"));
      data.setKickHeight(ser.read_type_6("kick_height"));
      data.setKickImpulse(ser.read_type_6("kick_impulse"));
      data.setKickTargetDistance(ser.read_type_6("kick_target_distance"));
      data.setPrekickWeightDistribution(ser.read_type_6("prekick_weight_distribution"));
   }

   public static void staticCopy(behavior_msgs.msg.dds.KickDoorActionDefinitionMessage src, behavior_msgs.msg.dds.KickDoorActionDefinitionMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.KickDoorActionDefinitionMessage createData()
   {
      return new behavior_msgs.msg.dds.KickDoorActionDefinitionMessage();
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
   
   public void serialize(behavior_msgs.msg.dds.KickDoorActionDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.KickDoorActionDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.KickDoorActionDefinitionMessage src, behavior_msgs.msg.dds.KickDoorActionDefinitionMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public KickDoorActionDefinitionMessagePubSubType newInstance()
   {
      return new KickDoorActionDefinitionMessagePubSubType();
   }
}
