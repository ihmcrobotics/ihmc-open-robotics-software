package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "PsyonicAbilityHandCommandActionDefinitionMessage" defined in "PsyonicAbilityHandCommandActionDefinitionMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from PsyonicAbilityHandCommandActionDefinitionMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit PsyonicAbilityHandCommandActionDefinitionMessage_.idl instead.
*
*/
public class PsyonicAbilityHandCommandActionDefinitionMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.PsyonicAbilityHandCommandActionDefinitionMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::PsyonicAbilityHandCommandActionDefinitionMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "af75b4b21b9f4d63ffff1f561df972442e157ff33e712c37b22afa16fc7362aa";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.PsyonicAbilityHandCommandActionDefinitionMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.PsyonicAbilityHandCommandActionDefinitionMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.PsyonicAbilityHandCommandActionDefinitionMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.PsyonicAbilityHandCommandActionDefinitionMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType.getCdrSerializedSize(data.getDefinition(), current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getGripType().length() + 1;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getGripSpeed().length() + 1;


      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.PsyonicAbilityHandCommandActionDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType.write(data.getDefinition(), cdr);
      cdr.write_type_9(data.getRobotSide());

      if(data.getGripType().length() <= 255)
      cdr.write_type_d(data.getGripType());else
          throw new RuntimeException("grip_type field exceeds the maximum length");

      if(data.getGripSpeed().length() <= 255)
      cdr.write_type_d(data.getGripSpeed());else
          throw new RuntimeException("grip_speed field exceeds the maximum length");

   }

   public static void read(behavior_msgs.msg.dds.PsyonicAbilityHandCommandActionDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType.read(data.getDefinition(), cdr);	
      data.setRobotSide(cdr.read_type_9());
      	
      cdr.read_type_d(data.getGripType());	
      cdr.read_type_d(data.getGripSpeed());	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.PsyonicAbilityHandCommandActionDefinitionMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("definition", new behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType(), data.getDefinition());

      ser.write_type_9("robot_side", data.getRobotSide());
      ser.write_type_d("grip_type", data.getGripType());
      ser.write_type_d("grip_speed", data.getGripSpeed());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.PsyonicAbilityHandCommandActionDefinitionMessage data)
   {
      ser.read_type_a("definition", new behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType(), data.getDefinition());

      data.setRobotSide(ser.read_type_9("robot_side"));
      ser.read_type_d("grip_type", data.getGripType());
      ser.read_type_d("grip_speed", data.getGripSpeed());
   }

   public static void staticCopy(behavior_msgs.msg.dds.PsyonicAbilityHandCommandActionDefinitionMessage src, behavior_msgs.msg.dds.PsyonicAbilityHandCommandActionDefinitionMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.PsyonicAbilityHandCommandActionDefinitionMessage createData()
   {
      return new behavior_msgs.msg.dds.PsyonicAbilityHandCommandActionDefinitionMessage();
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
   
   public void serialize(behavior_msgs.msg.dds.PsyonicAbilityHandCommandActionDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.PsyonicAbilityHandCommandActionDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.PsyonicAbilityHandCommandActionDefinitionMessage src, behavior_msgs.msg.dds.PsyonicAbilityHandCommandActionDefinitionMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public PsyonicAbilityHandCommandActionDefinitionMessagePubSubType newInstance()
   {
      return new PsyonicAbilityHandCommandActionDefinitionMessagePubSubType();
   }
}
