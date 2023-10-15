package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "SakeHandCommandActionDefinitionMessage" defined in "SakeHandCommandActionDefinitionMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from SakeHandCommandActionDefinitionMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit SakeHandCommandActionDefinitionMessage_.idl instead.
*
*/
public class SakeHandCommandActionDefinitionMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.SakeHandCommandActionDefinitionMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::SakeHandCommandActionDefinitionMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "40d5d7d6997216fb6180ab0fd61f8eb48edd353ba30bae91af0f856c3a8e5e0b";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.SakeHandCommandActionDefinitionMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.SakeHandCommandActionDefinitionMessage data) throws java.io.IOException
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

      current_alignment += behavior_msgs.msg.dds.BehaviorActionDefinitionMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.SakeHandCommandActionDefinitionMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.SakeHandCommandActionDefinitionMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += behavior_msgs.msg.dds.BehaviorActionDefinitionMessagePubSubType.getCdrSerializedSize(data.getActionDefinition(), current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.SakeHandCommandActionDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.BehaviorActionDefinitionMessagePubSubType.write(data.getActionDefinition(), cdr);
      cdr.write_type_9(data.getRobotSide());

      cdr.write_type_4(data.getConfiguration());

      cdr.write_type_6(data.getPositionRatio());

      cdr.write_type_6(data.getTorqueRatio());

      cdr.write_type_7(data.getExecuteWithNextAction());

   }

   public static void read(behavior_msgs.msg.dds.SakeHandCommandActionDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.BehaviorActionDefinitionMessagePubSubType.read(data.getActionDefinition(), cdr);	
      data.setRobotSide(cdr.read_type_9());
      	
      data.setConfiguration(cdr.read_type_4());
      	
      data.setPositionRatio(cdr.read_type_6());
      	
      data.setTorqueRatio(cdr.read_type_6());
      	
      data.setExecuteWithNextAction(cdr.read_type_7());
      	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.SakeHandCommandActionDefinitionMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("action_definition", new behavior_msgs.msg.dds.BehaviorActionDefinitionMessagePubSubType(), data.getActionDefinition());

      ser.write_type_9("robot_side", data.getRobotSide());
      ser.write_type_4("configuration", data.getConfiguration());
      ser.write_type_6("position_ratio", data.getPositionRatio());
      ser.write_type_6("torque_ratio", data.getTorqueRatio());
      ser.write_type_7("execute_with_next_action", data.getExecuteWithNextAction());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.SakeHandCommandActionDefinitionMessage data)
   {
      ser.read_type_a("action_definition", new behavior_msgs.msg.dds.BehaviorActionDefinitionMessagePubSubType(), data.getActionDefinition());

      data.setRobotSide(ser.read_type_9("robot_side"));
      data.setConfiguration(ser.read_type_4("configuration"));
      data.setPositionRatio(ser.read_type_6("position_ratio"));
      data.setTorqueRatio(ser.read_type_6("torque_ratio"));
      data.setExecuteWithNextAction(ser.read_type_7("execute_with_next_action"));
   }

   public static void staticCopy(behavior_msgs.msg.dds.SakeHandCommandActionDefinitionMessage src, behavior_msgs.msg.dds.SakeHandCommandActionDefinitionMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.SakeHandCommandActionDefinitionMessage createData()
   {
      return new behavior_msgs.msg.dds.SakeHandCommandActionDefinitionMessage();
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
   
   public void serialize(behavior_msgs.msg.dds.SakeHandCommandActionDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.SakeHandCommandActionDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.SakeHandCommandActionDefinitionMessage src, behavior_msgs.msg.dds.SakeHandCommandActionDefinitionMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public SakeHandCommandActionDefinitionMessagePubSubType newInstance()
   {
      return new SakeHandCommandActionDefinitionMessagePubSubType();
   }
}
