package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "SakeHandCommandActionDescriptionMessage" defined in "SakeHandCommandActionDescriptionMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from SakeHandCommandActionDescriptionMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit SakeHandCommandActionDescriptionMessage_.idl instead.
*
*/
public class SakeHandCommandActionDescriptionMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.SakeHandCommandActionDescriptionMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::SakeHandCommandActionDescriptionMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "063d0f2f6272a4abff03966696076ce034b372638a674ecdf4c22b2d6c6b4a2b";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.SakeHandCommandActionDescriptionMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.SakeHandCommandActionDescriptionMessage data) throws java.io.IOException
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

      current_alignment += behavior_msgs.msg.dds.ActionInformationMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.SakeHandCommandActionDescriptionMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.SakeHandCommandActionDescriptionMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += behavior_msgs.msg.dds.ActionInformationMessagePubSubType.getCdrSerializedSize(data.getActionInformation(), current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.SakeHandCommandActionDescriptionMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.ActionInformationMessagePubSubType.write(data.getActionInformation(), cdr);
      cdr.write_type_9(data.getRobotSide());

      cdr.write_type_4(data.getConfiguration());

      cdr.write_type_6(data.getPositionRatio());

      cdr.write_type_6(data.getTorqueRatio());

      cdr.write_type_7(data.getExecuteWithNextAction());

   }

   public static void read(behavior_msgs.msg.dds.SakeHandCommandActionDescriptionMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.ActionInformationMessagePubSubType.read(data.getActionInformation(), cdr);	
      data.setRobotSide(cdr.read_type_9());
      	
      data.setConfiguration(cdr.read_type_4());
      	
      data.setPositionRatio(cdr.read_type_6());
      	
      data.setTorqueRatio(cdr.read_type_6());
      	
      data.setExecuteWithNextAction(cdr.read_type_7());
      	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.SakeHandCommandActionDescriptionMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("action_information", new behavior_msgs.msg.dds.ActionInformationMessagePubSubType(), data.getActionInformation());

      ser.write_type_9("robot_side", data.getRobotSide());
      ser.write_type_4("configuration", data.getConfiguration());
      ser.write_type_6("position_ratio", data.getPositionRatio());
      ser.write_type_6("torque_ratio", data.getTorqueRatio());
      ser.write_type_7("execute_with_next_action", data.getExecuteWithNextAction());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.SakeHandCommandActionDescriptionMessage data)
   {
      ser.read_type_a("action_information", new behavior_msgs.msg.dds.ActionInformationMessagePubSubType(), data.getActionInformation());

      data.setRobotSide(ser.read_type_9("robot_side"));
      data.setConfiguration(ser.read_type_4("configuration"));
      data.setPositionRatio(ser.read_type_6("position_ratio"));
      data.setTorqueRatio(ser.read_type_6("torque_ratio"));
      data.setExecuteWithNextAction(ser.read_type_7("execute_with_next_action"));
   }

   public static void staticCopy(behavior_msgs.msg.dds.SakeHandCommandActionDescriptionMessage src, behavior_msgs.msg.dds.SakeHandCommandActionDescriptionMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.SakeHandCommandActionDescriptionMessage createData()
   {
      return new behavior_msgs.msg.dds.SakeHandCommandActionDescriptionMessage();
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
   
   public void serialize(behavior_msgs.msg.dds.SakeHandCommandActionDescriptionMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.SakeHandCommandActionDescriptionMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.SakeHandCommandActionDescriptionMessage src, behavior_msgs.msg.dds.SakeHandCommandActionDescriptionMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public SakeHandCommandActionDescriptionMessagePubSubType newInstance()
   {
      return new SakeHandCommandActionDescriptionMessagePubSubType();
   }
}
