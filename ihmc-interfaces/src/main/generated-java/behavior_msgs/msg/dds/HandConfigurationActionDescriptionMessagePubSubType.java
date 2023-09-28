package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "HandConfigurationActionDescriptionMessage" defined in "HandConfigurationActionDescriptionMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from HandConfigurationActionDescriptionMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit HandConfigurationActionDescriptionMessage_.idl instead.
*
*/
public class HandConfigurationActionDescriptionMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.HandConfigurationActionDescriptionMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::HandConfigurationActionDescriptionMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "276d11d82a2163cd2d3516c949b6c759f727a2d5d87107440da4e1cdbde6a6ef";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.HandConfigurationActionDescriptionMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.HandConfigurationActionDescriptionMessage data) throws java.io.IOException
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

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.HandConfigurationActionDescriptionMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.HandConfigurationActionDescriptionMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += behavior_msgs.msg.dds.ActionInformationMessagePubSubType.getCdrSerializedSize(data.getActionInformation(), current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.HandConfigurationActionDescriptionMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.ActionInformationMessagePubSubType.write(data.getActionInformation(), cdr);
      cdr.write_type_9(data.getRobotSide());

      cdr.write_type_4(data.getGrip());

      cdr.write_type_7(data.getExecuteWithNextAction());

   }

   public static void read(behavior_msgs.msg.dds.HandConfigurationActionDescriptionMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.ActionInformationMessagePubSubType.read(data.getActionInformation(), cdr);	
      data.setRobotSide(cdr.read_type_9());
      	
      data.setGrip(cdr.read_type_4());
      	
      data.setExecuteWithNextAction(cdr.read_type_7());
      	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.HandConfigurationActionDescriptionMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("action_information", new behavior_msgs.msg.dds.ActionInformationMessagePubSubType(), data.getActionInformation());

      ser.write_type_9("robot_side", data.getRobotSide());
      ser.write_type_4("grip", data.getGrip());
      ser.write_type_7("execute_with_next_action", data.getExecuteWithNextAction());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.HandConfigurationActionDescriptionMessage data)
   {
      ser.read_type_a("action_information", new behavior_msgs.msg.dds.ActionInformationMessagePubSubType(), data.getActionInformation());

      data.setRobotSide(ser.read_type_9("robot_side"));
      data.setGrip(ser.read_type_4("grip"));
      data.setExecuteWithNextAction(ser.read_type_7("execute_with_next_action"));
   }

   public static void staticCopy(behavior_msgs.msg.dds.HandConfigurationActionDescriptionMessage src, behavior_msgs.msg.dds.HandConfigurationActionDescriptionMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.HandConfigurationActionDescriptionMessage createData()
   {
      return new behavior_msgs.msg.dds.HandConfigurationActionDescriptionMessage();
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
   
   public void serialize(behavior_msgs.msg.dds.HandConfigurationActionDescriptionMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.HandConfigurationActionDescriptionMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.HandConfigurationActionDescriptionMessage src, behavior_msgs.msg.dds.HandConfigurationActionDescriptionMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public HandConfigurationActionDescriptionMessagePubSubType newInstance()
   {
      return new HandConfigurationActionDescriptionMessagePubSubType();
   }
}
