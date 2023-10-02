package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "ArmJointAnglesActionDefinitionMessage" defined in "ArmJointAnglesActionDefinitionMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from ArmJointAnglesActionDefinitionMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit ArmJointAnglesActionDefinitionMessage_.idl instead.
*
*/
public class ArmJointAnglesActionDefinitionMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.ArmJointAnglesActionDefinitionMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::ArmJointAnglesActionDefinitionMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "67ae749d285a690687f4ca0a751c81b5f901ebcdede6d2b246129223b4116278";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.ArmJointAnglesActionDefinitionMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.ArmJointAnglesActionDefinitionMessage data) throws java.io.IOException
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

      current_alignment += ((7) * 8) + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.ArmJointAnglesActionDefinitionMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.ArmJointAnglesActionDefinitionMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += behavior_msgs.msg.dds.ActionInformationMessagePubSubType.getCdrSerializedSize(data.getActionInformation(), current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += ((7) * 8) + us.ihmc.idl.CDR.alignment(current_alignment, 8);
      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.ArmJointAnglesActionDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.ActionInformationMessagePubSubType.write(data.getActionInformation(), cdr);
      cdr.write_type_9(data.getRobotSide());

      cdr.write_type_2(data.getPreset());

      for(int i0 = 0; i0 < data.getJointAngles().length; ++i0)
      {
        	cdr.write_type_6(data.getJointAngles()[i0]);	
      }

      cdr.write_type_6(data.getTrajectoryDuration());

   }

   public static void read(behavior_msgs.msg.dds.ArmJointAnglesActionDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.ActionInformationMessagePubSubType.read(data.getActionInformation(), cdr);	
      data.setRobotSide(cdr.read_type_9());
      	
      data.setPreset(cdr.read_type_2());
      	
      for(int i0 = 0; i0 < data.getJointAngles().length; ++i0)
      {
        	data.getJointAngles()[i0] = cdr.read_type_6();
        	
      }
      	
      data.setTrajectoryDuration(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.ArmJointAnglesActionDefinitionMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("action_information", new behavior_msgs.msg.dds.ActionInformationMessagePubSubType(), data.getActionInformation());

      ser.write_type_9("robot_side", data.getRobotSide());
      ser.write_type_2("preset", data.getPreset());
      ser.write_type_f("joint_angles", data.getJointAngles());
      ser.write_type_6("trajectory_duration", data.getTrajectoryDuration());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.ArmJointAnglesActionDefinitionMessage data)
   {
      ser.read_type_a("action_information", new behavior_msgs.msg.dds.ActionInformationMessagePubSubType(), data.getActionInformation());

      data.setRobotSide(ser.read_type_9("robot_side"));
      data.setPreset(ser.read_type_2("preset"));
      ser.read_type_f("joint_angles", data.getJointAngles());
      data.setTrajectoryDuration(ser.read_type_6("trajectory_duration"));
   }

   public static void staticCopy(behavior_msgs.msg.dds.ArmJointAnglesActionDefinitionMessage src, behavior_msgs.msg.dds.ArmJointAnglesActionDefinitionMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.ArmJointAnglesActionDefinitionMessage createData()
   {
      return new behavior_msgs.msg.dds.ArmJointAnglesActionDefinitionMessage();
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
   
   public void serialize(behavior_msgs.msg.dds.ArmJointAnglesActionDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.ArmJointAnglesActionDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.ArmJointAnglesActionDefinitionMessage src, behavior_msgs.msg.dds.ArmJointAnglesActionDefinitionMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public ArmJointAnglesActionDefinitionMessagePubSubType newInstance()
   {
      return new ArmJointAnglesActionDefinitionMessagePubSubType();
   }
}
