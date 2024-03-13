package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "HandWrenchActionDefinitionMessage" defined in "HandWrenchActionDefinitionMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from HandWrenchActionDefinitionMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit HandWrenchActionDefinitionMessage_.idl instead.
*
*/
public class HandWrenchActionDefinitionMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.HandWrenchActionDefinitionMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::HandWrenchActionDefinitionMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "5f085b71da58425596e95035e0aaf687c1e856ee21b246ec2dbaa652483174a4";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.HandWrenchActionDefinitionMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.HandWrenchActionDefinitionMessage data) throws java.io.IOException
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
      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.HandWrenchActionDefinitionMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.HandWrenchActionDefinitionMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType.getCdrSerializedSize(data.getDefinition(), current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getParentFrameName().length() + 1;

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.HandWrenchActionDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType.write(data.getDefinition(), cdr);
      cdr.write_type_9(data.getRobotSide());

      if(data.getParentFrameName().length() <= 255)
      cdr.write_type_d(data.getParentFrameName());else
          throw new RuntimeException("parent_frame_name field exceeds the maximum length");

      cdr.write_type_6(data.getTrajectoryDuration());

      cdr.write_type_6(data.getForceX());

      cdr.write_type_6(data.getForceY());

      cdr.write_type_6(data.getForceZ());

      cdr.write_type_6(data.getTorqueX());

      cdr.write_type_6(data.getTorqueY());

      cdr.write_type_6(data.getTorqueZ());

   }

   public static void read(behavior_msgs.msg.dds.HandWrenchActionDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType.read(data.getDefinition(), cdr);	
      data.setRobotSide(cdr.read_type_9());
      	
      cdr.read_type_d(data.getParentFrameName());
      data.setTrajectoryDuration(cdr.read_type_6());
      	
      data.setForceX(cdr.read_type_6());

      data.setForceY(cdr.read_type_6());

      data.setForceZ(cdr.read_type_6());

      data.setTorqueX(cdr.read_type_6());

      data.setTorqueY(cdr.read_type_6());

      data.setTorqueZ(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.HandWrenchActionDefinitionMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("definition", new behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType(), data.getDefinition());

      ser.write_type_9("robot_side", data.getRobotSide());
      ser.write_type_d("parent_frame_name", data.getParentFrameName());
      ser.write_type_6("trajectory_duration", data.getTrajectoryDuration());
      ser.write_type_6("force_x", data.getForceX());
      ser.write_type_6("force_y", data.getForceY());
      ser.write_type_6("force_z", data.getForceZ());
      ser.write_type_6("torque_x", data.getTorqueX());
      ser.write_type_6("torque_y", data.getTorqueY());
      ser.write_type_6("torque_z", data.getTorqueZ());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.HandWrenchActionDefinitionMessage data)
   {
      ser.read_type_a("definition", new behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType(), data.getDefinition());

      data.setRobotSide(ser.read_type_9("robot_side"));
      ser.read_type_d("parent_frame_name", data.getParentFrameName());
      data.setTrajectoryDuration(ser.read_type_6("trajectory_duration"));
      data.setForceX(ser.read_type_6("force_x"));
      data.setForceY(ser.read_type_6("force_y"));
      data.setForceZ(ser.read_type_6("force_z"));
      data.setTorqueX(ser.read_type_6("torque_x"));
      data.setTorqueY(ser.read_type_6("torque_y"));
      data.setTorqueZ(ser.read_type_6("torque_z"));
   }

   public static void staticCopy(behavior_msgs.msg.dds.HandWrenchActionDefinitionMessage src, behavior_msgs.msg.dds.HandWrenchActionDefinitionMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.HandWrenchActionDefinitionMessage createData()
   {
      return new behavior_msgs.msg.dds.HandWrenchActionDefinitionMessage();
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
   
   public void serialize(behavior_msgs.msg.dds.HandWrenchActionDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.HandWrenchActionDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.HandWrenchActionDefinitionMessage src, behavior_msgs.msg.dds.HandWrenchActionDefinitionMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public HandWrenchActionDefinitionMessagePubSubType newInstance()
   {
      return new HandWrenchActionDefinitionMessagePubSubType();
   }
}
