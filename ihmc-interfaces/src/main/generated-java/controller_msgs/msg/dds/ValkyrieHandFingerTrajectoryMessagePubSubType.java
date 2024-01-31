package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "ValkyrieHandFingerTrajectoryMessage" defined in "ValkyrieHandFingerTrajectoryMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from ValkyrieHandFingerTrajectoryMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit ValkyrieHandFingerTrajectoryMessage_.idl instead.
*
*/
public class ValkyrieHandFingerTrajectoryMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.ValkyrieHandFingerTrajectoryMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::ValkyrieHandFingerTrajectoryMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "27f2868a7ecbaeae24b343c80df6296edb2deed67fea53eed4311aa73ce7ee1a";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.ValkyrieHandFingerTrajectoryMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.ValkyrieHandFingerTrajectoryMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (6 * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += controller_msgs.msg.dds.JointspaceTrajectoryMessagePubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.ValkyrieHandFingerTrajectoryMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.ValkyrieHandFingerTrajectoryMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getValkyrieFingerMotorNames().size() * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += controller_msgs.msg.dds.JointspaceTrajectoryMessagePubSubType.getCdrSerializedSize(data.getJointspaceTrajectory(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.ValkyrieHandFingerTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_9(data.getRobotSide());

      if(data.getValkyrieFingerMotorNames().size() <= 6)
      cdr.write_type_e(data.getValkyrieFingerMotorNames());else
          throw new RuntimeException("valkyrie_finger_motor_names field exceeds the maximum length");

      controller_msgs.msg.dds.JointspaceTrajectoryMessagePubSubType.write(data.getJointspaceTrajectory(), cdr);
   }

   public static void read(controller_msgs.msg.dds.ValkyrieHandFingerTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setRobotSide(cdr.read_type_9());
      	
      cdr.read_type_e(data.getValkyrieFingerMotorNames());	
      controller_msgs.msg.dds.JointspaceTrajectoryMessagePubSubType.read(data.getJointspaceTrajectory(), cdr);	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.ValkyrieHandFingerTrajectoryMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_9("robot_side", data.getRobotSide());
      ser.write_type_e("valkyrie_finger_motor_names", data.getValkyrieFingerMotorNames());
      ser.write_type_a("jointspace_trajectory", new controller_msgs.msg.dds.JointspaceTrajectoryMessagePubSubType(), data.getJointspaceTrajectory());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.ValkyrieHandFingerTrajectoryMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setRobotSide(ser.read_type_9("robot_side"));
      ser.read_type_e("valkyrie_finger_motor_names", data.getValkyrieFingerMotorNames());
      ser.read_type_a("jointspace_trajectory", new controller_msgs.msg.dds.JointspaceTrajectoryMessagePubSubType(), data.getJointspaceTrajectory());

   }

   public static void staticCopy(controller_msgs.msg.dds.ValkyrieHandFingerTrajectoryMessage src, controller_msgs.msg.dds.ValkyrieHandFingerTrajectoryMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.ValkyrieHandFingerTrajectoryMessage createData()
   {
      return new controller_msgs.msg.dds.ValkyrieHandFingerTrajectoryMessage();
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
   
   public void serialize(controller_msgs.msg.dds.ValkyrieHandFingerTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.ValkyrieHandFingerTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.ValkyrieHandFingerTrajectoryMessage src, controller_msgs.msg.dds.ValkyrieHandFingerTrajectoryMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public ValkyrieHandFingerTrajectoryMessagePubSubType newInstance()
   {
      return new ValkyrieHandFingerTrajectoryMessagePubSubType();
   }
}
