package valkyrie_msgs.msg.dds;

/**
* 
* Topic data type of the struct "AthenaTrajectoryMessage" defined in "AthenaTrajectoryMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from AthenaTrajectoryMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit AthenaTrajectoryMessage_.idl instead.
*
*/
public class AthenaTrajectoryMessagePubSubType implements us.ihmc.pubsub.TopicDataType<valkyrie_msgs.msg.dds.AthenaTrajectoryMessage>
{
   public static final java.lang.String name = "valkyrie_msgs::msg::dds_::AthenaTrajectoryMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "0870d73c2699467d04b4755bc4beb86348a91567e8a71fe06c587096a774c2e0";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(valkyrie_msgs.msg.dds.AthenaTrajectoryMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, valkyrie_msgs.msg.dds.AthenaTrajectoryMessage data) throws java.io.IOException
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

   public final static int getCdrSerializedSize(valkyrie_msgs.msg.dds.AthenaTrajectoryMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(valkyrie_msgs.msg.dds.AthenaTrajectoryMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getFingerMotorNames().size() * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += controller_msgs.msg.dds.JointspaceTrajectoryMessagePubSubType.getCdrSerializedSize(data.getJointspaceTrajectory(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(valkyrie_msgs.msg.dds.AthenaTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_9(data.getRobotSide());

      if(data.getFingerMotorNames().size() <= 6)
      cdr.write_type_e(data.getFingerMotorNames());else
          throw new RuntimeException("finger_motor_names field exceeds the maximum length");

      controller_msgs.msg.dds.JointspaceTrajectoryMessagePubSubType.write(data.getJointspaceTrajectory(), cdr);
   }

   public static void read(valkyrie_msgs.msg.dds.AthenaTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setRobotSide(cdr.read_type_9());
      	
      cdr.read_type_e(data.getFingerMotorNames());	
      controller_msgs.msg.dds.JointspaceTrajectoryMessagePubSubType.read(data.getJointspaceTrajectory(), cdr);	

   }

   @Override
   public final void serialize(valkyrie_msgs.msg.dds.AthenaTrajectoryMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_9("robot_side", data.getRobotSide());
      ser.write_type_e("finger_motor_names", data.getFingerMotorNames());
      ser.write_type_a("jointspace_trajectory", new controller_msgs.msg.dds.JointspaceTrajectoryMessagePubSubType(), data.getJointspaceTrajectory());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, valkyrie_msgs.msg.dds.AthenaTrajectoryMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setRobotSide(ser.read_type_9("robot_side"));
      ser.read_type_e("finger_motor_names", data.getFingerMotorNames());
      ser.read_type_a("jointspace_trajectory", new controller_msgs.msg.dds.JointspaceTrajectoryMessagePubSubType(), data.getJointspaceTrajectory());

   }

   public static void staticCopy(valkyrie_msgs.msg.dds.AthenaTrajectoryMessage src, valkyrie_msgs.msg.dds.AthenaTrajectoryMessage dest)
   {
      dest.set(src);
   }

   @Override
   public valkyrie_msgs.msg.dds.AthenaTrajectoryMessage createData()
   {
      return new valkyrie_msgs.msg.dds.AthenaTrajectoryMessage();
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
   
   public void serialize(valkyrie_msgs.msg.dds.AthenaTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(valkyrie_msgs.msg.dds.AthenaTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(valkyrie_msgs.msg.dds.AthenaTrajectoryMessage src, valkyrie_msgs.msg.dds.AthenaTrajectoryMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public AthenaTrajectoryMessagePubSubType newInstance()
   {
      return new AthenaTrajectoryMessagePubSubType();
   }
}
