package valkyrie_msgs.msg.dds;

/**
* 
* Topic data type of the struct "PsyonicTrajectoryMessage" defined in "PsyonicTrajectoryMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from PsyonicTrajectoryMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit PsyonicTrajectoryMessage_.idl instead.
*
*/
public class PsyonicTrajectoryMessagePubSubType implements us.ihmc.pubsub.TopicDataType<valkyrie_msgs.msg.dds.PsyonicTrajectoryMessage>
{
   public static final java.lang.String name = "valkyrie_msgs::msg::dds_::PsyonicTrajectoryMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "e280d9786ad97769d83a4f22612810f5317526d0de937a230859e4a9c469ad0c";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(valkyrie_msgs.msg.dds.PsyonicTrajectoryMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, valkyrie_msgs.msg.dds.PsyonicTrajectoryMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (10 * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += controller_msgs.msg.dds.JointspaceTrajectoryMessagePubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(valkyrie_msgs.msg.dds.PsyonicTrajectoryMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(valkyrie_msgs.msg.dds.PsyonicTrajectoryMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getJointNames().size() * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += controller_msgs.msg.dds.JointspaceTrajectoryMessagePubSubType.getCdrSerializedSize(data.getJointspaceTrajectory(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(valkyrie_msgs.msg.dds.PsyonicTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_9(data.getRobotSide());

      if(data.getJointNames().size() <= 10)
      cdr.write_type_e(data.getJointNames());else
          throw new RuntimeException("joint_names field exceeds the maximum length");

      controller_msgs.msg.dds.JointspaceTrajectoryMessagePubSubType.write(data.getJointspaceTrajectory(), cdr);
   }

   public static void read(valkyrie_msgs.msg.dds.PsyonicTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setRobotSide(cdr.read_type_9());
      	
      cdr.read_type_e(data.getJointNames());	
      controller_msgs.msg.dds.JointspaceTrajectoryMessagePubSubType.read(data.getJointspaceTrajectory(), cdr);	

   }

   @Override
   public final void serialize(valkyrie_msgs.msg.dds.PsyonicTrajectoryMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_9("robot_side", data.getRobotSide());
      ser.write_type_e("joint_names", data.getJointNames());
      ser.write_type_a("jointspace_trajectory", new controller_msgs.msg.dds.JointspaceTrajectoryMessagePubSubType(), data.getJointspaceTrajectory());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, valkyrie_msgs.msg.dds.PsyonicTrajectoryMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setRobotSide(ser.read_type_9("robot_side"));
      ser.read_type_e("joint_names", data.getJointNames());
      ser.read_type_a("jointspace_trajectory", new controller_msgs.msg.dds.JointspaceTrajectoryMessagePubSubType(), data.getJointspaceTrajectory());

   }

   public static void staticCopy(valkyrie_msgs.msg.dds.PsyonicTrajectoryMessage src, valkyrie_msgs.msg.dds.PsyonicTrajectoryMessage dest)
   {
      dest.set(src);
   }

   @Override
   public valkyrie_msgs.msg.dds.PsyonicTrajectoryMessage createData()
   {
      return new valkyrie_msgs.msg.dds.PsyonicTrajectoryMessage();
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
   
   public void serialize(valkyrie_msgs.msg.dds.PsyonicTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(valkyrie_msgs.msg.dds.PsyonicTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(valkyrie_msgs.msg.dds.PsyonicTrajectoryMessage src, valkyrie_msgs.msg.dds.PsyonicTrajectoryMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public PsyonicTrajectoryMessagePubSubType newInstance()
   {
      return new PsyonicTrajectoryMessagePubSubType();
   }
}
