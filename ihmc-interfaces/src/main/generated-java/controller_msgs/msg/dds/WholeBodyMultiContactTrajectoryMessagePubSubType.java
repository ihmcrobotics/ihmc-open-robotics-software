package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "WholeBodyMultiContactTrajectoryMessage" defined in "WholeBodyMultiContactTrajectoryMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from WholeBodyMultiContactTrajectoryMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit WholeBodyMultiContactTrajectoryMessage_.idl instead.
*
*/
public class WholeBodyMultiContactTrajectoryMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.WholeBodyMultiContactTrajectoryMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::WholeBodyMultiContactTrajectoryMessage_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.WholeBodyMultiContactTrajectoryMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.WholeBodyMultiContactTrajectoryMessage data) throws java.io.IOException
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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (50 * 8) + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.WholeBodyMultiContactTrajectoryMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.WholeBodyMultiContactTrajectoryMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getJointAngles().size() * 8) + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getCdrSerializedSize(data.getRootJointPose(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.WholeBodyMultiContactTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_6(data.getTrajectoryDuration());

      if(data.getJointAngles().size() <= 50)
      cdr.write_type_e(data.getJointAngles());else
          throw new RuntimeException("joint_angles field exceeds the maximum length");

      geometry_msgs.msg.dds.PosePubSubType.write(data.getRootJointPose(), cdr);
      cdr.write_type_2(data.getJointNameHash());

   }

   public static void read(controller_msgs.msg.dds.WholeBodyMultiContactTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setTrajectoryDuration(cdr.read_type_6());
      	
      cdr.read_type_e(data.getJointAngles());	
      geometry_msgs.msg.dds.PosePubSubType.read(data.getRootJointPose(), cdr);	
      data.setJointNameHash(cdr.read_type_2());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.WholeBodyMultiContactTrajectoryMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_6("trajectory_duration", data.getTrajectoryDuration());
      ser.write_type_e("joint_angles", data.getJointAngles());
      ser.write_type_a("root_joint_pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getRootJointPose());

      ser.write_type_2("joint_name_hash", data.getJointNameHash());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.WholeBodyMultiContactTrajectoryMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setTrajectoryDuration(ser.read_type_6("trajectory_duration"));
      ser.read_type_e("joint_angles", data.getJointAngles());
      ser.read_type_a("root_joint_pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getRootJointPose());

      data.setJointNameHash(ser.read_type_2("joint_name_hash"));
   }

   public static void staticCopy(controller_msgs.msg.dds.WholeBodyMultiContactTrajectoryMessage src, controller_msgs.msg.dds.WholeBodyMultiContactTrajectoryMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.WholeBodyMultiContactTrajectoryMessage createData()
   {
      return new controller_msgs.msg.dds.WholeBodyMultiContactTrajectoryMessage();
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
   
   public void serialize(controller_msgs.msg.dds.WholeBodyMultiContactTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.WholeBodyMultiContactTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.WholeBodyMultiContactTrajectoryMessage src, controller_msgs.msg.dds.WholeBodyMultiContactTrajectoryMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public WholeBodyMultiContactTrajectoryMessagePubSubType newInstance()
   {
      return new WholeBodyMultiContactTrajectoryMessagePubSubType();
   }
}
