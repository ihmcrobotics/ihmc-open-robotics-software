package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "WholeBodyTrajectoryMessage" defined in "WholeBodyTrajectoryMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from WholeBodyTrajectoryMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit WholeBodyTrajectoryMessage_.idl instead.
*
*/
public class WholeBodyTrajectoryMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.WholeBodyTrajectoryMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::WholeBodyTrajectoryMessage_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.WholeBodyTrajectoryMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.WholeBodyTrajectoryMessage data) throws java.io.IOException
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

      current_alignment += controller_msgs.msg.dds.HandTrajectoryMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += controller_msgs.msg.dds.HandTrajectoryMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += controller_msgs.msg.dds.ArmTrajectoryMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += controller_msgs.msg.dds.ArmTrajectoryMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += controller_msgs.msg.dds.ChestTrajectoryMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += controller_msgs.msg.dds.SpineTrajectoryMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += controller_msgs.msg.dds.PelvisTrajectoryMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += controller_msgs.msg.dds.FootTrajectoryMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += controller_msgs.msg.dds.FootTrajectoryMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += controller_msgs.msg.dds.HeadTrajectoryMessagePubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.WholeBodyTrajectoryMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.WholeBodyTrajectoryMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += controller_msgs.msg.dds.HandTrajectoryMessagePubSubType.getCdrSerializedSize(data.getLeftHandTrajectoryMessage(), current_alignment);

      current_alignment += controller_msgs.msg.dds.HandTrajectoryMessagePubSubType.getCdrSerializedSize(data.getRightHandTrajectoryMessage(), current_alignment);

      current_alignment += controller_msgs.msg.dds.ArmTrajectoryMessagePubSubType.getCdrSerializedSize(data.getLeftArmTrajectoryMessage(), current_alignment);

      current_alignment += controller_msgs.msg.dds.ArmTrajectoryMessagePubSubType.getCdrSerializedSize(data.getRightArmTrajectoryMessage(), current_alignment);

      current_alignment += controller_msgs.msg.dds.ChestTrajectoryMessagePubSubType.getCdrSerializedSize(data.getChestTrajectoryMessage(), current_alignment);

      current_alignment += controller_msgs.msg.dds.SpineTrajectoryMessagePubSubType.getCdrSerializedSize(data.getSpineTrajectoryMessage(), current_alignment);

      current_alignment += controller_msgs.msg.dds.PelvisTrajectoryMessagePubSubType.getCdrSerializedSize(data.getPelvisTrajectoryMessage(), current_alignment);

      current_alignment += controller_msgs.msg.dds.FootTrajectoryMessagePubSubType.getCdrSerializedSize(data.getLeftFootTrajectoryMessage(), current_alignment);

      current_alignment += controller_msgs.msg.dds.FootTrajectoryMessagePubSubType.getCdrSerializedSize(data.getRightFootTrajectoryMessage(), current_alignment);

      current_alignment += controller_msgs.msg.dds.HeadTrajectoryMessagePubSubType.getCdrSerializedSize(data.getHeadTrajectoryMessage(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.WholeBodyTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      controller_msgs.msg.dds.HandTrajectoryMessagePubSubType.write(data.getLeftHandTrajectoryMessage(), cdr);
      controller_msgs.msg.dds.HandTrajectoryMessagePubSubType.write(data.getRightHandTrajectoryMessage(), cdr);
      controller_msgs.msg.dds.ArmTrajectoryMessagePubSubType.write(data.getLeftArmTrajectoryMessage(), cdr);
      controller_msgs.msg.dds.ArmTrajectoryMessagePubSubType.write(data.getRightArmTrajectoryMessage(), cdr);
      controller_msgs.msg.dds.ChestTrajectoryMessagePubSubType.write(data.getChestTrajectoryMessage(), cdr);
      controller_msgs.msg.dds.SpineTrajectoryMessagePubSubType.write(data.getSpineTrajectoryMessage(), cdr);
      controller_msgs.msg.dds.PelvisTrajectoryMessagePubSubType.write(data.getPelvisTrajectoryMessage(), cdr);
      controller_msgs.msg.dds.FootTrajectoryMessagePubSubType.write(data.getLeftFootTrajectoryMessage(), cdr);
      controller_msgs.msg.dds.FootTrajectoryMessagePubSubType.write(data.getRightFootTrajectoryMessage(), cdr);
      controller_msgs.msg.dds.HeadTrajectoryMessagePubSubType.write(data.getHeadTrajectoryMessage(), cdr);
   }

   public static void read(controller_msgs.msg.dds.WholeBodyTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      controller_msgs.msg.dds.HandTrajectoryMessagePubSubType.read(data.getLeftHandTrajectoryMessage(), cdr);	
      controller_msgs.msg.dds.HandTrajectoryMessagePubSubType.read(data.getRightHandTrajectoryMessage(), cdr);	
      controller_msgs.msg.dds.ArmTrajectoryMessagePubSubType.read(data.getLeftArmTrajectoryMessage(), cdr);	
      controller_msgs.msg.dds.ArmTrajectoryMessagePubSubType.read(data.getRightArmTrajectoryMessage(), cdr);	
      controller_msgs.msg.dds.ChestTrajectoryMessagePubSubType.read(data.getChestTrajectoryMessage(), cdr);	
      controller_msgs.msg.dds.SpineTrajectoryMessagePubSubType.read(data.getSpineTrajectoryMessage(), cdr);	
      controller_msgs.msg.dds.PelvisTrajectoryMessagePubSubType.read(data.getPelvisTrajectoryMessage(), cdr);	
      controller_msgs.msg.dds.FootTrajectoryMessagePubSubType.read(data.getLeftFootTrajectoryMessage(), cdr);	
      controller_msgs.msg.dds.FootTrajectoryMessagePubSubType.read(data.getRightFootTrajectoryMessage(), cdr);	
      controller_msgs.msg.dds.HeadTrajectoryMessagePubSubType.read(data.getHeadTrajectoryMessage(), cdr);	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.WholeBodyTrajectoryMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_a("left_hand_trajectory_message", new controller_msgs.msg.dds.HandTrajectoryMessagePubSubType(), data.getLeftHandTrajectoryMessage());

      ser.write_type_a("right_hand_trajectory_message", new controller_msgs.msg.dds.HandTrajectoryMessagePubSubType(), data.getRightHandTrajectoryMessage());

      ser.write_type_a("left_arm_trajectory_message", new controller_msgs.msg.dds.ArmTrajectoryMessagePubSubType(), data.getLeftArmTrajectoryMessage());

      ser.write_type_a("right_arm_trajectory_message", new controller_msgs.msg.dds.ArmTrajectoryMessagePubSubType(), data.getRightArmTrajectoryMessage());

      ser.write_type_a("chest_trajectory_message", new controller_msgs.msg.dds.ChestTrajectoryMessagePubSubType(), data.getChestTrajectoryMessage());

      ser.write_type_a("spine_trajectory_message", new controller_msgs.msg.dds.SpineTrajectoryMessagePubSubType(), data.getSpineTrajectoryMessage());

      ser.write_type_a("pelvis_trajectory_message", new controller_msgs.msg.dds.PelvisTrajectoryMessagePubSubType(), data.getPelvisTrajectoryMessage());

      ser.write_type_a("left_foot_trajectory_message", new controller_msgs.msg.dds.FootTrajectoryMessagePubSubType(), data.getLeftFootTrajectoryMessage());

      ser.write_type_a("right_foot_trajectory_message", new controller_msgs.msg.dds.FootTrajectoryMessagePubSubType(), data.getRightFootTrajectoryMessage());

      ser.write_type_a("head_trajectory_message", new controller_msgs.msg.dds.HeadTrajectoryMessagePubSubType(), data.getHeadTrajectoryMessage());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.WholeBodyTrajectoryMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      ser.read_type_a("left_hand_trajectory_message", new controller_msgs.msg.dds.HandTrajectoryMessagePubSubType(), data.getLeftHandTrajectoryMessage());

      ser.read_type_a("right_hand_trajectory_message", new controller_msgs.msg.dds.HandTrajectoryMessagePubSubType(), data.getRightHandTrajectoryMessage());

      ser.read_type_a("left_arm_trajectory_message", new controller_msgs.msg.dds.ArmTrajectoryMessagePubSubType(), data.getLeftArmTrajectoryMessage());

      ser.read_type_a("right_arm_trajectory_message", new controller_msgs.msg.dds.ArmTrajectoryMessagePubSubType(), data.getRightArmTrajectoryMessage());

      ser.read_type_a("chest_trajectory_message", new controller_msgs.msg.dds.ChestTrajectoryMessagePubSubType(), data.getChestTrajectoryMessage());

      ser.read_type_a("spine_trajectory_message", new controller_msgs.msg.dds.SpineTrajectoryMessagePubSubType(), data.getSpineTrajectoryMessage());

      ser.read_type_a("pelvis_trajectory_message", new controller_msgs.msg.dds.PelvisTrajectoryMessagePubSubType(), data.getPelvisTrajectoryMessage());

      ser.read_type_a("left_foot_trajectory_message", new controller_msgs.msg.dds.FootTrajectoryMessagePubSubType(), data.getLeftFootTrajectoryMessage());

      ser.read_type_a("right_foot_trajectory_message", new controller_msgs.msg.dds.FootTrajectoryMessagePubSubType(), data.getRightFootTrajectoryMessage());

      ser.read_type_a("head_trajectory_message", new controller_msgs.msg.dds.HeadTrajectoryMessagePubSubType(), data.getHeadTrajectoryMessage());

   }

   public static void staticCopy(controller_msgs.msg.dds.WholeBodyTrajectoryMessage src, controller_msgs.msg.dds.WholeBodyTrajectoryMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.WholeBodyTrajectoryMessage createData()
   {
      return new controller_msgs.msg.dds.WholeBodyTrajectoryMessage();
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
   
   public void serialize(controller_msgs.msg.dds.WholeBodyTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.WholeBodyTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.WholeBodyTrajectoryMessage src, controller_msgs.msg.dds.WholeBodyTrajectoryMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public WholeBodyTrajectoryMessagePubSubType newInstance()
   {
      return new WholeBodyTrajectoryMessagePubSubType();
   }
}
