package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "PelvisHeightTrajectoryMessage" defined in "PelvisHeightTrajectoryMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from PelvisHeightTrajectoryMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit PelvisHeightTrajectoryMessage_.idl instead.
*
*/
public class PelvisHeightTrajectoryMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.PelvisHeightTrajectoryMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::PelvisHeightTrajectoryMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "a3556820a1dab29433957adff61c8b6d05b8f65dd7c3733f3ec4f6f49d690851";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.PelvisHeightTrajectoryMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.PelvisHeightTrajectoryMessage data) throws java.io.IOException
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

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += ihmc_common_msgs.msg.dds.EuclideanTrajectoryMessagePubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.PelvisHeightTrajectoryMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.PelvisHeightTrajectoryMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += ihmc_common_msgs.msg.dds.EuclideanTrajectoryMessagePubSubType.getCdrSerializedSize(data.getEuclideanTrajectory(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.PelvisHeightTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_7(data.getEnableUserPelvisControl());

      cdr.write_type_7(data.getEnableUserPelvisControlDuringWalking());

      ihmc_common_msgs.msg.dds.EuclideanTrajectoryMessagePubSubType.write(data.getEuclideanTrajectory(), cdr);
   }

   public static void read(controller_msgs.msg.dds.PelvisHeightTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setEnableUserPelvisControl(cdr.read_type_7());
      	
      data.setEnableUserPelvisControlDuringWalking(cdr.read_type_7());
      	
      ihmc_common_msgs.msg.dds.EuclideanTrajectoryMessagePubSubType.read(data.getEuclideanTrajectory(), cdr);	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.PelvisHeightTrajectoryMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_7("enable_user_pelvis_control", data.getEnableUserPelvisControl());
      ser.write_type_7("enable_user_pelvis_control_during_walking", data.getEnableUserPelvisControlDuringWalking());
      ser.write_type_a("euclidean_trajectory", new ihmc_common_msgs.msg.dds.EuclideanTrajectoryMessagePubSubType(), data.getEuclideanTrajectory());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.PelvisHeightTrajectoryMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setEnableUserPelvisControl(ser.read_type_7("enable_user_pelvis_control"));
      data.setEnableUserPelvisControlDuringWalking(ser.read_type_7("enable_user_pelvis_control_during_walking"));
      ser.read_type_a("euclidean_trajectory", new ihmc_common_msgs.msg.dds.EuclideanTrajectoryMessagePubSubType(), data.getEuclideanTrajectory());

   }

   public static void staticCopy(controller_msgs.msg.dds.PelvisHeightTrajectoryMessage src, controller_msgs.msg.dds.PelvisHeightTrajectoryMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.PelvisHeightTrajectoryMessage createData()
   {
      return new controller_msgs.msg.dds.PelvisHeightTrajectoryMessage();
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
   
   public void serialize(controller_msgs.msg.dds.PelvisHeightTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.PelvisHeightTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.PelvisHeightTrajectoryMessage src, controller_msgs.msg.dds.PelvisHeightTrajectoryMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public PelvisHeightTrajectoryMessagePubSubType newInstance()
   {
      return new PelvisHeightTrajectoryMessagePubSubType();
   }
}
