package controller_msgs.msg.dds;

/**
 * Topic data type of the struct "PelvisHeightTrajectoryMessage" defined in "PelvisHeightTrajectoryMessage_.idl". Use this class to provide the TopicDataType to a Participant.
 *
 * This file was automatically generated from PelvisHeightTrajectoryMessage_.idl by us.ihmc.idl.generator.IDLGenerator.
 * Do not update this file directly, edit PelvisHeightTrajectoryMessage_.idl instead.
 */
public class PelvisHeightTrajectoryMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.PelvisHeightTrajectoryMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::PelvisHeightTrajectoryMessage_";
   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   public PelvisHeightTrajectoryMessagePubSubType()
   {

   }

   public static int getMaxCdrSerializedSize()
   {
      return getMaxCdrSerializedSize(0);
   }

   public static int getMaxCdrSerializedSize(int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += controller_msgs.msg.dds.EuclideanTrajectoryMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.PelvisHeightTrajectoryMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.PelvisHeightTrajectoryMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += controller_msgs.msg.dds.EuclideanTrajectoryMessagePubSubType.getCdrSerializedSize(data.getEuclideanTrajectory(), current_alignment);

      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.PelvisHeightTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {

      cdr.write_type_7(data.getEnableUserPelvisControl());

      cdr.write_type_7(data.getEnableUserPelvisHeightControlDuringWalking());

      controller_msgs.msg.dds.EuclideanTrajectoryMessagePubSubType.write(data.getEuclideanTrajectory(), cdr);
   }

   public static void read(controller_msgs.msg.dds.PelvisHeightTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {

      data.setEnableUserPelvisControl(cdr.read_type_7());

      data.setEnableUserPelvisHeightControlDuringWalking(cdr.read_type_7());

      controller_msgs.msg.dds.EuclideanTrajectoryMessagePubSubType.read(data.getEuclideanTrajectory(), cdr);
   }

   public static void staticCopy(controller_msgs.msg.dds.PelvisHeightTrajectoryMessage src, controller_msgs.msg.dds.PelvisHeightTrajectoryMessage dest)
   {
      dest.set(src);
   }

   @Override
   public void serialize(controller_msgs.msg.dds.PelvisHeightTrajectoryMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload)
         throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.PelvisHeightTrajectoryMessage data)
         throws java.io.IOException
   {
      deserializeCDR.deserialize(serializedPayload);
      read(data, deserializeCDR);
      deserializeCDR.finishDeserialize();
   }

   @Override
   public final void serialize(controller_msgs.msg.dds.PelvisHeightTrajectoryMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_7("enable_user_pelvis_control", data.getEnableUserPelvisControl());

      ser.write_type_7("enable_user_pelvis_height_control_during_walking", data.getEnableUserPelvisHeightControlDuringWalking());

      ser.write_type_a("euclidean_trajectory", new controller_msgs.msg.dds.EuclideanTrajectoryMessagePubSubType(), data.getEuclideanTrajectory());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.PelvisHeightTrajectoryMessage data)
   {
      data.setEnableUserPelvisControl(ser.read_type_7("enable_user_pelvis_control"));

      data.setEnableUserPelvisHeightControlDuringWalking(ser.read_type_7("enable_user_pelvis_height_control_during_walking"));

      ser.read_type_a("euclidean_trajectory", new controller_msgs.msg.dds.EuclideanTrajectoryMessagePubSubType(), data.getEuclideanTrajectory());
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