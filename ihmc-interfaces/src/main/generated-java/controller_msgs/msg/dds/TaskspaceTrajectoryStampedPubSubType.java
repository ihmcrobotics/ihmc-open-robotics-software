package controller_msgs.msg.dds;

/**
 * Topic data type of the struct "TaskspaceTrajectoryStamped" defined in "TaskspaceTrajectoryStamped_.idl". Use this class to provide the TopicDataType to a Participant.
 *
 * This file was automatically generated from TaskspaceTrajectoryStamped_.idl by us.ihmc.idl.generator.IDLGenerator.
 * Do not update this file directly, edit TaskspaceTrajectoryStamped_.idl instead.
 */
public class TaskspaceTrajectoryStampedPubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.TaskspaceTrajectoryStamped>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::TaskspaceTrajectoryStamped_";
   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   public TaskspaceTrajectoryStampedPubSubType()
   {

   }

   public static int getMaxCdrSerializedSize()
   {
      return getMaxCdrSerializedSize(0);
   }

   public static int getMaxCdrSerializedSize(int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += std_msgs.msg.dds.HeaderPubSubType.getMaxCdrSerializedSize(current_alignment);
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for (int a = 0; a < 100; ++a)
      {
         current_alignment += geometry_msgs.msg.dds.PoseStampedPubSubType.getMaxCdrSerializedSize(current_alignment);
      }

      current_alignment += builtin_interfaces.msg.dds.DurationPubSubType.getMaxCdrSerializedSize(current_alignment);

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.TaskspaceTrajectoryStamped data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.TaskspaceTrajectoryStamped data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += std_msgs.msg.dds.HeaderPubSubType.getCdrSerializedSize(data.getHeader(), current_alignment);
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for (int a = 0; a < data.getTrajectory_points_stamped().size(); ++a)
      {
         current_alignment += geometry_msgs.msg.dds.PoseStampedPubSubType.getCdrSerializedSize(data.getTrajectory_points_stamped().get(a), current_alignment);
      }

      current_alignment += builtin_interfaces.msg.dds.DurationPubSubType.getCdrSerializedSize(data.getTime_from_start(), current_alignment);

      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.TaskspaceTrajectoryStamped data, us.ihmc.idl.CDR cdr)
   {

      std_msgs.msg.dds.HeaderPubSubType.write(data.getHeader(), cdr);

      if (data.getTrajectory_points_stamped().size() <= 100)
         cdr.write_type_e(data.getTrajectory_points_stamped());
      else
         throw new RuntimeException("trajectory_points_stamped field exceeds the maximum length");

      builtin_interfaces.msg.dds.DurationPubSubType.write(data.getTime_from_start(), cdr);
   }

   public static void read(controller_msgs.msg.dds.TaskspaceTrajectoryStamped data, us.ihmc.idl.CDR cdr)
   {

      std_msgs.msg.dds.HeaderPubSubType.read(data.getHeader(), cdr);

      cdr.read_type_e(data.getTrajectory_points_stamped());

      builtin_interfaces.msg.dds.DurationPubSubType.read(data.getTime_from_start(), cdr);
   }

   public static void staticCopy(controller_msgs.msg.dds.TaskspaceTrajectoryStamped src, controller_msgs.msg.dds.TaskspaceTrajectoryStamped dest)
   {
      dest.set(src);
   }

   @Override
   public void serialize(controller_msgs.msg.dds.TaskspaceTrajectoryStamped data, us.ihmc.pubsub.common.SerializedPayload serializedPayload)
         throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.TaskspaceTrajectoryStamped data)
         throws java.io.IOException
   {
      deserializeCDR.deserialize(serializedPayload);
      read(data, deserializeCDR);
      deserializeCDR.finishDeserialize();
   }

   @Override
   public final void serialize(controller_msgs.msg.dds.TaskspaceTrajectoryStamped data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("header", new std_msgs.msg.dds.HeaderPubSubType(), data.getHeader());

      ser.write_type_e("trajectory_points_stamped", data.getTrajectory_points_stamped());

      ser.write_type_a("time_from_start", new builtin_interfaces.msg.dds.DurationPubSubType(), data.getTime_from_start());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.TaskspaceTrajectoryStamped data)
   {
      ser.read_type_a("header", new std_msgs.msg.dds.HeaderPubSubType(), data.getHeader());

      ser.read_type_e("trajectory_points_stamped", data.getTrajectory_points_stamped());

      ser.read_type_a("time_from_start", new builtin_interfaces.msg.dds.DurationPubSubType(), data.getTime_from_start());
   }

   @Override
   public controller_msgs.msg.dds.TaskspaceTrajectoryStamped createData()
   {
      return new controller_msgs.msg.dds.TaskspaceTrajectoryStamped();
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

   public void serialize(controller_msgs.msg.dds.TaskspaceTrajectoryStamped data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.TaskspaceTrajectoryStamped data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }

   public void copy(controller_msgs.msg.dds.TaskspaceTrajectoryStamped src, controller_msgs.msg.dds.TaskspaceTrajectoryStamped dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public TaskspaceTrajectoryStampedPubSubType newInstance()
   {
      return new TaskspaceTrajectoryStampedPubSubType();
   }
}