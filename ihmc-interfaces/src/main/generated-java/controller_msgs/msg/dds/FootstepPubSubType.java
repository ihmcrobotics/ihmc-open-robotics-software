package controller_msgs.msg.dds;

/**
 * Topic data type of the struct "Footstep" defined in "Footstep_.idl". Use this class to provide the TopicDataType to a Participant.
 *
 * This file was automatically generated from Footstep_.idl by us.ihmc.idl.generator.IDLGenerator.
 * Do not update this file directly, edit Footstep_.idl instead.
 */
public class FootstepPubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.Footstep>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::Footstep_";
   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   public FootstepPubSubType()
   {

   }

   public static int getMaxCdrSerializedSize()
   {
      return getMaxCdrSerializedSize(0);
   }

   public static int getMaxCdrSerializedSize(int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);
      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getMaxCdrSerializedSize(current_alignment);
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for (int a = 0; a < 100; ++a)
      {
         current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);
      }

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for (int a = 0; a < 100; ++a)
      {
         current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);
      }

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for (int a = 0; a < 100; ++a)
      {
         current_alignment += controller_msgs.msg.dds.TaskspaceTrajectoryStampedPubSubType.getMaxCdrSerializedSize(current_alignment);
      }

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.Footstep data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.Footstep data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getLocation(), current_alignment);
      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getCdrSerializedSize(data.getOrientation(), current_alignment);
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for (int a = 0; a < data.getPredictedContactPoints2d().size(); ++a)
      {
         current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getPredictedContactPoints2d().get(a), current_alignment);
      }

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for (int a = 0; a < data.getPositionWaypoints().size(); ++a)
      {
         current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getPositionWaypoints().get(a), current_alignment);
      }

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for (int a = 0; a < data.getSwingTrajectory().size(); ++a)
      {
         current_alignment += controller_msgs.msg.dds.TaskspaceTrajectoryStampedPubSubType
               .getCdrSerializedSize(data.getSwingTrajectory().get(a), current_alignment);
      }

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.Footstep data, us.ihmc.idl.CDR cdr)
   {

      cdr.write_type_12(data.getUniqueId());

      cdr.write_type_9(data.getRobotSide());

      geometry_msgs.msg.dds.PointPubSubType.write(data.getLocation(), cdr);

      geometry_msgs.msg.dds.QuaternionPubSubType.write(data.getOrientation(), cdr);

      if (data.getPredictedContactPoints2d().size() <= 100)
         cdr.write_type_e(data.getPredictedContactPoints2d());
      else
         throw new RuntimeException("predicted_contact_points_2d field exceeds the maximum length");

      cdr.write_type_9(data.getTrajectoryType());

      cdr.write_type_6(data.getSwingHeight());

      if (data.getPositionWaypoints().size() <= 100)
         cdr.write_type_e(data.getPositionWaypoints());
      else
         throw new RuntimeException("position_waypoints field exceeds the maximum length");

      if (data.getSwingTrajectory().size() <= 100)
         cdr.write_type_e(data.getSwingTrajectory());
      else
         throw new RuntimeException("swing_trajectory field exceeds the maximum length");

      cdr.write_type_6(data.getSwingTrajectoryBlendDuration());

      cdr.write_type_6(data.getSwingDuration());

      cdr.write_type_6(data.getTransferDuration());
   }

   public static void read(controller_msgs.msg.dds.Footstep data, us.ihmc.idl.CDR cdr)
   {

      data.setUniqueId(cdr.read_type_12());

      data.setRobotSide(cdr.read_type_9());

      geometry_msgs.msg.dds.PointPubSubType.read(data.getLocation(), cdr);

      geometry_msgs.msg.dds.QuaternionPubSubType.read(data.getOrientation(), cdr);

      cdr.read_type_e(data.getPredictedContactPoints2d());

      data.setTrajectoryType(cdr.read_type_9());

      data.setSwingHeight(cdr.read_type_6());

      cdr.read_type_e(data.getPositionWaypoints());

      cdr.read_type_e(data.getSwingTrajectory());

      data.setSwingTrajectoryBlendDuration(cdr.read_type_6());

      data.setSwingDuration(cdr.read_type_6());

      data.setTransferDuration(cdr.read_type_6());
   }

   public static void staticCopy(controller_msgs.msg.dds.Footstep src, controller_msgs.msg.dds.Footstep dest)
   {
      dest.set(src);
   }

   @Override
   public void serialize(controller_msgs.msg.dds.Footstep data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.Footstep data) throws java.io.IOException
   {
      deserializeCDR.deserialize(serializedPayload);
      read(data, deserializeCDR);
      deserializeCDR.finishDeserialize();
   }

   @Override
   public final void serialize(controller_msgs.msg.dds.Footstep data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_12("unique_id", data.getUniqueId());

      ser.write_type_9("robot_side", data.getRobotSide());

      ser.write_type_a("location", new geometry_msgs.msg.dds.PointPubSubType(), data.getLocation());

      ser.write_type_a("orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getOrientation());

      ser.write_type_e("predicted_contact_points_2d", data.getPredictedContactPoints2d());

      ser.write_type_9("trajectory_type", data.getTrajectoryType());

      ser.write_type_6("swing_height", data.getSwingHeight());

      ser.write_type_e("position_waypoints", data.getPositionWaypoints());

      ser.write_type_e("swing_trajectory", data.getSwingTrajectory());

      ser.write_type_6("swing_trajectory_blend_duration", data.getSwingTrajectoryBlendDuration());

      ser.write_type_6("swing_duration", data.getSwingDuration());

      ser.write_type_6("transfer_duration", data.getTransferDuration());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.Footstep data)
   {
      data.setUniqueId(ser.read_type_12("unique_id"));

      data.setRobotSide(ser.read_type_9("robot_side"));

      ser.read_type_a("location", new geometry_msgs.msg.dds.PointPubSubType(), data.getLocation());

      ser.read_type_a("orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getOrientation());

      ser.read_type_e("predicted_contact_points_2d", data.getPredictedContactPoints2d());

      data.setTrajectoryType(ser.read_type_9("trajectory_type"));

      data.setSwingHeight(ser.read_type_6("swing_height"));

      ser.read_type_e("position_waypoints", data.getPositionWaypoints());

      ser.read_type_e("swing_trajectory", data.getSwingTrajectory());

      data.setSwingTrajectoryBlendDuration(ser.read_type_6("swing_trajectory_blend_duration"));

      data.setSwingDuration(ser.read_type_6("swing_duration"));

      data.setTransferDuration(ser.read_type_6("transfer_duration"));
   }

   @Override
   public controller_msgs.msg.dds.Footstep createData()
   {
      return new controller_msgs.msg.dds.Footstep();
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

   public void serialize(controller_msgs.msg.dds.Footstep data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.Footstep data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }

   public void copy(controller_msgs.msg.dds.Footstep src, controller_msgs.msg.dds.Footstep dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public FootstepPubSubType newInstance()
   {
      return new FootstepPubSubType();
   }
}