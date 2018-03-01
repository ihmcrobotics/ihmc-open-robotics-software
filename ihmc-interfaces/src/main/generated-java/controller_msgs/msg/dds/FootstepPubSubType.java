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
      for (int a = 0; a < data.getPredicted_contact_points_2d().size(); ++a)
      {
         current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getPredicted_contact_points_2d().get(a), current_alignment);
      }

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for (int a = 0; a < data.getPosition_waypoints().size(); ++a)
      {
         current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getPosition_waypoints().get(a), current_alignment);
      }

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for (int a = 0; a < data.getSwing_trajectory().size(); ++a)
      {
         current_alignment += controller_msgs.msg.dds.TaskspaceTrajectoryStampedPubSubType
               .getCdrSerializedSize(data.getSwing_trajectory().get(a), current_alignment);
      }

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.Footstep data, us.ihmc.idl.CDR cdr)
   {

      cdr.write_type_12(data.getUnique_id());

      cdr.write_type_9(data.getRobot_side());

      geometry_msgs.msg.dds.PointPubSubType.write(data.getLocation(), cdr);

      geometry_msgs.msg.dds.QuaternionPubSubType.write(data.getOrientation(), cdr);

      if (data.getPredicted_contact_points_2d().size() <= 100)
         cdr.write_type_e(data.getPredicted_contact_points_2d());
      else
         throw new RuntimeException("predicted_contact_points_2d field exceeds the maximum length");

      cdr.write_type_9(data.getTrajectory_type());

      cdr.write_type_6(data.getSwing_height());

      if (data.getPosition_waypoints().size() <= 100)
         cdr.write_type_e(data.getPosition_waypoints());
      else
         throw new RuntimeException("position_waypoints field exceeds the maximum length");

      if (data.getSwing_trajectory().size() <= 100)
         cdr.write_type_e(data.getSwing_trajectory());
      else
         throw new RuntimeException("swing_trajectory field exceeds the maximum length");

      cdr.write_type_6(data.getSwing_trajectory_blend_duration());

      cdr.write_type_6(data.getSwing_duration());

      cdr.write_type_6(data.getTransfer_duration());
   }

   public static void read(controller_msgs.msg.dds.Footstep data, us.ihmc.idl.CDR cdr)
   {

      data.setUnique_id(cdr.read_type_12());

      data.setRobot_side(cdr.read_type_9());

      geometry_msgs.msg.dds.PointPubSubType.read(data.getLocation(), cdr);

      geometry_msgs.msg.dds.QuaternionPubSubType.read(data.getOrientation(), cdr);

      cdr.read_type_e(data.getPredicted_contact_points_2d());

      data.setTrajectory_type(cdr.read_type_9());

      data.setSwing_height(cdr.read_type_6());

      cdr.read_type_e(data.getPosition_waypoints());

      cdr.read_type_e(data.getSwing_trajectory());

      data.setSwing_trajectory_blend_duration(cdr.read_type_6());

      data.setSwing_duration(cdr.read_type_6());

      data.setTransfer_duration(cdr.read_type_6());
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
      ser.write_type_12("unique_id", data.getUnique_id());

      ser.write_type_9("robot_side", data.getRobot_side());

      ser.write_type_a("location", new geometry_msgs.msg.dds.PointPubSubType(), data.getLocation());

      ser.write_type_a("orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getOrientation());

      ser.write_type_e("predicted_contact_points_2d", data.getPredicted_contact_points_2d());

      ser.write_type_9("trajectory_type", data.getTrajectory_type());

      ser.write_type_6("swing_height", data.getSwing_height());

      ser.write_type_e("position_waypoints", data.getPosition_waypoints());

      ser.write_type_e("swing_trajectory", data.getSwing_trajectory());

      ser.write_type_6("swing_trajectory_blend_duration", data.getSwing_trajectory_blend_duration());

      ser.write_type_6("swing_duration", data.getSwing_duration());

      ser.write_type_6("transfer_duration", data.getTransfer_duration());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.Footstep data)
   {
      data.setUnique_id(ser.read_type_12("unique_id"));

      data.setRobot_side(ser.read_type_9("robot_side"));

      ser.read_type_a("location", new geometry_msgs.msg.dds.PointPubSubType(), data.getLocation());

      ser.read_type_a("orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getOrientation());

      ser.read_type_e("predicted_contact_points_2d", data.getPredicted_contact_points_2d());

      data.setTrajectory_type(ser.read_type_9("trajectory_type"));

      data.setSwing_height(ser.read_type_6("swing_height"));

      ser.read_type_e("position_waypoints", data.getPosition_waypoints());

      ser.read_type_e("swing_trajectory", data.getSwing_trajectory());

      data.setSwing_trajectory_blend_duration(ser.read_type_6("swing_trajectory_blend_duration"));

      data.setSwing_duration(ser.read_type_6("swing_duration"));

      data.setTransfer_duration(ser.read_type_6("transfer_duration"));
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