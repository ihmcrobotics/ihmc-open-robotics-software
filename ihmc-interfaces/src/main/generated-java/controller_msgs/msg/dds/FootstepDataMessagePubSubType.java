package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "FootstepDataMessage" defined in "FootstepDataMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from FootstepDataMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit FootstepDataMessage_.idl instead.
*
*/
public class FootstepDataMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.FootstepDataMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::FootstepDataMessage_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.FootstepDataMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.FootstepDataMessage data) throws java.io.IOException
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

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 10; ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (2 * 8) + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 10; ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 10; ++i0)
      {
          current_alignment += controller_msgs.msg.dds.SE3TrajectoryPointMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.FootstepDataMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.FootstepDataMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getLocation(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getCdrSerializedSize(data.getOrientation(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getPredictedContactPoints2d().size(); ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getPredictedContactPoints2d().get(i0), current_alignment);}

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getCustomWaypointProportions().size() * 8) + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getCustomPositionWaypoints().size(); ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getCustomPositionWaypoints().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getSwingTrajectory().size(); ++i0)
      {
          current_alignment += controller_msgs.msg.dds.SE3TrajectoryPointMessagePubSubType.getCdrSerializedSize(data.getSwingTrajectory().get(i0), current_alignment);}

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.FootstepDataMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_9(data.getRobotSide());

      geometry_msgs.msg.dds.PointPubSubType.write(data.getLocation(), cdr);
      geometry_msgs.msg.dds.QuaternionPubSubType.write(data.getOrientation(), cdr);
      if(data.getPredictedContactPoints2d().size() <= 10)
      cdr.write_type_e(data.getPredictedContactPoints2d());else
          throw new RuntimeException("predicted_contact_points_2d field exceeds the maximum length");

      cdr.write_type_9(data.getTrajectoryType());

      cdr.write_type_6(data.getSwingHeight());

      if(data.getCustomWaypointProportions().size() <= 2)
      cdr.write_type_e(data.getCustomWaypointProportions());else
          throw new RuntimeException("custom_waypoint_proportions field exceeds the maximum length");

      if(data.getCustomPositionWaypoints().size() <= 10)
      cdr.write_type_e(data.getCustomPositionWaypoints());else
          throw new RuntimeException("custom_position_waypoints field exceeds the maximum length");

      if(data.getSwingTrajectory().size() <= 10)
      cdr.write_type_e(data.getSwingTrajectory());else
          throw new RuntimeException("swing_trajectory field exceeds the maximum length");

      cdr.write_type_6(data.getSwingTrajectoryBlendDuration());

      cdr.write_type_6(data.getSwingDuration());

      cdr.write_type_6(data.getTransferDuration());

      cdr.write_type_6(data.getExecutionDelayTime());

      cdr.write_type_6(data.getTransferSplitFraction());

      cdr.write_type_6(data.getTouchdownDuration());

      cdr.write_type_6(data.getLiftoffDuration());

   }

   public static void read(controller_msgs.msg.dds.FootstepDataMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setRobotSide(cdr.read_type_9());
      	
      geometry_msgs.msg.dds.PointPubSubType.read(data.getLocation(), cdr);	
      geometry_msgs.msg.dds.QuaternionPubSubType.read(data.getOrientation(), cdr);	
      cdr.read_type_e(data.getPredictedContactPoints2d());	
      data.setTrajectoryType(cdr.read_type_9());
      	
      data.setSwingHeight(cdr.read_type_6());
      	
      cdr.read_type_e(data.getCustomWaypointProportions());	
      cdr.read_type_e(data.getCustomPositionWaypoints());	
      cdr.read_type_e(data.getSwingTrajectory());	
      data.setSwingTrajectoryBlendDuration(cdr.read_type_6());
      	
      data.setSwingDuration(cdr.read_type_6());
      	
      data.setTransferDuration(cdr.read_type_6());
      	
      data.setExecutionDelayTime(cdr.read_type_6());
      	
      data.setTransferSplitFraction(cdr.read_type_6());
      	
      data.setTouchdownDuration(cdr.read_type_6());
      	
      data.setLiftoffDuration(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.FootstepDataMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_9("robot_side", data.getRobotSide());
      ser.write_type_a("location", new geometry_msgs.msg.dds.PointPubSubType(), data.getLocation());

      ser.write_type_a("orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getOrientation());

      ser.write_type_e("predicted_contact_points_2d", data.getPredictedContactPoints2d());
      ser.write_type_9("trajectory_type", data.getTrajectoryType());
      ser.write_type_6("swing_height", data.getSwingHeight());
      ser.write_type_e("custom_waypoint_proportions", data.getCustomWaypointProportions());
      ser.write_type_e("custom_position_waypoints", data.getCustomPositionWaypoints());
      ser.write_type_e("swing_trajectory", data.getSwingTrajectory());
      ser.write_type_6("swing_trajectory_blend_duration", data.getSwingTrajectoryBlendDuration());
      ser.write_type_6("swing_duration", data.getSwingDuration());
      ser.write_type_6("transfer_duration", data.getTransferDuration());
      ser.write_type_6("execution_delay_time", data.getExecutionDelayTime());
      ser.write_type_6("transfer_split_fraction", data.getTransferSplitFraction());
      ser.write_type_6("touchdown_duration", data.getTouchdownDuration());
      ser.write_type_6("liftoff_duration", data.getLiftoffDuration());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.FootstepDataMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setRobotSide(ser.read_type_9("robot_side"));
      ser.read_type_a("location", new geometry_msgs.msg.dds.PointPubSubType(), data.getLocation());

      ser.read_type_a("orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getOrientation());

      ser.read_type_e("predicted_contact_points_2d", data.getPredictedContactPoints2d());
      data.setTrajectoryType(ser.read_type_9("trajectory_type"));
      data.setSwingHeight(ser.read_type_6("swing_height"));
      ser.read_type_e("custom_waypoint_proportions", data.getCustomWaypointProportions());
      ser.read_type_e("custom_position_waypoints", data.getCustomPositionWaypoints());
      ser.read_type_e("swing_trajectory", data.getSwingTrajectory());
      data.setSwingTrajectoryBlendDuration(ser.read_type_6("swing_trajectory_blend_duration"));
      data.setSwingDuration(ser.read_type_6("swing_duration"));
      data.setTransferDuration(ser.read_type_6("transfer_duration"));
      data.setExecutionDelayTime(ser.read_type_6("execution_delay_time"));
      data.setTransferSplitFraction(ser.read_type_6("transfer_split_fraction"));
      data.setTouchdownDuration(ser.read_type_6("touchdown_duration"));
      data.setLiftoffDuration(ser.read_type_6("liftoff_duration"));
   }

   public static void staticCopy(controller_msgs.msg.dds.FootstepDataMessage src, controller_msgs.msg.dds.FootstepDataMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.FootstepDataMessage createData()
   {
      return new controller_msgs.msg.dds.FootstepDataMessage();
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
   
   public void serialize(controller_msgs.msg.dds.FootstepDataMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.FootstepDataMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.FootstepDataMessage src, controller_msgs.msg.dds.FootstepDataMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public FootstepDataMessagePubSubType newInstance()
   {
      return new FootstepDataMessagePubSubType();
   }
}
