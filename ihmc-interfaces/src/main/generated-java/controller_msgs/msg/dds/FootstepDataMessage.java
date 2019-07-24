package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC whole-body controller API.
       * This message specifies the position, orientation and side (left or right) of a desired footstep in world frame.
       */
public class FootstepDataMessage extends Packet<FootstepDataMessage> implements Settable<FootstepDataMessage>, EpsilonComparable<FootstepDataMessage>
{
   public static final byte ROBOT_SIDE_LEFT = (byte) 0;
   public static final byte ROBOT_SIDE_RIGHT = (byte) 1;
   public static final byte TRAJECTORY_TYPE_DEFAULT = (byte) 0;
   public static final byte TRAJECTORY_TYPE_OBSTACLE_CLEARANCE = (byte) 1;
   public static final byte TRAJECTORY_TYPE_CUSTOM = (byte) 2;
   public static final byte TRAJECTORY_TYPE_WAYPOINTS = (byte) 3;
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   /**
            * Specifies which foot will swing to reach the footstep.
            */
   public byte robot_side_ = (byte) 255;
   /**
            * Specifies the position of the footstep (sole frame) in world frame.
            */
   public us.ihmc.euclid.tuple3D.Point3D location_;
   /**
            * Specifies the orientation of the footstep (sole frame) in world frame.
            */
   public us.ihmc.euclid.tuple4D.Quaternion orientation_;
   /**
            * Predicted contact points represent the vertices of the expected contact polygon between the foot and the world.
            * An empty list will request the controller to use the default foot support polygon.
            * Contact points  are expressed in sole frame. The ordering does not matter.
            * For example: to tell the controller to use the entire foot, the predicted contact points would be:
            * - x: 0.5 * foot_length, y: -0.5 * toe_width
            * - x: 0.5 * foot_length, y: 0.5 * toe_width
            * - x: -0.5 * foot_length, y: -0.5 * heel_width
            * - x: -0.5 * foot_length, y: 0.5 * heel_width
            * Note: The z coordinate of each point is ignored.
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D>  predicted_contact_points_2d_;
   /**
            * This contains information on what the swing trajectory should be for each step. Recommended is TRAJECTORY_TYPE_DEFAULT.
            */
   public byte trajectory_type_;
   /**
            * Contains information on how high the robot should swing its foot.
            * This affects trajectory types TRAJECTORY_TYPE_DEFAULT and TRAJECTORY_TYPE_OBSTACLE_CLEARANCE.
            * If a value smaller then the minimal swing height is chosen (e.g. 0.0) the swing height will be changed to a default value.
            */
   public double swing_height_ = -1.0;
   /**
            * In case the trajectory type is set to TRAJECTORY_TYPE_DEFAULT or TRAJECTORY_TYPE_OBSTACLE_CLEARANCE, custom waypoint proportions
            * can be requested. These proportions encode the xy positions of the swing trajectory's two waypoints. A proportion of 0.0 and 1.0 will
            * place a waypoint's xy-position at the start and end of the trajectory, respectively. If this value is empty, the default proportions are used.
            */
   public us.ihmc.idl.IDLSequence.Double  custom_waypoint_proportions_;
   /**
            * In case the trajectory type is set to TRAJECTORY_TYPE_CUSTOM two swing waypoints can be specified here.
            * The waypoints define sole positions.
            * The controller will compute times and velocities at the waypoints.
            * This is a convenient way to shape the trajectory of the swing.
            * If full control over the swing trajectory is desired use the trajectory type TRAJECTORY_TYPE_WAYPOINTS instead.
            * The position waypoints are expected in the trajectory frame.
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D>  custom_position_waypoints_;
   /**
            * In case the trajectory type is set to TRAJECTORY_TYPE_WAYPOINTS, swing waypoints can be specified here.
            * The waypoints do not include the start point (which is set to the current foot state at lift-off) and the touch down point
            * (which is specified by the location and orientation fields).
            * All waypoints are for the sole frame and expressed in the trajectory frame.
            * The maximum number of points can be found in the Footstep class.
            */
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.SE3TrajectoryPointMessage>  swing_trajectory_;
   /**
            * In case the trajectory type is set to TRAJECTORY_TYPE_WAYPOINTS, this value can be used to specify the trajectory blend duration in seconds.
            * If greater than zero, waypoints that fall within the valid time window (beginning at the start of the swing phase and spanning the desired blend duration)
            * will be adjusted to account for the initial error between the actual and expected position and orientation of the swing foot.
            * Note that the expected_initial_location and expected_initial_orientation fields must be defined in order to enable trajectory blending.
            */
   public double swing_trajectory_blend_duration_;
   /**
            * The swingDuration is the time a foot is not in ground contact during a step.
            * If the value of this field is invalid (not positive) it will be replaced by a default swing_duration.
            */
   public double swing_duration_ = -1.0;
   /**
            * The transferDuration is the time spent with the feet in ground contact before a step.
            * If the value of this field is invalid (not positive) it will be replaced by a default transfer_duration.
            */
   public double transfer_duration_ = -1.0;
   /**
            * The time to delay this command on the controller side before being executed.
            */
   public double execution_delay_time_;
   /**
            * The swing_duration_shift_fraction is the fraction of the swing duration spent shifting the weight from the heel of the foot to the toe of the foot.
            * A higher split fraction means that the weight is shifted to the toe slowly, then spends very little time on the toe.
            * A lower split fraction means that the weight is shifted to the toe quickly, then spends a long time on the toe.
            */
   public double swing_duration_shift_fraction_ = -1.0;
   /**
            * The swing_split_fraction is the fraction of the shift portion of swing duration spent shifting the weight from the heel of the foot to the ball of the foot.
            * A higher split fraction means that the weight is shifted to the ball slowly, then to the toe quickly.
            * A lower split fraction means that the weight is shifted to the ball quickly, then to the toe slowly.
            */
   public double swing_split_fraction_ = -1.0;
   /**
            * The transfer_split_fraction is the fraction of the transfer duration spent shifting the weight from the trailing foot to the middle of the stance.
            * A higher split fraction means that the weight is shifted to the center slowly, then to the upcoming support foot quickly.
            * A lower split fraction means that the weight is shifted to the center quickly, then to the upcoming support foot slowly.
            */
   public double transfer_split_fraction_ = -1.0;
   /**
            * Time spent after touchdown to transition from heel or toe support to full foot support. Note, that this only has an
            * effect if the foot touches down non-flat. More specific: the foot pitch (in sole z-up frame) at touchdown must be
            * different from the pitch of the foothold pose provided in this message.
            */
   public double touchdown_duration_ = -1.0;
   /**
            * Time spent in toe or heel support before the step. This duration is part of the transfer duration and must therefore
            * be shorter then the transfer. Note, that this only has an effect if the swing trajectory is provided, the swing trajectory
            * has its first waypoint at time 0.0, and the pitch of the first swing waypoint (in sole z-up frame) is different from the
            * foot pitch.
            */
   public double liftoff_duration_ = -1.0;

   public FootstepDataMessage()
   {
      location_ = new us.ihmc.euclid.tuple3D.Point3D();
      orientation_ = new us.ihmc.euclid.tuple4D.Quaternion();
      predicted_contact_points_2d_ = new us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D> (10, new geometry_msgs.msg.dds.PointPubSubType());
      custom_waypoint_proportions_ = new us.ihmc.idl.IDLSequence.Double (2, "type_6");

      custom_position_waypoints_ = new us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D> (10, new geometry_msgs.msg.dds.PointPubSubType());
      swing_trajectory_ = new us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.SE3TrajectoryPointMessage> (10, new controller_msgs.msg.dds.SE3TrajectoryPointMessagePubSubType());

   }

   public FootstepDataMessage(FootstepDataMessage other)
   {
      this();
      set(other);
   }

   public void set(FootstepDataMessage other)
   {
      sequence_id_ = other.sequence_id_;

      robot_side_ = other.robot_side_;

      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.location_, location_);
      geometry_msgs.msg.dds.QuaternionPubSubType.staticCopy(other.orientation_, orientation_);
      predicted_contact_points_2d_.set(other.predicted_contact_points_2d_);
      trajectory_type_ = other.trajectory_type_;

      swing_height_ = other.swing_height_;

      custom_waypoint_proportions_.set(other.custom_waypoint_proportions_);
      custom_position_waypoints_.set(other.custom_position_waypoints_);
      swing_trajectory_.set(other.swing_trajectory_);
      swing_trajectory_blend_duration_ = other.swing_trajectory_blend_duration_;

      swing_duration_ = other.swing_duration_;

      transfer_duration_ = other.transfer_duration_;

      execution_delay_time_ = other.execution_delay_time_;

      swing_duration_shift_fraction_ = other.swing_duration_shift_fraction_;

      swing_split_fraction_ = other.swing_split_fraction_;

      transfer_split_fraction_ = other.transfer_split_fraction_;

      touchdown_duration_ = other.touchdown_duration_;

      liftoff_duration_ = other.liftoff_duration_;

   }

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public void setSequenceId(long sequence_id)
   {
      sequence_id_ = sequence_id;
   }
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long getSequenceId()
   {
      return sequence_id_;
   }

   /**
            * Specifies which foot will swing to reach the footstep.
            */
   public void setRobotSide(byte robot_side)
   {
      robot_side_ = robot_side;
   }
   /**
            * Specifies which foot will swing to reach the footstep.
            */
   public byte getRobotSide()
   {
      return robot_side_;
   }


   /**
            * Specifies the position of the footstep (sole frame) in world frame.
            */
   public us.ihmc.euclid.tuple3D.Point3D getLocation()
   {
      return location_;
   }


   /**
            * Specifies the orientation of the footstep (sole frame) in world frame.
            */
   public us.ihmc.euclid.tuple4D.Quaternion getOrientation()
   {
      return orientation_;
   }


   /**
            * Predicted contact points represent the vertices of the expected contact polygon between the foot and the world.
            * An empty list will request the controller to use the default foot support polygon.
            * Contact points  are expressed in sole frame. The ordering does not matter.
            * For example: to tell the controller to use the entire foot, the predicted contact points would be:
            * - x: 0.5 * foot_length, y: -0.5 * toe_width
            * - x: 0.5 * foot_length, y: 0.5 * toe_width
            * - x: -0.5 * foot_length, y: -0.5 * heel_width
            * - x: -0.5 * foot_length, y: 0.5 * heel_width
            * Note: The z coordinate of each point is ignored.
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D>  getPredictedContactPoints2d()
   {
      return predicted_contact_points_2d_;
   }

   /**
            * This contains information on what the swing trajectory should be for each step. Recommended is TRAJECTORY_TYPE_DEFAULT.
            */
   public void setTrajectoryType(byte trajectory_type)
   {
      trajectory_type_ = trajectory_type;
   }
   /**
            * This contains information on what the swing trajectory should be for each step. Recommended is TRAJECTORY_TYPE_DEFAULT.
            */
   public byte getTrajectoryType()
   {
      return trajectory_type_;
   }

   /**
            * Contains information on how high the robot should swing its foot.
            * This affects trajectory types TRAJECTORY_TYPE_DEFAULT and TRAJECTORY_TYPE_OBSTACLE_CLEARANCE.
            * If a value smaller then the minimal swing height is chosen (e.g. 0.0) the swing height will be changed to a default value.
            */
   public void setSwingHeight(double swing_height)
   {
      swing_height_ = swing_height;
   }
   /**
            * Contains information on how high the robot should swing its foot.
            * This affects trajectory types TRAJECTORY_TYPE_DEFAULT and TRAJECTORY_TYPE_OBSTACLE_CLEARANCE.
            * If a value smaller then the minimal swing height is chosen (e.g. 0.0) the swing height will be changed to a default value.
            */
   public double getSwingHeight()
   {
      return swing_height_;
   }


   /**
            * In case the trajectory type is set to TRAJECTORY_TYPE_DEFAULT or TRAJECTORY_TYPE_OBSTACLE_CLEARANCE, custom waypoint proportions
            * can be requested. These proportions encode the xy positions of the swing trajectory's two waypoints. A proportion of 0.0 and 1.0 will
            * place a waypoint's xy-position at the start and end of the trajectory, respectively. If this value is empty, the default proportions are used.
            */
   public us.ihmc.idl.IDLSequence.Double  getCustomWaypointProportions()
   {
      return custom_waypoint_proportions_;
   }


   /**
            * In case the trajectory type is set to TRAJECTORY_TYPE_CUSTOM two swing waypoints can be specified here.
            * The waypoints define sole positions.
            * The controller will compute times and velocities at the waypoints.
            * This is a convenient way to shape the trajectory of the swing.
            * If full control over the swing trajectory is desired use the trajectory type TRAJECTORY_TYPE_WAYPOINTS instead.
            * The position waypoints are expected in the trajectory frame.
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D>  getCustomPositionWaypoints()
   {
      return custom_position_waypoints_;
   }


   /**
            * In case the trajectory type is set to TRAJECTORY_TYPE_WAYPOINTS, swing waypoints can be specified here.
            * The waypoints do not include the start point (which is set to the current foot state at lift-off) and the touch down point
            * (which is specified by the location and orientation fields).
            * All waypoints are for the sole frame and expressed in the trajectory frame.
            * The maximum number of points can be found in the Footstep class.
            */
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.SE3TrajectoryPointMessage>  getSwingTrajectory()
   {
      return swing_trajectory_;
   }

   /**
            * In case the trajectory type is set to TRAJECTORY_TYPE_WAYPOINTS, this value can be used to specify the trajectory blend duration in seconds.
            * If greater than zero, waypoints that fall within the valid time window (beginning at the start of the swing phase and spanning the desired blend duration)
            * will be adjusted to account for the initial error between the actual and expected position and orientation of the swing foot.
            * Note that the expected_initial_location and expected_initial_orientation fields must be defined in order to enable trajectory blending.
            */
   public void setSwingTrajectoryBlendDuration(double swing_trajectory_blend_duration)
   {
      swing_trajectory_blend_duration_ = swing_trajectory_blend_duration;
   }
   /**
            * In case the trajectory type is set to TRAJECTORY_TYPE_WAYPOINTS, this value can be used to specify the trajectory blend duration in seconds.
            * If greater than zero, waypoints that fall within the valid time window (beginning at the start of the swing phase and spanning the desired blend duration)
            * will be adjusted to account for the initial error between the actual and expected position and orientation of the swing foot.
            * Note that the expected_initial_location and expected_initial_orientation fields must be defined in order to enable trajectory blending.
            */
   public double getSwingTrajectoryBlendDuration()
   {
      return swing_trajectory_blend_duration_;
   }

   /**
            * The swingDuration is the time a foot is not in ground contact during a step.
            * If the value of this field is invalid (not positive) it will be replaced by a default swing_duration.
            */
   public void setSwingDuration(double swing_duration)
   {
      swing_duration_ = swing_duration;
   }
   /**
            * The swingDuration is the time a foot is not in ground contact during a step.
            * If the value of this field is invalid (not positive) it will be replaced by a default swing_duration.
            */
   public double getSwingDuration()
   {
      return swing_duration_;
   }

   /**
            * The transferDuration is the time spent with the feet in ground contact before a step.
            * If the value of this field is invalid (not positive) it will be replaced by a default transfer_duration.
            */
   public void setTransferDuration(double transfer_duration)
   {
      transfer_duration_ = transfer_duration;
   }
   /**
            * The transferDuration is the time spent with the feet in ground contact before a step.
            * If the value of this field is invalid (not positive) it will be replaced by a default transfer_duration.
            */
   public double getTransferDuration()
   {
      return transfer_duration_;
   }

   /**
            * The time to delay this command on the controller side before being executed.
            */
   public void setExecutionDelayTime(double execution_delay_time)
   {
      execution_delay_time_ = execution_delay_time;
   }
   /**
            * The time to delay this command on the controller side before being executed.
            */
   public double getExecutionDelayTime()
   {
      return execution_delay_time_;
   }

   /**
            * The swing_duration_shift_fraction is the fraction of the swing duration spent shifting the weight from the heel of the foot to the toe of the foot.
            * A higher split fraction means that the weight is shifted to the toe slowly, then spends very little time on the toe.
            * A lower split fraction means that the weight is shifted to the toe quickly, then spends a long time on the toe.
            */
   public void setSwingDurationShiftFraction(double swing_duration_shift_fraction)
   {
      swing_duration_shift_fraction_ = swing_duration_shift_fraction;
   }
   /**
            * The swing_duration_shift_fraction is the fraction of the swing duration spent shifting the weight from the heel of the foot to the toe of the foot.
            * A higher split fraction means that the weight is shifted to the toe slowly, then spends very little time on the toe.
            * A lower split fraction means that the weight is shifted to the toe quickly, then spends a long time on the toe.
            */
   public double getSwingDurationShiftFraction()
   {
      return swing_duration_shift_fraction_;
   }

   /**
            * The swing_split_fraction is the fraction of the shift portion of swing duration spent shifting the weight from the heel of the foot to the ball of the foot.
            * A higher split fraction means that the weight is shifted to the ball slowly, then to the toe quickly.
            * A lower split fraction means that the weight is shifted to the ball quickly, then to the toe slowly.
            */
   public void setSwingSplitFraction(double swing_split_fraction)
   {
      swing_split_fraction_ = swing_split_fraction;
   }
   /**
            * The swing_split_fraction is the fraction of the shift portion of swing duration spent shifting the weight from the heel of the foot to the ball of the foot.
            * A higher split fraction means that the weight is shifted to the ball slowly, then to the toe quickly.
            * A lower split fraction means that the weight is shifted to the ball quickly, then to the toe slowly.
            */
   public double getSwingSplitFraction()
   {
      return swing_split_fraction_;
   }

   /**
            * The transfer_split_fraction is the fraction of the transfer duration spent shifting the weight from the trailing foot to the middle of the stance.
            * A higher split fraction means that the weight is shifted to the center slowly, then to the upcoming support foot quickly.
            * A lower split fraction means that the weight is shifted to the center quickly, then to the upcoming support foot slowly.
            */
   public void setTransferSplitFraction(double transfer_split_fraction)
   {
      transfer_split_fraction_ = transfer_split_fraction;
   }
   /**
            * The transfer_split_fraction is the fraction of the transfer duration spent shifting the weight from the trailing foot to the middle of the stance.
            * A higher split fraction means that the weight is shifted to the center slowly, then to the upcoming support foot quickly.
            * A lower split fraction means that the weight is shifted to the center quickly, then to the upcoming support foot slowly.
            */
   public double getTransferSplitFraction()
   {
      return transfer_split_fraction_;
   }

   /**
            * Time spent after touchdown to transition from heel or toe support to full foot support. Note, that this only has an
            * effect if the foot touches down non-flat. More specific: the foot pitch (in sole z-up frame) at touchdown must be
            * different from the pitch of the foothold pose provided in this message.
            */
   public void setTouchdownDuration(double touchdown_duration)
   {
      touchdown_duration_ = touchdown_duration;
   }
   /**
            * Time spent after touchdown to transition from heel or toe support to full foot support. Note, that this only has an
            * effect if the foot touches down non-flat. More specific: the foot pitch (in sole z-up frame) at touchdown must be
            * different from the pitch of the foothold pose provided in this message.
            */
   public double getTouchdownDuration()
   {
      return touchdown_duration_;
   }

   /**
            * Time spent in toe or heel support before the step. This duration is part of the transfer duration and must therefore
            * be shorter then the transfer. Note, that this only has an effect if the swing trajectory is provided, the swing trajectory
            * has its first waypoint at time 0.0, and the pitch of the first swing waypoint (in sole z-up frame) is different from the
            * foot pitch.
            */
   public void setLiftoffDuration(double liftoff_duration)
   {
      liftoff_duration_ = liftoff_duration;
   }
   /**
            * Time spent in toe or heel support before the step. This duration is part of the transfer duration and must therefore
            * be shorter then the transfer. Note, that this only has an effect if the swing trajectory is provided, the swing trajectory
            * has its first waypoint at time 0.0, and the pitch of the first swing waypoint (in sole z-up frame) is different from the
            * foot pitch.
            */
   public double getLiftoffDuration()
   {
      return liftoff_duration_;
   }


   public static Supplier<FootstepDataMessagePubSubType> getPubSubType()
   {
      return FootstepDataMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return FootstepDataMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(FootstepDataMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.robot_side_, other.robot_side_, epsilon)) return false;

      if (!this.location_.epsilonEquals(other.location_, epsilon)) return false;
      if (!this.orientation_.epsilonEquals(other.orientation_, epsilon)) return false;
      if (this.predicted_contact_points_2d_.size() != other.predicted_contact_points_2d_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.predicted_contact_points_2d_.size(); i++)
         {  if (!this.predicted_contact_points_2d_.get(i).epsilonEquals(other.predicted_contact_points_2d_.get(i), epsilon)) return false; }
      }

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.trajectory_type_, other.trajectory_type_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.swing_height_, other.swing_height_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsDoubleSequence(this.custom_waypoint_proportions_, other.custom_waypoint_proportions_, epsilon)) return false;

      if (this.custom_position_waypoints_.size() != other.custom_position_waypoints_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.custom_position_waypoints_.size(); i++)
         {  if (!this.custom_position_waypoints_.get(i).epsilonEquals(other.custom_position_waypoints_.get(i), epsilon)) return false; }
      }

      if (this.swing_trajectory_.size() != other.swing_trajectory_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.swing_trajectory_.size(); i++)
         {  if (!this.swing_trajectory_.get(i).epsilonEquals(other.swing_trajectory_.get(i), epsilon)) return false; }
      }

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.swing_trajectory_blend_duration_, other.swing_trajectory_blend_duration_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.swing_duration_, other.swing_duration_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.transfer_duration_, other.transfer_duration_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.execution_delay_time_, other.execution_delay_time_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.swing_duration_shift_fraction_, other.swing_duration_shift_fraction_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.swing_split_fraction_, other.swing_split_fraction_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.transfer_split_fraction_, other.transfer_split_fraction_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.touchdown_duration_, other.touchdown_duration_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.liftoff_duration_, other.liftoff_duration_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof FootstepDataMessage)) return false;

      FootstepDataMessage otherMyClass = (FootstepDataMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.robot_side_ != otherMyClass.robot_side_) return false;

      if (!this.location_.equals(otherMyClass.location_)) return false;
      if (!this.orientation_.equals(otherMyClass.orientation_)) return false;
      if (!this.predicted_contact_points_2d_.equals(otherMyClass.predicted_contact_points_2d_)) return false;
      if(this.trajectory_type_ != otherMyClass.trajectory_type_) return false;

      if(this.swing_height_ != otherMyClass.swing_height_) return false;

      if (!this.custom_waypoint_proportions_.equals(otherMyClass.custom_waypoint_proportions_)) return false;
      if (!this.custom_position_waypoints_.equals(otherMyClass.custom_position_waypoints_)) return false;
      if (!this.swing_trajectory_.equals(otherMyClass.swing_trajectory_)) return false;
      if(this.swing_trajectory_blend_duration_ != otherMyClass.swing_trajectory_blend_duration_) return false;

      if(this.swing_duration_ != otherMyClass.swing_duration_) return false;

      if(this.transfer_duration_ != otherMyClass.transfer_duration_) return false;

      if(this.execution_delay_time_ != otherMyClass.execution_delay_time_) return false;

      if(this.swing_duration_shift_fraction_ != otherMyClass.swing_duration_shift_fraction_) return false;

      if(this.swing_split_fraction_ != otherMyClass.swing_split_fraction_) return false;

      if(this.transfer_split_fraction_ != otherMyClass.transfer_split_fraction_) return false;

      if(this.touchdown_duration_ != otherMyClass.touchdown_duration_) return false;

      if(this.liftoff_duration_ != otherMyClass.liftoff_duration_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("FootstepDataMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("robot_side=");
      builder.append(this.robot_side_);      builder.append(", ");
      builder.append("location=");
      builder.append(this.location_);      builder.append(", ");
      builder.append("orientation=");
      builder.append(this.orientation_);      builder.append(", ");
      builder.append("predicted_contact_points_2d=");
      builder.append(this.predicted_contact_points_2d_);      builder.append(", ");
      builder.append("trajectory_type=");
      builder.append(this.trajectory_type_);      builder.append(", ");
      builder.append("swing_height=");
      builder.append(this.swing_height_);      builder.append(", ");
      builder.append("custom_waypoint_proportions=");
      builder.append(this.custom_waypoint_proportions_);      builder.append(", ");
      builder.append("custom_position_waypoints=");
      builder.append(this.custom_position_waypoints_);      builder.append(", ");
      builder.append("swing_trajectory=");
      builder.append(this.swing_trajectory_);      builder.append(", ");
      builder.append("swing_trajectory_blend_duration=");
      builder.append(this.swing_trajectory_blend_duration_);      builder.append(", ");
      builder.append("swing_duration=");
      builder.append(this.swing_duration_);      builder.append(", ");
      builder.append("transfer_duration=");
      builder.append(this.transfer_duration_);      builder.append(", ");
      builder.append("execution_delay_time=");
      builder.append(this.execution_delay_time_);      builder.append(", ");
      builder.append("swing_duration_shift_fraction=");
      builder.append(this.swing_duration_shift_fraction_);      builder.append(", ");
      builder.append("swing_split_fraction=");
      builder.append(this.swing_split_fraction_);      builder.append(", ");
      builder.append("transfer_split_fraction=");
      builder.append(this.transfer_split_fraction_);      builder.append(", ");
      builder.append("touchdown_duration=");
      builder.append(this.touchdown_duration_);      builder.append(", ");
      builder.append("liftoff_duration=");
      builder.append(this.liftoff_duration_);
      builder.append("}");
      return builder.toString();
   }
}
