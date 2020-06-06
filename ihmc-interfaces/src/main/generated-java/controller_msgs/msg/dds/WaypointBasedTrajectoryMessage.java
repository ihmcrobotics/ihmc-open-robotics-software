package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * Configure a constrained trajectory for a given end-effector.
       * Main usage is the IHMC WholeBodyTrajectoryToolbox.
       */
public class WaypointBasedTrajectoryMessage extends Packet<WaypointBasedTrajectoryMessage> implements Settable<WaypointBasedTrajectoryMessage>, EpsilonComparable<WaypointBasedTrajectoryMessage>
{

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   public int end_effector_hash_code_;

   public us.ihmc.idl.IDLSequence.Double  waypoint_times_;

   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.geometry.Pose3D>  waypoints_;

   public controller_msgs.msg.dds.SelectionMatrix3DMessage angular_selection_matrix_;

   public controller_msgs.msg.dds.SelectionMatrix3DMessage linear_selection_matrix_;

   public us.ihmc.euclid.tuple3D.Point3D control_frame_position_in_end_effector_;

   public us.ihmc.euclid.tuple4D.Quaternion control_frame_orientation_in_end_effector_;

   public double weight_ = -1.0;

   public WaypointBasedTrajectoryMessage()
   {



      waypoint_times_ = new us.ihmc.idl.IDLSequence.Double (100, "type_6");


      waypoints_ = new us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.geometry.Pose3D> (100, new geometry_msgs.msg.dds.PosePubSubType());

      angular_selection_matrix_ = new controller_msgs.msg.dds.SelectionMatrix3DMessage();

      linear_selection_matrix_ = new controller_msgs.msg.dds.SelectionMatrix3DMessage();

      control_frame_position_in_end_effector_ = new us.ihmc.euclid.tuple3D.Point3D();

      control_frame_orientation_in_end_effector_ = new us.ihmc.euclid.tuple4D.Quaternion();


   }

   public WaypointBasedTrajectoryMessage(WaypointBasedTrajectoryMessage other)
   {
      this();
      set(other);
   }

   public void set(WaypointBasedTrajectoryMessage other)
   {

      sequence_id_ = other.sequence_id_;


      end_effector_hash_code_ = other.end_effector_hash_code_;


      waypoint_times_.set(other.waypoint_times_);

      waypoints_.set(other.waypoints_);

      controller_msgs.msg.dds.SelectionMatrix3DMessagePubSubType.staticCopy(other.angular_selection_matrix_, angular_selection_matrix_);

      controller_msgs.msg.dds.SelectionMatrix3DMessagePubSubType.staticCopy(other.linear_selection_matrix_, linear_selection_matrix_);

      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.control_frame_position_in_end_effector_, control_frame_position_in_end_effector_);

      geometry_msgs.msg.dds.QuaternionPubSubType.staticCopy(other.control_frame_orientation_in_end_effector_, control_frame_orientation_in_end_effector_);

      weight_ = other.weight_;

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


   public void setEndEffectorHashCode(int end_effector_hash_code)
   {
      end_effector_hash_code_ = end_effector_hash_code;
   }
   public int getEndEffectorHashCode()
   {
      return end_effector_hash_code_;
   }



   public us.ihmc.idl.IDLSequence.Double  getWaypointTimes()
   {
      return waypoint_times_;
   }



   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.geometry.Pose3D>  getWaypoints()
   {
      return waypoints_;
   }



   public controller_msgs.msg.dds.SelectionMatrix3DMessage getAngularSelectionMatrix()
   {
      return angular_selection_matrix_;
   }



   public controller_msgs.msg.dds.SelectionMatrix3DMessage getLinearSelectionMatrix()
   {
      return linear_selection_matrix_;
   }



   public us.ihmc.euclid.tuple3D.Point3D getControlFramePositionInEndEffector()
   {
      return control_frame_position_in_end_effector_;
   }



   public us.ihmc.euclid.tuple4D.Quaternion getControlFrameOrientationInEndEffector()
   {
      return control_frame_orientation_in_end_effector_;
   }


   public void setWeight(double weight)
   {
      weight_ = weight;
   }
   public double getWeight()
   {
      return weight_;
   }


   public static Supplier<WaypointBasedTrajectoryMessagePubSubType> getPubSubType()
   {
      return WaypointBasedTrajectoryMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return WaypointBasedTrajectoryMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(WaypointBasedTrajectoryMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.end_effector_hash_code_, other.end_effector_hash_code_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsDoubleSequence(this.waypoint_times_, other.waypoint_times_, epsilon)) return false;


      if (this.waypoints_.size() != other.waypoints_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.waypoints_.size(); i++)
         {  if (!this.waypoints_.get(i).epsilonEquals(other.waypoints_.get(i), epsilon)) return false; }
      }


      if (!this.angular_selection_matrix_.epsilonEquals(other.angular_selection_matrix_, epsilon)) return false;

      if (!this.linear_selection_matrix_.epsilonEquals(other.linear_selection_matrix_, epsilon)) return false;

      if (!this.control_frame_position_in_end_effector_.epsilonEquals(other.control_frame_position_in_end_effector_, epsilon)) return false;

      if (!this.control_frame_orientation_in_end_effector_.epsilonEquals(other.control_frame_orientation_in_end_effector_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.weight_, other.weight_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof WaypointBasedTrajectoryMessage)) return false;

      WaypointBasedTrajectoryMessage otherMyClass = (WaypointBasedTrajectoryMessage) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if(this.end_effector_hash_code_ != otherMyClass.end_effector_hash_code_) return false;


      if (!this.waypoint_times_.equals(otherMyClass.waypoint_times_)) return false;

      if (!this.waypoints_.equals(otherMyClass.waypoints_)) return false;

      if (!this.angular_selection_matrix_.equals(otherMyClass.angular_selection_matrix_)) return false;

      if (!this.linear_selection_matrix_.equals(otherMyClass.linear_selection_matrix_)) return false;

      if (!this.control_frame_position_in_end_effector_.equals(otherMyClass.control_frame_position_in_end_effector_)) return false;

      if (!this.control_frame_orientation_in_end_effector_.equals(otherMyClass.control_frame_orientation_in_end_effector_)) return false;

      if(this.weight_ != otherMyClass.weight_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("WaypointBasedTrajectoryMessage {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("end_effector_hash_code=");
      builder.append(this.end_effector_hash_code_);      builder.append(", ");

      builder.append("waypoint_times=");
      builder.append(this.waypoint_times_);      builder.append(", ");

      builder.append("waypoints=");
      builder.append(this.waypoints_);      builder.append(", ");

      builder.append("angular_selection_matrix=");
      builder.append(this.angular_selection_matrix_);      builder.append(", ");

      builder.append("linear_selection_matrix=");
      builder.append(this.linear_selection_matrix_);      builder.append(", ");

      builder.append("control_frame_position_in_end_effector=");
      builder.append(this.control_frame_position_in_end_effector_);      builder.append(", ");

      builder.append("control_frame_orientation_in_end_effector=");
      builder.append(this.control_frame_orientation_in_end_effector_);      builder.append(", ");

      builder.append("weight=");
      builder.append(this.weight_);
      builder.append("}");
      return builder.toString();
   }
}
