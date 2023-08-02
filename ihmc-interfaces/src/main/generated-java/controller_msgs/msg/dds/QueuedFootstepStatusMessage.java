package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC whole-body controller API.
       * This message specifies the position, orientation and side (left or right) of a footstep in the queue in world frame.
       */
public class QueuedFootstepStatusMessage extends Packet<QueuedFootstepStatusMessage> implements Settable<QueuedFootstepStatusMessage>, EpsilonComparable<QueuedFootstepStatusMessage>
{
   public static final byte ROBOT_SIDE_LEFT = (byte) 0;
   public static final byte ROBOT_SIDE_RIGHT = (byte) 1;
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
            * Step constraint regions to be used by this footstep
            */
   public controller_msgs.msg.dds.StepConstraintsListMessage step_constraints_;

   public QueuedFootstepStatusMessage()
   {
      location_ = new us.ihmc.euclid.tuple3D.Point3D();
      orientation_ = new us.ihmc.euclid.tuple4D.Quaternion();
      predicted_contact_points_2d_ = new us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D> (10, new geometry_msgs.msg.dds.PointPubSubType());
      step_constraints_ = new controller_msgs.msg.dds.StepConstraintsListMessage();

   }

   public QueuedFootstepStatusMessage(QueuedFootstepStatusMessage other)
   {
      this();
      set(other);
   }

   public void set(QueuedFootstepStatusMessage other)
   {
      sequence_id_ = other.sequence_id_;

      robot_side_ = other.robot_side_;

      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.location_, location_);
      geometry_msgs.msg.dds.QuaternionPubSubType.staticCopy(other.orientation_, orientation_);
      predicted_contact_points_2d_.set(other.predicted_contact_points_2d_);
      swing_duration_ = other.swing_duration_;

      transfer_duration_ = other.transfer_duration_;

      controller_msgs.msg.dds.StepConstraintsListMessagePubSubType.staticCopy(other.step_constraints_, step_constraints_);
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
            * Step constraint regions to be used by this footstep
            */
   public controller_msgs.msg.dds.StepConstraintsListMessage getStepConstraints()
   {
      return step_constraints_;
   }


   public static Supplier<QueuedFootstepStatusMessagePubSubType> getPubSubType()
   {
      return QueuedFootstepStatusMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return QueuedFootstepStatusMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(QueuedFootstepStatusMessage other, double epsilon)
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

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.swing_duration_, other.swing_duration_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.transfer_duration_, other.transfer_duration_, epsilon)) return false;

      if (!this.step_constraints_.epsilonEquals(other.step_constraints_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof QueuedFootstepStatusMessage)) return false;

      QueuedFootstepStatusMessage otherMyClass = (QueuedFootstepStatusMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.robot_side_ != otherMyClass.robot_side_) return false;

      if (!this.location_.equals(otherMyClass.location_)) return false;
      if (!this.orientation_.equals(otherMyClass.orientation_)) return false;
      if (!this.predicted_contact_points_2d_.equals(otherMyClass.predicted_contact_points_2d_)) return false;
      if(this.swing_duration_ != otherMyClass.swing_duration_) return false;

      if(this.transfer_duration_ != otherMyClass.transfer_duration_) return false;

      if (!this.step_constraints_.equals(otherMyClass.step_constraints_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("QueuedFootstepStatusMessage {");
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
      builder.append("swing_duration=");
      builder.append(this.swing_duration_);      builder.append(", ");
      builder.append("transfer_duration=");
      builder.append(this.transfer_duration_);      builder.append(", ");
      builder.append("step_constraints=");
      builder.append(this.step_constraints_);
      builder.append("}");
      return builder.toString();
   }
}
