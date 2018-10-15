package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class KinematicsPlanningToolboxMessage extends Packet<KinematicsPlanningToolboxMessage> implements Settable<KinematicsPlanningToolboxMessage>, EpsilonComparable<KinematicsPlanningToolboxMessage>
{
   public long sequence_id_;
   public long end_effector_name_based_hash_code_;
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.geometry.Pose3D>  key_frame_poses_;
   public controller_msgs.msg.dds.SelectionMatrix3DMessage angular_selection_matrix_;
   public controller_msgs.msg.dds.SelectionMatrix3DMessage linear_selection_matrix_;
   public us.ihmc.euclid.tuple3D.Point3D control_frame_position_in_end_effector_;
   public us.ihmc.euclid.tuple4D.Quaternion control_frame_orientation_in_end_effector_;
   public double allowable_displacement_;

   public KinematicsPlanningToolboxMessage()
   {
      key_frame_poses_ = new us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.geometry.Pose3D> (100, new geometry_msgs.msg.dds.PosePubSubType());
      angular_selection_matrix_ = new controller_msgs.msg.dds.SelectionMatrix3DMessage();
      linear_selection_matrix_ = new controller_msgs.msg.dds.SelectionMatrix3DMessage();
      control_frame_position_in_end_effector_ = new us.ihmc.euclid.tuple3D.Point3D();
      control_frame_orientation_in_end_effector_ = new us.ihmc.euclid.tuple4D.Quaternion();

   }

   public KinematicsPlanningToolboxMessage(KinematicsPlanningToolboxMessage other)
   {
      this();
      set(other);
   }

   public void set(KinematicsPlanningToolboxMessage other)
   {
      sequence_id_ = other.sequence_id_;

      end_effector_name_based_hash_code_ = other.end_effector_name_based_hash_code_;

      key_frame_poses_.set(other.key_frame_poses_);
      controller_msgs.msg.dds.SelectionMatrix3DMessagePubSubType.staticCopy(other.angular_selection_matrix_, angular_selection_matrix_);
      controller_msgs.msg.dds.SelectionMatrix3DMessagePubSubType.staticCopy(other.linear_selection_matrix_, linear_selection_matrix_);
      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.control_frame_position_in_end_effector_, control_frame_position_in_end_effector_);
      geometry_msgs.msg.dds.QuaternionPubSubType.staticCopy(other.control_frame_orientation_in_end_effector_, control_frame_orientation_in_end_effector_);
      allowable_displacement_ = other.allowable_displacement_;

   }

   public void setSequenceId(long sequence_id)
   {
      sequence_id_ = sequence_id;
   }
   public long getSequenceId()
   {
      return sequence_id_;
   }

   public void setEndEffectorNameBasedHashCode(long end_effector_name_based_hash_code)
   {
      end_effector_name_based_hash_code_ = end_effector_name_based_hash_code;
   }
   public long getEndEffectorNameBasedHashCode()
   {
      return end_effector_name_based_hash_code_;
   }


   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.geometry.Pose3D>  getKeyFramePoses()
   {
      return key_frame_poses_;
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

   public void setAllowableDisplacement(double allowable_displacement)
   {
      allowable_displacement_ = allowable_displacement;
   }
   public double getAllowableDisplacement()
   {
      return allowable_displacement_;
   }


   public static Supplier<KinematicsPlanningToolboxMessagePubSubType> getPubSubType()
   {
      return KinematicsPlanningToolboxMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return KinematicsPlanningToolboxMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(KinematicsPlanningToolboxMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.end_effector_name_based_hash_code_, other.end_effector_name_based_hash_code_, epsilon)) return false;

      if (this.key_frame_poses_.size() != other.key_frame_poses_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.key_frame_poses_.size(); i++)
         {  if (!this.key_frame_poses_.get(i).epsilonEquals(other.key_frame_poses_.get(i), epsilon)) return false; }
      }

      if (!this.angular_selection_matrix_.epsilonEquals(other.angular_selection_matrix_, epsilon)) return false;
      if (!this.linear_selection_matrix_.epsilonEquals(other.linear_selection_matrix_, epsilon)) return false;
      if (!this.control_frame_position_in_end_effector_.epsilonEquals(other.control_frame_position_in_end_effector_, epsilon)) return false;
      if (!this.control_frame_orientation_in_end_effector_.epsilonEquals(other.control_frame_orientation_in_end_effector_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.allowable_displacement_, other.allowable_displacement_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof KinematicsPlanningToolboxMessage)) return false;

      KinematicsPlanningToolboxMessage otherMyClass = (KinematicsPlanningToolboxMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.end_effector_name_based_hash_code_ != otherMyClass.end_effector_name_based_hash_code_) return false;

      if (!this.key_frame_poses_.equals(otherMyClass.key_frame_poses_)) return false;
      if (!this.angular_selection_matrix_.equals(otherMyClass.angular_selection_matrix_)) return false;
      if (!this.linear_selection_matrix_.equals(otherMyClass.linear_selection_matrix_)) return false;
      if (!this.control_frame_position_in_end_effector_.equals(otherMyClass.control_frame_position_in_end_effector_)) return false;
      if (!this.control_frame_orientation_in_end_effector_.equals(otherMyClass.control_frame_orientation_in_end_effector_)) return false;
      if(this.allowable_displacement_ != otherMyClass.allowable_displacement_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("KinematicsPlanningToolboxMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("end_effector_name_based_hash_code=");
      builder.append(this.end_effector_name_based_hash_code_);      builder.append(", ");
      builder.append("key_frame_poses=");
      builder.append(this.key_frame_poses_);      builder.append(", ");
      builder.append("angular_selection_matrix=");
      builder.append(this.angular_selection_matrix_);      builder.append(", ");
      builder.append("linear_selection_matrix=");
      builder.append(this.linear_selection_matrix_);      builder.append(", ");
      builder.append("control_frame_position_in_end_effector=");
      builder.append(this.control_frame_position_in_end_effector_);      builder.append(", ");
      builder.append("control_frame_orientation_in_end_effector=");
      builder.append(this.control_frame_orientation_in_end_effector_);      builder.append(", ");
      builder.append("allowable_displacement=");
      builder.append(this.allowable_displacement_);
      builder.append("}");
      return builder.toString();
   }
}
