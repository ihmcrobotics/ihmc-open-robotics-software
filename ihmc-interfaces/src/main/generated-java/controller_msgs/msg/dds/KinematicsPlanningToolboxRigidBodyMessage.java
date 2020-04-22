package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class KinematicsPlanningToolboxRigidBodyMessage extends Packet<KinematicsPlanningToolboxRigidBodyMessage> implements Settable<KinematicsPlanningToolboxRigidBodyMessage>, EpsilonComparable<KinematicsPlanningToolboxRigidBodyMessage>
{

   /**
            * This message is part of the IHMC whole-body inverse kinematics module.
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   public int end_effector_hash_code_;

   /**
            * This is the list of desired times for each key frames.
            */
   public us.ihmc.idl.IDLSequence.Double  key_frame_times_;

   /**
            * This is the list of desired key frames for end effector.
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.geometry.Pose3D>  key_frame_poses_;

   /**
            * The selection frames coming along with the given selection matrix are used to determine to
            * what reference frame the selected axes are referring to. For instance, if only the hand height
            * in world should be controlled on the linear z component of the selection matrix should be
            * selected and the reference frame should be world frame. When no reference frame is provided
            * with the selection matrix, it will be used as it is in the control frame, i.e. the body-fixed
            * frame if not defined otherwise.
            */
   public controller_msgs.msg.dds.SelectionMatrix3DMessage angular_selection_matrix_;

   /**
            * The selection matrix is used to determinate which degree of freedom of the end-effector should
            * be controlled.
            * The selection frames coming along with the given selection matrix are used to determine to
            * what reference frame the selected axes are referring to. For instance, if only the hand height
            * in world should be controlled on the linear z component of the selection matrix should be
            * selected and the reference frame should be world frame. When no reference frame is provided
            * with the selection matrix, it will be used as it is in the control frame, i.e. the body-fixed
            * frame if not defined otherwise.
            */
   public controller_msgs.msg.dds.SelectionMatrix3DMessage linear_selection_matrix_;

   /**
            * Weight matrix used to define the priority of controlling the rotation around each axis on the solver side.
            */
   public controller_msgs.msg.dds.WeightMatrix3DMessage angular_weight_matrix_;

   /**
            * Weight matrix used to define the priority of controlling the translation around each axis on the solver side.
            */
   public controller_msgs.msg.dds.WeightMatrix3DMessage linear_weight_matrix_;

   /**
            * This is the position of the control frame's origin expressed in endEffector.getBodyFixedFrame().
            * By default, the control frame is coincident to endEffector.getBodyFixedFrame().
            * The control frame is rigidly attached to the end-effector.
            */
   public us.ihmc.euclid.tuple3D.Point3D control_frame_position_in_end_effector_;

   /**
            * This is the orientation of the control frame expressed in endEffector.getBodyFixedFrame().
            * By default, the control frame is coincident to endEffector.getBodyFixedFrame().
            */
   public us.ihmc.euclid.tuple4D.Quaternion control_frame_orientation_in_end_effector_;

   /**
            * This is the allowable displacement of the position for each key frame.
            * By default, the solver will try to find a solution without modifying the position of the key frames.
            * When a positive value is provided, the solver may adjust a key frame to improve the overall solution quality.
            */
   public us.ihmc.idl.IDLSequence.Double  allowable_position_displacement_;

   /**
            * This is the allowable displacement of the orientation for each key frame.
            * By default, the solver will try to find a solution without modifying the orientation of the key frames.
            * When a positive value is provided, the solver may adjust a key frame to improve the overall solution quality.
            */
   public us.ihmc.idl.IDLSequence.Double  allowable_orientation_displacement_;

   public KinematicsPlanningToolboxRigidBodyMessage()
   {



      key_frame_times_ = new us.ihmc.idl.IDLSequence.Double (100, "type_6");


      key_frame_poses_ = new us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.geometry.Pose3D> (100, new geometry_msgs.msg.dds.PosePubSubType());

      angular_selection_matrix_ = new controller_msgs.msg.dds.SelectionMatrix3DMessage();

      linear_selection_matrix_ = new controller_msgs.msg.dds.SelectionMatrix3DMessage();

      angular_weight_matrix_ = new controller_msgs.msg.dds.WeightMatrix3DMessage();

      linear_weight_matrix_ = new controller_msgs.msg.dds.WeightMatrix3DMessage();

      control_frame_position_in_end_effector_ = new us.ihmc.euclid.tuple3D.Point3D();

      control_frame_orientation_in_end_effector_ = new us.ihmc.euclid.tuple4D.Quaternion();

      allowable_position_displacement_ = new us.ihmc.idl.IDLSequence.Double (100, "type_6");


      allowable_orientation_displacement_ = new us.ihmc.idl.IDLSequence.Double (100, "type_6");


   }

   public KinematicsPlanningToolboxRigidBodyMessage(KinematicsPlanningToolboxRigidBodyMessage other)
   {
      this();
      set(other);
   }

   public void set(KinematicsPlanningToolboxRigidBodyMessage other)
   {

      sequence_id_ = other.sequence_id_;


      end_effector_hash_code_ = other.end_effector_hash_code_;


      key_frame_times_.set(other.key_frame_times_);

      key_frame_poses_.set(other.key_frame_poses_);

      controller_msgs.msg.dds.SelectionMatrix3DMessagePubSubType.staticCopy(other.angular_selection_matrix_, angular_selection_matrix_);

      controller_msgs.msg.dds.SelectionMatrix3DMessagePubSubType.staticCopy(other.linear_selection_matrix_, linear_selection_matrix_);

      controller_msgs.msg.dds.WeightMatrix3DMessagePubSubType.staticCopy(other.angular_weight_matrix_, angular_weight_matrix_);

      controller_msgs.msg.dds.WeightMatrix3DMessagePubSubType.staticCopy(other.linear_weight_matrix_, linear_weight_matrix_);

      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.control_frame_position_in_end_effector_, control_frame_position_in_end_effector_);

      geometry_msgs.msg.dds.QuaternionPubSubType.staticCopy(other.control_frame_orientation_in_end_effector_, control_frame_orientation_in_end_effector_);

      allowable_position_displacement_.set(other.allowable_position_displacement_);

      allowable_orientation_displacement_.set(other.allowable_orientation_displacement_);
   }


   /**
            * This message is part of the IHMC whole-body inverse kinematics module.
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public void setSequenceId(long sequence_id)
   {
      sequence_id_ = sequence_id;
   }
   /**
            * This message is part of the IHMC whole-body inverse kinematics module.
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



   /**
            * This is the list of desired times for each key frames.
            */
   public us.ihmc.idl.IDLSequence.Double  getKeyFrameTimes()
   {
      return key_frame_times_;
   }



   /**
            * This is the list of desired key frames for end effector.
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.geometry.Pose3D>  getKeyFramePoses()
   {
      return key_frame_poses_;
   }



   /**
            * The selection frames coming along with the given selection matrix are used to determine to
            * what reference frame the selected axes are referring to. For instance, if only the hand height
            * in world should be controlled on the linear z component of the selection matrix should be
            * selected and the reference frame should be world frame. When no reference frame is provided
            * with the selection matrix, it will be used as it is in the control frame, i.e. the body-fixed
            * frame if not defined otherwise.
            */
   public controller_msgs.msg.dds.SelectionMatrix3DMessage getAngularSelectionMatrix()
   {
      return angular_selection_matrix_;
   }



   /**
            * The selection matrix is used to determinate which degree of freedom of the end-effector should
            * be controlled.
            * The selection frames coming along with the given selection matrix are used to determine to
            * what reference frame the selected axes are referring to. For instance, if only the hand height
            * in world should be controlled on the linear z component of the selection matrix should be
            * selected and the reference frame should be world frame. When no reference frame is provided
            * with the selection matrix, it will be used as it is in the control frame, i.e. the body-fixed
            * frame if not defined otherwise.
            */
   public controller_msgs.msg.dds.SelectionMatrix3DMessage getLinearSelectionMatrix()
   {
      return linear_selection_matrix_;
   }



   /**
            * Weight matrix used to define the priority of controlling the rotation around each axis on the solver side.
            */
   public controller_msgs.msg.dds.WeightMatrix3DMessage getAngularWeightMatrix()
   {
      return angular_weight_matrix_;
   }



   /**
            * Weight matrix used to define the priority of controlling the translation around each axis on the solver side.
            */
   public controller_msgs.msg.dds.WeightMatrix3DMessage getLinearWeightMatrix()
   {
      return linear_weight_matrix_;
   }



   /**
            * This is the position of the control frame's origin expressed in endEffector.getBodyFixedFrame().
            * By default, the control frame is coincident to endEffector.getBodyFixedFrame().
            * The control frame is rigidly attached to the end-effector.
            */
   public us.ihmc.euclid.tuple3D.Point3D getControlFramePositionInEndEffector()
   {
      return control_frame_position_in_end_effector_;
   }



   /**
            * This is the orientation of the control frame expressed in endEffector.getBodyFixedFrame().
            * By default, the control frame is coincident to endEffector.getBodyFixedFrame().
            */
   public us.ihmc.euclid.tuple4D.Quaternion getControlFrameOrientationInEndEffector()
   {
      return control_frame_orientation_in_end_effector_;
   }



   /**
            * This is the allowable displacement of the position for each key frame.
            * By default, the solver will try to find a solution without modifying the position of the key frames.
            * When a positive value is provided, the solver may adjust a key frame to improve the overall solution quality.
            */
   public us.ihmc.idl.IDLSequence.Double  getAllowablePositionDisplacement()
   {
      return allowable_position_displacement_;
   }



   /**
            * This is the allowable displacement of the orientation for each key frame.
            * By default, the solver will try to find a solution without modifying the orientation of the key frames.
            * When a positive value is provided, the solver may adjust a key frame to improve the overall solution quality.
            */
   public us.ihmc.idl.IDLSequence.Double  getAllowableOrientationDisplacement()
   {
      return allowable_orientation_displacement_;
   }


   public static Supplier<KinematicsPlanningToolboxRigidBodyMessagePubSubType> getPubSubType()
   {
      return KinematicsPlanningToolboxRigidBodyMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return KinematicsPlanningToolboxRigidBodyMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(KinematicsPlanningToolboxRigidBodyMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.end_effector_hash_code_, other.end_effector_hash_code_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsDoubleSequence(this.key_frame_times_, other.key_frame_times_, epsilon)) return false;


      if (this.key_frame_poses_.size() != other.key_frame_poses_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.key_frame_poses_.size(); i++)
         {  if (!this.key_frame_poses_.get(i).epsilonEquals(other.key_frame_poses_.get(i), epsilon)) return false; }
      }


      if (!this.angular_selection_matrix_.epsilonEquals(other.angular_selection_matrix_, epsilon)) return false;

      if (!this.linear_selection_matrix_.epsilonEquals(other.linear_selection_matrix_, epsilon)) return false;

      if (!this.angular_weight_matrix_.epsilonEquals(other.angular_weight_matrix_, epsilon)) return false;

      if (!this.linear_weight_matrix_.epsilonEquals(other.linear_weight_matrix_, epsilon)) return false;

      if (!this.control_frame_position_in_end_effector_.epsilonEquals(other.control_frame_position_in_end_effector_, epsilon)) return false;

      if (!this.control_frame_orientation_in_end_effector_.epsilonEquals(other.control_frame_orientation_in_end_effector_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsDoubleSequence(this.allowable_position_displacement_, other.allowable_position_displacement_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsDoubleSequence(this.allowable_orientation_displacement_, other.allowable_orientation_displacement_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof KinematicsPlanningToolboxRigidBodyMessage)) return false;

      KinematicsPlanningToolboxRigidBodyMessage otherMyClass = (KinematicsPlanningToolboxRigidBodyMessage) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if(this.end_effector_hash_code_ != otherMyClass.end_effector_hash_code_) return false;


      if (!this.key_frame_times_.equals(otherMyClass.key_frame_times_)) return false;

      if (!this.key_frame_poses_.equals(otherMyClass.key_frame_poses_)) return false;

      if (!this.angular_selection_matrix_.equals(otherMyClass.angular_selection_matrix_)) return false;

      if (!this.linear_selection_matrix_.equals(otherMyClass.linear_selection_matrix_)) return false;

      if (!this.angular_weight_matrix_.equals(otherMyClass.angular_weight_matrix_)) return false;

      if (!this.linear_weight_matrix_.equals(otherMyClass.linear_weight_matrix_)) return false;

      if (!this.control_frame_position_in_end_effector_.equals(otherMyClass.control_frame_position_in_end_effector_)) return false;

      if (!this.control_frame_orientation_in_end_effector_.equals(otherMyClass.control_frame_orientation_in_end_effector_)) return false;

      if (!this.allowable_position_displacement_.equals(otherMyClass.allowable_position_displacement_)) return false;

      if (!this.allowable_orientation_displacement_.equals(otherMyClass.allowable_orientation_displacement_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("KinematicsPlanningToolboxRigidBodyMessage {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("end_effector_hash_code=");
      builder.append(this.end_effector_hash_code_);      builder.append(", ");

      builder.append("key_frame_times=");
      builder.append(this.key_frame_times_);      builder.append(", ");

      builder.append("key_frame_poses=");
      builder.append(this.key_frame_poses_);      builder.append(", ");

      builder.append("angular_selection_matrix=");
      builder.append(this.angular_selection_matrix_);      builder.append(", ");

      builder.append("linear_selection_matrix=");
      builder.append(this.linear_selection_matrix_);      builder.append(", ");

      builder.append("angular_weight_matrix=");
      builder.append(this.angular_weight_matrix_);      builder.append(", ");

      builder.append("linear_weight_matrix=");
      builder.append(this.linear_weight_matrix_);      builder.append(", ");

      builder.append("control_frame_position_in_end_effector=");
      builder.append(this.control_frame_position_in_end_effector_);      builder.append(", ");

      builder.append("control_frame_orientation_in_end_effector=");
      builder.append(this.control_frame_orientation_in_end_effector_);      builder.append(", ");

      builder.append("allowable_position_displacement=");
      builder.append(this.allowable_position_displacement_);      builder.append(", ");

      builder.append("allowable_orientation_displacement=");
      builder.append(this.allowable_orientation_displacement_);
      builder.append("}");
      return builder.toString();
   }
}
