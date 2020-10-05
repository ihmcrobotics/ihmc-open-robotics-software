package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC whole-body inverse kinematics module.
       * It holds all the information needed for detailing the type of constraint to apply to the center of mass.
       */
public class KinematicsToolboxCenterOfMassMessage extends Packet<KinematicsToolboxCenterOfMassMessage> implements Settable<KinematicsToolboxCenterOfMassMessage>, EpsilonComparable<KinematicsToolboxCenterOfMassMessage>
{
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   /**
            * Specifies the desired center of mass position.
            * The data is assumed to be expressed in world frame.
            */
   public us.ihmc.euclid.tuple3D.Point3D desired_position_in_world_;
   /**
            * The selection matrix is used to determinate which degree of freedom of the center of mass
            * should be controlled.
            * The selection frame coming along with the given selection matrix is used to determine to what
            * reference frame the selected axes are referring to. For instance, if only the hand height in
            * world should be controlled on the linear z component of the selection matrix should be
            * selected and the reference frame should world frame. When no reference frame is provided with
            * the selection matrix, it will be used as it is in the control frame, i.e. the body-fixed frame
            * if not defined otherwise.
            */
   public controller_msgs.msg.dds.SelectionMatrix3DMessage selection_matrix_;
   /**
            * Specifies the priority of controller the position along each axis independently.
            * If no frame is provided, the weight matrix will be applied in the center of mass frame which is
            * aligned with the world axes.
            */
   public controller_msgs.msg.dds.WeightMatrix3DMessage weights_;

   public KinematicsToolboxCenterOfMassMessage()
   {
      desired_position_in_world_ = new us.ihmc.euclid.tuple3D.Point3D();
      selection_matrix_ = new controller_msgs.msg.dds.SelectionMatrix3DMessage();
      weights_ = new controller_msgs.msg.dds.WeightMatrix3DMessage();
   }

   public KinematicsToolboxCenterOfMassMessage(KinematicsToolboxCenterOfMassMessage other)
   {
      this();
      set(other);
   }

   public void set(KinematicsToolboxCenterOfMassMessage other)
   {
      sequence_id_ = other.sequence_id_;

      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.desired_position_in_world_, desired_position_in_world_);
      controller_msgs.msg.dds.SelectionMatrix3DMessagePubSubType.staticCopy(other.selection_matrix_, selection_matrix_);
      controller_msgs.msg.dds.WeightMatrix3DMessagePubSubType.staticCopy(other.weights_, weights_);
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
            * Specifies the desired center of mass position.
            * The data is assumed to be expressed in world frame.
            */
   public us.ihmc.euclid.tuple3D.Point3D getDesiredPositionInWorld()
   {
      return desired_position_in_world_;
   }


   /**
            * The selection matrix is used to determinate which degree of freedom of the center of mass
            * should be controlled.
            * The selection frame coming along with the given selection matrix is used to determine to what
            * reference frame the selected axes are referring to. For instance, if only the hand height in
            * world should be controlled on the linear z component of the selection matrix should be
            * selected and the reference frame should world frame. When no reference frame is provided with
            * the selection matrix, it will be used as it is in the control frame, i.e. the body-fixed frame
            * if not defined otherwise.
            */
   public controller_msgs.msg.dds.SelectionMatrix3DMessage getSelectionMatrix()
   {
      return selection_matrix_;
   }


   /**
            * Specifies the priority of controller the position along each axis independently.
            * If no frame is provided, the weight matrix will be applied in the center of mass frame which is
            * aligned with the world axes.
            */
   public controller_msgs.msg.dds.WeightMatrix3DMessage getWeights()
   {
      return weights_;
   }


   public static Supplier<KinematicsToolboxCenterOfMassMessagePubSubType> getPubSubType()
   {
      return KinematicsToolboxCenterOfMassMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return KinematicsToolboxCenterOfMassMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(KinematicsToolboxCenterOfMassMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!this.desired_position_in_world_.epsilonEquals(other.desired_position_in_world_, epsilon)) return false;
      if (!this.selection_matrix_.epsilonEquals(other.selection_matrix_, epsilon)) return false;
      if (!this.weights_.epsilonEquals(other.weights_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof KinematicsToolboxCenterOfMassMessage)) return false;

      KinematicsToolboxCenterOfMassMessage otherMyClass = (KinematicsToolboxCenterOfMassMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if (!this.desired_position_in_world_.equals(otherMyClass.desired_position_in_world_)) return false;
      if (!this.selection_matrix_.equals(otherMyClass.selection_matrix_)) return false;
      if (!this.weights_.equals(otherMyClass.weights_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("KinematicsToolboxCenterOfMassMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("desired_position_in_world=");
      builder.append(this.desired_position_in_world_);      builder.append(", ");
      builder.append("selection_matrix=");
      builder.append(this.selection_matrix_);      builder.append(", ");
      builder.append("weights=");
      builder.append(this.weights_);
      builder.append("}");
      return builder.toString();
   }
}
