package toolbox_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC whole-body inverse kinematics module.
       * It holds all the information needed for detailing the type of constraint to apply to the center of mass.
       */
public class KinematicsPlanningToolboxCenterOfMassMessage extends Packet<KinematicsPlanningToolboxCenterOfMassMessage> implements Settable<KinematicsPlanningToolboxCenterOfMassMessage>, EpsilonComparable<KinematicsPlanningToolboxCenterOfMassMessage>
{
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   /**
            * This is the list of desired times for each key frames.
            */
   public us.ihmc.idl.IDLSequence.Double  way_point_times_;
   /**
            * Specifies the desired center of mass position.
            * The data is assumed to be expressed in world frame.
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D>  desired_way_point_positions_in_world_;
   /**
            * The selection matrix is used to determinate which degree of freedom of the center of mass
            * should be controlled.
            * The selection frame coming along with the given selection matrix is used to determine to what
            * reference frame the selected axes are referring to. For instance, if only the hand height in
            * world should be controlled only the linear z component of the selection matrix should be
            * selected and the reference frame should world frame. When no reference frame is provided with
            * the selection matrix, it will be used as it is in the center of mass frame which is aligned with the world axes.
            */
   public ihmc_common_msgs.msg.dds.SelectionMatrix3DMessage selection_matrix_;
   /**
            * Specifies the priority of controller the position along each axis independently.
            * If no frame is provided, the weight matrix will be applied in the center of mass frame which is
            * aligned with the world axes.
            */
   public ihmc_common_msgs.msg.dds.WeightMatrix3DMessage weights_;

   public KinematicsPlanningToolboxCenterOfMassMessage()
   {
      way_point_times_ = new us.ihmc.idl.IDLSequence.Double (100, "type_6");

      desired_way_point_positions_in_world_ = new us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D> (100, new geometry_msgs.msg.dds.PointPubSubType());
      selection_matrix_ = new ihmc_common_msgs.msg.dds.SelectionMatrix3DMessage();
      weights_ = new ihmc_common_msgs.msg.dds.WeightMatrix3DMessage();

   }

   public KinematicsPlanningToolboxCenterOfMassMessage(KinematicsPlanningToolboxCenterOfMassMessage other)
   {
      this();
      set(other);
   }

   public void set(KinematicsPlanningToolboxCenterOfMassMessage other)
   {
      sequence_id_ = other.sequence_id_;

      way_point_times_.set(other.way_point_times_);
      desired_way_point_positions_in_world_.set(other.desired_way_point_positions_in_world_);
      ihmc_common_msgs.msg.dds.SelectionMatrix3DMessagePubSubType.staticCopy(other.selection_matrix_, selection_matrix_);
      ihmc_common_msgs.msg.dds.WeightMatrix3DMessagePubSubType.staticCopy(other.weights_, weights_);
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
            * This is the list of desired times for each key frames.
            */
   public us.ihmc.idl.IDLSequence.Double  getWayPointTimes()
   {
      return way_point_times_;
   }


   /**
            * Specifies the desired center of mass position.
            * The data is assumed to be expressed in world frame.
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D>  getDesiredWayPointPositionsInWorld()
   {
      return desired_way_point_positions_in_world_;
   }


   /**
            * The selection matrix is used to determinate which degree of freedom of the center of mass
            * should be controlled.
            * The selection frame coming along with the given selection matrix is used to determine to what
            * reference frame the selected axes are referring to. For instance, if only the hand height in
            * world should be controlled only the linear z component of the selection matrix should be
            * selected and the reference frame should world frame. When no reference frame is provided with
            * the selection matrix, it will be used as it is in the center of mass frame which is aligned with the world axes.
            */
   public ihmc_common_msgs.msg.dds.SelectionMatrix3DMessage getSelectionMatrix()
   {
      return selection_matrix_;
   }


   /**
            * Specifies the priority of controller the position along each axis independently.
            * If no frame is provided, the weight matrix will be applied in the center of mass frame which is
            * aligned with the world axes.
            */
   public ihmc_common_msgs.msg.dds.WeightMatrix3DMessage getWeights()
   {
      return weights_;
   }


   public static Supplier<KinematicsPlanningToolboxCenterOfMassMessagePubSubType> getPubSubType()
   {
      return KinematicsPlanningToolboxCenterOfMassMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return KinematicsPlanningToolboxCenterOfMassMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(KinematicsPlanningToolboxCenterOfMassMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsDoubleSequence(this.way_point_times_, other.way_point_times_, epsilon)) return false;

      if (this.desired_way_point_positions_in_world_.size() != other.desired_way_point_positions_in_world_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.desired_way_point_positions_in_world_.size(); i++)
         {  if (!this.desired_way_point_positions_in_world_.get(i).epsilonEquals(other.desired_way_point_positions_in_world_.get(i), epsilon)) return false; }
      }

      if (!this.selection_matrix_.epsilonEquals(other.selection_matrix_, epsilon)) return false;
      if (!this.weights_.epsilonEquals(other.weights_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof KinematicsPlanningToolboxCenterOfMassMessage)) return false;

      KinematicsPlanningToolboxCenterOfMassMessage otherMyClass = (KinematicsPlanningToolboxCenterOfMassMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if (!this.way_point_times_.equals(otherMyClass.way_point_times_)) return false;
      if (!this.desired_way_point_positions_in_world_.equals(otherMyClass.desired_way_point_positions_in_world_)) return false;
      if (!this.selection_matrix_.equals(otherMyClass.selection_matrix_)) return false;
      if (!this.weights_.equals(otherMyClass.weights_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("KinematicsPlanningToolboxCenterOfMassMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("way_point_times=");
      builder.append(this.way_point_times_);      builder.append(", ");
      builder.append("desired_way_point_positions_in_world=");
      builder.append(this.desired_way_point_positions_in_world_);      builder.append(", ");
      builder.append("selection_matrix=");
      builder.append(this.selection_matrix_);      builder.append(", ");
      builder.append("weights=");
      builder.append(this.weights_);
      builder.append("}");
      return builder.toString();
   }
}
