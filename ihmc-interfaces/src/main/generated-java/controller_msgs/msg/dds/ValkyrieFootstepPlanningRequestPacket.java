package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class ValkyrieFootstepPlanningRequestPacket extends Packet<ValkyrieFootstepPlanningRequestPacket> implements Settable<ValkyrieFootstepPlanningRequestPacket>, EpsilonComparable<ValkyrieFootstepPlanningRequestPacket>
{
   /**
            * Set this id to keep track of your request
            */
   public int planner_request_id_ = -1;
   /**
            * Left foot pose
            */
   public us.ihmc.euclid.geometry.Pose3D left_foot_pose_;
   /**
            * Right foot pose
            */
   public us.ihmc.euclid.geometry.Pose3D right_foot_pose_;
   /**
            * Goal poses
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.geometry.Pose3D>  goal_poses_;
   /**
            * Parameters for the planner to use
            */
   public controller_msgs.msg.dds.ValkyrieFootstepPlannerParametersPacket parameters_;
   /**
            * Acceptable xy distance from the given goal for the planner to terminate
            */
   public double goal_distance_proximity_ = -1.0;
   /**
            * Acceptable yaw offset from the given goal for the planner to terminate
            */
   public double goal_yaw_proximity_ = -1.0;
   /**
            * Timeout in seconds
            */
   public double timeout_;
   /**
            * Planar regions to use, if you don't want to assume flat ground
            */
   public controller_msgs.msg.dds.PlanarRegionsListMessage planar_regions_list_message_;
   /**
            * Explicitly tell the planner to use flat ground
            */
   public boolean assume_flat_ground_;
   /**
            * If true a body path is computed as part of the planning process. If true, only one goal pose is used
            */
   public boolean compute_body_path_;

   public ValkyrieFootstepPlanningRequestPacket()
   {
      left_foot_pose_ = new us.ihmc.euclid.geometry.Pose3D();
      right_foot_pose_ = new us.ihmc.euclid.geometry.Pose3D();
      goal_poses_ = new us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.geometry.Pose3D> (100, new geometry_msgs.msg.dds.PosePubSubType());
      parameters_ = new controller_msgs.msg.dds.ValkyrieFootstepPlannerParametersPacket();
      planar_regions_list_message_ = new controller_msgs.msg.dds.PlanarRegionsListMessage();

   }

   public ValkyrieFootstepPlanningRequestPacket(ValkyrieFootstepPlanningRequestPacket other)
   {
      this();
      set(other);
   }

   public void set(ValkyrieFootstepPlanningRequestPacket other)
   {
      planner_request_id_ = other.planner_request_id_;

      geometry_msgs.msg.dds.PosePubSubType.staticCopy(other.left_foot_pose_, left_foot_pose_);
      geometry_msgs.msg.dds.PosePubSubType.staticCopy(other.right_foot_pose_, right_foot_pose_);
      goal_poses_.set(other.goal_poses_);
      controller_msgs.msg.dds.ValkyrieFootstepPlannerParametersPacketPubSubType.staticCopy(other.parameters_, parameters_);
      goal_distance_proximity_ = other.goal_distance_proximity_;

      goal_yaw_proximity_ = other.goal_yaw_proximity_;

      timeout_ = other.timeout_;

      controller_msgs.msg.dds.PlanarRegionsListMessagePubSubType.staticCopy(other.planar_regions_list_message_, planar_regions_list_message_);
      assume_flat_ground_ = other.assume_flat_ground_;

      compute_body_path_ = other.compute_body_path_;

   }

   /**
            * Set this id to keep track of your request
            */
   public void setPlannerRequestId(int planner_request_id)
   {
      planner_request_id_ = planner_request_id;
   }
   /**
            * Set this id to keep track of your request
            */
   public int getPlannerRequestId()
   {
      return planner_request_id_;
   }


   /**
            * Left foot pose
            */
   public us.ihmc.euclid.geometry.Pose3D getLeftFootPose()
   {
      return left_foot_pose_;
   }


   /**
            * Right foot pose
            */
   public us.ihmc.euclid.geometry.Pose3D getRightFootPose()
   {
      return right_foot_pose_;
   }


   /**
            * Goal poses
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.geometry.Pose3D>  getGoalPoses()
   {
      return goal_poses_;
   }


   /**
            * Parameters for the planner to use
            */
   public controller_msgs.msg.dds.ValkyrieFootstepPlannerParametersPacket getParameters()
   {
      return parameters_;
   }

   /**
            * Acceptable xy distance from the given goal for the planner to terminate
            */
   public void setGoalDistanceProximity(double goal_distance_proximity)
   {
      goal_distance_proximity_ = goal_distance_proximity;
   }
   /**
            * Acceptable xy distance from the given goal for the planner to terminate
            */
   public double getGoalDistanceProximity()
   {
      return goal_distance_proximity_;
   }

   /**
            * Acceptable yaw offset from the given goal for the planner to terminate
            */
   public void setGoalYawProximity(double goal_yaw_proximity)
   {
      goal_yaw_proximity_ = goal_yaw_proximity;
   }
   /**
            * Acceptable yaw offset from the given goal for the planner to terminate
            */
   public double getGoalYawProximity()
   {
      return goal_yaw_proximity_;
   }

   /**
            * Timeout in seconds
            */
   public void setTimeout(double timeout)
   {
      timeout_ = timeout;
   }
   /**
            * Timeout in seconds
            */
   public double getTimeout()
   {
      return timeout_;
   }


   /**
            * Planar regions to use, if you don't want to assume flat ground
            */
   public controller_msgs.msg.dds.PlanarRegionsListMessage getPlanarRegionsListMessage()
   {
      return planar_regions_list_message_;
   }

   /**
            * Explicitly tell the planner to use flat ground
            */
   public void setAssumeFlatGround(boolean assume_flat_ground)
   {
      assume_flat_ground_ = assume_flat_ground;
   }
   /**
            * Explicitly tell the planner to use flat ground
            */
   public boolean getAssumeFlatGround()
   {
      return assume_flat_ground_;
   }

   /**
            * If true a body path is computed as part of the planning process. If true, only one goal pose is used
            */
   public void setComputeBodyPath(boolean compute_body_path)
   {
      compute_body_path_ = compute_body_path;
   }
   /**
            * If true a body path is computed as part of the planning process. If true, only one goal pose is used
            */
   public boolean getComputeBodyPath()
   {
      return compute_body_path_;
   }


   public static Supplier<ValkyrieFootstepPlanningRequestPacketPubSubType> getPubSubType()
   {
      return ValkyrieFootstepPlanningRequestPacketPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return ValkyrieFootstepPlanningRequestPacketPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(ValkyrieFootstepPlanningRequestPacket other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.planner_request_id_, other.planner_request_id_, epsilon)) return false;

      if (!this.left_foot_pose_.epsilonEquals(other.left_foot_pose_, epsilon)) return false;
      if (!this.right_foot_pose_.epsilonEquals(other.right_foot_pose_, epsilon)) return false;
      if (this.goal_poses_.size() != other.goal_poses_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.goal_poses_.size(); i++)
         {  if (!this.goal_poses_.get(i).epsilonEquals(other.goal_poses_.get(i), epsilon)) return false; }
      }

      if (!this.parameters_.epsilonEquals(other.parameters_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.goal_distance_proximity_, other.goal_distance_proximity_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.goal_yaw_proximity_, other.goal_yaw_proximity_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.timeout_, other.timeout_, epsilon)) return false;

      if (!this.planar_regions_list_message_.epsilonEquals(other.planar_regions_list_message_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.assume_flat_ground_, other.assume_flat_ground_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.compute_body_path_, other.compute_body_path_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof ValkyrieFootstepPlanningRequestPacket)) return false;

      ValkyrieFootstepPlanningRequestPacket otherMyClass = (ValkyrieFootstepPlanningRequestPacket) other;

      if(this.planner_request_id_ != otherMyClass.planner_request_id_) return false;

      if (!this.left_foot_pose_.equals(otherMyClass.left_foot_pose_)) return false;
      if (!this.right_foot_pose_.equals(otherMyClass.right_foot_pose_)) return false;
      if (!this.goal_poses_.equals(otherMyClass.goal_poses_)) return false;
      if (!this.parameters_.equals(otherMyClass.parameters_)) return false;
      if(this.goal_distance_proximity_ != otherMyClass.goal_distance_proximity_) return false;

      if(this.goal_yaw_proximity_ != otherMyClass.goal_yaw_proximity_) return false;

      if(this.timeout_ != otherMyClass.timeout_) return false;

      if (!this.planar_regions_list_message_.equals(otherMyClass.planar_regions_list_message_)) return false;
      if(this.assume_flat_ground_ != otherMyClass.assume_flat_ground_) return false;

      if(this.compute_body_path_ != otherMyClass.compute_body_path_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ValkyrieFootstepPlanningRequestPacket {");
      builder.append("planner_request_id=");
      builder.append(this.planner_request_id_);      builder.append(", ");
      builder.append("left_foot_pose=");
      builder.append(this.left_foot_pose_);      builder.append(", ");
      builder.append("right_foot_pose=");
      builder.append(this.right_foot_pose_);      builder.append(", ");
      builder.append("goal_poses=");
      builder.append(this.goal_poses_);      builder.append(", ");
      builder.append("parameters=");
      builder.append(this.parameters_);      builder.append(", ");
      builder.append("goal_distance_proximity=");
      builder.append(this.goal_distance_proximity_);      builder.append(", ");
      builder.append("goal_yaw_proximity=");
      builder.append(this.goal_yaw_proximity_);      builder.append(", ");
      builder.append("timeout=");
      builder.append(this.timeout_);      builder.append(", ");
      builder.append("planar_regions_list_message=");
      builder.append(this.planar_regions_list_message_);      builder.append(", ");
      builder.append("assume_flat_ground=");
      builder.append(this.assume_flat_ground_);      builder.append(", ");
      builder.append("compute_body_path=");
      builder.append(this.compute_body_path_);
      builder.append("}");
      return builder.toString();
   }
}
