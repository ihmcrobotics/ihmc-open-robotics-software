package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC quadruped footstep planning module.
       */
public class QuadrupedFootstepPlannerParametersPacket extends Packet<QuadrupedFootstepPlannerParametersPacket> implements Settable<QuadrupedFootstepPlannerParametersPacket>, EpsilonComparable<QuadrupedFootstepPlannerParametersPacket>
{
   public static final double NO_VALUE_DOUBLE = -11.1;
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   public double maximum_step_reach_ = -11.1;
   public double maximum_step_length_ = -11.1;
   public double minimum_step_length_ = -11.1;
   public double maximum_step_width_ = -11.1;
   public double minimum_step_width_ = -11.1;
   public double minimum_step_yaw_ = -11.1;
   public double maximum_step_yaw_ = -11.1;
   public double maximum_step_change_z_ = -11.1;
   public double body_ground_clearance_ = -11.1;
   public double distance_heuristic_weight_ = -11.1;
   public double yaw_weight_ = -11.1;
   public double x_gait_weight_ = -11.1;
   public double cost_per_step_ = -11.1;
   public double step_up_weight_ = -11.1;
   public double step_down_weight_ = -11.1;
   public double heuristics_weight_ = -11.1;
   public double min_x_clearance_from_foot_ = -11.1;
   public double min_y_clearance_from_foot_ = -11.1;
   public double crawl_speed_ = -11.1;
   public double trot_speed_ = -11.1;
   public double pace_speed_ = -11.1;
   public double projection_inside_distance_ = -11.1;
   public double minimum_surface_incline_radians_ = -11.1;
   public double cliff_height_to_avoid_ = -11.1;
   public double minimum_distance_from_cliff_bottoms_ = -11.1;
   public double minimum_distance_from_cliff_tops_ = -11.1;

   public QuadrupedFootstepPlannerParametersPacket()
   {
   }

   public QuadrupedFootstepPlannerParametersPacket(QuadrupedFootstepPlannerParametersPacket other)
   {
      this();
      set(other);
   }

   public void set(QuadrupedFootstepPlannerParametersPacket other)
   {
      sequence_id_ = other.sequence_id_;

      maximum_step_reach_ = other.maximum_step_reach_;

      maximum_step_length_ = other.maximum_step_length_;

      minimum_step_length_ = other.minimum_step_length_;

      maximum_step_width_ = other.maximum_step_width_;

      minimum_step_width_ = other.minimum_step_width_;

      minimum_step_yaw_ = other.minimum_step_yaw_;

      maximum_step_yaw_ = other.maximum_step_yaw_;

      maximum_step_change_z_ = other.maximum_step_change_z_;

      body_ground_clearance_ = other.body_ground_clearance_;

      distance_heuristic_weight_ = other.distance_heuristic_weight_;

      yaw_weight_ = other.yaw_weight_;

      x_gait_weight_ = other.x_gait_weight_;

      cost_per_step_ = other.cost_per_step_;

      step_up_weight_ = other.step_up_weight_;

      step_down_weight_ = other.step_down_weight_;

      heuristics_weight_ = other.heuristics_weight_;

      min_x_clearance_from_foot_ = other.min_x_clearance_from_foot_;

      min_y_clearance_from_foot_ = other.min_y_clearance_from_foot_;

      crawl_speed_ = other.crawl_speed_;

      trot_speed_ = other.trot_speed_;

      pace_speed_ = other.pace_speed_;

      projection_inside_distance_ = other.projection_inside_distance_;

      minimum_surface_incline_radians_ = other.minimum_surface_incline_radians_;

      cliff_height_to_avoid_ = other.cliff_height_to_avoid_;

      minimum_distance_from_cliff_bottoms_ = other.minimum_distance_from_cliff_bottoms_;

      minimum_distance_from_cliff_tops_ = other.minimum_distance_from_cliff_tops_;

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

   public void setMaximumStepReach(double maximum_step_reach)
   {
      maximum_step_reach_ = maximum_step_reach;
   }
   public double getMaximumStepReach()
   {
      return maximum_step_reach_;
   }

   public void setMaximumStepLength(double maximum_step_length)
   {
      maximum_step_length_ = maximum_step_length;
   }
   public double getMaximumStepLength()
   {
      return maximum_step_length_;
   }

   public void setMinimumStepLength(double minimum_step_length)
   {
      minimum_step_length_ = minimum_step_length;
   }
   public double getMinimumStepLength()
   {
      return minimum_step_length_;
   }

   public void setMaximumStepWidth(double maximum_step_width)
   {
      maximum_step_width_ = maximum_step_width;
   }
   public double getMaximumStepWidth()
   {
      return maximum_step_width_;
   }

   public void setMinimumStepWidth(double minimum_step_width)
   {
      minimum_step_width_ = minimum_step_width;
   }
   public double getMinimumStepWidth()
   {
      return minimum_step_width_;
   }

   public void setMinimumStepYaw(double minimum_step_yaw)
   {
      minimum_step_yaw_ = minimum_step_yaw;
   }
   public double getMinimumStepYaw()
   {
      return minimum_step_yaw_;
   }

   public void setMaximumStepYaw(double maximum_step_yaw)
   {
      maximum_step_yaw_ = maximum_step_yaw;
   }
   public double getMaximumStepYaw()
   {
      return maximum_step_yaw_;
   }

   public void setMaximumStepChangeZ(double maximum_step_change_z)
   {
      maximum_step_change_z_ = maximum_step_change_z;
   }
   public double getMaximumStepChangeZ()
   {
      return maximum_step_change_z_;
   }

   public void setBodyGroundClearance(double body_ground_clearance)
   {
      body_ground_clearance_ = body_ground_clearance;
   }
   public double getBodyGroundClearance()
   {
      return body_ground_clearance_;
   }

   public void setDistanceHeuristicWeight(double distance_heuristic_weight)
   {
      distance_heuristic_weight_ = distance_heuristic_weight;
   }
   public double getDistanceHeuristicWeight()
   {
      return distance_heuristic_weight_;
   }

   public void setYawWeight(double yaw_weight)
   {
      yaw_weight_ = yaw_weight;
   }
   public double getYawWeight()
   {
      return yaw_weight_;
   }

   public void setXGaitWeight(double x_gait_weight)
   {
      x_gait_weight_ = x_gait_weight;
   }
   public double getXGaitWeight()
   {
      return x_gait_weight_;
   }

   public void setCostPerStep(double cost_per_step)
   {
      cost_per_step_ = cost_per_step;
   }
   public double getCostPerStep()
   {
      return cost_per_step_;
   }

   public void setStepUpWeight(double step_up_weight)
   {
      step_up_weight_ = step_up_weight;
   }
   public double getStepUpWeight()
   {
      return step_up_weight_;
   }

   public void setStepDownWeight(double step_down_weight)
   {
      step_down_weight_ = step_down_weight;
   }
   public double getStepDownWeight()
   {
      return step_down_weight_;
   }

   public void setHeuristicsWeight(double heuristics_weight)
   {
      heuristics_weight_ = heuristics_weight;
   }
   public double getHeuristicsWeight()
   {
      return heuristics_weight_;
   }

   public void setMinXClearanceFromFoot(double min_x_clearance_from_foot)
   {
      min_x_clearance_from_foot_ = min_x_clearance_from_foot;
   }
   public double getMinXClearanceFromFoot()
   {
      return min_x_clearance_from_foot_;
   }

   public void setMinYClearanceFromFoot(double min_y_clearance_from_foot)
   {
      min_y_clearance_from_foot_ = min_y_clearance_from_foot;
   }
   public double getMinYClearanceFromFoot()
   {
      return min_y_clearance_from_foot_;
   }

   public void setCrawlSpeed(double crawl_speed)
   {
      crawl_speed_ = crawl_speed;
   }
   public double getCrawlSpeed()
   {
      return crawl_speed_;
   }

   public void setTrotSpeed(double trot_speed)
   {
      trot_speed_ = trot_speed;
   }
   public double getTrotSpeed()
   {
      return trot_speed_;
   }

   public void setPaceSpeed(double pace_speed)
   {
      pace_speed_ = pace_speed;
   }
   public double getPaceSpeed()
   {
      return pace_speed_;
   }

   public void setProjectionInsideDistance(double projection_inside_distance)
   {
      projection_inside_distance_ = projection_inside_distance;
   }
   public double getProjectionInsideDistance()
   {
      return projection_inside_distance_;
   }

   public void setMinimumSurfaceInclineRadians(double minimum_surface_incline_radians)
   {
      minimum_surface_incline_radians_ = minimum_surface_incline_radians;
   }
   public double getMinimumSurfaceInclineRadians()
   {
      return minimum_surface_incline_radians_;
   }

   public void setCliffHeightToAvoid(double cliff_height_to_avoid)
   {
      cliff_height_to_avoid_ = cliff_height_to_avoid;
   }
   public double getCliffHeightToAvoid()
   {
      return cliff_height_to_avoid_;
   }

   public void setMinimumDistanceFromCliffBottoms(double minimum_distance_from_cliff_bottoms)
   {
      minimum_distance_from_cliff_bottoms_ = minimum_distance_from_cliff_bottoms;
   }
   public double getMinimumDistanceFromCliffBottoms()
   {
      return minimum_distance_from_cliff_bottoms_;
   }

   public void setMinimumDistanceFromCliffTops(double minimum_distance_from_cliff_tops)
   {
      minimum_distance_from_cliff_tops_ = minimum_distance_from_cliff_tops;
   }
   public double getMinimumDistanceFromCliffTops()
   {
      return minimum_distance_from_cliff_tops_;
   }


   public static Supplier<QuadrupedFootstepPlannerParametersPacketPubSubType> getPubSubType()
   {
      return QuadrupedFootstepPlannerParametersPacketPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return QuadrupedFootstepPlannerParametersPacketPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(QuadrupedFootstepPlannerParametersPacket other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.maximum_step_reach_, other.maximum_step_reach_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.maximum_step_length_, other.maximum_step_length_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.minimum_step_length_, other.minimum_step_length_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.maximum_step_width_, other.maximum_step_width_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.minimum_step_width_, other.minimum_step_width_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.minimum_step_yaw_, other.minimum_step_yaw_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.maximum_step_yaw_, other.maximum_step_yaw_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.maximum_step_change_z_, other.maximum_step_change_z_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.body_ground_clearance_, other.body_ground_clearance_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.distance_heuristic_weight_, other.distance_heuristic_weight_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.yaw_weight_, other.yaw_weight_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.x_gait_weight_, other.x_gait_weight_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.cost_per_step_, other.cost_per_step_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.step_up_weight_, other.step_up_weight_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.step_down_weight_, other.step_down_weight_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.heuristics_weight_, other.heuristics_weight_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.min_x_clearance_from_foot_, other.min_x_clearance_from_foot_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.min_y_clearance_from_foot_, other.min_y_clearance_from_foot_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.crawl_speed_, other.crawl_speed_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.trot_speed_, other.trot_speed_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.pace_speed_, other.pace_speed_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.projection_inside_distance_, other.projection_inside_distance_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.minimum_surface_incline_radians_, other.minimum_surface_incline_radians_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.cliff_height_to_avoid_, other.cliff_height_to_avoid_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.minimum_distance_from_cliff_bottoms_, other.minimum_distance_from_cliff_bottoms_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.minimum_distance_from_cliff_tops_, other.minimum_distance_from_cliff_tops_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof QuadrupedFootstepPlannerParametersPacket)) return false;

      QuadrupedFootstepPlannerParametersPacket otherMyClass = (QuadrupedFootstepPlannerParametersPacket) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.maximum_step_reach_ != otherMyClass.maximum_step_reach_) return false;

      if(this.maximum_step_length_ != otherMyClass.maximum_step_length_) return false;

      if(this.minimum_step_length_ != otherMyClass.minimum_step_length_) return false;

      if(this.maximum_step_width_ != otherMyClass.maximum_step_width_) return false;

      if(this.minimum_step_width_ != otherMyClass.minimum_step_width_) return false;

      if(this.minimum_step_yaw_ != otherMyClass.minimum_step_yaw_) return false;

      if(this.maximum_step_yaw_ != otherMyClass.maximum_step_yaw_) return false;

      if(this.maximum_step_change_z_ != otherMyClass.maximum_step_change_z_) return false;

      if(this.body_ground_clearance_ != otherMyClass.body_ground_clearance_) return false;

      if(this.distance_heuristic_weight_ != otherMyClass.distance_heuristic_weight_) return false;

      if(this.yaw_weight_ != otherMyClass.yaw_weight_) return false;

      if(this.x_gait_weight_ != otherMyClass.x_gait_weight_) return false;

      if(this.cost_per_step_ != otherMyClass.cost_per_step_) return false;

      if(this.step_up_weight_ != otherMyClass.step_up_weight_) return false;

      if(this.step_down_weight_ != otherMyClass.step_down_weight_) return false;

      if(this.heuristics_weight_ != otherMyClass.heuristics_weight_) return false;

      if(this.min_x_clearance_from_foot_ != otherMyClass.min_x_clearance_from_foot_) return false;

      if(this.min_y_clearance_from_foot_ != otherMyClass.min_y_clearance_from_foot_) return false;

      if(this.crawl_speed_ != otherMyClass.crawl_speed_) return false;

      if(this.trot_speed_ != otherMyClass.trot_speed_) return false;

      if(this.pace_speed_ != otherMyClass.pace_speed_) return false;

      if(this.projection_inside_distance_ != otherMyClass.projection_inside_distance_) return false;

      if(this.minimum_surface_incline_radians_ != otherMyClass.minimum_surface_incline_radians_) return false;

      if(this.cliff_height_to_avoid_ != otherMyClass.cliff_height_to_avoid_) return false;

      if(this.minimum_distance_from_cliff_bottoms_ != otherMyClass.minimum_distance_from_cliff_bottoms_) return false;

      if(this.minimum_distance_from_cliff_tops_ != otherMyClass.minimum_distance_from_cliff_tops_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("QuadrupedFootstepPlannerParametersPacket {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("maximum_step_reach=");
      builder.append(this.maximum_step_reach_);      builder.append(", ");
      builder.append("maximum_step_length=");
      builder.append(this.maximum_step_length_);      builder.append(", ");
      builder.append("minimum_step_length=");
      builder.append(this.minimum_step_length_);      builder.append(", ");
      builder.append("maximum_step_width=");
      builder.append(this.maximum_step_width_);      builder.append(", ");
      builder.append("minimum_step_width=");
      builder.append(this.minimum_step_width_);      builder.append(", ");
      builder.append("minimum_step_yaw=");
      builder.append(this.minimum_step_yaw_);      builder.append(", ");
      builder.append("maximum_step_yaw=");
      builder.append(this.maximum_step_yaw_);      builder.append(", ");
      builder.append("maximum_step_change_z=");
      builder.append(this.maximum_step_change_z_);      builder.append(", ");
      builder.append("body_ground_clearance=");
      builder.append(this.body_ground_clearance_);      builder.append(", ");
      builder.append("distance_heuristic_weight=");
      builder.append(this.distance_heuristic_weight_);      builder.append(", ");
      builder.append("yaw_weight=");
      builder.append(this.yaw_weight_);      builder.append(", ");
      builder.append("x_gait_weight=");
      builder.append(this.x_gait_weight_);      builder.append(", ");
      builder.append("cost_per_step=");
      builder.append(this.cost_per_step_);      builder.append(", ");
      builder.append("step_up_weight=");
      builder.append(this.step_up_weight_);      builder.append(", ");
      builder.append("step_down_weight=");
      builder.append(this.step_down_weight_);      builder.append(", ");
      builder.append("heuristics_weight=");
      builder.append(this.heuristics_weight_);      builder.append(", ");
      builder.append("min_x_clearance_from_foot=");
      builder.append(this.min_x_clearance_from_foot_);      builder.append(", ");
      builder.append("min_y_clearance_from_foot=");
      builder.append(this.min_y_clearance_from_foot_);      builder.append(", ");
      builder.append("crawl_speed=");
      builder.append(this.crawl_speed_);      builder.append(", ");
      builder.append("trot_speed=");
      builder.append(this.trot_speed_);      builder.append(", ");
      builder.append("pace_speed=");
      builder.append(this.pace_speed_);      builder.append(", ");
      builder.append("projection_inside_distance=");
      builder.append(this.projection_inside_distance_);      builder.append(", ");
      builder.append("minimum_surface_incline_radians=");
      builder.append(this.minimum_surface_incline_radians_);      builder.append(", ");
      builder.append("cliff_height_to_avoid=");
      builder.append(this.cliff_height_to_avoid_);      builder.append(", ");
      builder.append("minimum_distance_from_cliff_bottoms=");
      builder.append(this.minimum_distance_from_cliff_bottoms_);      builder.append(", ");
      builder.append("minimum_distance_from_cliff_tops=");
      builder.append(this.minimum_distance_from_cliff_tops_);
      builder.append("}");
      return builder.toString();
   }
}
