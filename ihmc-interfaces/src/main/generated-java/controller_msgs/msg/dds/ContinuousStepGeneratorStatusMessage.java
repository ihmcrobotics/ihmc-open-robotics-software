package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * Current status of various CSG parameters
       */
public class ContinuousStepGeneratorStatusMessage extends Packet<ContinuousStepGeneratorStatusMessage> implements Settable<ContinuousStepGeneratorStatusMessage>, EpsilonComparable<ContinuousStepGeneratorStatusMessage>
{
   public boolean is_walking_;
   public boolean are_velocities_normalized_;
   public double current_forward_velocity_;
   public double current_lateral_velocity_;
   public double current_turn_velocity_;
   public double current_swing_height_;
   public double current_swing_duration_;
   public double current_transfer_duration_;
   public double current_max_step_length_forwards_;
   public double current_max_step_length_backwards_;
   public double current_max_step_width_;
   public double current_min_step_width_;
   public double current_default_step_width_;
   public double current_turn_max_angle_inward_;
   public double current_turn_max_angle_outward_;
   public boolean are_steps_adjustable_;
   public boolean snapping_to_heightmap_;
   public boolean accounting_for_ground_drift_;

   public ContinuousStepGeneratorStatusMessage()
   {
   }

   public ContinuousStepGeneratorStatusMessage(ContinuousStepGeneratorStatusMessage other)
   {
      this();
      set(other);
   }

   public void set(ContinuousStepGeneratorStatusMessage other)
   {
      is_walking_ = other.is_walking_;

      are_velocities_normalized_ = other.are_velocities_normalized_;

      current_forward_velocity_ = other.current_forward_velocity_;

      current_lateral_velocity_ = other.current_lateral_velocity_;

      current_turn_velocity_ = other.current_turn_velocity_;

      current_swing_height_ = other.current_swing_height_;

      current_swing_duration_ = other.current_swing_duration_;

      current_transfer_duration_ = other.current_transfer_duration_;

      current_max_step_length_forwards_ = other.current_max_step_length_forwards_;

      current_max_step_length_backwards_ = other.current_max_step_length_backwards_;

      current_max_step_width_ = other.current_max_step_width_;

      current_min_step_width_ = other.current_min_step_width_;

      current_default_step_width_ = other.current_default_step_width_;

      current_turn_max_angle_inward_ = other.current_turn_max_angle_inward_;

      current_turn_max_angle_outward_ = other.current_turn_max_angle_outward_;

      are_steps_adjustable_ = other.are_steps_adjustable_;

      snapping_to_heightmap_ = other.snapping_to_heightmap_;

      accounting_for_ground_drift_ = other.accounting_for_ground_drift_;

   }

   public void setIsWalking(boolean is_walking)
   {
      is_walking_ = is_walking;
   }
   public boolean getIsWalking()
   {
      return is_walking_;
   }

   public void setAreVelocitiesNormalized(boolean are_velocities_normalized)
   {
      are_velocities_normalized_ = are_velocities_normalized;
   }
   public boolean getAreVelocitiesNormalized()
   {
      return are_velocities_normalized_;
   }

   public void setCurrentForwardVelocity(double current_forward_velocity)
   {
      current_forward_velocity_ = current_forward_velocity;
   }
   public double getCurrentForwardVelocity()
   {
      return current_forward_velocity_;
   }

   public void setCurrentLateralVelocity(double current_lateral_velocity)
   {
      current_lateral_velocity_ = current_lateral_velocity;
   }
   public double getCurrentLateralVelocity()
   {
      return current_lateral_velocity_;
   }

   public void setCurrentTurnVelocity(double current_turn_velocity)
   {
      current_turn_velocity_ = current_turn_velocity;
   }
   public double getCurrentTurnVelocity()
   {
      return current_turn_velocity_;
   }

   public void setCurrentSwingHeight(double current_swing_height)
   {
      current_swing_height_ = current_swing_height;
   }
   public double getCurrentSwingHeight()
   {
      return current_swing_height_;
   }

   public void setCurrentSwingDuration(double current_swing_duration)
   {
      current_swing_duration_ = current_swing_duration;
   }
   public double getCurrentSwingDuration()
   {
      return current_swing_duration_;
   }

   public void setCurrentTransferDuration(double current_transfer_duration)
   {
      current_transfer_duration_ = current_transfer_duration;
   }
   public double getCurrentTransferDuration()
   {
      return current_transfer_duration_;
   }

   public void setCurrentMaxStepLengthForwards(double current_max_step_length_forwards)
   {
      current_max_step_length_forwards_ = current_max_step_length_forwards;
   }
   public double getCurrentMaxStepLengthForwards()
   {
      return current_max_step_length_forwards_;
   }

   public void setCurrentMaxStepLengthBackwards(double current_max_step_length_backwards)
   {
      current_max_step_length_backwards_ = current_max_step_length_backwards;
   }
   public double getCurrentMaxStepLengthBackwards()
   {
      return current_max_step_length_backwards_;
   }

   public void setCurrentMaxStepWidth(double current_max_step_width)
   {
      current_max_step_width_ = current_max_step_width;
   }
   public double getCurrentMaxStepWidth()
   {
      return current_max_step_width_;
   }

   public void setCurrentMinStepWidth(double current_min_step_width)
   {
      current_min_step_width_ = current_min_step_width;
   }
   public double getCurrentMinStepWidth()
   {
      return current_min_step_width_;
   }

   public void setCurrentDefaultStepWidth(double current_default_step_width)
   {
      current_default_step_width_ = current_default_step_width;
   }
   public double getCurrentDefaultStepWidth()
   {
      return current_default_step_width_;
   }

   public void setCurrentTurnMaxAngleInward(double current_turn_max_angle_inward)
   {
      current_turn_max_angle_inward_ = current_turn_max_angle_inward;
   }
   public double getCurrentTurnMaxAngleInward()
   {
      return current_turn_max_angle_inward_;
   }

   public void setCurrentTurnMaxAngleOutward(double current_turn_max_angle_outward)
   {
      current_turn_max_angle_outward_ = current_turn_max_angle_outward;
   }
   public double getCurrentTurnMaxAngleOutward()
   {
      return current_turn_max_angle_outward_;
   }

   public void setAreStepsAdjustable(boolean are_steps_adjustable)
   {
      are_steps_adjustable_ = are_steps_adjustable;
   }
   public boolean getAreStepsAdjustable()
   {
      return are_steps_adjustable_;
   }

   public void setSnappingToHeightmap(boolean snapping_to_heightmap)
   {
      snapping_to_heightmap_ = snapping_to_heightmap;
   }
   public boolean getSnappingToHeightmap()
   {
      return snapping_to_heightmap_;
   }

   public void setAccountingForGroundDrift(boolean accounting_for_ground_drift)
   {
      accounting_for_ground_drift_ = accounting_for_ground_drift;
   }
   public boolean getAccountingForGroundDrift()
   {
      return accounting_for_ground_drift_;
   }


   public static Supplier<ContinuousStepGeneratorStatusMessagePubSubType> getPubSubType()
   {
      return ContinuousStepGeneratorStatusMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return ContinuousStepGeneratorStatusMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(ContinuousStepGeneratorStatusMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.is_walking_, other.is_walking_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.are_velocities_normalized_, other.are_velocities_normalized_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.current_forward_velocity_, other.current_forward_velocity_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.current_lateral_velocity_, other.current_lateral_velocity_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.current_turn_velocity_, other.current_turn_velocity_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.current_swing_height_, other.current_swing_height_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.current_swing_duration_, other.current_swing_duration_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.current_transfer_duration_, other.current_transfer_duration_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.current_max_step_length_forwards_, other.current_max_step_length_forwards_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.current_max_step_length_backwards_, other.current_max_step_length_backwards_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.current_max_step_width_, other.current_max_step_width_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.current_min_step_width_, other.current_min_step_width_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.current_default_step_width_, other.current_default_step_width_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.current_turn_max_angle_inward_, other.current_turn_max_angle_inward_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.current_turn_max_angle_outward_, other.current_turn_max_angle_outward_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.are_steps_adjustable_, other.are_steps_adjustable_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.snapping_to_heightmap_, other.snapping_to_heightmap_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.accounting_for_ground_drift_, other.accounting_for_ground_drift_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof ContinuousStepGeneratorStatusMessage)) return false;

      ContinuousStepGeneratorStatusMessage otherMyClass = (ContinuousStepGeneratorStatusMessage) other;

      if(this.is_walking_ != otherMyClass.is_walking_) return false;

      if(this.are_velocities_normalized_ != otherMyClass.are_velocities_normalized_) return false;

      if(this.current_forward_velocity_ != otherMyClass.current_forward_velocity_) return false;

      if(this.current_lateral_velocity_ != otherMyClass.current_lateral_velocity_) return false;

      if(this.current_turn_velocity_ != otherMyClass.current_turn_velocity_) return false;

      if(this.current_swing_height_ != otherMyClass.current_swing_height_) return false;

      if(this.current_swing_duration_ != otherMyClass.current_swing_duration_) return false;

      if(this.current_transfer_duration_ != otherMyClass.current_transfer_duration_) return false;

      if(this.current_max_step_length_forwards_ != otherMyClass.current_max_step_length_forwards_) return false;

      if(this.current_max_step_length_backwards_ != otherMyClass.current_max_step_length_backwards_) return false;

      if(this.current_max_step_width_ != otherMyClass.current_max_step_width_) return false;

      if(this.current_min_step_width_ != otherMyClass.current_min_step_width_) return false;

      if(this.current_default_step_width_ != otherMyClass.current_default_step_width_) return false;

      if(this.current_turn_max_angle_inward_ != otherMyClass.current_turn_max_angle_inward_) return false;

      if(this.current_turn_max_angle_outward_ != otherMyClass.current_turn_max_angle_outward_) return false;

      if(this.are_steps_adjustable_ != otherMyClass.are_steps_adjustable_) return false;

      if(this.snapping_to_heightmap_ != otherMyClass.snapping_to_heightmap_) return false;

      if(this.accounting_for_ground_drift_ != otherMyClass.accounting_for_ground_drift_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ContinuousStepGeneratorStatusMessage {");
      builder.append("is_walking=");
      builder.append(this.is_walking_);      builder.append(", ");
      builder.append("are_velocities_normalized=");
      builder.append(this.are_velocities_normalized_);      builder.append(", ");
      builder.append("current_forward_velocity=");
      builder.append(this.current_forward_velocity_);      builder.append(", ");
      builder.append("current_lateral_velocity=");
      builder.append(this.current_lateral_velocity_);      builder.append(", ");
      builder.append("current_turn_velocity=");
      builder.append(this.current_turn_velocity_);      builder.append(", ");
      builder.append("current_swing_height=");
      builder.append(this.current_swing_height_);      builder.append(", ");
      builder.append("current_swing_duration=");
      builder.append(this.current_swing_duration_);      builder.append(", ");
      builder.append("current_transfer_duration=");
      builder.append(this.current_transfer_duration_);      builder.append(", ");
      builder.append("current_max_step_length_forwards=");
      builder.append(this.current_max_step_length_forwards_);      builder.append(", ");
      builder.append("current_max_step_length_backwards=");
      builder.append(this.current_max_step_length_backwards_);      builder.append(", ");
      builder.append("current_max_step_width=");
      builder.append(this.current_max_step_width_);      builder.append(", ");
      builder.append("current_min_step_width=");
      builder.append(this.current_min_step_width_);      builder.append(", ");
      builder.append("current_default_step_width=");
      builder.append(this.current_default_step_width_);      builder.append(", ");
      builder.append("current_turn_max_angle_inward=");
      builder.append(this.current_turn_max_angle_inward_);      builder.append(", ");
      builder.append("current_turn_max_angle_outward=");
      builder.append(this.current_turn_max_angle_outward_);      builder.append(", ");
      builder.append("are_steps_adjustable=");
      builder.append(this.are_steps_adjustable_);      builder.append(", ");
      builder.append("snapping_to_heightmap=");
      builder.append(this.snapping_to_heightmap_);      builder.append(", ");
      builder.append("accounting_for_ground_drift=");
      builder.append(this.accounting_for_ground_drift_);
      builder.append("}");
      return builder.toString();
   }
}
