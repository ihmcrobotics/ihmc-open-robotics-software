package toolbox_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC whole-body inverse kinematics module.
       * Similar to KinematicsToolboxConfigurationMessage, this contains auxiliary information that allows to further customized the behavior of the solver.
       * The parameters exposed through this message are specific to application to humanoid robots.
       */
public class HumanoidKinematicsToolboxConfigurationMessage extends Packet<HumanoidKinematicsToolboxConfigurationMessage> implements Settable<HumanoidKinematicsToolboxConfigurationMessage>, EpsilonComparable<HumanoidKinematicsToolboxConfigurationMessage>
{
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   /**
            * When set to true, the solver will maintain, if possible, the current x and y coordinates of the center
            * of mass. By 'current', it means that the solver will use the robot configuration data
            * broadcasted by the controller to obtain the center of mass position.
            */
   public boolean hold_current_center_of_mass_xy_position_ = true;
   /**
            * When set to true, the solver will process the balance status from an active controller session and build a support polygon.
            * Note that the auto support polygon feature is overidden by a user specified support polygon when provided.
            * - when the walking controller is running, the CapturabilityBasedStatus message is used to identify the current support polygon.
            * - when the multi-contact controller is running, the MultiContactBalanceStatus message is used to identify the current support polygon.
            */
   public boolean enable_auto_support_polygon_ = true;
   /**
            * If true, enables an objective for improving the stability margin of the robot when the upper body is load-bearing.
            */
   public boolean enable_stability_objective_;
   /**
            * If true, enables a preview of the feasible region and corresponding contact-point adjustment.
            */
   public boolean enable_contact_adjustment_;
   /**
            * If enable_contact_adjustment is true this the origin position of the frame describing the upcoming bracing region
            */
   public us.ihmc.euclid.tuple3D.Point3D region_point_;
   /**
            * If enable_contact_adjustment is true this the origin orientation of the frame describing the upcoming bracing region. The normal is along z in the given plane
            */
   public us.ihmc.euclid.tuple4D.Quaternion region_orientation_;
   /**
            * Normal in world. This is over-specific but available to use if necessary
            */
   public us.ihmc.euclid.tuple3D.Vector3D region_normal_;
   /**
            * If enable_contact_adjustment is true this the convex hull of the upcoming bracing region
            */
   public us.ihmc.idl.IDLSequence.Object<ihmc_common_msgs.msg.dds.Point2DMessage>  region_vertices_;
   /**
            * Whether restrictive joint limits are enabled, in order to have the IK avoid a solution at the joint limit.
            */
   public boolean enable_joint_limit_reduction_ = true;
   /**
            * By default, the hip joint limits are restricted by 0.05 of the RoM.
            * When this and the subsequent list are set, the default limit restrictions are replaced with these values.
            * This list are the new joint limit reduction factors to be used, such that a value 0.05 means the restricted RoM will be 0.95 of the nominal (0.025 from either end).
            */
   public us.ihmc.idl.IDLSequence.Float  joint_limit_reduction_factors_;
   /**
            * The list of joints that the field joint_limit_reduction_values correspond to, by hash-code. The hash-code is computed from OneDoFJoint#hashcode().
            */
   public us.ihmc.idl.IDLSequence.Integer  joint_limit_reduction_hash_codes_;

   public HumanoidKinematicsToolboxConfigurationMessage()
   {
      region_point_ = new us.ihmc.euclid.tuple3D.Point3D();
      region_orientation_ = new us.ihmc.euclid.tuple4D.Quaternion();
      region_normal_ = new us.ihmc.euclid.tuple3D.Vector3D();
      region_vertices_ = new us.ihmc.idl.IDLSequence.Object<ihmc_common_msgs.msg.dds.Point2DMessage> (200, new ihmc_common_msgs.msg.dds.Point2DMessagePubSubType());
      joint_limit_reduction_factors_ = new us.ihmc.idl.IDLSequence.Float (20, "type_5");

      joint_limit_reduction_hash_codes_ = new us.ihmc.idl.IDLSequence.Integer (20, "type_2");


   }

   public HumanoidKinematicsToolboxConfigurationMessage(HumanoidKinematicsToolboxConfigurationMessage other)
   {
      this();
      set(other);
   }

   public void set(HumanoidKinematicsToolboxConfigurationMessage other)
   {
      sequence_id_ = other.sequence_id_;

      hold_current_center_of_mass_xy_position_ = other.hold_current_center_of_mass_xy_position_;

      enable_auto_support_polygon_ = other.enable_auto_support_polygon_;

      enable_stability_objective_ = other.enable_stability_objective_;

      enable_contact_adjustment_ = other.enable_contact_adjustment_;

      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.region_point_, region_point_);
      geometry_msgs.msg.dds.QuaternionPubSubType.staticCopy(other.region_orientation_, region_orientation_);
      geometry_msgs.msg.dds.Vector3PubSubType.staticCopy(other.region_normal_, region_normal_);
      region_vertices_.set(other.region_vertices_);
      enable_joint_limit_reduction_ = other.enable_joint_limit_reduction_;

      joint_limit_reduction_factors_.set(other.joint_limit_reduction_factors_);
      joint_limit_reduction_hash_codes_.set(other.joint_limit_reduction_hash_codes_);
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
            * When set to true, the solver will maintain, if possible, the current x and y coordinates of the center
            * of mass. By 'current', it means that the solver will use the robot configuration data
            * broadcasted by the controller to obtain the center of mass position.
            */
   public void setHoldCurrentCenterOfMassXyPosition(boolean hold_current_center_of_mass_xy_position)
   {
      hold_current_center_of_mass_xy_position_ = hold_current_center_of_mass_xy_position;
   }
   /**
            * When set to true, the solver will maintain, if possible, the current x and y coordinates of the center
            * of mass. By 'current', it means that the solver will use the robot configuration data
            * broadcasted by the controller to obtain the center of mass position.
            */
   public boolean getHoldCurrentCenterOfMassXyPosition()
   {
      return hold_current_center_of_mass_xy_position_;
   }

   /**
            * When set to true, the solver will process the balance status from an active controller session and build a support polygon.
            * Note that the auto support polygon feature is overidden by a user specified support polygon when provided.
            * - when the walking controller is running, the CapturabilityBasedStatus message is used to identify the current support polygon.
            * - when the multi-contact controller is running, the MultiContactBalanceStatus message is used to identify the current support polygon.
            */
   public void setEnableAutoSupportPolygon(boolean enable_auto_support_polygon)
   {
      enable_auto_support_polygon_ = enable_auto_support_polygon;
   }
   /**
            * When set to true, the solver will process the balance status from an active controller session and build a support polygon.
            * Note that the auto support polygon feature is overidden by a user specified support polygon when provided.
            * - when the walking controller is running, the CapturabilityBasedStatus message is used to identify the current support polygon.
            * - when the multi-contact controller is running, the MultiContactBalanceStatus message is used to identify the current support polygon.
            */
   public boolean getEnableAutoSupportPolygon()
   {
      return enable_auto_support_polygon_;
   }

   /**
            * If true, enables an objective for improving the stability margin of the robot when the upper body is load-bearing.
            */
   public void setEnableStabilityObjective(boolean enable_stability_objective)
   {
      enable_stability_objective_ = enable_stability_objective;
   }
   /**
            * If true, enables an objective for improving the stability margin of the robot when the upper body is load-bearing.
            */
   public boolean getEnableStabilityObjective()
   {
      return enable_stability_objective_;
   }

   /**
            * If true, enables a preview of the feasible region and corresponding contact-point adjustment.
            */
   public void setEnableContactAdjustment(boolean enable_contact_adjustment)
   {
      enable_contact_adjustment_ = enable_contact_adjustment;
   }
   /**
            * If true, enables a preview of the feasible region and corresponding contact-point adjustment.
            */
   public boolean getEnableContactAdjustment()
   {
      return enable_contact_adjustment_;
   }


   /**
            * If enable_contact_adjustment is true this the origin position of the frame describing the upcoming bracing region
            */
   public us.ihmc.euclid.tuple3D.Point3D getRegionPoint()
   {
      return region_point_;
   }


   /**
            * If enable_contact_adjustment is true this the origin orientation of the frame describing the upcoming bracing region. The normal is along z in the given plane
            */
   public us.ihmc.euclid.tuple4D.Quaternion getRegionOrientation()
   {
      return region_orientation_;
   }


   /**
            * Normal in world. This is over-specific but available to use if necessary
            */
   public us.ihmc.euclid.tuple3D.Vector3D getRegionNormal()
   {
      return region_normal_;
   }


   /**
            * If enable_contact_adjustment is true this the convex hull of the upcoming bracing region
            */
   public us.ihmc.idl.IDLSequence.Object<ihmc_common_msgs.msg.dds.Point2DMessage>  getRegionVertices()
   {
      return region_vertices_;
   }

   /**
            * Whether restrictive joint limits are enabled, in order to have the IK avoid a solution at the joint limit.
            */
   public void setEnableJointLimitReduction(boolean enable_joint_limit_reduction)
   {
      enable_joint_limit_reduction_ = enable_joint_limit_reduction;
   }
   /**
            * Whether restrictive joint limits are enabled, in order to have the IK avoid a solution at the joint limit.
            */
   public boolean getEnableJointLimitReduction()
   {
      return enable_joint_limit_reduction_;
   }


   /**
            * By default, the hip joint limits are restricted by 0.05 of the RoM.
            * When this and the subsequent list are set, the default limit restrictions are replaced with these values.
            * This list are the new joint limit reduction factors to be used, such that a value 0.05 means the restricted RoM will be 0.95 of the nominal (0.025 from either end).
            */
   public us.ihmc.idl.IDLSequence.Float  getJointLimitReductionFactors()
   {
      return joint_limit_reduction_factors_;
   }


   /**
            * The list of joints that the field joint_limit_reduction_values correspond to, by hash-code. The hash-code is computed from OneDoFJoint#hashcode().
            */
   public us.ihmc.idl.IDLSequence.Integer  getJointLimitReductionHashCodes()
   {
      return joint_limit_reduction_hash_codes_;
   }


   public static Supplier<HumanoidKinematicsToolboxConfigurationMessagePubSubType> getPubSubType()
   {
      return HumanoidKinematicsToolboxConfigurationMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return HumanoidKinematicsToolboxConfigurationMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(HumanoidKinematicsToolboxConfigurationMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.hold_current_center_of_mass_xy_position_, other.hold_current_center_of_mass_xy_position_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.enable_auto_support_polygon_, other.enable_auto_support_polygon_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.enable_stability_objective_, other.enable_stability_objective_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.enable_contact_adjustment_, other.enable_contact_adjustment_, epsilon)) return false;

      if (!this.region_point_.epsilonEquals(other.region_point_, epsilon)) return false;
      if (!this.region_orientation_.epsilonEquals(other.region_orientation_, epsilon)) return false;
      if (!this.region_normal_.epsilonEquals(other.region_normal_, epsilon)) return false;
      if (this.region_vertices_.size() != other.region_vertices_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.region_vertices_.size(); i++)
         {  if (!this.region_vertices_.get(i).epsilonEquals(other.region_vertices_.get(i), epsilon)) return false; }
      }

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.enable_joint_limit_reduction_, other.enable_joint_limit_reduction_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsFloatSequence(this.joint_limit_reduction_factors_, other.joint_limit_reduction_factors_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsIntegerSequence(this.joint_limit_reduction_hash_codes_, other.joint_limit_reduction_hash_codes_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof HumanoidKinematicsToolboxConfigurationMessage)) return false;

      HumanoidKinematicsToolboxConfigurationMessage otherMyClass = (HumanoidKinematicsToolboxConfigurationMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.hold_current_center_of_mass_xy_position_ != otherMyClass.hold_current_center_of_mass_xy_position_) return false;

      if(this.enable_auto_support_polygon_ != otherMyClass.enable_auto_support_polygon_) return false;

      if(this.enable_stability_objective_ != otherMyClass.enable_stability_objective_) return false;

      if(this.enable_contact_adjustment_ != otherMyClass.enable_contact_adjustment_) return false;

      if (!this.region_point_.equals(otherMyClass.region_point_)) return false;
      if (!this.region_orientation_.equals(otherMyClass.region_orientation_)) return false;
      if (!this.region_normal_.equals(otherMyClass.region_normal_)) return false;
      if (!this.region_vertices_.equals(otherMyClass.region_vertices_)) return false;
      if(this.enable_joint_limit_reduction_ != otherMyClass.enable_joint_limit_reduction_) return false;

      if (!this.joint_limit_reduction_factors_.equals(otherMyClass.joint_limit_reduction_factors_)) return false;
      if (!this.joint_limit_reduction_hash_codes_.equals(otherMyClass.joint_limit_reduction_hash_codes_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("HumanoidKinematicsToolboxConfigurationMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("hold_current_center_of_mass_xy_position=");
      builder.append(this.hold_current_center_of_mass_xy_position_);      builder.append(", ");
      builder.append("enable_auto_support_polygon=");
      builder.append(this.enable_auto_support_polygon_);      builder.append(", ");
      builder.append("enable_stability_objective=");
      builder.append(this.enable_stability_objective_);      builder.append(", ");
      builder.append("enable_contact_adjustment=");
      builder.append(this.enable_contact_adjustment_);      builder.append(", ");
      builder.append("region_point=");
      builder.append(this.region_point_);      builder.append(", ");
      builder.append("region_orientation=");
      builder.append(this.region_orientation_);      builder.append(", ");
      builder.append("region_normal=");
      builder.append(this.region_normal_);      builder.append(", ");
      builder.append("region_vertices=");
      builder.append(this.region_vertices_);      builder.append(", ");
      builder.append("enable_joint_limit_reduction=");
      builder.append(this.enable_joint_limit_reduction_);      builder.append(", ");
      builder.append("joint_limit_reduction_factors=");
      builder.append(this.joint_limit_reduction_factors_);      builder.append(", ");
      builder.append("joint_limit_reduction_hash_codes=");
      builder.append(this.joint_limit_reduction_hash_codes_);
      builder.append("}");
      return builder.toString();
   }
}
