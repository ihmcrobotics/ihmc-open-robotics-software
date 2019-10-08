package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC whole-body inverse kinematics module.
       * It contains auxiliary information that allows to further customized the behavior of the solver.
       */
public class KinematicsToolboxConfigurationMessage extends Packet<KinematicsToolboxConfigurationMessage> implements Settable<KinematicsToolboxConfigurationMessage>, EpsilonComparable<KinematicsToolboxConfigurationMessage>
{
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   /**
            * Indicates whether the privileged_root_joint_position is to be used or not.
            */
   public boolean use_privileged_root_joint_position_;
   /**
            * Indicates whether the privileged_root_joint_orientation is to be used or not.
            */
   public boolean use_privileged_root_joint_orientation_;
   /**
            * When provided, the solver will attempt to find the solution that is the closest to the privileged configuration.
            */
   public us.ihmc.euclid.tuple3D.Point3D privileged_root_joint_position_;
   /**
            * When provided, the solver will attempt to find the solution that is the closest to the privileged configuration.
            */
   public us.ihmc.euclid.tuple4D.Quaternion privileged_root_joint_orientation_;
   /**
            * This array is used to identify to which joint each angle in privileged_joint_angles belongs to.
            * See Joint.hashCode() for the computation of a joint hash code.
            */
   public us.ihmc.idl.IDLSequence.Integer  privileged_joint_hash_codes_;
   /**
            * When provided, the solver will attempt to find the solution that is the closest to the privileged configuration.
            */
   public us.ihmc.idl.IDLSequence.Float  privileged_joint_angles_;
   /**
            * The weight to use in the optimization for the privileged configuration.
            * When remaining close to the privileged configuration is important, raise this weight to a value higher than the
            * weight of the main objectives.
            * Any value less than zero will be ignored.
            * A value of -1 will result in the solver using its default value.
            */
   public double privileged_weight_ = -1.0;
   /**
            * The feedback proportional gain to use for the privileged configuration.
            * It is coupled to some extent to the privileged_weight
            * A value of -1 will result in the solver using its default value.
            */
   public double privileged_gain_ = -1.0;
   /**
            * Specifies how much high joint velocity values should be penalized in the optimization problem.
            * A low value generally results in a reduce number of iterations before convergence but it also decreases the general stability of the solver.
            * A value of -1 will result in the solver using its default value.
            */
   public double joint_velocity_weight_ = -1.0;
   /**
            * Specifying how much high joint acceleration values should be penalized in the optimization problem.
            * A value of -1 will result in the solver using its default value.
            */
   public double joint_acceleration_weight_ = -1.0;
   /**
            * When true, the solver will enforce the joint velocity limits as defined in the robot model.
            * Enabling this restriction will augment the number of iteration before converging to a robot configuration for a given set of end-effector positions.
            */
   public boolean enable_joint_velocity_limits_;
   /**
            * When true, the solver will ignore the joint velocity limits.
            * Enabling this restriction will reduce the number of iteration before converging to a robot configuration for a given set of end-effector positions.
            */
   public boolean disable_joint_velocity_limits_;
   /**
            * If the toolbox has been setup with the collision model of the robot, it will by default handle self-collision avoidance.
            * In case it has undesirable effects, use this flag to disable it.
            */
   public boolean disable_collision_avoidance_;
   /**
            * In case collision avoidance has been disabled, use this flag to re-enable it while leaving disable_collision_avoidance to false.
            */
   public boolean enable_collision_avoidance_;

   public KinematicsToolboxConfigurationMessage()
   {
      privileged_root_joint_position_ = new us.ihmc.euclid.tuple3D.Point3D();
      privileged_root_joint_orientation_ = new us.ihmc.euclid.tuple4D.Quaternion();
      privileged_joint_hash_codes_ = new us.ihmc.idl.IDLSequence.Integer (100, "type_2");

      privileged_joint_angles_ = new us.ihmc.idl.IDLSequence.Float (100, "type_5");

   }

   public KinematicsToolboxConfigurationMessage(KinematicsToolboxConfigurationMessage other)
   {
      this();
      set(other);
   }

   public void set(KinematicsToolboxConfigurationMessage other)
   {
      sequence_id_ = other.sequence_id_;

      use_privileged_root_joint_position_ = other.use_privileged_root_joint_position_;

      use_privileged_root_joint_orientation_ = other.use_privileged_root_joint_orientation_;

      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.privileged_root_joint_position_, privileged_root_joint_position_);
      geometry_msgs.msg.dds.QuaternionPubSubType.staticCopy(other.privileged_root_joint_orientation_, privileged_root_joint_orientation_);
      privileged_joint_hash_codes_.set(other.privileged_joint_hash_codes_);
      privileged_joint_angles_.set(other.privileged_joint_angles_);
      privileged_weight_ = other.privileged_weight_;

      privileged_gain_ = other.privileged_gain_;

      joint_velocity_weight_ = other.joint_velocity_weight_;

      joint_acceleration_weight_ = other.joint_acceleration_weight_;

      enable_joint_velocity_limits_ = other.enable_joint_velocity_limits_;

      disable_joint_velocity_limits_ = other.disable_joint_velocity_limits_;

      disable_collision_avoidance_ = other.disable_collision_avoidance_;

      enable_collision_avoidance_ = other.enable_collision_avoidance_;

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
            * Indicates whether the privileged_root_joint_position is to be used or not.
            */
   public void setUsePrivilegedRootJointPosition(boolean use_privileged_root_joint_position)
   {
      use_privileged_root_joint_position_ = use_privileged_root_joint_position;
   }
   /**
            * Indicates whether the privileged_root_joint_position is to be used or not.
            */
   public boolean getUsePrivilegedRootJointPosition()
   {
      return use_privileged_root_joint_position_;
   }

   /**
            * Indicates whether the privileged_root_joint_orientation is to be used or not.
            */
   public void setUsePrivilegedRootJointOrientation(boolean use_privileged_root_joint_orientation)
   {
      use_privileged_root_joint_orientation_ = use_privileged_root_joint_orientation;
   }
   /**
            * Indicates whether the privileged_root_joint_orientation is to be used or not.
            */
   public boolean getUsePrivilegedRootJointOrientation()
   {
      return use_privileged_root_joint_orientation_;
   }


   /**
            * When provided, the solver will attempt to find the solution that is the closest to the privileged configuration.
            */
   public us.ihmc.euclid.tuple3D.Point3D getPrivilegedRootJointPosition()
   {
      return privileged_root_joint_position_;
   }


   /**
            * When provided, the solver will attempt to find the solution that is the closest to the privileged configuration.
            */
   public us.ihmc.euclid.tuple4D.Quaternion getPrivilegedRootJointOrientation()
   {
      return privileged_root_joint_orientation_;
   }


   /**
            * This array is used to identify to which joint each angle in privileged_joint_angles belongs to.
            * See Joint.hashCode() for the computation of a joint hash code.
            */
   public us.ihmc.idl.IDLSequence.Integer  getPrivilegedJointHashCodes()
   {
      return privileged_joint_hash_codes_;
   }


   /**
            * When provided, the solver will attempt to find the solution that is the closest to the privileged configuration.
            */
   public us.ihmc.idl.IDLSequence.Float  getPrivilegedJointAngles()
   {
      return privileged_joint_angles_;
   }

   /**
            * The weight to use in the optimization for the privileged configuration.
            * When remaining close to the privileged configuration is important, raise this weight to a value higher than the
            * weight of the main objectives.
            * Any value less than zero will be ignored.
            * A value of -1 will result in the solver using its default value.
            */
   public void setPrivilegedWeight(double privileged_weight)
   {
      privileged_weight_ = privileged_weight;
   }
   /**
            * The weight to use in the optimization for the privileged configuration.
            * When remaining close to the privileged configuration is important, raise this weight to a value higher than the
            * weight of the main objectives.
            * Any value less than zero will be ignored.
            * A value of -1 will result in the solver using its default value.
            */
   public double getPrivilegedWeight()
   {
      return privileged_weight_;
   }

   /**
            * The feedback proportional gain to use for the privileged configuration.
            * It is coupled to some extent to the privileged_weight
            * A value of -1 will result in the solver using its default value.
            */
   public void setPrivilegedGain(double privileged_gain)
   {
      privileged_gain_ = privileged_gain;
   }
   /**
            * The feedback proportional gain to use for the privileged configuration.
            * It is coupled to some extent to the privileged_weight
            * A value of -1 will result in the solver using its default value.
            */
   public double getPrivilegedGain()
   {
      return privileged_gain_;
   }

   /**
            * Specifies how much high joint velocity values should be penalized in the optimization problem.
            * A low value generally results in a reduce number of iterations before convergence but it also decreases the general stability of the solver.
            * A value of -1 will result in the solver using its default value.
            */
   public void setJointVelocityWeight(double joint_velocity_weight)
   {
      joint_velocity_weight_ = joint_velocity_weight;
   }
   /**
            * Specifies how much high joint velocity values should be penalized in the optimization problem.
            * A low value generally results in a reduce number of iterations before convergence but it also decreases the general stability of the solver.
            * A value of -1 will result in the solver using its default value.
            */
   public double getJointVelocityWeight()
   {
      return joint_velocity_weight_;
   }

   /**
            * Specifying how much high joint acceleration values should be penalized in the optimization problem.
            * A value of -1 will result in the solver using its default value.
            */
   public void setJointAccelerationWeight(double joint_acceleration_weight)
   {
      joint_acceleration_weight_ = joint_acceleration_weight;
   }
   /**
            * Specifying how much high joint acceleration values should be penalized in the optimization problem.
            * A value of -1 will result in the solver using its default value.
            */
   public double getJointAccelerationWeight()
   {
      return joint_acceleration_weight_;
   }

   /**
            * When true, the solver will enforce the joint velocity limits as defined in the robot model.
            * Enabling this restriction will augment the number of iteration before converging to a robot configuration for a given set of end-effector positions.
            */
   public void setEnableJointVelocityLimits(boolean enable_joint_velocity_limits)
   {
      enable_joint_velocity_limits_ = enable_joint_velocity_limits;
   }
   /**
            * When true, the solver will enforce the joint velocity limits as defined in the robot model.
            * Enabling this restriction will augment the number of iteration before converging to a robot configuration for a given set of end-effector positions.
            */
   public boolean getEnableJointVelocityLimits()
   {
      return enable_joint_velocity_limits_;
   }

   /**
            * When true, the solver will ignore the joint velocity limits.
            * Enabling this restriction will reduce the number of iteration before converging to a robot configuration for a given set of end-effector positions.
            */
   public void setDisableJointVelocityLimits(boolean disable_joint_velocity_limits)
   {
      disable_joint_velocity_limits_ = disable_joint_velocity_limits;
   }
   /**
            * When true, the solver will ignore the joint velocity limits.
            * Enabling this restriction will reduce the number of iteration before converging to a robot configuration for a given set of end-effector positions.
            */
   public boolean getDisableJointVelocityLimits()
   {
      return disable_joint_velocity_limits_;
   }

   /**
            * If the toolbox has been setup with the collision model of the robot, it will by default handle self-collision avoidance.
            * In case it has undesirable effects, use this flag to disable it.
            */
   public void setDisableCollisionAvoidance(boolean disable_collision_avoidance)
   {
      disable_collision_avoidance_ = disable_collision_avoidance;
   }
   /**
            * If the toolbox has been setup with the collision model of the robot, it will by default handle self-collision avoidance.
            * In case it has undesirable effects, use this flag to disable it.
            */
   public boolean getDisableCollisionAvoidance()
   {
      return disable_collision_avoidance_;
   }

   /**
            * In case collision avoidance has been disabled, use this flag to re-enable it while leaving disable_collision_avoidance to false.
            */
   public void setEnableCollisionAvoidance(boolean enable_collision_avoidance)
   {
      enable_collision_avoidance_ = enable_collision_avoidance;
   }
   /**
            * In case collision avoidance has been disabled, use this flag to re-enable it while leaving disable_collision_avoidance to false.
            */
   public boolean getEnableCollisionAvoidance()
   {
      return enable_collision_avoidance_;
   }


   public static Supplier<KinematicsToolboxConfigurationMessagePubSubType> getPubSubType()
   {
      return KinematicsToolboxConfigurationMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return KinematicsToolboxConfigurationMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(KinematicsToolboxConfigurationMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.use_privileged_root_joint_position_, other.use_privileged_root_joint_position_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.use_privileged_root_joint_orientation_, other.use_privileged_root_joint_orientation_, epsilon)) return false;

      if (!this.privileged_root_joint_position_.epsilonEquals(other.privileged_root_joint_position_, epsilon)) return false;
      if (!this.privileged_root_joint_orientation_.epsilonEquals(other.privileged_root_joint_orientation_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsIntegerSequence(this.privileged_joint_hash_codes_, other.privileged_joint_hash_codes_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsFloatSequence(this.privileged_joint_angles_, other.privileged_joint_angles_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.privileged_weight_, other.privileged_weight_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.privileged_gain_, other.privileged_gain_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.joint_velocity_weight_, other.joint_velocity_weight_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.joint_acceleration_weight_, other.joint_acceleration_weight_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.enable_joint_velocity_limits_, other.enable_joint_velocity_limits_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.disable_joint_velocity_limits_, other.disable_joint_velocity_limits_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.disable_collision_avoidance_, other.disable_collision_avoidance_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.enable_collision_avoidance_, other.enable_collision_avoidance_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof KinematicsToolboxConfigurationMessage)) return false;

      KinematicsToolboxConfigurationMessage otherMyClass = (KinematicsToolboxConfigurationMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.use_privileged_root_joint_position_ != otherMyClass.use_privileged_root_joint_position_) return false;

      if(this.use_privileged_root_joint_orientation_ != otherMyClass.use_privileged_root_joint_orientation_) return false;

      if (!this.privileged_root_joint_position_.equals(otherMyClass.privileged_root_joint_position_)) return false;
      if (!this.privileged_root_joint_orientation_.equals(otherMyClass.privileged_root_joint_orientation_)) return false;
      if (!this.privileged_joint_hash_codes_.equals(otherMyClass.privileged_joint_hash_codes_)) return false;
      if (!this.privileged_joint_angles_.equals(otherMyClass.privileged_joint_angles_)) return false;
      if(this.privileged_weight_ != otherMyClass.privileged_weight_) return false;

      if(this.privileged_gain_ != otherMyClass.privileged_gain_) return false;

      if(this.joint_velocity_weight_ != otherMyClass.joint_velocity_weight_) return false;

      if(this.joint_acceleration_weight_ != otherMyClass.joint_acceleration_weight_) return false;

      if(this.enable_joint_velocity_limits_ != otherMyClass.enable_joint_velocity_limits_) return false;

      if(this.disable_joint_velocity_limits_ != otherMyClass.disable_joint_velocity_limits_) return false;

      if(this.disable_collision_avoidance_ != otherMyClass.disable_collision_avoidance_) return false;

      if(this.enable_collision_avoidance_ != otherMyClass.enable_collision_avoidance_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("KinematicsToolboxConfigurationMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("use_privileged_root_joint_position=");
      builder.append(this.use_privileged_root_joint_position_);      builder.append(", ");
      builder.append("use_privileged_root_joint_orientation=");
      builder.append(this.use_privileged_root_joint_orientation_);      builder.append(", ");
      builder.append("privileged_root_joint_position=");
      builder.append(this.privileged_root_joint_position_);      builder.append(", ");
      builder.append("privileged_root_joint_orientation=");
      builder.append(this.privileged_root_joint_orientation_);      builder.append(", ");
      builder.append("privileged_joint_hash_codes=");
      builder.append(this.privileged_joint_hash_codes_);      builder.append(", ");
      builder.append("privileged_joint_angles=");
      builder.append(this.privileged_joint_angles_);      builder.append(", ");
      builder.append("privileged_weight=");
      builder.append(this.privileged_weight_);      builder.append(", ");
      builder.append("privileged_gain=");
      builder.append(this.privileged_gain_);      builder.append(", ");
      builder.append("joint_velocity_weight=");
      builder.append(this.joint_velocity_weight_);      builder.append(", ");
      builder.append("joint_acceleration_weight=");
      builder.append(this.joint_acceleration_weight_);      builder.append(", ");
      builder.append("enable_joint_velocity_limits=");
      builder.append(this.enable_joint_velocity_limits_);      builder.append(", ");
      builder.append("disable_joint_velocity_limits=");
      builder.append(this.disable_joint_velocity_limits_);      builder.append(", ");
      builder.append("disable_collision_avoidance=");
      builder.append(this.disable_collision_avoidance_);      builder.append(", ");
      builder.append("enable_collision_avoidance=");
      builder.append(this.enable_collision_avoidance_);
      builder.append("}");
      return builder.toString();
   }
}
