package toolbox_msgs.msg.dds;

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
   /**
            * By default, the kinematics toolbox will preserve all input that has been provided until it is reinitialized.
            * For instance, if the user sends a command for controlling a certain body, the command will persist even if the user stops sending it.
            * A command that persists is overridden if another is received for the same body.
            * Priviliged configuration inputs are not affected by this flag.
            * In case persistence of inputs is not desired, use this flag to disable it.
            * When persistence is disabled, it is strongly recommended to use KinematicsToolboxInputCollectionMessage to send inputs to the toolbox.
            */
   public boolean disable_input_persistence_;
   /**
            * In case input persistence has been disabled, use this flat to re-enable it while leaving disable_input_persistence to false.
            */
   public boolean enable_input_persistence_;
   /**
            * When true, the solver enforces the solution to have the projection of the center of mass contained
            * inside the current support polygon. The support polygon can be determined automatically using the controller's ouput if running.
            * The support polygon can also be defined by the user by sending a KinematicsToolboxContactStateMessage.
            */
   public boolean enable_support_polygon_constraint_;
   /**
            * When true, this disables the support polygon constraint on the center of mass.
            */
   public boolean disable_support_polygon_constraint_;
   /**
            * Hash codes of the joints to be deactivated in the solver
            */
   public us.ihmc.idl.IDLSequence.Integer  joints_to_deactivate_;
   /**
            * Hash codes of the joints to be re-activated in the solver
            */
   public us.ihmc.idl.IDLSequence.Integer  joints_to_activate_;

   public KinematicsToolboxConfigurationMessage()
   {
      joints_to_deactivate_ = new us.ihmc.idl.IDLSequence.Integer (10, "type_2");

      joints_to_activate_ = new us.ihmc.idl.IDLSequence.Integer (10, "type_2");

   }

   public KinematicsToolboxConfigurationMessage(KinematicsToolboxConfigurationMessage other)
   {
      this();
      set(other);
   }

   public void set(KinematicsToolboxConfigurationMessage other)
   {
      sequence_id_ = other.sequence_id_;

      joint_velocity_weight_ = other.joint_velocity_weight_;

      joint_acceleration_weight_ = other.joint_acceleration_weight_;

      enable_joint_velocity_limits_ = other.enable_joint_velocity_limits_;

      disable_joint_velocity_limits_ = other.disable_joint_velocity_limits_;

      disable_collision_avoidance_ = other.disable_collision_avoidance_;

      enable_collision_avoidance_ = other.enable_collision_avoidance_;

      disable_input_persistence_ = other.disable_input_persistence_;

      enable_input_persistence_ = other.enable_input_persistence_;

      enable_support_polygon_constraint_ = other.enable_support_polygon_constraint_;

      disable_support_polygon_constraint_ = other.disable_support_polygon_constraint_;

      joints_to_deactivate_.set(other.joints_to_deactivate_);
      joints_to_activate_.set(other.joints_to_activate_);
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

   /**
            * By default, the kinematics toolbox will preserve all input that has been provided until it is reinitialized.
            * For instance, if the user sends a command for controlling a certain body, the command will persist even if the user stops sending it.
            * A command that persists is overridden if another is received for the same body.
            * Priviliged configuration inputs are not affected by this flag.
            * In case persistence of inputs is not desired, use this flag to disable it.
            * When persistence is disabled, it is strongly recommended to use KinematicsToolboxInputCollectionMessage to send inputs to the toolbox.
            */
   public void setDisableInputPersistence(boolean disable_input_persistence)
   {
      disable_input_persistence_ = disable_input_persistence;
   }
   /**
            * By default, the kinematics toolbox will preserve all input that has been provided until it is reinitialized.
            * For instance, if the user sends a command for controlling a certain body, the command will persist even if the user stops sending it.
            * A command that persists is overridden if another is received for the same body.
            * Priviliged configuration inputs are not affected by this flag.
            * In case persistence of inputs is not desired, use this flag to disable it.
            * When persistence is disabled, it is strongly recommended to use KinematicsToolboxInputCollectionMessage to send inputs to the toolbox.
            */
   public boolean getDisableInputPersistence()
   {
      return disable_input_persistence_;
   }

   /**
            * In case input persistence has been disabled, use this flat to re-enable it while leaving disable_input_persistence to false.
            */
   public void setEnableInputPersistence(boolean enable_input_persistence)
   {
      enable_input_persistence_ = enable_input_persistence;
   }
   /**
            * In case input persistence has been disabled, use this flat to re-enable it while leaving disable_input_persistence to false.
            */
   public boolean getEnableInputPersistence()
   {
      return enable_input_persistence_;
   }

   /**
            * When true, the solver enforces the solution to have the projection of the center of mass contained
            * inside the current support polygon. The support polygon can be determined automatically using the controller's ouput if running.
            * The support polygon can also be defined by the user by sending a KinematicsToolboxContactStateMessage.
            */
   public void setEnableSupportPolygonConstraint(boolean enable_support_polygon_constraint)
   {
      enable_support_polygon_constraint_ = enable_support_polygon_constraint;
   }
   /**
            * When true, the solver enforces the solution to have the projection of the center of mass contained
            * inside the current support polygon. The support polygon can be determined automatically using the controller's ouput if running.
            * The support polygon can also be defined by the user by sending a KinematicsToolboxContactStateMessage.
            */
   public boolean getEnableSupportPolygonConstraint()
   {
      return enable_support_polygon_constraint_;
   }

   /**
            * When true, this disables the support polygon constraint on the center of mass.
            */
   public void setDisableSupportPolygonConstraint(boolean disable_support_polygon_constraint)
   {
      disable_support_polygon_constraint_ = disable_support_polygon_constraint;
   }
   /**
            * When true, this disables the support polygon constraint on the center of mass.
            */
   public boolean getDisableSupportPolygonConstraint()
   {
      return disable_support_polygon_constraint_;
   }


   /**
            * Hash codes of the joints to be deactivated in the solver
            */
   public us.ihmc.idl.IDLSequence.Integer  getJointsToDeactivate()
   {
      return joints_to_deactivate_;
   }


   /**
            * Hash codes of the joints to be re-activated in the solver
            */
   public us.ihmc.idl.IDLSequence.Integer  getJointsToActivate()
   {
      return joints_to_activate_;
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

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.joint_velocity_weight_, other.joint_velocity_weight_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.joint_acceleration_weight_, other.joint_acceleration_weight_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.enable_joint_velocity_limits_, other.enable_joint_velocity_limits_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.disable_joint_velocity_limits_, other.disable_joint_velocity_limits_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.disable_collision_avoidance_, other.disable_collision_avoidance_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.enable_collision_avoidance_, other.enable_collision_avoidance_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.disable_input_persistence_, other.disable_input_persistence_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.enable_input_persistence_, other.enable_input_persistence_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.enable_support_polygon_constraint_, other.enable_support_polygon_constraint_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.disable_support_polygon_constraint_, other.disable_support_polygon_constraint_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsIntegerSequence(this.joints_to_deactivate_, other.joints_to_deactivate_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsIntegerSequence(this.joints_to_activate_, other.joints_to_activate_, epsilon)) return false;


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

      if(this.joint_velocity_weight_ != otherMyClass.joint_velocity_weight_) return false;

      if(this.joint_acceleration_weight_ != otherMyClass.joint_acceleration_weight_) return false;

      if(this.enable_joint_velocity_limits_ != otherMyClass.enable_joint_velocity_limits_) return false;

      if(this.disable_joint_velocity_limits_ != otherMyClass.disable_joint_velocity_limits_) return false;

      if(this.disable_collision_avoidance_ != otherMyClass.disable_collision_avoidance_) return false;

      if(this.enable_collision_avoidance_ != otherMyClass.enable_collision_avoidance_) return false;

      if(this.disable_input_persistence_ != otherMyClass.disable_input_persistence_) return false;

      if(this.enable_input_persistence_ != otherMyClass.enable_input_persistence_) return false;

      if(this.enable_support_polygon_constraint_ != otherMyClass.enable_support_polygon_constraint_) return false;

      if(this.disable_support_polygon_constraint_ != otherMyClass.disable_support_polygon_constraint_) return false;

      if (!this.joints_to_deactivate_.equals(otherMyClass.joints_to_deactivate_)) return false;
      if (!this.joints_to_activate_.equals(otherMyClass.joints_to_activate_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("KinematicsToolboxConfigurationMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
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
      builder.append(this.enable_collision_avoidance_);      builder.append(", ");
      builder.append("disable_input_persistence=");
      builder.append(this.disable_input_persistence_);      builder.append(", ");
      builder.append("enable_input_persistence=");
      builder.append(this.enable_input_persistence_);      builder.append(", ");
      builder.append("enable_support_polygon_constraint=");
      builder.append(this.enable_support_polygon_constraint_);      builder.append(", ");
      builder.append("disable_support_polygon_constraint=");
      builder.append(this.disable_support_polygon_constraint_);      builder.append(", ");
      builder.append("joints_to_deactivate=");
      builder.append(this.joints_to_deactivate_);      builder.append(", ");
      builder.append("joints_to_activate=");
      builder.append(this.joints_to_activate_);
      builder.append("}");
      return builder.toString();
   }
}
