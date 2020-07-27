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

   public KinematicsToolboxConfigurationMessage()
   {










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
      builder.append(this.enable_input_persistence_);
      builder.append("}");
      return builder.toString();
   }
}
