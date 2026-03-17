package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * A message for sending necessary startup, shutdown, and high-level operational command variables to the robot
       */
public class EStopMasterGainCommandMessage extends Packet<EStopMasterGainCommandMessage> implements Settable<EStopMasterGainCommandMessage>, EpsilonComparable<EStopMasterGainCommandMessage>
{
   /**
            * Auto startup and shutdown
            */
   public boolean request_startup_;
   public boolean request_shutdown_;
   public boolean execute_startup_shutdown_;
   /**
            * SOFT-E-STOP to set
            */
   public boolean estop_;
   public boolean execute_estop_;
   /**
            * AvatarLowLevelOutputProcessor masterGain variable to set
            */
   public double desired_master_gain_;
   public boolean execute_master_gain_;
   /**
            * True if this is a request to servo the robot
            * This is used to enable and ramp up/down the robot's controller gains, to go into freeze or stand prep
            */
   public boolean servo_robot_;
   public boolean execute_servo_robot_;
   /**
            * True if this is a request to immediately zero the gains which can be violent and dangerous
            */
   public boolean unservo_quickly_;
   public boolean execute_unservo_quickly_;
   /**
            * Enable publishing of ROS commands to the robot
            */
   public boolean enable_publishing_to_robot_;
   public boolean execute_enable_publishing_to_robot_;
   /**
            * Clear robot faults
            */
   public boolean clear_faults_;
   public boolean execute_clear_faults_;
   /**
            * Calibrate robot
            */
   public boolean calibrate_robot_;
   public boolean execute_calibrate_robot_;
   /**
            * Enable all actuators
            */
   public boolean enable_actuators_;
   public boolean execute_enable_actuators_;

   public EStopMasterGainCommandMessage()
   {
   }

   public EStopMasterGainCommandMessage(EStopMasterGainCommandMessage other)
   {
      this();
      set(other);
   }

   public void set(EStopMasterGainCommandMessage other)
   {
      request_startup_ = other.request_startup_;

      request_shutdown_ = other.request_shutdown_;

      execute_startup_shutdown_ = other.execute_startup_shutdown_;

      estop_ = other.estop_;

      execute_estop_ = other.execute_estop_;

      desired_master_gain_ = other.desired_master_gain_;

      execute_master_gain_ = other.execute_master_gain_;

      servo_robot_ = other.servo_robot_;

      execute_servo_robot_ = other.execute_servo_robot_;

      unservo_quickly_ = other.unservo_quickly_;

      execute_unservo_quickly_ = other.execute_unservo_quickly_;

      enable_publishing_to_robot_ = other.enable_publishing_to_robot_;

      execute_enable_publishing_to_robot_ = other.execute_enable_publishing_to_robot_;

      clear_faults_ = other.clear_faults_;

      execute_clear_faults_ = other.execute_clear_faults_;

      calibrate_robot_ = other.calibrate_robot_;

      execute_calibrate_robot_ = other.execute_calibrate_robot_;

      enable_actuators_ = other.enable_actuators_;

      execute_enable_actuators_ = other.execute_enable_actuators_;

   }

   /**
            * Auto startup and shutdown
            */
   public void setRequestStartup(boolean request_startup)
   {
      request_startup_ = request_startup;
   }
   /**
            * Auto startup and shutdown
            */
   public boolean getRequestStartup()
   {
      return request_startup_;
   }

   public void setRequestShutdown(boolean request_shutdown)
   {
      request_shutdown_ = request_shutdown;
   }
   public boolean getRequestShutdown()
   {
      return request_shutdown_;
   }

   public void setExecuteStartupShutdown(boolean execute_startup_shutdown)
   {
      execute_startup_shutdown_ = execute_startup_shutdown;
   }
   public boolean getExecuteStartupShutdown()
   {
      return execute_startup_shutdown_;
   }

   /**
            * SOFT-E-STOP to set
            */
   public void setEstop(boolean estop)
   {
      estop_ = estop;
   }
   /**
            * SOFT-E-STOP to set
            */
   public boolean getEstop()
   {
      return estop_;
   }

   public void setExecuteEstop(boolean execute_estop)
   {
      execute_estop_ = execute_estop;
   }
   public boolean getExecuteEstop()
   {
      return execute_estop_;
   }

   /**
            * AvatarLowLevelOutputProcessor masterGain variable to set
            */
   public void setDesiredMasterGain(double desired_master_gain)
   {
      desired_master_gain_ = desired_master_gain;
   }
   /**
            * AvatarLowLevelOutputProcessor masterGain variable to set
            */
   public double getDesiredMasterGain()
   {
      return desired_master_gain_;
   }

   public void setExecuteMasterGain(boolean execute_master_gain)
   {
      execute_master_gain_ = execute_master_gain;
   }
   public boolean getExecuteMasterGain()
   {
      return execute_master_gain_;
   }

   /**
            * True if this is a request to servo the robot
            * This is used to enable and ramp up/down the robot's controller gains, to go into freeze or stand prep
            */
   public void setServoRobot(boolean servo_robot)
   {
      servo_robot_ = servo_robot;
   }
   /**
            * True if this is a request to servo the robot
            * This is used to enable and ramp up/down the robot's controller gains, to go into freeze or stand prep
            */
   public boolean getServoRobot()
   {
      return servo_robot_;
   }

   public void setExecuteServoRobot(boolean execute_servo_robot)
   {
      execute_servo_robot_ = execute_servo_robot;
   }
   public boolean getExecuteServoRobot()
   {
      return execute_servo_robot_;
   }

   /**
            * True if this is a request to immediately zero the gains which can be violent and dangerous
            */
   public void setUnservoQuickly(boolean unservo_quickly)
   {
      unservo_quickly_ = unservo_quickly;
   }
   /**
            * True if this is a request to immediately zero the gains which can be violent and dangerous
            */
   public boolean getUnservoQuickly()
   {
      return unservo_quickly_;
   }

   public void setExecuteUnservoQuickly(boolean execute_unservo_quickly)
   {
      execute_unservo_quickly_ = execute_unservo_quickly;
   }
   public boolean getExecuteUnservoQuickly()
   {
      return execute_unservo_quickly_;
   }

   /**
            * Enable publishing of ROS commands to the robot
            */
   public void setEnablePublishingToRobot(boolean enable_publishing_to_robot)
   {
      enable_publishing_to_robot_ = enable_publishing_to_robot;
   }
   /**
            * Enable publishing of ROS commands to the robot
            */
   public boolean getEnablePublishingToRobot()
   {
      return enable_publishing_to_robot_;
   }

   public void setExecuteEnablePublishingToRobot(boolean execute_enable_publishing_to_robot)
   {
      execute_enable_publishing_to_robot_ = execute_enable_publishing_to_robot;
   }
   public boolean getExecuteEnablePublishingToRobot()
   {
      return execute_enable_publishing_to_robot_;
   }

   /**
            * Clear robot faults
            */
   public void setClearFaults(boolean clear_faults)
   {
      clear_faults_ = clear_faults;
   }
   /**
            * Clear robot faults
            */
   public boolean getClearFaults()
   {
      return clear_faults_;
   }

   public void setExecuteClearFaults(boolean execute_clear_faults)
   {
      execute_clear_faults_ = execute_clear_faults;
   }
   public boolean getExecuteClearFaults()
   {
      return execute_clear_faults_;
   }

   /**
            * Calibrate robot
            */
   public void setCalibrateRobot(boolean calibrate_robot)
   {
      calibrate_robot_ = calibrate_robot;
   }
   /**
            * Calibrate robot
            */
   public boolean getCalibrateRobot()
   {
      return calibrate_robot_;
   }

   public void setExecuteCalibrateRobot(boolean execute_calibrate_robot)
   {
      execute_calibrate_robot_ = execute_calibrate_robot;
   }
   public boolean getExecuteCalibrateRobot()
   {
      return execute_calibrate_robot_;
   }

   /**
            * Enable all actuators
            */
   public void setEnableActuators(boolean enable_actuators)
   {
      enable_actuators_ = enable_actuators;
   }
   /**
            * Enable all actuators
            */
   public boolean getEnableActuators()
   {
      return enable_actuators_;
   }

   public void setExecuteEnableActuators(boolean execute_enable_actuators)
   {
      execute_enable_actuators_ = execute_enable_actuators;
   }
   public boolean getExecuteEnableActuators()
   {
      return execute_enable_actuators_;
   }


   public static Supplier<EStopMasterGainCommandMessagePubSubType> getPubSubType()
   {
      return EStopMasterGainCommandMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return EStopMasterGainCommandMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(EStopMasterGainCommandMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.request_startup_, other.request_startup_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.request_shutdown_, other.request_shutdown_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.execute_startup_shutdown_, other.execute_startup_shutdown_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.estop_, other.estop_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.execute_estop_, other.execute_estop_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.desired_master_gain_, other.desired_master_gain_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.execute_master_gain_, other.execute_master_gain_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.servo_robot_, other.servo_robot_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.execute_servo_robot_, other.execute_servo_robot_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.unservo_quickly_, other.unservo_quickly_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.execute_unservo_quickly_, other.execute_unservo_quickly_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.enable_publishing_to_robot_, other.enable_publishing_to_robot_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.execute_enable_publishing_to_robot_, other.execute_enable_publishing_to_robot_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.clear_faults_, other.clear_faults_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.execute_clear_faults_, other.execute_clear_faults_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.calibrate_robot_, other.calibrate_robot_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.execute_calibrate_robot_, other.execute_calibrate_robot_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.enable_actuators_, other.enable_actuators_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.execute_enable_actuators_, other.execute_enable_actuators_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof EStopMasterGainCommandMessage)) return false;

      EStopMasterGainCommandMessage otherMyClass = (EStopMasterGainCommandMessage) other;

      if(this.request_startup_ != otherMyClass.request_startup_) return false;

      if(this.request_shutdown_ != otherMyClass.request_shutdown_) return false;

      if(this.execute_startup_shutdown_ != otherMyClass.execute_startup_shutdown_) return false;

      if(this.estop_ != otherMyClass.estop_) return false;

      if(this.execute_estop_ != otherMyClass.execute_estop_) return false;

      if(this.desired_master_gain_ != otherMyClass.desired_master_gain_) return false;

      if(this.execute_master_gain_ != otherMyClass.execute_master_gain_) return false;

      if(this.servo_robot_ != otherMyClass.servo_robot_) return false;

      if(this.execute_servo_robot_ != otherMyClass.execute_servo_robot_) return false;

      if(this.unservo_quickly_ != otherMyClass.unservo_quickly_) return false;

      if(this.execute_unservo_quickly_ != otherMyClass.execute_unservo_quickly_) return false;

      if(this.enable_publishing_to_robot_ != otherMyClass.enable_publishing_to_robot_) return false;

      if(this.execute_enable_publishing_to_robot_ != otherMyClass.execute_enable_publishing_to_robot_) return false;

      if(this.clear_faults_ != otherMyClass.clear_faults_) return false;

      if(this.execute_clear_faults_ != otherMyClass.execute_clear_faults_) return false;

      if(this.calibrate_robot_ != otherMyClass.calibrate_robot_) return false;

      if(this.execute_calibrate_robot_ != otherMyClass.execute_calibrate_robot_) return false;

      if(this.enable_actuators_ != otherMyClass.enable_actuators_) return false;

      if(this.execute_enable_actuators_ != otherMyClass.execute_enable_actuators_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("EStopMasterGainCommandMessage {");
      builder.append("request_startup=");
      builder.append(this.request_startup_);      builder.append(", ");
      builder.append("request_shutdown=");
      builder.append(this.request_shutdown_);      builder.append(", ");
      builder.append("execute_startup_shutdown=");
      builder.append(this.execute_startup_shutdown_);      builder.append(", ");
      builder.append("estop=");
      builder.append(this.estop_);      builder.append(", ");
      builder.append("execute_estop=");
      builder.append(this.execute_estop_);      builder.append(", ");
      builder.append("desired_master_gain=");
      builder.append(this.desired_master_gain_);      builder.append(", ");
      builder.append("execute_master_gain=");
      builder.append(this.execute_master_gain_);      builder.append(", ");
      builder.append("servo_robot=");
      builder.append(this.servo_robot_);      builder.append(", ");
      builder.append("execute_servo_robot=");
      builder.append(this.execute_servo_robot_);      builder.append(", ");
      builder.append("unservo_quickly=");
      builder.append(this.unservo_quickly_);      builder.append(", ");
      builder.append("execute_unservo_quickly=");
      builder.append(this.execute_unservo_quickly_);      builder.append(", ");
      builder.append("enable_publishing_to_robot=");
      builder.append(this.enable_publishing_to_robot_);      builder.append(", ");
      builder.append("execute_enable_publishing_to_robot=");
      builder.append(this.execute_enable_publishing_to_robot_);      builder.append(", ");
      builder.append("clear_faults=");
      builder.append(this.clear_faults_);      builder.append(", ");
      builder.append("execute_clear_faults=");
      builder.append(this.execute_clear_faults_);      builder.append(", ");
      builder.append("calibrate_robot=");
      builder.append(this.calibrate_robot_);      builder.append(", ");
      builder.append("execute_calibrate_robot=");
      builder.append(this.execute_calibrate_robot_);      builder.append(", ");
      builder.append("enable_actuators=");
      builder.append(this.enable_actuators_);      builder.append(", ");
      builder.append("execute_enable_actuators=");
      builder.append(this.execute_enable_actuators_);
      builder.append("}");
      return builder.toString();
   }
}
