package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * A message for allowing operator to observe the status of various startup/shutdown variables often required for robot operation
       */
public class EStopMasterGainStatusMessage extends Packet<EStopMasterGainStatusMessage> implements Settable<EStopMasterGainStatusMessage>, EpsilonComparable<EStopMasterGainStatusMessage>
{
   /**
            * Current SOFT-E-STOP value
            */
   public boolean is_estopped_;
   /**
            * Is robot faulted
            */
   public boolean robot_is_faulted_;
   /**
            * Is robot servod
            */
   public boolean robot_is_servod_;
   /**
            * Is robot calibrated
            */
   public boolean robot_is_calibrated_;
   /**
            * Is publishing ROS commands to robot enabled
            */
   public boolean publishing_to_robot_is_enabled_;
   /**
            * Are the actuators enabled
            */
   public boolean actuators_are_enabled_;
   /**
            * The current master gain
            */
   public double current_master_gain_;

   public EStopMasterGainStatusMessage()
   {
   }

   public EStopMasterGainStatusMessage(EStopMasterGainStatusMessage other)
   {
      this();
      set(other);
   }

   public void set(EStopMasterGainStatusMessage other)
   {
      is_estopped_ = other.is_estopped_;

      robot_is_faulted_ = other.robot_is_faulted_;

      robot_is_servod_ = other.robot_is_servod_;

      robot_is_calibrated_ = other.robot_is_calibrated_;

      publishing_to_robot_is_enabled_ = other.publishing_to_robot_is_enabled_;

      actuators_are_enabled_ = other.actuators_are_enabled_;

      current_master_gain_ = other.current_master_gain_;

   }

   /**
            * Current SOFT-E-STOP value
            */
   public void setIsEstopped(boolean is_estopped)
   {
      is_estopped_ = is_estopped;
   }
   /**
            * Current SOFT-E-STOP value
            */
   public boolean getIsEstopped()
   {
      return is_estopped_;
   }

   /**
            * Is robot faulted
            */
   public void setRobotIsFaulted(boolean robot_is_faulted)
   {
      robot_is_faulted_ = robot_is_faulted;
   }
   /**
            * Is robot faulted
            */
   public boolean getRobotIsFaulted()
   {
      return robot_is_faulted_;
   }

   /**
            * Is robot servod
            */
   public void setRobotIsServod(boolean robot_is_servod)
   {
      robot_is_servod_ = robot_is_servod;
   }
   /**
            * Is robot servod
            */
   public boolean getRobotIsServod()
   {
      return robot_is_servod_;
   }

   /**
            * Is robot calibrated
            */
   public void setRobotIsCalibrated(boolean robot_is_calibrated)
   {
      robot_is_calibrated_ = robot_is_calibrated;
   }
   /**
            * Is robot calibrated
            */
   public boolean getRobotIsCalibrated()
   {
      return robot_is_calibrated_;
   }

   /**
            * Is publishing ROS commands to robot enabled
            */
   public void setPublishingToRobotIsEnabled(boolean publishing_to_robot_is_enabled)
   {
      publishing_to_robot_is_enabled_ = publishing_to_robot_is_enabled;
   }
   /**
            * Is publishing ROS commands to robot enabled
            */
   public boolean getPublishingToRobotIsEnabled()
   {
      return publishing_to_robot_is_enabled_;
   }

   /**
            * Are the actuators enabled
            */
   public void setActuatorsAreEnabled(boolean actuators_are_enabled)
   {
      actuators_are_enabled_ = actuators_are_enabled;
   }
   /**
            * Are the actuators enabled
            */
   public boolean getActuatorsAreEnabled()
   {
      return actuators_are_enabled_;
   }

   /**
            * The current master gain
            */
   public void setCurrentMasterGain(double current_master_gain)
   {
      current_master_gain_ = current_master_gain;
   }
   /**
            * The current master gain
            */
   public double getCurrentMasterGain()
   {
      return current_master_gain_;
   }


   public static Supplier<EStopMasterGainStatusMessagePubSubType> getPubSubType()
   {
      return EStopMasterGainStatusMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return EStopMasterGainStatusMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(EStopMasterGainStatusMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.is_estopped_, other.is_estopped_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.robot_is_faulted_, other.robot_is_faulted_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.robot_is_servod_, other.robot_is_servod_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.robot_is_calibrated_, other.robot_is_calibrated_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.publishing_to_robot_is_enabled_, other.publishing_to_robot_is_enabled_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.actuators_are_enabled_, other.actuators_are_enabled_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.current_master_gain_, other.current_master_gain_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof EStopMasterGainStatusMessage)) return false;

      EStopMasterGainStatusMessage otherMyClass = (EStopMasterGainStatusMessage) other;

      if(this.is_estopped_ != otherMyClass.is_estopped_) return false;

      if(this.robot_is_faulted_ != otherMyClass.robot_is_faulted_) return false;

      if(this.robot_is_servod_ != otherMyClass.robot_is_servod_) return false;

      if(this.robot_is_calibrated_ != otherMyClass.robot_is_calibrated_) return false;

      if(this.publishing_to_robot_is_enabled_ != otherMyClass.publishing_to_robot_is_enabled_) return false;

      if(this.actuators_are_enabled_ != otherMyClass.actuators_are_enabled_) return false;

      if(this.current_master_gain_ != otherMyClass.current_master_gain_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("EStopMasterGainStatusMessage {");
      builder.append("is_estopped=");
      builder.append(this.is_estopped_);      builder.append(", ");
      builder.append("robot_is_faulted=");
      builder.append(this.robot_is_faulted_);      builder.append(", ");
      builder.append("robot_is_servod=");
      builder.append(this.robot_is_servod_);      builder.append(", ");
      builder.append("robot_is_calibrated=");
      builder.append(this.robot_is_calibrated_);      builder.append(", ");
      builder.append("publishing_to_robot_is_enabled=");
      builder.append(this.publishing_to_robot_is_enabled_);      builder.append(", ");
      builder.append("actuators_are_enabled=");
      builder.append(this.actuators_are_enabled_);      builder.append(", ");
      builder.append("current_master_gain=");
      builder.append(this.current_master_gain_);
      builder.append("}");
      return builder.toString();
   }
}
