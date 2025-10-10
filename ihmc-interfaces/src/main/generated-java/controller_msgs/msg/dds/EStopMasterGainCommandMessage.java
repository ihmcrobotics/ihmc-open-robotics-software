package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * A message for operating the e-stop and master gain controller variables
       */
public class EStopMasterGainCommandMessage extends Packet<EStopMasterGainCommandMessage> implements Settable<EStopMasterGainCommandMessage>, EpsilonComparable<EStopMasterGainCommandMessage>
{
   /**
            * SOFT-E-STOP to set
            */
   public boolean estop_;
   /**
            * AvatarLowLevelOutputProcessor masterGain variable to set
            */
   public double master_gain_;
   /**
            * True if this is a request to servo the robot
            * This is used to enable and ramp up the robot's controller gains, to go into freeze or stand prep
            */
   public boolean servo_robot_;
   /**
            * True is this is a request to immediately zero the gains which can be violent and dangerous
            */
   public boolean unservo_immediately_;
   /**
            * True if this is a request to slowly ramp down the robot's controller gains to zero, in prepration for robot shutdown
            */
   public boolean unservo_slowly_;

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
      estop_ = other.estop_;

      master_gain_ = other.master_gain_;

      servo_robot_ = other.servo_robot_;

      unservo_immediately_ = other.unservo_immediately_;

      unservo_slowly_ = other.unservo_slowly_;

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

   /**
            * AvatarLowLevelOutputProcessor masterGain variable to set
            */
   public void setMasterGain(double master_gain)
   {
      master_gain_ = master_gain;
   }
   /**
            * AvatarLowLevelOutputProcessor masterGain variable to set
            */
   public double getMasterGain()
   {
      return master_gain_;
   }

   /**
            * True if this is a request to servo the robot
            * This is used to enable and ramp up the robot's controller gains, to go into freeze or stand prep
            */
   public void setServoRobot(boolean servo_robot)
   {
      servo_robot_ = servo_robot;
   }
   /**
            * True if this is a request to servo the robot
            * This is used to enable and ramp up the robot's controller gains, to go into freeze or stand prep
            */
   public boolean getServoRobot()
   {
      return servo_robot_;
   }

   /**
            * True is this is a request to immediately zero the gains which can be violent and dangerous
            */
   public void setUnservoImmediately(boolean unservo_immediately)
   {
      unservo_immediately_ = unservo_immediately;
   }
   /**
            * True is this is a request to immediately zero the gains which can be violent and dangerous
            */
   public boolean getUnservoImmediately()
   {
      return unservo_immediately_;
   }

   /**
            * True if this is a request to slowly ramp down the robot's controller gains to zero, in prepration for robot shutdown
            */
   public void setUnservoSlowly(boolean unservo_slowly)
   {
      unservo_slowly_ = unservo_slowly;
   }
   /**
            * True if this is a request to slowly ramp down the robot's controller gains to zero, in prepration for robot shutdown
            */
   public boolean getUnservoSlowly()
   {
      return unservo_slowly_;
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

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.estop_, other.estop_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.master_gain_, other.master_gain_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.servo_robot_, other.servo_robot_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.unservo_immediately_, other.unservo_immediately_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.unservo_slowly_, other.unservo_slowly_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof EStopMasterGainCommandMessage)) return false;

      EStopMasterGainCommandMessage otherMyClass = (EStopMasterGainCommandMessage) other;

      if(this.estop_ != otherMyClass.estop_) return false;

      if(this.master_gain_ != otherMyClass.master_gain_) return false;

      if(this.servo_robot_ != otherMyClass.servo_robot_) return false;

      if(this.unservo_immediately_ != otherMyClass.unservo_immediately_) return false;

      if(this.unservo_slowly_ != otherMyClass.unservo_slowly_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("EStopMasterGainCommandMessage {");
      builder.append("estop=");
      builder.append(this.estop_);      builder.append(", ");
      builder.append("master_gain=");
      builder.append(this.master_gain_);      builder.append(", ");
      builder.append("servo_robot=");
      builder.append(this.servo_robot_);      builder.append(", ");
      builder.append("unservo_immediately=");
      builder.append(this.unservo_immediately_);      builder.append(", ");
      builder.append("unservo_slowly=");
      builder.append(this.unservo_slowly_);
      builder.append("}");
      return builder.toString();
   }
}
