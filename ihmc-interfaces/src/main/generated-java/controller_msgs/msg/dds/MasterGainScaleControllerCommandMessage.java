package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class MasterGainScaleControllerCommandMessage extends Packet<MasterGainScaleControllerCommandMessage> implements Settable<MasterGainScaleControllerCommandMessage>, EpsilonComparable<MasterGainScaleControllerCommandMessage>
{
   /**
            * True if this is a request to servo the robot
            * This is used to enable and ramp up the robot's controller gains, to go into freeze or stand prep
            */
   public boolean servo_robot_;
   /**
            * True is this is a request to immediately zero the gains which can be violent and dangerous
            * This will also turn off the HPU
            */
   public boolean unservo_immediately_;
   /**
            * True if this is a request to slowly ramp down the robot's controller gains to zero, in prepration for robot shutdown
            * This will also turn off the HPU at the end
            */
   public boolean unservo_slowly_;

   public MasterGainScaleControllerCommandMessage()
   {
   }

   public MasterGainScaleControllerCommandMessage(MasterGainScaleControllerCommandMessage other)
   {
      this();
      set(other);
   }

   public void set(MasterGainScaleControllerCommandMessage other)
   {
      servo_robot_ = other.servo_robot_;

      unservo_immediately_ = other.unservo_immediately_;

      unservo_slowly_ = other.unservo_slowly_;

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
            * This will also turn off the HPU
            */
   public void setUnservoImmediately(boolean unservo_immediately)
   {
      unservo_immediately_ = unservo_immediately;
   }
   /**
            * True is this is a request to immediately zero the gains which can be violent and dangerous
            * This will also turn off the HPU
            */
   public boolean getUnservoImmediately()
   {
      return unservo_immediately_;
   }

   /**
            * True if this is a request to slowly ramp down the robot's controller gains to zero, in prepration for robot shutdown
            * This will also turn off the HPU at the end
            */
   public void setUnservoSlowly(boolean unservo_slowly)
   {
      unservo_slowly_ = unservo_slowly;
   }
   /**
            * True if this is a request to slowly ramp down the robot's controller gains to zero, in prepration for robot shutdown
            * This will also turn off the HPU at the end
            */
   public boolean getUnservoSlowly()
   {
      return unservo_slowly_;
   }


   public static Supplier<MasterGainScaleControllerCommandMessagePubSubType> getPubSubType()
   {
      return MasterGainScaleControllerCommandMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return MasterGainScaleControllerCommandMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(MasterGainScaleControllerCommandMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

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
      if(!(other instanceof MasterGainScaleControllerCommandMessage)) return false;

      MasterGainScaleControllerCommandMessage otherMyClass = (MasterGainScaleControllerCommandMessage) other;

      if(this.servo_robot_ != otherMyClass.servo_robot_) return false;

      if(this.unservo_immediately_ != otherMyClass.unservo_immediately_) return false;

      if(this.unservo_slowly_ != otherMyClass.unservo_slowly_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("MasterGainScaleControllerCommandMessage {");
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
