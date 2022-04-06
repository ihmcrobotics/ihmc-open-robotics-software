package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message sends gripper commands/feedback to nadia
       */
public class NadiaGripperMessage extends Packet<NadiaGripperMessage> implements Settable<NadiaGripperMessage>, EpsilonComparable<NadiaGripperMessage>
{
   public static final byte IDLE = (byte) 0;
   public static final byte CALIBRATE = (byte) 1;
   public static final byte CLOSE = (byte) 2;
   public static final byte OPEN = (byte) 3;
   public static final byte RELEASE = (byte) 4;
   /**
            * High level gripper command, sent to Gripper class
            */
   public byte command_ = (byte) 255;
   /**
            * Desired torque on 0 to 1
            */
   public double desired_torque_ratio_;
   /**
            * Desired position on 0 to 1
            */
   public double desired_position_ratio_;
   /**
            * Measured position on 0 to 1
            */
   public double measured_position_ratio_;

   public NadiaGripperMessage()
   {
   }

   public NadiaGripperMessage(NadiaGripperMessage other)
   {
      this();
      set(other);
   }

   public void set(NadiaGripperMessage other)
   {
      command_ = other.command_;

      desired_torque_ratio_ = other.desired_torque_ratio_;

      desired_position_ratio_ = other.desired_position_ratio_;

      measured_position_ratio_ = other.measured_position_ratio_;

   }

   /**
            * High level gripper command, sent to Gripper class
            */
   public void setCommand(byte command)
   {
      command_ = command;
   }
   /**
            * High level gripper command, sent to Gripper class
            */
   public byte getCommand()
   {
      return command_;
   }

   /**
            * Desired torque on 0 to 1
            */
   public void setDesiredTorqueRatio(double desired_torque_ratio)
   {
      desired_torque_ratio_ = desired_torque_ratio;
   }
   /**
            * Desired torque on 0 to 1
            */
   public double getDesiredTorqueRatio()
   {
      return desired_torque_ratio_;
   }

   /**
            * Desired position on 0 to 1
            */
   public void setDesiredPositionRatio(double desired_position_ratio)
   {
      desired_position_ratio_ = desired_position_ratio;
   }
   /**
            * Desired position on 0 to 1
            */
   public double getDesiredPositionRatio()
   {
      return desired_position_ratio_;
   }

   /**
            * Measured position on 0 to 1
            */
   public void setMeasuredPositionRatio(double measured_position_ratio)
   {
      measured_position_ratio_ = measured_position_ratio;
   }
   /**
            * Measured position on 0 to 1
            */
   public double getMeasuredPositionRatio()
   {
      return measured_position_ratio_;
   }


   public static Supplier<NadiaGripperMessagePubSubType> getPubSubType()
   {
      return NadiaGripperMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return NadiaGripperMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(NadiaGripperMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.command_, other.command_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.desired_torque_ratio_, other.desired_torque_ratio_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.desired_position_ratio_, other.desired_position_ratio_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.measured_position_ratio_, other.measured_position_ratio_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof NadiaGripperMessage)) return false;

      NadiaGripperMessage otherMyClass = (NadiaGripperMessage) other;

      if(this.command_ != otherMyClass.command_) return false;

      if(this.desired_torque_ratio_ != otherMyClass.desired_torque_ratio_) return false;

      if(this.desired_position_ratio_ != otherMyClass.desired_position_ratio_) return false;

      if(this.measured_position_ratio_ != otherMyClass.measured_position_ratio_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("NadiaGripperMessage {");
      builder.append("command=");
      builder.append(this.command_);      builder.append(", ");
      builder.append("desired_torque_ratio=");
      builder.append(this.desired_torque_ratio_);      builder.append(", ");
      builder.append("desired_position_ratio=");
      builder.append(this.desired_position_ratio_);      builder.append(", ");
      builder.append("measured_position_ratio=");
      builder.append(this.measured_position_ratio_);
      builder.append("}");
      return builder.toString();
   }
}
