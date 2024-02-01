package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * Message for commanding the Sake hands to perform various predefined grasps.
       * Also allows for custom grasps with set positions/torques
       */
public class SakeHandDesiredCommandMessage extends Packet<SakeHandDesiredCommandMessage> implements Settable<SakeHandDesiredCommandMessage>, EpsilonComparable<SakeHandDesiredCommandMessage>
{
   public static final byte ROBOT_SIDE_LEFT = (byte) 0;
   public static final byte ROBOT_SIDE_RIGHT = (byte) 1;
   public static final byte HAND_CONFIGURATION_CALIBRATE = (byte) 0;
   public static final byte HAND_CONFIGURATION_RESET = (byte) 1;
   public static final byte HAND_CONFIGURATION_OPEN = (byte) 2;
   public static final byte HAND_CONFIGURATION_CLOSE = (byte) 3;
   public static final byte HAND_CONFIGURATION_RELEASE = (byte) 4;
   public static final byte HAND_CONFIGURATION_GOTO_POSITION_WITH_TORQUE = (byte) 5;
   public static final byte HAND_CONFIGURATION_GRIP_WITH_TORQUE = (byte) 6;
   public static final byte HAND_CONFIGURATION_GRIP_HARD = (byte) 7;
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   /**
            * Specifies the side of the robot that will execute the trajectory
            */
   public byte robot_side_ = (byte) 255;
   /**
            * Specifies the grasp to perform
            */
   public byte desired_hand_configuration_ = (byte) 255;
   /**
            * 0.0 is closed, 1.0 is open
            */
   public double postion_ratio_;
   /**
            * Specifies desired torque of grasp, if not specified by hand configuration. 0.0 min, 1.0 max
            */
   public double torque_ratio_;

   public SakeHandDesiredCommandMessage()
   {
   }

   public SakeHandDesiredCommandMessage(SakeHandDesiredCommandMessage other)
   {
      this();
      set(other);
   }

   public void set(SakeHandDesiredCommandMessage other)
   {
      sequence_id_ = other.sequence_id_;

      robot_side_ = other.robot_side_;

      desired_hand_configuration_ = other.desired_hand_configuration_;

      postion_ratio_ = other.postion_ratio_;

      torque_ratio_ = other.torque_ratio_;

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
            * Specifies the side of the robot that will execute the trajectory
            */
   public void setRobotSide(byte robot_side)
   {
      robot_side_ = robot_side;
   }
   /**
            * Specifies the side of the robot that will execute the trajectory
            */
   public byte getRobotSide()
   {
      return robot_side_;
   }

   /**
            * Specifies the grasp to perform
            */
   public void setDesiredHandConfiguration(byte desired_hand_configuration)
   {
      desired_hand_configuration_ = desired_hand_configuration;
   }
   /**
            * Specifies the grasp to perform
            */
   public byte getDesiredHandConfiguration()
   {
      return desired_hand_configuration_;
   }

   /**
            * 0.0 is closed, 1.0 is open
            */
   public void setPostionRatio(double postion_ratio)
   {
      postion_ratio_ = postion_ratio;
   }
   /**
            * 0.0 is closed, 1.0 is open
            */
   public double getPostionRatio()
   {
      return postion_ratio_;
   }

   /**
            * Specifies desired torque of grasp, if not specified by hand configuration. 0.0 min, 1.0 max
            */
   public void setTorqueRatio(double torque_ratio)
   {
      torque_ratio_ = torque_ratio;
   }
   /**
            * Specifies desired torque of grasp, if not specified by hand configuration. 0.0 min, 1.0 max
            */
   public double getTorqueRatio()
   {
      return torque_ratio_;
   }


   public static Supplier<SakeHandDesiredCommandMessagePubSubType> getPubSubType()
   {
      return SakeHandDesiredCommandMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return SakeHandDesiredCommandMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(SakeHandDesiredCommandMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.robot_side_, other.robot_side_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.desired_hand_configuration_, other.desired_hand_configuration_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.postion_ratio_, other.postion_ratio_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.torque_ratio_, other.torque_ratio_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof SakeHandDesiredCommandMessage)) return false;

      SakeHandDesiredCommandMessage otherMyClass = (SakeHandDesiredCommandMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.robot_side_ != otherMyClass.robot_side_) return false;

      if(this.desired_hand_configuration_ != otherMyClass.desired_hand_configuration_) return false;

      if(this.postion_ratio_ != otherMyClass.postion_ratio_) return false;

      if(this.torque_ratio_ != otherMyClass.torque_ratio_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("SakeHandDesiredCommandMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("robot_side=");
      builder.append(this.robot_side_);      builder.append(", ");
      builder.append("desired_hand_configuration=");
      builder.append(this.desired_hand_configuration_);      builder.append(", ");
      builder.append("postion_ratio=");
      builder.append(this.postion_ratio_);      builder.append(", ");
      builder.append("torque_ratio=");
      builder.append(this.torque_ratio_);
      builder.append("}");
      return builder.toString();
   }
}
