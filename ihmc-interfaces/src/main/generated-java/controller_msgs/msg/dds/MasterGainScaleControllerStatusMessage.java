package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class MasterGainScaleControllerStatusMessage extends Packet<MasterGainScaleControllerStatusMessage> implements Settable<MasterGainScaleControllerStatusMessage>, EpsilonComparable<MasterGainScaleControllerStatusMessage>
{
   /**
            * Whether the robot's gains are currently active in holding the controller's desireds
            */
   public boolean is_robot_servoed_;

   public MasterGainScaleControllerStatusMessage()
   {
   }

   public MasterGainScaleControllerStatusMessage(MasterGainScaleControllerStatusMessage other)
   {
      this();
      set(other);
   }

   public void set(MasterGainScaleControllerStatusMessage other)
   {
      is_robot_servoed_ = other.is_robot_servoed_;

   }

   /**
            * Whether the robot's gains are currently active in holding the controller's desireds
            */
   public void setIsRobotServoed(boolean is_robot_servoed)
   {
      is_robot_servoed_ = is_robot_servoed;
   }
   /**
            * Whether the robot's gains are currently active in holding the controller's desireds
            */
   public boolean getIsRobotServoed()
   {
      return is_robot_servoed_;
   }


   public static Supplier<MasterGainScaleControllerStatusMessagePubSubType> getPubSubType()
   {
      return MasterGainScaleControllerStatusMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return MasterGainScaleControllerStatusMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(MasterGainScaleControllerStatusMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.is_robot_servoed_, other.is_robot_servoed_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof MasterGainScaleControllerStatusMessage)) return false;

      MasterGainScaleControllerStatusMessage otherMyClass = (MasterGainScaleControllerStatusMessage) other;

      if(this.is_robot_servoed_ != otherMyClass.is_robot_servoed_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("MasterGainScaleControllerStatusMessage {");
      builder.append("is_robot_servoed=");
      builder.append(this.is_robot_servoed_);
      builder.append("}");
      return builder.toString();
   }
}
