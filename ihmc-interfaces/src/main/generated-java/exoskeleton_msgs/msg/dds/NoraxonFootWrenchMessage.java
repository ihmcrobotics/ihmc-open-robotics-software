package exoskeleton_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the Noraxon MR3 to Eva Exoskeleton Interface
       * This message acts as a go-between between the Noraxon HTTP intepreter program and the Eva Controller
       */
public class NoraxonFootWrenchMessage extends Packet<NoraxonFootWrenchMessage> implements Settable<NoraxonFootWrenchMessage>, EpsilonComparable<NoraxonFootWrenchMessage>
{
   public static final byte ROBOT_SIDE_LEFT = (byte) 0;
   public static final byte ROBOT_SIDE_RIGHT = (byte) 1;
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   /**
            * Which side the Wrench is on
            */
   public byte robot_side_;
   /**
            * This is the wrench at the foot, with respect to the sole frame
            */
   public double linear_force_x_;
   public double linear_force_y_;
   public double linear_force_z_;
   public double angular_moment_x_;
   public double angular_moment_y_;
   public double angular_moment_z_;

   public NoraxonFootWrenchMessage()
   {
   }

   public NoraxonFootWrenchMessage(NoraxonFootWrenchMessage other)
   {
      this();
      set(other);
   }

   public void set(NoraxonFootWrenchMessage other)
   {
      sequence_id_ = other.sequence_id_;

      robot_side_ = other.robot_side_;

      linear_force_x_ = other.linear_force_x_;

      linear_force_y_ = other.linear_force_y_;

      linear_force_z_ = other.linear_force_z_;

      angular_moment_x_ = other.angular_moment_x_;

      angular_moment_y_ = other.angular_moment_y_;

      angular_moment_z_ = other.angular_moment_z_;

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
            * Which side the Wrench is on
            */
   public void setRobotSide(byte robot_side)
   {
      robot_side_ = robot_side;
   }
   /**
            * Which side the Wrench is on
            */
   public byte getRobotSide()
   {
      return robot_side_;
   }

   /**
            * This is the wrench at the foot, with respect to the sole frame
            */
   public void setLinearForceX(double linear_force_x)
   {
      linear_force_x_ = linear_force_x;
   }
   /**
            * This is the wrench at the foot, with respect to the sole frame
            */
   public double getLinearForceX()
   {
      return linear_force_x_;
   }

   public void setLinearForceY(double linear_force_y)
   {
      linear_force_y_ = linear_force_y;
   }
   public double getLinearForceY()
   {
      return linear_force_y_;
   }

   public void setLinearForceZ(double linear_force_z)
   {
      linear_force_z_ = linear_force_z;
   }
   public double getLinearForceZ()
   {
      return linear_force_z_;
   }

   public void setAngularMomentX(double angular_moment_x)
   {
      angular_moment_x_ = angular_moment_x;
   }
   public double getAngularMomentX()
   {
      return angular_moment_x_;
   }

   public void setAngularMomentY(double angular_moment_y)
   {
      angular_moment_y_ = angular_moment_y;
   }
   public double getAngularMomentY()
   {
      return angular_moment_y_;
   }

   public void setAngularMomentZ(double angular_moment_z)
   {
      angular_moment_z_ = angular_moment_z;
   }
   public double getAngularMomentZ()
   {
      return angular_moment_z_;
   }


   public static Supplier<NoraxonFootWrenchMessagePubSubType> getPubSubType()
   {
      return NoraxonFootWrenchMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return NoraxonFootWrenchMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(NoraxonFootWrenchMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.robot_side_, other.robot_side_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.linear_force_x_, other.linear_force_x_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.linear_force_y_, other.linear_force_y_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.linear_force_z_, other.linear_force_z_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.angular_moment_x_, other.angular_moment_x_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.angular_moment_y_, other.angular_moment_y_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.angular_moment_z_, other.angular_moment_z_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof NoraxonFootWrenchMessage)) return false;

      NoraxonFootWrenchMessage otherMyClass = (NoraxonFootWrenchMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.robot_side_ != otherMyClass.robot_side_) return false;

      if(this.linear_force_x_ != otherMyClass.linear_force_x_) return false;

      if(this.linear_force_y_ != otherMyClass.linear_force_y_) return false;

      if(this.linear_force_z_ != otherMyClass.linear_force_z_) return false;

      if(this.angular_moment_x_ != otherMyClass.angular_moment_x_) return false;

      if(this.angular_moment_y_ != otherMyClass.angular_moment_y_) return false;

      if(this.angular_moment_z_ != otherMyClass.angular_moment_z_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("NoraxonFootWrenchMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("robot_side=");
      builder.append(this.robot_side_);      builder.append(", ");
      builder.append("linear_force_x=");
      builder.append(this.linear_force_x_);      builder.append(", ");
      builder.append("linear_force_y=");
      builder.append(this.linear_force_y_);      builder.append(", ");
      builder.append("linear_force_z=");
      builder.append(this.linear_force_z_);      builder.append(", ");
      builder.append("angular_moment_x=");
      builder.append(this.angular_moment_x_);      builder.append(", ");
      builder.append("angular_moment_y=");
      builder.append(this.angular_moment_y_);      builder.append(", ");
      builder.append("angular_moment_z=");
      builder.append(this.angular_moment_z_);
      builder.append("}");
      return builder.toString();
   }
}
