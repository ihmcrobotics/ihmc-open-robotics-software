package toolbox_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class KinematicsToolboxFootStatus extends Packet<KinematicsToolboxFootStatus> implements Settable<KinematicsToolboxFootStatus>, EpsilonComparable<KinematicsToolboxFootStatus>
{
   /**
            * General purpose message normally used to report the solution of a whole-body inverse kinematics solver.
            * Which hand to receive the object
            */
   public byte side_ = (byte) 255;
   /**
            * Legged robot-specific contact information (false if not a legged robot)
            */
   public boolean foot_in_contact_;
   public us.ihmc.euclid.tuple3D.Point3D desired_relative_foot_position_from_stance_;
   public us.ihmc.euclid.tuple4D.Quaternion desired_relative_foot_orientation_from_stance_;

   public KinematicsToolboxFootStatus()
   {
      desired_relative_foot_position_from_stance_ = new us.ihmc.euclid.tuple3D.Point3D();
      desired_relative_foot_orientation_from_stance_ = new us.ihmc.euclid.tuple4D.Quaternion();
   }

   public KinematicsToolboxFootStatus(KinematicsToolboxFootStatus other)
   {
      this();
      set(other);
   }

   public void set(KinematicsToolboxFootStatus other)
   {
      side_ = other.side_;

      foot_in_contact_ = other.foot_in_contact_;

      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.desired_relative_foot_position_from_stance_, desired_relative_foot_position_from_stance_);
      geometry_msgs.msg.dds.QuaternionPubSubType.staticCopy(other.desired_relative_foot_orientation_from_stance_, desired_relative_foot_orientation_from_stance_);
   }

   /**
            * General purpose message normally used to report the solution of a whole-body inverse kinematics solver.
            * Which hand to receive the object
            */
   public void setSide(byte side)
   {
      side_ = side;
   }
   /**
            * General purpose message normally used to report the solution of a whole-body inverse kinematics solver.
            * Which hand to receive the object
            */
   public byte getSide()
   {
      return side_;
   }

   /**
            * Legged robot-specific contact information (false if not a legged robot)
            */
   public void setFootInContact(boolean foot_in_contact)
   {
      foot_in_contact_ = foot_in_contact;
   }
   /**
            * Legged robot-specific contact information (false if not a legged robot)
            */
   public boolean getFootInContact()
   {
      return foot_in_contact_;
   }


   public us.ihmc.euclid.tuple3D.Point3D getDesiredRelativeFootPositionFromStance()
   {
      return desired_relative_foot_position_from_stance_;
   }


   public us.ihmc.euclid.tuple4D.Quaternion getDesiredRelativeFootOrientationFromStance()
   {
      return desired_relative_foot_orientation_from_stance_;
   }


   public static Supplier<KinematicsToolboxFootStatusPubSubType> getPubSubType()
   {
      return KinematicsToolboxFootStatusPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return KinematicsToolboxFootStatusPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(KinematicsToolboxFootStatus other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.side_, other.side_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.foot_in_contact_, other.foot_in_contact_, epsilon)) return false;

      if (!this.desired_relative_foot_position_from_stance_.epsilonEquals(other.desired_relative_foot_position_from_stance_, epsilon)) return false;
      if (!this.desired_relative_foot_orientation_from_stance_.epsilonEquals(other.desired_relative_foot_orientation_from_stance_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof KinematicsToolboxFootStatus)) return false;

      KinematicsToolboxFootStatus otherMyClass = (KinematicsToolboxFootStatus) other;

      if(this.side_ != otherMyClass.side_) return false;

      if(this.foot_in_contact_ != otherMyClass.foot_in_contact_) return false;

      if (!this.desired_relative_foot_position_from_stance_.equals(otherMyClass.desired_relative_foot_position_from_stance_)) return false;
      if (!this.desired_relative_foot_orientation_from_stance_.equals(otherMyClass.desired_relative_foot_orientation_from_stance_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("KinematicsToolboxFootStatus {");
      builder.append("side=");
      builder.append(this.side_);      builder.append(", ");
      builder.append("foot_in_contact=");
      builder.append(this.foot_in_contact_);      builder.append(", ");
      builder.append("desired_relative_foot_position_from_stance=");
      builder.append(this.desired_relative_foot_position_from_stance_);      builder.append(", ");
      builder.append("desired_relative_foot_orientation_from_stance=");
      builder.append(this.desired_relative_foot_orientation_from_stance_);
      builder.append("}");
      return builder.toString();
   }
}
