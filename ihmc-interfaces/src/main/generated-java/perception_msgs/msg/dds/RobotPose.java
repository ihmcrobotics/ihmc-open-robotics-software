package perception_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * Isaac Sim robot ground-truth root pose + joint angles.
       * Published by the Isaac Sim bridge control process (alex_isaac_sim_dds.py) at the
       * control rate so RDX can pin the robot mesh to Isaac ground truth (the estimator
       * pose in RobotConfigurationData drifts from Isaac truth). Visualization only.
       */
public class RobotPose extends Packet<RobotPose> implements Settable<RobotPose>, EpsilonComparable<RobotPose>
{
   /**
            * Sequence number for detecting out-of-order / dropped messages
            */
   public long sequence_number_;
   /**
            * Root (pelvis) world pose
            */
   public us.ihmc.euclid.tuple3D.Point3D position_;
   public us.ihmc.euclid.tuple4D.Quaternion orientation_;
   /**
            * Joint angles by name (parallel arrays; applied by name, order-independent)
            */
   public us.ihmc.idl.IDLSequence.StringBuilderHolder  joint_names_;
   public us.ihmc.idl.IDLSequence.Float  joint_positions_;

   public RobotPose()
   {
      position_ = new us.ihmc.euclid.tuple3D.Point3D();
      orientation_ = new us.ihmc.euclid.tuple4D.Quaternion();
      joint_names_ = new us.ihmc.idl.IDLSequence.StringBuilderHolder (100, "type_d");
      joint_positions_ = new us.ihmc.idl.IDLSequence.Float (100, "type_5");

   }

   public RobotPose(RobotPose other)
   {
      this();
      set(other);
   }

   public void set(RobotPose other)
   {
      sequence_number_ = other.sequence_number_;

      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.position_, position_);
      geometry_msgs.msg.dds.QuaternionPubSubType.staticCopy(other.orientation_, orientation_);
      joint_names_.set(other.joint_names_);
      joint_positions_.set(other.joint_positions_);
   }

   /**
            * Sequence number for detecting out-of-order / dropped messages
            */
   public void setSequenceNumber(long sequence_number)
   {
      sequence_number_ = sequence_number;
   }
   /**
            * Sequence number for detecting out-of-order / dropped messages
            */
   public long getSequenceNumber()
   {
      return sequence_number_;
   }


   /**
            * Root (pelvis) world pose
            */
   public us.ihmc.euclid.tuple3D.Point3D getPosition()
   {
      return position_;
   }


   public us.ihmc.euclid.tuple4D.Quaternion getOrientation()
   {
      return orientation_;
   }


   /**
            * Joint angles by name (parallel arrays; applied by name, order-independent)
            */
   public us.ihmc.idl.IDLSequence.StringBuilderHolder  getJointNames()
   {
      return joint_names_;
   }


   public us.ihmc.idl.IDLSequence.Float  getJointPositions()
   {
      return joint_positions_;
   }


   public static Supplier<RobotPosePubSubType> getPubSubType()
   {
      return RobotPosePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return RobotPosePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(RobotPose other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_number_, other.sequence_number_, epsilon)) return false;

      if (!this.position_.epsilonEquals(other.position_, epsilon)) return false;
      if (!this.orientation_.epsilonEquals(other.orientation_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilderSequence(this.joint_names_, other.joint_names_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsFloatSequence(this.joint_positions_, other.joint_positions_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof RobotPose)) return false;

      RobotPose otherMyClass = (RobotPose) other;

      if(this.sequence_number_ != otherMyClass.sequence_number_) return false;

      if (!this.position_.equals(otherMyClass.position_)) return false;
      if (!this.orientation_.equals(otherMyClass.orientation_)) return false;
      if (!this.joint_names_.equals(otherMyClass.joint_names_)) return false;
      if (!this.joint_positions_.equals(otherMyClass.joint_positions_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("RobotPose {");
      builder.append("sequence_number=");
      builder.append(this.sequence_number_);      builder.append(", ");
      builder.append("position=");
      builder.append(this.position_);      builder.append(", ");
      builder.append("orientation=");
      builder.append(this.orientation_);      builder.append(", ");
      builder.append("joint_names=");
      builder.append(this.joint_names_);      builder.append(", ");
      builder.append("joint_positions=");
      builder.append(this.joint_positions_);
      builder.append("}");
      return builder.toString();
   }
}
