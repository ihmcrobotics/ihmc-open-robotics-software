package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC whole-body controller API.
       * This message contains a list of joint-level desired values that are output from the whole-body controller
       */
public class RobotDesiredConfigurationData extends Packet<RobotDesiredConfigurationData> implements Settable<RobotDesiredConfigurationData>, EpsilonComparable<RobotDesiredConfigurationData>
{

   /**
            * Wall-time in nanoseconds
            */
   public long wall_time_;

   /**
            * Hash of the joint names included in joint_desired_output_list
            */
   public int joint_name_hash_;

   /**
            * Contains list of joint desireds, ordered according
            */
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.JointDesiredOutputMessage>  joint_desired_output_list_;

   public boolean has_desired_root_joint_position_data_;

   public us.ihmc.euclid.tuple3D.Vector3D desired_root_joint_translation_;

   public us.ihmc.euclid.tuple4D.Quaternion desired_root_joint_orientation_;

   public boolean has_desired_root_joint_velocity_data_;

   public us.ihmc.euclid.tuple3D.Vector3D desired_root_joint_linear_velocity_;

   public us.ihmc.euclid.tuple3D.Vector3D desired_root_joint_angular_velocity_;

   public boolean has_desired_root_joint_acceleration_data_;

   public us.ihmc.euclid.tuple3D.Vector3D desired_root_joint_linear_acceleration_;

   public us.ihmc.euclid.tuple3D.Vector3D desired_root_joint_angular_acceleration_;

   public RobotDesiredConfigurationData()
   {



      joint_desired_output_list_ = new us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.JointDesiredOutputMessage> (50, new controller_msgs.msg.dds.JointDesiredOutputMessagePubSubType());


      desired_root_joint_translation_ = new us.ihmc.euclid.tuple3D.Vector3D();

      desired_root_joint_orientation_ = new us.ihmc.euclid.tuple4D.Quaternion();


      desired_root_joint_linear_velocity_ = new us.ihmc.euclid.tuple3D.Vector3D();

      desired_root_joint_angular_velocity_ = new us.ihmc.euclid.tuple3D.Vector3D();


      desired_root_joint_linear_acceleration_ = new us.ihmc.euclid.tuple3D.Vector3D();

      desired_root_joint_angular_acceleration_ = new us.ihmc.euclid.tuple3D.Vector3D();

   }

   public RobotDesiredConfigurationData(RobotDesiredConfigurationData other)
   {
      this();
      set(other);
   }

   public void set(RobotDesiredConfigurationData other)
   {

      wall_time_ = other.wall_time_;


      joint_name_hash_ = other.joint_name_hash_;


      joint_desired_output_list_.set(other.joint_desired_output_list_);

      has_desired_root_joint_position_data_ = other.has_desired_root_joint_position_data_;


      geometry_msgs.msg.dds.Vector3PubSubType.staticCopy(other.desired_root_joint_translation_, desired_root_joint_translation_);

      geometry_msgs.msg.dds.QuaternionPubSubType.staticCopy(other.desired_root_joint_orientation_, desired_root_joint_orientation_);

      has_desired_root_joint_velocity_data_ = other.has_desired_root_joint_velocity_data_;


      geometry_msgs.msg.dds.Vector3PubSubType.staticCopy(other.desired_root_joint_linear_velocity_, desired_root_joint_linear_velocity_);

      geometry_msgs.msg.dds.Vector3PubSubType.staticCopy(other.desired_root_joint_angular_velocity_, desired_root_joint_angular_velocity_);

      has_desired_root_joint_acceleration_data_ = other.has_desired_root_joint_acceleration_data_;


      geometry_msgs.msg.dds.Vector3PubSubType.staticCopy(other.desired_root_joint_linear_acceleration_, desired_root_joint_linear_acceleration_);

      geometry_msgs.msg.dds.Vector3PubSubType.staticCopy(other.desired_root_joint_angular_acceleration_, desired_root_joint_angular_acceleration_);
   }


   /**
            * Wall-time in nanoseconds
            */
   public void setWallTime(long wall_time)
   {
      wall_time_ = wall_time;
   }
   /**
            * Wall-time in nanoseconds
            */
   public long getWallTime()
   {
      return wall_time_;
   }


   /**
            * Hash of the joint names included in joint_desired_output_list
            */
   public void setJointNameHash(int joint_name_hash)
   {
      joint_name_hash_ = joint_name_hash;
   }
   /**
            * Hash of the joint names included in joint_desired_output_list
            */
   public int getJointNameHash()
   {
      return joint_name_hash_;
   }



   /**
            * Contains list of joint desireds, ordered according
            */
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.JointDesiredOutputMessage>  getJointDesiredOutputList()
   {
      return joint_desired_output_list_;
   }


   public void setHasDesiredRootJointPositionData(boolean has_desired_root_joint_position_data)
   {
      has_desired_root_joint_position_data_ = has_desired_root_joint_position_data;
   }
   public boolean getHasDesiredRootJointPositionData()
   {
      return has_desired_root_joint_position_data_;
   }



   public us.ihmc.euclid.tuple3D.Vector3D getDesiredRootJointTranslation()
   {
      return desired_root_joint_translation_;
   }



   public us.ihmc.euclid.tuple4D.Quaternion getDesiredRootJointOrientation()
   {
      return desired_root_joint_orientation_;
   }


   public void setHasDesiredRootJointVelocityData(boolean has_desired_root_joint_velocity_data)
   {
      has_desired_root_joint_velocity_data_ = has_desired_root_joint_velocity_data;
   }
   public boolean getHasDesiredRootJointVelocityData()
   {
      return has_desired_root_joint_velocity_data_;
   }



   public us.ihmc.euclid.tuple3D.Vector3D getDesiredRootJointLinearVelocity()
   {
      return desired_root_joint_linear_velocity_;
   }



   public us.ihmc.euclid.tuple3D.Vector3D getDesiredRootJointAngularVelocity()
   {
      return desired_root_joint_angular_velocity_;
   }


   public void setHasDesiredRootJointAccelerationData(boolean has_desired_root_joint_acceleration_data)
   {
      has_desired_root_joint_acceleration_data_ = has_desired_root_joint_acceleration_data;
   }
   public boolean getHasDesiredRootJointAccelerationData()
   {
      return has_desired_root_joint_acceleration_data_;
   }



   public us.ihmc.euclid.tuple3D.Vector3D getDesiredRootJointLinearAcceleration()
   {
      return desired_root_joint_linear_acceleration_;
   }



   public us.ihmc.euclid.tuple3D.Vector3D getDesiredRootJointAngularAcceleration()
   {
      return desired_root_joint_angular_acceleration_;
   }


   public static Supplier<RobotDesiredConfigurationDataPubSubType> getPubSubType()
   {
      return RobotDesiredConfigurationDataPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return RobotDesiredConfigurationDataPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(RobotDesiredConfigurationData other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.wall_time_, other.wall_time_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.joint_name_hash_, other.joint_name_hash_, epsilon)) return false;


      if (this.joint_desired_output_list_.size() != other.joint_desired_output_list_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.joint_desired_output_list_.size(); i++)
         {  if (!this.joint_desired_output_list_.get(i).epsilonEquals(other.joint_desired_output_list_.get(i), epsilon)) return false; }
      }


      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.has_desired_root_joint_position_data_, other.has_desired_root_joint_position_data_, epsilon)) return false;


      if (!this.desired_root_joint_translation_.epsilonEquals(other.desired_root_joint_translation_, epsilon)) return false;

      if (!this.desired_root_joint_orientation_.epsilonEquals(other.desired_root_joint_orientation_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.has_desired_root_joint_velocity_data_, other.has_desired_root_joint_velocity_data_, epsilon)) return false;


      if (!this.desired_root_joint_linear_velocity_.epsilonEquals(other.desired_root_joint_linear_velocity_, epsilon)) return false;

      if (!this.desired_root_joint_angular_velocity_.epsilonEquals(other.desired_root_joint_angular_velocity_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.has_desired_root_joint_acceleration_data_, other.has_desired_root_joint_acceleration_data_, epsilon)) return false;


      if (!this.desired_root_joint_linear_acceleration_.epsilonEquals(other.desired_root_joint_linear_acceleration_, epsilon)) return false;

      if (!this.desired_root_joint_angular_acceleration_.epsilonEquals(other.desired_root_joint_angular_acceleration_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof RobotDesiredConfigurationData)) return false;

      RobotDesiredConfigurationData otherMyClass = (RobotDesiredConfigurationData) other;


      if(this.wall_time_ != otherMyClass.wall_time_) return false;


      if(this.joint_name_hash_ != otherMyClass.joint_name_hash_) return false;


      if (!this.joint_desired_output_list_.equals(otherMyClass.joint_desired_output_list_)) return false;

      if(this.has_desired_root_joint_position_data_ != otherMyClass.has_desired_root_joint_position_data_) return false;


      if (!this.desired_root_joint_translation_.equals(otherMyClass.desired_root_joint_translation_)) return false;

      if (!this.desired_root_joint_orientation_.equals(otherMyClass.desired_root_joint_orientation_)) return false;

      if(this.has_desired_root_joint_velocity_data_ != otherMyClass.has_desired_root_joint_velocity_data_) return false;


      if (!this.desired_root_joint_linear_velocity_.equals(otherMyClass.desired_root_joint_linear_velocity_)) return false;

      if (!this.desired_root_joint_angular_velocity_.equals(otherMyClass.desired_root_joint_angular_velocity_)) return false;

      if(this.has_desired_root_joint_acceleration_data_ != otherMyClass.has_desired_root_joint_acceleration_data_) return false;


      if (!this.desired_root_joint_linear_acceleration_.equals(otherMyClass.desired_root_joint_linear_acceleration_)) return false;

      if (!this.desired_root_joint_angular_acceleration_.equals(otherMyClass.desired_root_joint_angular_acceleration_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("RobotDesiredConfigurationData {");

      builder.append("wall_time=");
      builder.append(this.wall_time_);      builder.append(", ");

      builder.append("joint_name_hash=");
      builder.append(this.joint_name_hash_);      builder.append(", ");

      builder.append("joint_desired_output_list=");
      builder.append(this.joint_desired_output_list_);      builder.append(", ");

      builder.append("has_desired_root_joint_position_data=");
      builder.append(this.has_desired_root_joint_position_data_);      builder.append(", ");

      builder.append("desired_root_joint_translation=");
      builder.append(this.desired_root_joint_translation_);      builder.append(", ");

      builder.append("desired_root_joint_orientation=");
      builder.append(this.desired_root_joint_orientation_);      builder.append(", ");

      builder.append("has_desired_root_joint_velocity_data=");
      builder.append(this.has_desired_root_joint_velocity_data_);      builder.append(", ");

      builder.append("desired_root_joint_linear_velocity=");
      builder.append(this.desired_root_joint_linear_velocity_);      builder.append(", ");

      builder.append("desired_root_joint_angular_velocity=");
      builder.append(this.desired_root_joint_angular_velocity_);      builder.append(", ");

      builder.append("has_desired_root_joint_acceleration_data=");
      builder.append(this.has_desired_root_joint_acceleration_data_);      builder.append(", ");

      builder.append("desired_root_joint_linear_acceleration=");
      builder.append(this.desired_root_joint_linear_acceleration_);      builder.append(", ");

      builder.append("desired_root_joint_angular_acceleration=");
      builder.append(this.desired_root_joint_angular_acceleration_);
      builder.append("}");
      return builder.toString();
   }
}
