package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC whole-body controller API.
       * This message commands the controller to brace against the given planar region
       */
public class HandContactMessage extends Packet<HandContactMessage> implements Settable<HandContactMessage>, EpsilonComparable<HandContactMessage>
{
   public static final byte ROBOT_SIDE_LEFT = (byte) 0;
   public static final byte ROBOT_SIDE_RIGHT = (byte) 1;
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   /**
            * If true it will load the contact point, otherwise the hand will stop bearing load.
            */
   public boolean load_;
   /**
            * Specifies which hand will execute the trajectory.
            */
   public byte robot_side_ = (byte) 255;
   /**
            * Trajectory duration in seconds
            */
   public double trajectory_duration_;
   /**
            * Specifies the desired bracing point
            */
   public us.ihmc.euclid.tuple3D.Point3D bracing_point_;
   /**
            * Specifies the normal vector of the bracing surface
            */
   public us.ihmc.euclid.tuple3D.Vector3D bracing_normal_;
   /**
            * Initial support polygon in mid-feet zup frame on contact, to warm start region solved through LP
            */
   public us.ihmc.idl.IDLSequence.Object<ihmc_common_msgs.msg.dds.Point2DMessage>  support_region_in_mid_feet_frame_;
   public controller_msgs.msg.dds.RigidBodyTransformMessage region_transform_;
   public us.ihmc.idl.IDLSequence.Object<ihmc_common_msgs.msg.dds.Point2DMessage>  scaled_convex_hull_;

   public HandContactMessage()
   {
      bracing_point_ = new us.ihmc.euclid.tuple3D.Point3D();
      bracing_normal_ = new us.ihmc.euclid.tuple3D.Vector3D();
      support_region_in_mid_feet_frame_ = new us.ihmc.idl.IDLSequence.Object<ihmc_common_msgs.msg.dds.Point2DMessage> (18, new ihmc_common_msgs.msg.dds.Point2DMessagePubSubType());
      region_transform_ = new controller_msgs.msg.dds.RigidBodyTransformMessage();
      scaled_convex_hull_ = new us.ihmc.idl.IDLSequence.Object<ihmc_common_msgs.msg.dds.Point2DMessage> (50, new ihmc_common_msgs.msg.dds.Point2DMessagePubSubType());

   }

   public HandContactMessage(HandContactMessage other)
   {
      this();
      set(other);
   }

   public void set(HandContactMessage other)
   {
      sequence_id_ = other.sequence_id_;

      load_ = other.load_;

      robot_side_ = other.robot_side_;

      trajectory_duration_ = other.trajectory_duration_;

      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.bracing_point_, bracing_point_);
      geometry_msgs.msg.dds.Vector3PubSubType.staticCopy(other.bracing_normal_, bracing_normal_);
      support_region_in_mid_feet_frame_.set(other.support_region_in_mid_feet_frame_);
      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.staticCopy(other.region_transform_, region_transform_);
      scaled_convex_hull_.set(other.scaled_convex_hull_);
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
            * If true it will load the contact point, otherwise the hand will stop bearing load.
            */
   public void setLoad(boolean load)
   {
      load_ = load;
   }
   /**
            * If true it will load the contact point, otherwise the hand will stop bearing load.
            */
   public boolean getLoad()
   {
      return load_;
   }

   /**
            * Specifies which hand will execute the trajectory.
            */
   public void setRobotSide(byte robot_side)
   {
      robot_side_ = robot_side;
   }
   /**
            * Specifies which hand will execute the trajectory.
            */
   public byte getRobotSide()
   {
      return robot_side_;
   }

   /**
            * Trajectory duration in seconds
            */
   public void setTrajectoryDuration(double trajectory_duration)
   {
      trajectory_duration_ = trajectory_duration;
   }
   /**
            * Trajectory duration in seconds
            */
   public double getTrajectoryDuration()
   {
      return trajectory_duration_;
   }


   /**
            * Specifies the desired bracing point
            */
   public us.ihmc.euclid.tuple3D.Point3D getBracingPoint()
   {
      return bracing_point_;
   }


   /**
            * Specifies the normal vector of the bracing surface
            */
   public us.ihmc.euclid.tuple3D.Vector3D getBracingNormal()
   {
      return bracing_normal_;
   }


   /**
            * Initial support polygon in mid-feet zup frame on contact, to warm start region solved through LP
            */
   public us.ihmc.idl.IDLSequence.Object<ihmc_common_msgs.msg.dds.Point2DMessage>  getSupportRegionInMidFeetFrame()
   {
      return support_region_in_mid_feet_frame_;
   }


   public controller_msgs.msg.dds.RigidBodyTransformMessage getRegionTransform()
   {
      return region_transform_;
   }


   public us.ihmc.idl.IDLSequence.Object<ihmc_common_msgs.msg.dds.Point2DMessage>  getScaledConvexHull()
   {
      return scaled_convex_hull_;
   }


   public static Supplier<HandContactMessagePubSubType> getPubSubType()
   {
      return HandContactMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return HandContactMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(HandContactMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.load_, other.load_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.robot_side_, other.robot_side_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.trajectory_duration_, other.trajectory_duration_, epsilon)) return false;

      if (!this.bracing_point_.epsilonEquals(other.bracing_point_, epsilon)) return false;
      if (!this.bracing_normal_.epsilonEquals(other.bracing_normal_, epsilon)) return false;
      if (this.support_region_in_mid_feet_frame_.size() != other.support_region_in_mid_feet_frame_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.support_region_in_mid_feet_frame_.size(); i++)
         {  if (!this.support_region_in_mid_feet_frame_.get(i).epsilonEquals(other.support_region_in_mid_feet_frame_.get(i), epsilon)) return false; }
      }

      if (!this.region_transform_.epsilonEquals(other.region_transform_, epsilon)) return false;
      if (this.scaled_convex_hull_.size() != other.scaled_convex_hull_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.scaled_convex_hull_.size(); i++)
         {  if (!this.scaled_convex_hull_.get(i).epsilonEquals(other.scaled_convex_hull_.get(i), epsilon)) return false; }
      }


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof HandContactMessage)) return false;

      HandContactMessage otherMyClass = (HandContactMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.load_ != otherMyClass.load_) return false;

      if(this.robot_side_ != otherMyClass.robot_side_) return false;

      if(this.trajectory_duration_ != otherMyClass.trajectory_duration_) return false;

      if (!this.bracing_point_.equals(otherMyClass.bracing_point_)) return false;
      if (!this.bracing_normal_.equals(otherMyClass.bracing_normal_)) return false;
      if (!this.support_region_in_mid_feet_frame_.equals(otherMyClass.support_region_in_mid_feet_frame_)) return false;
      if (!this.region_transform_.equals(otherMyClass.region_transform_)) return false;
      if (!this.scaled_convex_hull_.equals(otherMyClass.scaled_convex_hull_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("HandContactMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("load=");
      builder.append(this.load_);      builder.append(", ");
      builder.append("robot_side=");
      builder.append(this.robot_side_);      builder.append(", ");
      builder.append("trajectory_duration=");
      builder.append(this.trajectory_duration_);      builder.append(", ");
      builder.append("bracing_point=");
      builder.append(this.bracing_point_);      builder.append(", ");
      builder.append("bracing_normal=");
      builder.append(this.bracing_normal_);      builder.append(", ");
      builder.append("support_region_in_mid_feet_frame=");
      builder.append(this.support_region_in_mid_feet_frame_);      builder.append(", ");
      builder.append("region_transform=");
      builder.append(this.region_transform_);      builder.append(", ");
      builder.append("scaled_convex_hull=");
      builder.append(this.scaled_convex_hull_);
      builder.append("}");
      return builder.toString();
   }
}
