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
public class ReactiveBracingMessage extends Packet<ReactiveBracingMessage> implements Settable<ReactiveBracingMessage>, EpsilonComparable<ReactiveBracingMessage>
{
   public static final byte ROBOT_SIDE_LEFT = (byte) 0;
   public static final byte ROBOT_SIDE_RIGHT = (byte) 1;
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   /**
            * Specifies which hand will execute the trajectory.
            */
   public byte robot_side_ = (byte) 255;
   /**
            * Specifies the desired bracing point
            */
   public us.ihmc.euclid.tuple3D.Point3D bracing_point_;
   /**
            * Specifies the normal vector of the bracing surface
            */
   public us.ihmc.euclid.tuple3D.Vector3D bracing_normal_;

   public ReactiveBracingMessage()
   {
      bracing_point_ = new us.ihmc.euclid.tuple3D.Point3D();
      bracing_normal_ = new us.ihmc.euclid.tuple3D.Vector3D();
   }

   public ReactiveBracingMessage(ReactiveBracingMessage other)
   {
      this();
      set(other);
   }

   public void set(ReactiveBracingMessage other)
   {
      sequence_id_ = other.sequence_id_;

      robot_side_ = other.robot_side_;

      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.bracing_point_, bracing_point_);
      geometry_msgs.msg.dds.Vector3PubSubType.staticCopy(other.bracing_normal_, bracing_normal_);
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


   public static Supplier<ReactiveBracingMessagePubSubType> getPubSubType()
   {
      return ReactiveBracingMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return ReactiveBracingMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(ReactiveBracingMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.robot_side_, other.robot_side_, epsilon)) return false;

      if (!this.bracing_point_.epsilonEquals(other.bracing_point_, epsilon)) return false;
      if (!this.bracing_normal_.epsilonEquals(other.bracing_normal_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof ReactiveBracingMessage)) return false;

      ReactiveBracingMessage otherMyClass = (ReactiveBracingMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.robot_side_ != otherMyClass.robot_side_) return false;

      if (!this.bracing_point_.equals(otherMyClass.bracing_point_)) return false;
      if (!this.bracing_normal_.equals(otherMyClass.bracing_normal_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ReactiveBracingMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("robot_side=");
      builder.append(this.robot_side_);      builder.append(", ");
      builder.append("bracing_point=");
      builder.append(this.bracing_point_);      builder.append(", ");
      builder.append("bracing_normal=");
      builder.append(this.bracing_normal_);
      builder.append("}");
      return builder.toString();
   }
}
