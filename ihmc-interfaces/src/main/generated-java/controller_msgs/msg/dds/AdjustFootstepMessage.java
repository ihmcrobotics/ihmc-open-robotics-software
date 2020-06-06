package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC whole-body controller API.
       * The intent of this message is to adjust a footstep when the robot is executing it
       * (a foot is currently swinging to reach the footstep to be adjusted).
       */
public class AdjustFootstepMessage extends Packet<AdjustFootstepMessage> implements Settable<AdjustFootstepMessage>, EpsilonComparable<AdjustFootstepMessage>
{

   public static final byte ROBOT_SIDE_LEFT = (byte) 0;

   public static final byte ROBOT_SIDE_RIGHT = (byte) 1;

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   /**
            * Specifies which foot is expected to be executing the footstep to be adjusted.
            */
   public byte robot_side_ = (byte) 255;

   /**
            * Specifies the adjusted position of the footstep. It is expressed in world frame.
            */
   public us.ihmc.euclid.tuple3D.Point3D location_;

   /**
            * Specifies the adjusted orientation of the footstep. It is expressed in world frame.
            */
   public us.ihmc.euclid.tuple4D.Quaternion orientation_;

   /**
            * Predicted contact points represent the vertices of the expected contact polygon between the foot and the world.
            * An empty list will request the controller to use the default foot support polygon.
            * Contact points  are expressed in sole frame. The ordering does not matter.
            * For example: to tell the controller to use the entire foot, the predicted contact points would be:
            * - x: 0.5 * foot_length, y: -0.5 * toe_width
            * - x: 0.5 * foot_length, y: 0.5 * toe_width
            * - x: -0.5 * foot_length, y: -0.5 * heel_width
            * - x: -0.5 * foot_length, y: 0.5 * heel_width
            * Note: The z coordinate of each point is ignored.
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D>  predicted_contact_points_2d_;

   /**
            * The time to delay this command on the controller side before being executed.
            */
   public double execution_delay_time_;

   public AdjustFootstepMessage()
   {



      location_ = new us.ihmc.euclid.tuple3D.Point3D();

      orientation_ = new us.ihmc.euclid.tuple4D.Quaternion();

      predicted_contact_points_2d_ = new us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D> (100, new geometry_msgs.msg.dds.PointPubSubType());


   }

   public AdjustFootstepMessage(AdjustFootstepMessage other)
   {
      this();
      set(other);
   }

   public void set(AdjustFootstepMessage other)
   {

      sequence_id_ = other.sequence_id_;


      robot_side_ = other.robot_side_;


      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.location_, location_);

      geometry_msgs.msg.dds.QuaternionPubSubType.staticCopy(other.orientation_, orientation_);

      predicted_contact_points_2d_.set(other.predicted_contact_points_2d_);

      execution_delay_time_ = other.execution_delay_time_;

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
            * Specifies which foot is expected to be executing the footstep to be adjusted.
            */
   public void setRobotSide(byte robot_side)
   {
      robot_side_ = robot_side;
   }
   /**
            * Specifies which foot is expected to be executing the footstep to be adjusted.
            */
   public byte getRobotSide()
   {
      return robot_side_;
   }



   /**
            * Specifies the adjusted position of the footstep. It is expressed in world frame.
            */
   public us.ihmc.euclid.tuple3D.Point3D getLocation()
   {
      return location_;
   }



   /**
            * Specifies the adjusted orientation of the footstep. It is expressed in world frame.
            */
   public us.ihmc.euclid.tuple4D.Quaternion getOrientation()
   {
      return orientation_;
   }



   /**
            * Predicted contact points represent the vertices of the expected contact polygon between the foot and the world.
            * An empty list will request the controller to use the default foot support polygon.
            * Contact points  are expressed in sole frame. The ordering does not matter.
            * For example: to tell the controller to use the entire foot, the predicted contact points would be:
            * - x: 0.5 * foot_length, y: -0.5 * toe_width
            * - x: 0.5 * foot_length, y: 0.5 * toe_width
            * - x: -0.5 * foot_length, y: -0.5 * heel_width
            * - x: -0.5 * foot_length, y: 0.5 * heel_width
            * Note: The z coordinate of each point is ignored.
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D>  getPredictedContactPoints2d()
   {
      return predicted_contact_points_2d_;
   }


   /**
            * The time to delay this command on the controller side before being executed.
            */
   public void setExecutionDelayTime(double execution_delay_time)
   {
      execution_delay_time_ = execution_delay_time;
   }
   /**
            * The time to delay this command on the controller side before being executed.
            */
   public double getExecutionDelayTime()
   {
      return execution_delay_time_;
   }


   public static Supplier<AdjustFootstepMessagePubSubType> getPubSubType()
   {
      return AdjustFootstepMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return AdjustFootstepMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(AdjustFootstepMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.robot_side_, other.robot_side_, epsilon)) return false;


      if (!this.location_.epsilonEquals(other.location_, epsilon)) return false;

      if (!this.orientation_.epsilonEquals(other.orientation_, epsilon)) return false;

      if (this.predicted_contact_points_2d_.size() != other.predicted_contact_points_2d_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.predicted_contact_points_2d_.size(); i++)
         {  if (!this.predicted_contact_points_2d_.get(i).epsilonEquals(other.predicted_contact_points_2d_.get(i), epsilon)) return false; }
      }


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.execution_delay_time_, other.execution_delay_time_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof AdjustFootstepMessage)) return false;

      AdjustFootstepMessage otherMyClass = (AdjustFootstepMessage) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if(this.robot_side_ != otherMyClass.robot_side_) return false;


      if (!this.location_.equals(otherMyClass.location_)) return false;

      if (!this.orientation_.equals(otherMyClass.orientation_)) return false;

      if (!this.predicted_contact_points_2d_.equals(otherMyClass.predicted_contact_points_2d_)) return false;

      if(this.execution_delay_time_ != otherMyClass.execution_delay_time_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("AdjustFootstepMessage {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("robot_side=");
      builder.append(this.robot_side_);      builder.append(", ");

      builder.append("location=");
      builder.append(this.location_);      builder.append(", ");

      builder.append("orientation=");
      builder.append(this.orientation_);      builder.append(", ");

      builder.append("predicted_contact_points_2d=");
      builder.append(this.predicted_contact_points_2d_);      builder.append(", ");

      builder.append("execution_delay_time=");
      builder.append(this.execution_delay_time_);
      builder.append("}");
      return builder.toString();
   }
}
