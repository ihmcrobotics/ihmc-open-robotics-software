package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;

public class QuadrupedStepMessage extends Packet<QuadrupedStepMessage> implements Settable<QuadrupedStepMessage>, EpsilonComparable<QuadrupedStepMessage>
{
   public static final byte FRONT_LEFT = (byte) 0;
   public static final byte FRONT_RIGHT = (byte) 1;
   public static final byte HIND_RIGHT = (byte) 2;
   public static final byte HIND_LEFT = (byte) 3;
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   public byte robot_quadrant_ = (byte) 255;
   public us.ihmc.euclid.tuple3D.Point3D goal_position_;
   public double ground_clearance_;

   public QuadrupedStepMessage()
   {
      goal_position_ = new us.ihmc.euclid.tuple3D.Point3D();
   }

   public QuadrupedStepMessage(QuadrupedStepMessage other)
   {
      this();
      set(other);
   }

   public void set(QuadrupedStepMessage other)
   {
      sequence_id_ = other.sequence_id_;

      robot_quadrant_ = other.robot_quadrant_;

      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.goal_position_, goal_position_);
      ground_clearance_ = other.ground_clearance_;

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

   public void setRobotQuadrant(byte robot_quadrant)
   {
      robot_quadrant_ = robot_quadrant;
   }
   public byte getRobotQuadrant()
   {
      return robot_quadrant_;
   }


   public us.ihmc.euclid.tuple3D.Point3D getGoalPosition()
   {
      return goal_position_;
   }

   public void setGroundClearance(double ground_clearance)
   {
      ground_clearance_ = ground_clearance;
   }
   public double getGroundClearance()
   {
      return ground_clearance_;
   }


   @Override
   public boolean epsilonEquals(QuadrupedStepMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.robot_quadrant_, other.robot_quadrant_, epsilon)) return false;

      if (!this.goal_position_.epsilonEquals(other.goal_position_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.ground_clearance_, other.ground_clearance_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof QuadrupedStepMessage)) return false;

      QuadrupedStepMessage otherMyClass = (QuadrupedStepMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.robot_quadrant_ != otherMyClass.robot_quadrant_) return false;

      if (!this.goal_position_.equals(otherMyClass.goal_position_)) return false;
      if(this.ground_clearance_ != otherMyClass.ground_clearance_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("QuadrupedStepMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("robot_quadrant=");
      builder.append(this.robot_quadrant_);      builder.append(", ");
      builder.append("goal_position=");
      builder.append(this.goal_position_);      builder.append(", ");
      builder.append("ground_clearance=");
      builder.append(this.ground_clearance_);
      builder.append("}");
      return builder.toString();
   }
}
