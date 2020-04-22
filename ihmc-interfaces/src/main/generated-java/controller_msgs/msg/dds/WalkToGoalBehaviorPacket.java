package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC humanoid behavior module.
       */
public class WalkToGoalBehaviorPacket extends Packet<WalkToGoalBehaviorPacket> implements Settable<WalkToGoalBehaviorPacket>, EpsilonComparable<WalkToGoalBehaviorPacket>
{

   public static final byte WALK_TO_GOAL_ACTION_FIND_PATH = (byte) 0;

   public static final byte WALK_TO_GOAL_ACTION_EXECUTE = (byte) 1;

   public static final byte WALK_TO_GOAL_ACTION_EXECUTE_UNKNOWN = (byte) 2;

   public static final byte WALK_TO_GOAL_ACTION_STOP = (byte) 3;

   public static final byte ROBOT_SIDE_LEFT = (byte) 0;

   public static final byte ROBOT_SIDE_RIGHT = (byte) 1;

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   public byte walk_to_goal_action_ = (byte) 255;

   public double x_goal_;

   public double y_goal_;

   public double theta_goal_;

   public byte goal_robot_side_ = (byte) 255;

   public WalkToGoalBehaviorPacket()
   {







   }

   public WalkToGoalBehaviorPacket(WalkToGoalBehaviorPacket other)
   {
      this();
      set(other);
   }

   public void set(WalkToGoalBehaviorPacket other)
   {

      sequence_id_ = other.sequence_id_;


      walk_to_goal_action_ = other.walk_to_goal_action_;


      x_goal_ = other.x_goal_;


      y_goal_ = other.y_goal_;


      theta_goal_ = other.theta_goal_;


      goal_robot_side_ = other.goal_robot_side_;

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


   public void setWalkToGoalAction(byte walk_to_goal_action)
   {
      walk_to_goal_action_ = walk_to_goal_action;
   }
   public byte getWalkToGoalAction()
   {
      return walk_to_goal_action_;
   }


   public void setXGoal(double x_goal)
   {
      x_goal_ = x_goal;
   }
   public double getXGoal()
   {
      return x_goal_;
   }


   public void setYGoal(double y_goal)
   {
      y_goal_ = y_goal;
   }
   public double getYGoal()
   {
      return y_goal_;
   }


   public void setThetaGoal(double theta_goal)
   {
      theta_goal_ = theta_goal;
   }
   public double getThetaGoal()
   {
      return theta_goal_;
   }


   public void setGoalRobotSide(byte goal_robot_side)
   {
      goal_robot_side_ = goal_robot_side;
   }
   public byte getGoalRobotSide()
   {
      return goal_robot_side_;
   }


   public static Supplier<WalkToGoalBehaviorPacketPubSubType> getPubSubType()
   {
      return WalkToGoalBehaviorPacketPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return WalkToGoalBehaviorPacketPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(WalkToGoalBehaviorPacket other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.walk_to_goal_action_, other.walk_to_goal_action_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.x_goal_, other.x_goal_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.y_goal_, other.y_goal_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.theta_goal_, other.theta_goal_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.goal_robot_side_, other.goal_robot_side_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof WalkToGoalBehaviorPacket)) return false;

      WalkToGoalBehaviorPacket otherMyClass = (WalkToGoalBehaviorPacket) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if(this.walk_to_goal_action_ != otherMyClass.walk_to_goal_action_) return false;


      if(this.x_goal_ != otherMyClass.x_goal_) return false;


      if(this.y_goal_ != otherMyClass.y_goal_) return false;


      if(this.theta_goal_ != otherMyClass.theta_goal_) return false;


      if(this.goal_robot_side_ != otherMyClass.goal_robot_side_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("WalkToGoalBehaviorPacket {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("walk_to_goal_action=");
      builder.append(this.walk_to_goal_action_);      builder.append(", ");

      builder.append("x_goal=");
      builder.append(this.x_goal_);      builder.append(", ");

      builder.append("y_goal=");
      builder.append(this.y_goal_);      builder.append(", ");

      builder.append("theta_goal=");
      builder.append(this.theta_goal_);      builder.append(", ");

      builder.append("goal_robot_side=");
      builder.append(this.goal_robot_side_);
      builder.append("}");
      return builder.toString();
   }
}
