package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class AI2RCommandMessage extends Packet<AI2RCommandMessage> implements Settable<AI2RCommandMessage>, EpsilonComparable<AI2RCommandMessage>
{
   /**
            * Behavior to execute (checkpoint to jump to in the pre-loaded behavior collection)
            */
   public java.lang.StringBuilder behavior_to_execute_;
   /**
            * Goto action - Reference frame for the action
            */
   public java.lang.StringBuilder goto_reference_frame_name_;
   /**
            * Goto action - The position to which the goal stance is aligned
            */
   public us.ihmc.euclid.tuple3D.Point3D goto_goal_stance_point_;
   /**
            * Goto action - The point that the robot should be facing in the goal stance
            */
   public us.ihmc.euclid.tuple3D.Point3D goto_goal_focal_point_;

   public AI2RCommandMessage()
   {
      behavior_to_execute_ = new java.lang.StringBuilder(255);
      goto_reference_frame_name_ = new java.lang.StringBuilder(255);
      goto_goal_stance_point_ = new us.ihmc.euclid.tuple3D.Point3D();
      goto_goal_focal_point_ = new us.ihmc.euclid.tuple3D.Point3D();
   }

   public AI2RCommandMessage(AI2RCommandMessage other)
   {
      this();
      set(other);
   }

   public void set(AI2RCommandMessage other)
   {
      behavior_to_execute_.setLength(0);
      behavior_to_execute_.append(other.behavior_to_execute_);

      goto_reference_frame_name_.setLength(0);
      goto_reference_frame_name_.append(other.goto_reference_frame_name_);

      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.goto_goal_stance_point_, goto_goal_stance_point_);
      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.goto_goal_focal_point_, goto_goal_focal_point_);
   }

   /**
            * Behavior to execute (checkpoint to jump to in the pre-loaded behavior collection)
            */
   public void setBehaviorToExecute(java.lang.String behavior_to_execute)
   {
      behavior_to_execute_.setLength(0);
      behavior_to_execute_.append(behavior_to_execute);
   }

   /**
            * Behavior to execute (checkpoint to jump to in the pre-loaded behavior collection)
            */
   public java.lang.String getBehaviorToExecuteAsString()
   {
      return getBehaviorToExecute().toString();
   }
   /**
            * Behavior to execute (checkpoint to jump to in the pre-loaded behavior collection)
            */
   public java.lang.StringBuilder getBehaviorToExecute()
   {
      return behavior_to_execute_;
   }

   /**
            * Goto action - Reference frame for the action
            */
   public void setGotoReferenceFrameName(java.lang.String goto_reference_frame_name)
   {
      goto_reference_frame_name_.setLength(0);
      goto_reference_frame_name_.append(goto_reference_frame_name);
   }

   /**
            * Goto action - Reference frame for the action
            */
   public java.lang.String getGotoReferenceFrameNameAsString()
   {
      return getGotoReferenceFrameName().toString();
   }
   /**
            * Goto action - Reference frame for the action
            */
   public java.lang.StringBuilder getGotoReferenceFrameName()
   {
      return goto_reference_frame_name_;
   }


   /**
            * Goto action - The position to which the goal stance is aligned
            */
   public us.ihmc.euclid.tuple3D.Point3D getGotoGoalStancePoint()
   {
      return goto_goal_stance_point_;
   }


   /**
            * Goto action - The point that the robot should be facing in the goal stance
            */
   public us.ihmc.euclid.tuple3D.Point3D getGotoGoalFocalPoint()
   {
      return goto_goal_focal_point_;
   }


   public static Supplier<AI2RCommandMessagePubSubType> getPubSubType()
   {
      return AI2RCommandMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return AI2RCommandMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(AI2RCommandMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.behavior_to_execute_, other.behavior_to_execute_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.goto_reference_frame_name_, other.goto_reference_frame_name_, epsilon)) return false;

      if (!this.goto_goal_stance_point_.epsilonEquals(other.goto_goal_stance_point_, epsilon)) return false;
      if (!this.goto_goal_focal_point_.epsilonEquals(other.goto_goal_focal_point_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof AI2RCommandMessage)) return false;

      AI2RCommandMessage otherMyClass = (AI2RCommandMessage) other;

      if (!us.ihmc.idl.IDLTools.equals(this.behavior_to_execute_, otherMyClass.behavior_to_execute_)) return false;

      if (!us.ihmc.idl.IDLTools.equals(this.goto_reference_frame_name_, otherMyClass.goto_reference_frame_name_)) return false;

      if (!this.goto_goal_stance_point_.equals(otherMyClass.goto_goal_stance_point_)) return false;
      if (!this.goto_goal_focal_point_.equals(otherMyClass.goto_goal_focal_point_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("AI2RCommandMessage {");
      builder.append("behavior_to_execute=");
      builder.append(this.behavior_to_execute_);      builder.append(", ");
      builder.append("goto_reference_frame_name=");
      builder.append(this.goto_reference_frame_name_);      builder.append(", ");
      builder.append("goto_goal_stance_point=");
      builder.append(this.goto_goal_stance_point_);      builder.append(", ");
      builder.append("goto_goal_focal_point=");
      builder.append(this.goto_goal_focal_point_);
      builder.append("}");
      return builder.toString();
   }
}
