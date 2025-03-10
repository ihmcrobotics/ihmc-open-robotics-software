package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class AI2RNavigationMessage extends Packet<AI2RNavigationMessage> implements Settable<AI2RNavigationMessage>, EpsilonComparable<AI2RNavigationMessage>
{
   /**
            * Goto action - Reference frame for the action
            */
   public java.lang.StringBuilder reference_frame_name_;
   /**
            * Goto action - The position to which the goal stance is aligned
            */
   public us.ihmc.euclid.tuple3D.Point3D goal_stance_point_;
   /**
            * Goto action - The point that the robot should be facing in the goal stance
            */
   public us.ihmc.euclid.tuple3D.Point3D goal_focal_point_;

   public AI2RNavigationMessage()
   {
      reference_frame_name_ = new java.lang.StringBuilder(255);
      goal_stance_point_ = new us.ihmc.euclid.tuple3D.Point3D();
      goal_focal_point_ = new us.ihmc.euclid.tuple3D.Point3D();
   }

   public AI2RNavigationMessage(AI2RNavigationMessage other)
   {
      this();
      set(other);
   }

   public void set(AI2RNavigationMessage other)
   {
      reference_frame_name_.setLength(0);
      reference_frame_name_.append(other.reference_frame_name_);

      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.goal_stance_point_, goal_stance_point_);
      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.goal_focal_point_, goal_focal_point_);
   }

   /**
            * Goto action - Reference frame for the action
            */
   public void setReferenceFrameName(java.lang.String reference_frame_name)
   {
      reference_frame_name_.setLength(0);
      reference_frame_name_.append(reference_frame_name);
   }

   /**
            * Goto action - Reference frame for the action
            */
   public java.lang.String getReferenceFrameNameAsString()
   {
      return getReferenceFrameName().toString();
   }
   /**
            * Goto action - Reference frame for the action
            */
   public java.lang.StringBuilder getReferenceFrameName()
   {
      return reference_frame_name_;
   }


   /**
            * Goto action - The position to which the goal stance is aligned
            */
   public us.ihmc.euclid.tuple3D.Point3D getGoalStancePoint()
   {
      return goal_stance_point_;
   }


   /**
            * Goto action - The point that the robot should be facing in the goal stance
            */
   public us.ihmc.euclid.tuple3D.Point3D getGoalFocalPoint()
   {
      return goal_focal_point_;
   }


   public static Supplier<AI2RNavigationMessagePubSubType> getPubSubType()
   {
      return AI2RNavigationMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return AI2RNavigationMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(AI2RNavigationMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.reference_frame_name_, other.reference_frame_name_, epsilon)) return false;

      if (!this.goal_stance_point_.epsilonEquals(other.goal_stance_point_, epsilon)) return false;
      if (!this.goal_focal_point_.epsilonEquals(other.goal_focal_point_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof AI2RNavigationMessage)) return false;

      AI2RNavigationMessage otherMyClass = (AI2RNavigationMessage) other;

      if (!us.ihmc.idl.IDLTools.equals(this.reference_frame_name_, otherMyClass.reference_frame_name_)) return false;

      if (!this.goal_stance_point_.equals(otherMyClass.goal_stance_point_)) return false;
      if (!this.goal_focal_point_.equals(otherMyClass.goal_focal_point_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("AI2RNavigationMessage {");
      builder.append("reference_frame_name=");
      builder.append(this.reference_frame_name_);      builder.append(", ");
      builder.append("goal_stance_point=");
      builder.append(this.goal_stance_point_);      builder.append(", ");
      builder.append("goal_focal_point=");
      builder.append(this.goal_focal_point_);
      builder.append("}");
      return builder.toString();
   }
}
