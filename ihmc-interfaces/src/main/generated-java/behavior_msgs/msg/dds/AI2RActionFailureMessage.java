package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * For use by external AI to modify the behavior tree and operate behaviors
       */
public class AI2RActionFailureMessage extends Packet<AI2RActionFailureMessage> implements Settable<AI2RActionFailureMessage>, EpsilonComparable<AI2RActionFailureMessage>
{
   /**
            * Name of the action that has failed
            */
   public java.lang.StringBuilder action_name_;
   /**
            * Type of the action
            */
   public java.lang.StringBuilder action_type_;
   /**
            * Reference frame of the action
            */
   public java.lang.StringBuilder action_frame_;
   /**
            * Position tolerance for the action [m]
            */
   public double position_tolerance_;
   /**
            * Orientation tolerance for the action [deg]
            */
   public double orientation_tolerance_;
   /**
            * Error in position for the action
            */
   public us.ihmc.euclid.tuple3D.Point3D position_error_;
   /**
            * Error in orientation for the action
            */
   public us.ihmc.euclid.tuple4D.Quaternion orientation_error_;

   public AI2RActionFailureMessage()
   {
      action_name_ = new java.lang.StringBuilder(255);
      action_type_ = new java.lang.StringBuilder(255);
      action_frame_ = new java.lang.StringBuilder(255);
      position_error_ = new us.ihmc.euclid.tuple3D.Point3D();
      orientation_error_ = new us.ihmc.euclid.tuple4D.Quaternion();
   }

   public AI2RActionFailureMessage(AI2RActionFailureMessage other)
   {
      this();
      set(other);
   }

   public void set(AI2RActionFailureMessage other)
   {
      action_name_.setLength(0);
      action_name_.append(other.action_name_);

      action_type_.setLength(0);
      action_type_.append(other.action_type_);

      action_frame_.setLength(0);
      action_frame_.append(other.action_frame_);

      position_tolerance_ = other.position_tolerance_;

      orientation_tolerance_ = other.orientation_tolerance_;

      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.position_error_, position_error_);
      geometry_msgs.msg.dds.QuaternionPubSubType.staticCopy(other.orientation_error_, orientation_error_);
   }

   /**
            * Name of the action that has failed
            */
   public void setActionName(java.lang.String action_name)
   {
      action_name_.setLength(0);
      action_name_.append(action_name);
   }

   /**
            * Name of the action that has failed
            */
   public java.lang.String getActionNameAsString()
   {
      return getActionName().toString();
   }
   /**
            * Name of the action that has failed
            */
   public java.lang.StringBuilder getActionName()
   {
      return action_name_;
   }

   /**
            * Type of the action
            */
   public void setActionType(java.lang.String action_type)
   {
      action_type_.setLength(0);
      action_type_.append(action_type);
   }

   /**
            * Type of the action
            */
   public java.lang.String getActionTypeAsString()
   {
      return getActionType().toString();
   }
   /**
            * Type of the action
            */
   public java.lang.StringBuilder getActionType()
   {
      return action_type_;
   }

   /**
            * Reference frame of the action
            */
   public void setActionFrame(java.lang.String action_frame)
   {
      action_frame_.setLength(0);
      action_frame_.append(action_frame);
   }

   /**
            * Reference frame of the action
            */
   public java.lang.String getActionFrameAsString()
   {
      return getActionFrame().toString();
   }
   /**
            * Reference frame of the action
            */
   public java.lang.StringBuilder getActionFrame()
   {
      return action_frame_;
   }

   /**
            * Position tolerance for the action [m]
            */
   public void setPositionTolerance(double position_tolerance)
   {
      position_tolerance_ = position_tolerance;
   }
   /**
            * Position tolerance for the action [m]
            */
   public double getPositionTolerance()
   {
      return position_tolerance_;
   }

   /**
            * Orientation tolerance for the action [deg]
            */
   public void setOrientationTolerance(double orientation_tolerance)
   {
      orientation_tolerance_ = orientation_tolerance;
   }
   /**
            * Orientation tolerance for the action [deg]
            */
   public double getOrientationTolerance()
   {
      return orientation_tolerance_;
   }


   /**
            * Error in position for the action
            */
   public us.ihmc.euclid.tuple3D.Point3D getPositionError()
   {
      return position_error_;
   }


   /**
            * Error in orientation for the action
            */
   public us.ihmc.euclid.tuple4D.Quaternion getOrientationError()
   {
      return orientation_error_;
   }


   public static Supplier<AI2RActionFailureMessagePubSubType> getPubSubType()
   {
      return AI2RActionFailureMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return AI2RActionFailureMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(AI2RActionFailureMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.action_name_, other.action_name_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.action_type_, other.action_type_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.action_frame_, other.action_frame_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.position_tolerance_, other.position_tolerance_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.orientation_tolerance_, other.orientation_tolerance_, epsilon)) return false;

      if (!this.position_error_.epsilonEquals(other.position_error_, epsilon)) return false;
      if (!this.orientation_error_.epsilonEquals(other.orientation_error_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof AI2RActionFailureMessage)) return false;

      AI2RActionFailureMessage otherMyClass = (AI2RActionFailureMessage) other;

      if (!us.ihmc.idl.IDLTools.equals(this.action_name_, otherMyClass.action_name_)) return false;

      if (!us.ihmc.idl.IDLTools.equals(this.action_type_, otherMyClass.action_type_)) return false;

      if (!us.ihmc.idl.IDLTools.equals(this.action_frame_, otherMyClass.action_frame_)) return false;

      if(this.position_tolerance_ != otherMyClass.position_tolerance_) return false;

      if(this.orientation_tolerance_ != otherMyClass.orientation_tolerance_) return false;

      if (!this.position_error_.equals(otherMyClass.position_error_)) return false;
      if (!this.orientation_error_.equals(otherMyClass.orientation_error_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("AI2RActionFailureMessage {");
      builder.append("action_name=");
      builder.append(this.action_name_);      builder.append(", ");
      builder.append("action_type=");
      builder.append(this.action_type_);      builder.append(", ");
      builder.append("action_frame=");
      builder.append(this.action_frame_);      builder.append(", ");
      builder.append("position_tolerance=");
      builder.append(this.position_tolerance_);      builder.append(", ");
      builder.append("orientation_tolerance=");
      builder.append(this.orientation_tolerance_);      builder.append(", ");
      builder.append("position_error=");
      builder.append(this.position_error_);      builder.append(", ");
      builder.append("orientation_error=");
      builder.append(this.orientation_error_);
      builder.append("}");
      return builder.toString();
   }
}
