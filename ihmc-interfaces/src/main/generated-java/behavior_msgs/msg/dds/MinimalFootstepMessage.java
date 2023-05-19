package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class MinimalFootstepMessage extends Packet<MinimalFootstepMessage> implements Settable<MinimalFootstepMessage>, EpsilonComparable<MinimalFootstepMessage>
{
   /**
            * Specifies which foot will swing to reach the footstep.
            * Field default value 255
            */
   public byte robot_side_;
   /**
            * Specifies the position of the footstep (sole frame) in world frame.
            */
   public us.ihmc.euclid.tuple3D.Point3D position_;
   /**
            * Specifies the orientation of the footstep (sole frame) in world frame.
            */
   public us.ihmc.euclid.tuple4D.Quaternion orientation_;
   /**
            * Support polygon
            */
   public ihmc_common_msgs.msg.dds.Point2DMessage[] support_polygon_;
   /**
            * Description of the footstep
            */
   public java.lang.StringBuilder description_;

   public MinimalFootstepMessage()
   {
      position_ = new us.ihmc.euclid.tuple3D.Point3D();
      orientation_ = new us.ihmc.euclid.tuple4D.Quaternion();
      support_polygon_ = new ihmc_common_msgs.msg.dds.Point2DMessage[16];

      for(int i1 = 0; i1 < support_polygon_.length; ++i1)
      {
          support_polygon_[i1] = new ihmc_common_msgs.msg.dds.Point2DMessage();
      }
      description_ = new java.lang.StringBuilder(255);
   }

   public MinimalFootstepMessage(MinimalFootstepMessage other)
   {
      this();
      set(other);
   }

   public void set(MinimalFootstepMessage other)
   {
      robot_side_ = other.robot_side_;

      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.position_, position_);
      geometry_msgs.msg.dds.QuaternionPubSubType.staticCopy(other.orientation_, orientation_);
      for(int i3 = 0; i3 < support_polygon_.length; ++i3)
      {
            ihmc_common_msgs.msg.dds.Point2DMessagePubSubType.staticCopy(other.support_polygon_[i3], support_polygon_[i3]);}

      description_.setLength(0);
      description_.append(other.description_);

   }

   /**
            * Specifies which foot will swing to reach the footstep.
            * Field default value 255
            */
   public void setRobotSide(byte robot_side)
   {
      robot_side_ = robot_side;
   }
   /**
            * Specifies which foot will swing to reach the footstep.
            * Field default value 255
            */
   public byte getRobotSide()
   {
      return robot_side_;
   }


   /**
            * Specifies the position of the footstep (sole frame) in world frame.
            */
   public us.ihmc.euclid.tuple3D.Point3D getPosition()
   {
      return position_;
   }


   /**
            * Specifies the orientation of the footstep (sole frame) in world frame.
            */
   public us.ihmc.euclid.tuple4D.Quaternion getOrientation()
   {
      return orientation_;
   }


   /**
            * Support polygon
            */
   public ihmc_common_msgs.msg.dds.Point2DMessage[] getSupportPolygon()
   {
      return support_polygon_;
   }

   /**
            * Description of the footstep
            */
   public void setDescription(java.lang.String description)
   {
      description_.setLength(0);
      description_.append(description);
   }

   /**
            * Description of the footstep
            */
   public java.lang.String getDescriptionAsString()
   {
      return getDescription().toString();
   }
   /**
            * Description of the footstep
            */
   public java.lang.StringBuilder getDescription()
   {
      return description_;
   }


   public static Supplier<MinimalFootstepMessagePubSubType> getPubSubType()
   {
      return MinimalFootstepMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return MinimalFootstepMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(MinimalFootstepMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.robot_side_, other.robot_side_, epsilon)) return false;

      if (!this.position_.epsilonEquals(other.position_, epsilon)) return false;
      if (!this.orientation_.epsilonEquals(other.orientation_, epsilon)) return false;
      for(int i5 = 0; i5 < support_polygon_.length; ++i5)
      {
              if (!this.support_polygon_[i5].epsilonEquals(other.support_polygon_[i5], epsilon)) return false;
      }

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.description_, other.description_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof MinimalFootstepMessage)) return false;

      MinimalFootstepMessage otherMyClass = (MinimalFootstepMessage) other;

      if(this.robot_side_ != otherMyClass.robot_side_) return false;

      if (!this.position_.equals(otherMyClass.position_)) return false;
      if (!this.orientation_.equals(otherMyClass.orientation_)) return false;
      for(int i7 = 0; i7 < support_polygon_.length; ++i7)
      {
                if (!this.support_polygon_[i7].equals(otherMyClass.support_polygon_[i7])) return false;
      }
      if (!us.ihmc.idl.IDLTools.equals(this.description_, otherMyClass.description_)) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("MinimalFootstepMessage {");
      builder.append("robot_side=");
      builder.append(this.robot_side_);      builder.append(", ");
      builder.append("position=");
      builder.append(this.position_);      builder.append(", ");
      builder.append("orientation=");
      builder.append(this.orientation_);      builder.append(", ");
      builder.append("support_polygon=");
      builder.append(java.util.Arrays.toString(this.support_polygon_));      builder.append(", ");
      builder.append("description=");
      builder.append(this.description_);
      builder.append("}");
      return builder.toString();
   }
}
