package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC footstep planning module.
       */
public class VisibilityClusterMessage extends Packet<VisibilityClusterMessage> implements Settable<VisibilityClusterMessage>, EpsilonComparable<VisibilityClusterMessage>
{
   public static final byte EXTRUSION_INSIDE = (byte) 0;
   public static final byte EXTRUSION_OUTSIDE = (byte) 1;
   public static final byte TYPE_LINE = (byte) 0;
   public static final byte TYPE_MULTI_LINE = (byte) 1;
   public static final byte TYPE_POLYGON = (byte) 2;
   public byte extrusion_side_ = (byte) 255;
   public byte type_ = (byte) 255;
   public us.ihmc.euclid.geometry.Pose3D pose_in_world_;
   public controller_msgs.msg.dds.VisibilityClusterPointsMessage raw_points_in_local_;
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.VisibilityClusterPointsMessage>  preferred_navigable_extrusions_in_local_;
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.VisibilityClusterPointsMessage>  preferred_non_navigable_extrusions_in_local_;
   public controller_msgs.msg.dds.VisibilityClusterPointsMessage navigable_extrusions_in_local_;
   public controller_msgs.msg.dds.VisibilityClusterPointsMessage non_navigable_extrusions_in_local_;

   public VisibilityClusterMessage()
   {
      pose_in_world_ = new us.ihmc.euclid.geometry.Pose3D();
      raw_points_in_local_ = new controller_msgs.msg.dds.VisibilityClusterPointsMessage();
      preferred_navigable_extrusions_in_local_ = new us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.VisibilityClusterPointsMessage> (25, new controller_msgs.msg.dds.VisibilityClusterPointsMessagePubSubType());
      preferred_non_navigable_extrusions_in_local_ = new us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.VisibilityClusterPointsMessage> (25, new controller_msgs.msg.dds.VisibilityClusterPointsMessagePubSubType());
      navigable_extrusions_in_local_ = new controller_msgs.msg.dds.VisibilityClusterPointsMessage();
      non_navigable_extrusions_in_local_ = new controller_msgs.msg.dds.VisibilityClusterPointsMessage();

   }

   public VisibilityClusterMessage(VisibilityClusterMessage other)
   {
      this();
      set(other);
   }

   public void set(VisibilityClusterMessage other)
   {
      extrusion_side_ = other.extrusion_side_;

      type_ = other.type_;

      geometry_msgs.msg.dds.PosePubSubType.staticCopy(other.pose_in_world_, pose_in_world_);
      controller_msgs.msg.dds.VisibilityClusterPointsMessagePubSubType.staticCopy(other.raw_points_in_local_, raw_points_in_local_);
      preferred_navigable_extrusions_in_local_.set(other.preferred_navigable_extrusions_in_local_);
      preferred_non_navigable_extrusions_in_local_.set(other.preferred_non_navigable_extrusions_in_local_);
      controller_msgs.msg.dds.VisibilityClusterPointsMessagePubSubType.staticCopy(other.navigable_extrusions_in_local_, navigable_extrusions_in_local_);
      controller_msgs.msg.dds.VisibilityClusterPointsMessagePubSubType.staticCopy(other.non_navigable_extrusions_in_local_, non_navigable_extrusions_in_local_);
   }

   public void setExtrusionSide(byte extrusion_side)
   {
      extrusion_side_ = extrusion_side;
   }
   public byte getExtrusionSide()
   {
      return extrusion_side_;
   }

   public void setType(byte type)
   {
      type_ = type;
   }
   public byte getType()
   {
      return type_;
   }


   public us.ihmc.euclid.geometry.Pose3D getPoseInWorld()
   {
      return pose_in_world_;
   }


   public controller_msgs.msg.dds.VisibilityClusterPointsMessage getRawPointsInLocal()
   {
      return raw_points_in_local_;
   }


   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.VisibilityClusterPointsMessage>  getPreferredNavigableExtrusionsInLocal()
   {
      return preferred_navigable_extrusions_in_local_;
   }


   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.VisibilityClusterPointsMessage>  getPreferredNonNavigableExtrusionsInLocal()
   {
      return preferred_non_navigable_extrusions_in_local_;
   }


   public controller_msgs.msg.dds.VisibilityClusterPointsMessage getNavigableExtrusionsInLocal()
   {
      return navigable_extrusions_in_local_;
   }


   public controller_msgs.msg.dds.VisibilityClusterPointsMessage getNonNavigableExtrusionsInLocal()
   {
      return non_navigable_extrusions_in_local_;
   }


   public static Supplier<VisibilityClusterMessagePubSubType> getPubSubType()
   {
      return VisibilityClusterMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return VisibilityClusterMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(VisibilityClusterMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.extrusion_side_, other.extrusion_side_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.type_, other.type_, epsilon)) return false;

      if (!this.pose_in_world_.epsilonEquals(other.pose_in_world_, epsilon)) return false;
      if (!this.raw_points_in_local_.epsilonEquals(other.raw_points_in_local_, epsilon)) return false;
      if (this.preferred_navigable_extrusions_in_local_.size() != other.preferred_navigable_extrusions_in_local_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.preferred_navigable_extrusions_in_local_.size(); i++)
         {  if (!this.preferred_navigable_extrusions_in_local_.get(i).epsilonEquals(other.preferred_navigable_extrusions_in_local_.get(i), epsilon)) return false; }
      }

      if (this.preferred_non_navigable_extrusions_in_local_.size() != other.preferred_non_navigable_extrusions_in_local_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.preferred_non_navigable_extrusions_in_local_.size(); i++)
         {  if (!this.preferred_non_navigable_extrusions_in_local_.get(i).epsilonEquals(other.preferred_non_navigable_extrusions_in_local_.get(i), epsilon)) return false; }
      }

      if (!this.navigable_extrusions_in_local_.epsilonEquals(other.navigable_extrusions_in_local_, epsilon)) return false;
      if (!this.non_navigable_extrusions_in_local_.epsilonEquals(other.non_navigable_extrusions_in_local_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof VisibilityClusterMessage)) return false;

      VisibilityClusterMessage otherMyClass = (VisibilityClusterMessage) other;

      if(this.extrusion_side_ != otherMyClass.extrusion_side_) return false;

      if(this.type_ != otherMyClass.type_) return false;

      if (!this.pose_in_world_.equals(otherMyClass.pose_in_world_)) return false;
      if (!this.raw_points_in_local_.equals(otherMyClass.raw_points_in_local_)) return false;
      if (!this.preferred_navigable_extrusions_in_local_.equals(otherMyClass.preferred_navigable_extrusions_in_local_)) return false;
      if (!this.preferred_non_navigable_extrusions_in_local_.equals(otherMyClass.preferred_non_navigable_extrusions_in_local_)) return false;
      if (!this.navigable_extrusions_in_local_.equals(otherMyClass.navigable_extrusions_in_local_)) return false;
      if (!this.non_navigable_extrusions_in_local_.equals(otherMyClass.non_navigable_extrusions_in_local_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("VisibilityClusterMessage {");
      builder.append("extrusion_side=");
      builder.append(this.extrusion_side_);      builder.append(", ");
      builder.append("type=");
      builder.append(this.type_);      builder.append(", ");
      builder.append("pose_in_world=");
      builder.append(this.pose_in_world_);      builder.append(", ");
      builder.append("raw_points_in_local=");
      builder.append(this.raw_points_in_local_);      builder.append(", ");
      builder.append("preferred_navigable_extrusions_in_local=");
      builder.append(this.preferred_navigable_extrusions_in_local_);      builder.append(", ");
      builder.append("preferred_non_navigable_extrusions_in_local=");
      builder.append(this.preferred_non_navigable_extrusions_in_local_);      builder.append(", ");
      builder.append("navigable_extrusions_in_local=");
      builder.append(this.navigable_extrusions_in_local_);      builder.append(", ");
      builder.append("non_navigable_extrusions_in_local=");
      builder.append(this.non_navigable_extrusions_in_local_);
      builder.append("}");
      return builder.toString();
   }
}
