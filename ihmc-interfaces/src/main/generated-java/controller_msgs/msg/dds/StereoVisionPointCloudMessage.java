package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;

/**
 * Should disappear for the ROS equivalent.
 */
public class StereoVisionPointCloudMessage extends Packet<StereoVisionPointCloudMessage>
      implements Settable<StereoVisionPointCloudMessage>, EpsilonComparable<StereoVisionPointCloudMessage>
{
   /**
    * As of March 2018, the header for this message is only use for its sequence ID.
    */
   public std_msgs.msg.dds.Header header_;
   public long robot_timestamp_;
   public us.ihmc.idl.IDLSequence.Float point_cloud_;
   public us.ihmc.idl.IDLSequence.Integer colors_;

   public StereoVisionPointCloudMessage()
   {
      header_ = new std_msgs.msg.dds.Header();
      point_cloud_ = new us.ihmc.idl.IDLSequence.Float(100, "type_5");

      colors_ = new us.ihmc.idl.IDLSequence.Integer(100, "type_2");

   }

   public StereoVisionPointCloudMessage(StereoVisionPointCloudMessage other)
   {
      this();
      set(other);
   }

   public void set(StereoVisionPointCloudMessage other)
   {
      std_msgs.msg.dds.HeaderPubSubType.staticCopy(other.header_, header_);
      robot_timestamp_ = other.robot_timestamp_;

      point_cloud_.set(other.point_cloud_);
      colors_.set(other.colors_);
   }

   /**
    * As of March 2018, the header for this message is only use for its sequence ID.
    */
   public std_msgs.msg.dds.Header getHeader()
   {
      return header_;
   }

   public void setRobotTimestamp(long robot_timestamp)
   {
      robot_timestamp_ = robot_timestamp;
   }

   public long getRobotTimestamp()
   {
      return robot_timestamp_;
   }

   public us.ihmc.idl.IDLSequence.Float getPointCloud()
   {
      return point_cloud_;
   }

   public us.ihmc.idl.IDLSequence.Integer getColors()
   {
      return colors_;
   }

   @Override
   public boolean epsilonEquals(StereoVisionPointCloudMessage other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!this.header_.epsilonEquals(other.header_, epsilon))
         return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.robot_timestamp_, other.robot_timestamp_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsFloatSequence(this.point_cloud_, other.point_cloud_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsIntegerSequence(this.colors_, other.colors_, epsilon))
         return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;
      if (!(other instanceof StereoVisionPointCloudMessage))
         return false;

      StereoVisionPointCloudMessage otherMyClass = (StereoVisionPointCloudMessage) other;

      if (!this.header_.equals(otherMyClass.header_))
         return false;
      if (this.robot_timestamp_ != otherMyClass.robot_timestamp_)
         return false;

      if (!this.point_cloud_.equals(otherMyClass.point_cloud_))
         return false;
      if (!this.colors_.equals(otherMyClass.colors_))
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("StereoVisionPointCloudMessage {");
      builder.append("header=");
      builder.append(this.header_);
      builder.append(", ");
      builder.append("robot_timestamp=");
      builder.append(this.robot_timestamp_);
      builder.append(", ");
      builder.append("point_cloud=");
      builder.append(this.point_cloud_);
      builder.append(", ");
      builder.append("colors=");
      builder.append(this.colors_);
      builder.append("}");
      return builder.toString();
   }
}
