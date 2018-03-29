package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;

/**
 * This message should disappear to be replaced by ROS equivalent.
 */
public class LidarScanMessage extends Packet<LidarScanMessage> implements Settable<LidarScanMessage>, EpsilonComparable<LidarScanMessage>
{
   /**
    * As of March 2018, the header for this message is only use for its sequence ID.
    */
   public std_msgs.msg.dds.Header header_;
   public long robot_timestamp_;
   public us.ihmc.euclid.tuple3D.Point3D lidar_position_;
   public us.ihmc.euclid.tuple4D.Quaternion lidar_orientation_;
   public us.ihmc.idl.IDLSequence.Float scan_;

   public LidarScanMessage()
   {
      header_ = new std_msgs.msg.dds.Header();
      lidar_position_ = new us.ihmc.euclid.tuple3D.Point3D();
      lidar_orientation_ = new us.ihmc.euclid.tuple4D.Quaternion();
      scan_ = new us.ihmc.idl.IDLSequence.Float(100, "type_5");

   }

   public LidarScanMessage(LidarScanMessage other)
   {
      this();
      set(other);
   }

   public void set(LidarScanMessage other)
   {
      std_msgs.msg.dds.HeaderPubSubType.staticCopy(other.header_, header_);
      robot_timestamp_ = other.robot_timestamp_;

      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.lidar_position_, lidar_position_);
      geometry_msgs.msg.dds.QuaternionPubSubType.staticCopy(other.lidar_orientation_, lidar_orientation_);
      scan_.set(other.scan_);
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

   public us.ihmc.euclid.tuple3D.Point3D getLidarPosition()
   {
      return lidar_position_;
   }

   public us.ihmc.euclid.tuple4D.Quaternion getLidarOrientation()
   {
      return lidar_orientation_;
   }

   public us.ihmc.idl.IDLSequence.Float getScan()
   {
      return scan_;
   }

   @Override
   public boolean epsilonEquals(LidarScanMessage other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!this.header_.epsilonEquals(other.header_, epsilon))
         return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.robot_timestamp_, other.robot_timestamp_, epsilon))
         return false;

      if (!this.lidar_position_.epsilonEquals(other.lidar_position_, epsilon))
         return false;
      if (!this.lidar_orientation_.epsilonEquals(other.lidar_orientation_, epsilon))
         return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsFloatSequence(this.scan_, other.scan_, epsilon))
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
      if (!(other instanceof LidarScanMessage))
         return false;

      LidarScanMessage otherMyClass = (LidarScanMessage) other;

      if (!this.header_.equals(otherMyClass.header_))
         return false;
      if (this.robot_timestamp_ != otherMyClass.robot_timestamp_)
         return false;

      if (!this.lidar_position_.equals(otherMyClass.lidar_position_))
         return false;
      if (!this.lidar_orientation_.equals(otherMyClass.lidar_orientation_))
         return false;
      if (!this.scan_.equals(otherMyClass.scan_))
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("LidarScanMessage {");
      builder.append("header=");
      builder.append(this.header_);
      builder.append(", ");
      builder.append("robot_timestamp=");
      builder.append(this.robot_timestamp_);
      builder.append(", ");
      builder.append("lidar_position=");
      builder.append(this.lidar_position_);
      builder.append(", ");
      builder.append("lidar_orientation=");
      builder.append(this.lidar_orientation_);
      builder.append(", ");
      builder.append("scan=");
      builder.append(this.scan_);
      builder.append("}");
      return builder.toString();
   }
}
