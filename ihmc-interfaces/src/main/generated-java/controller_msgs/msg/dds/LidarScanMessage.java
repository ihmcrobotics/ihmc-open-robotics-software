package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message should disappear to be replaced by ROS equivalent.
       */
public class LidarScanMessage extends Packet<LidarScanMessage> implements Settable<LidarScanMessage>, EpsilonComparable<LidarScanMessage>
{
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   public long robot_timestamp_;
   public us.ihmc.euclid.tuple3D.Point3D lidar_position_;
   public us.ihmc.euclid.tuple4D.Quaternion lidar_orientation_;
   public us.ihmc.idl.IDLSequence.Float  scan_;

   public LidarScanMessage()
   {
      lidar_position_ = new us.ihmc.euclid.tuple3D.Point3D();
      lidar_orientation_ = new us.ihmc.euclid.tuple4D.Quaternion();
      scan_ = new us.ihmc.idl.IDLSequence.Float (200000, "type_5");

   }

   public LidarScanMessage(LidarScanMessage other)
   {
      this();
      set(other);
   }

   public void set(LidarScanMessage other)
   {
      sequence_id_ = other.sequence_id_;

      robot_timestamp_ = other.robot_timestamp_;

      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.lidar_position_, lidar_position_);
      geometry_msgs.msg.dds.QuaternionPubSubType.staticCopy(other.lidar_orientation_, lidar_orientation_);
      scan_.set(other.scan_);
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


   public us.ihmc.idl.IDLSequence.Float  getScan()
   {
      return scan_;
   }


   public static Supplier<LidarScanMessagePubSubType> getPubSubType()
   {
      return LidarScanMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return LidarScanMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(LidarScanMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.robot_timestamp_, other.robot_timestamp_, epsilon)) return false;

      if (!this.lidar_position_.epsilonEquals(other.lidar_position_, epsilon)) return false;
      if (!this.lidar_orientation_.epsilonEquals(other.lidar_orientation_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsFloatSequence(this.scan_, other.scan_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof LidarScanMessage)) return false;

      LidarScanMessage otherMyClass = (LidarScanMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.robot_timestamp_ != otherMyClass.robot_timestamp_) return false;

      if (!this.lidar_position_.equals(otherMyClass.lidar_position_)) return false;
      if (!this.lidar_orientation_.equals(otherMyClass.lidar_orientation_)) return false;
      if (!this.scan_.equals(otherMyClass.scan_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("LidarScanMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("robot_timestamp=");
      builder.append(this.robot_timestamp_);      builder.append(", ");
      builder.append("lidar_position=");
      builder.append(this.lidar_position_);      builder.append(", ");
      builder.append("lidar_orientation=");
      builder.append(this.lidar_orientation_);      builder.append(", ");
      builder.append("scan=");
      builder.append(this.scan_);
      builder.append("}");
      return builder.toString();
   }
}
