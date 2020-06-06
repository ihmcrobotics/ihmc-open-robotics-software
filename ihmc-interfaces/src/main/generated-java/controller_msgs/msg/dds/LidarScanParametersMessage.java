package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * Message used for simulated LIDAR
       */
public class LidarScanParametersMessage extends Packet<LidarScanParametersMessage> implements Settable<LidarScanParametersMessage>, EpsilonComparable<LidarScanParametersMessage>
{

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   public long timestamp_;

   public float sweep_yaw_max_;

   public float sweep_yaw_min_;

   public float height_pitch_max_;

   public float height_pitch_min_;

   public float time_increment_;

   public float scan_time_;

   public float min_range_;

   public float max_range_;

   public int points_per_sweep_;

   public int scan_height_;

   public LidarScanParametersMessage()
   {













   }

   public LidarScanParametersMessage(LidarScanParametersMessage other)
   {
      this();
      set(other);
   }

   public void set(LidarScanParametersMessage other)
   {

      sequence_id_ = other.sequence_id_;


      timestamp_ = other.timestamp_;


      sweep_yaw_max_ = other.sweep_yaw_max_;


      sweep_yaw_min_ = other.sweep_yaw_min_;


      height_pitch_max_ = other.height_pitch_max_;


      height_pitch_min_ = other.height_pitch_min_;


      time_increment_ = other.time_increment_;


      scan_time_ = other.scan_time_;


      min_range_ = other.min_range_;


      max_range_ = other.max_range_;


      points_per_sweep_ = other.points_per_sweep_;


      scan_height_ = other.scan_height_;

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


   public void setTimestamp(long timestamp)
   {
      timestamp_ = timestamp;
   }
   public long getTimestamp()
   {
      return timestamp_;
   }


   public void setSweepYawMax(float sweep_yaw_max)
   {
      sweep_yaw_max_ = sweep_yaw_max;
   }
   public float getSweepYawMax()
   {
      return sweep_yaw_max_;
   }


   public void setSweepYawMin(float sweep_yaw_min)
   {
      sweep_yaw_min_ = sweep_yaw_min;
   }
   public float getSweepYawMin()
   {
      return sweep_yaw_min_;
   }


   public void setHeightPitchMax(float height_pitch_max)
   {
      height_pitch_max_ = height_pitch_max;
   }
   public float getHeightPitchMax()
   {
      return height_pitch_max_;
   }


   public void setHeightPitchMin(float height_pitch_min)
   {
      height_pitch_min_ = height_pitch_min;
   }
   public float getHeightPitchMin()
   {
      return height_pitch_min_;
   }


   public void setTimeIncrement(float time_increment)
   {
      time_increment_ = time_increment;
   }
   public float getTimeIncrement()
   {
      return time_increment_;
   }


   public void setScanTime(float scan_time)
   {
      scan_time_ = scan_time;
   }
   public float getScanTime()
   {
      return scan_time_;
   }


   public void setMinRange(float min_range)
   {
      min_range_ = min_range;
   }
   public float getMinRange()
   {
      return min_range_;
   }


   public void setMaxRange(float max_range)
   {
      max_range_ = max_range;
   }
   public float getMaxRange()
   {
      return max_range_;
   }


   public void setPointsPerSweep(int points_per_sweep)
   {
      points_per_sweep_ = points_per_sweep;
   }
   public int getPointsPerSweep()
   {
      return points_per_sweep_;
   }


   public void setScanHeight(int scan_height)
   {
      scan_height_ = scan_height;
   }
   public int getScanHeight()
   {
      return scan_height_;
   }


   public static Supplier<LidarScanParametersMessagePubSubType> getPubSubType()
   {
      return LidarScanParametersMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return LidarScanParametersMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(LidarScanParametersMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.timestamp_, other.timestamp_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sweep_yaw_max_, other.sweep_yaw_max_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sweep_yaw_min_, other.sweep_yaw_min_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.height_pitch_max_, other.height_pitch_max_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.height_pitch_min_, other.height_pitch_min_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.time_increment_, other.time_increment_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.scan_time_, other.scan_time_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.min_range_, other.min_range_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.max_range_, other.max_range_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.points_per_sweep_, other.points_per_sweep_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.scan_height_, other.scan_height_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof LidarScanParametersMessage)) return false;

      LidarScanParametersMessage otherMyClass = (LidarScanParametersMessage) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if(this.timestamp_ != otherMyClass.timestamp_) return false;


      if(this.sweep_yaw_max_ != otherMyClass.sweep_yaw_max_) return false;


      if(this.sweep_yaw_min_ != otherMyClass.sweep_yaw_min_) return false;


      if(this.height_pitch_max_ != otherMyClass.height_pitch_max_) return false;


      if(this.height_pitch_min_ != otherMyClass.height_pitch_min_) return false;


      if(this.time_increment_ != otherMyClass.time_increment_) return false;


      if(this.scan_time_ != otherMyClass.scan_time_) return false;


      if(this.min_range_ != otherMyClass.min_range_) return false;


      if(this.max_range_ != otherMyClass.max_range_) return false;


      if(this.points_per_sweep_ != otherMyClass.points_per_sweep_) return false;


      if(this.scan_height_ != otherMyClass.scan_height_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("LidarScanParametersMessage {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("timestamp=");
      builder.append(this.timestamp_);      builder.append(", ");

      builder.append("sweep_yaw_max=");
      builder.append(this.sweep_yaw_max_);      builder.append(", ");

      builder.append("sweep_yaw_min=");
      builder.append(this.sweep_yaw_min_);      builder.append(", ");

      builder.append("height_pitch_max=");
      builder.append(this.height_pitch_max_);      builder.append(", ");

      builder.append("height_pitch_min=");
      builder.append(this.height_pitch_min_);      builder.append(", ");

      builder.append("time_increment=");
      builder.append(this.time_increment_);      builder.append(", ");

      builder.append("scan_time=");
      builder.append(this.scan_time_);      builder.append(", ");

      builder.append("min_range=");
      builder.append(this.min_range_);      builder.append(", ");

      builder.append("max_range=");
      builder.append(this.max_range_);      builder.append(", ");

      builder.append("points_per_sweep=");
      builder.append(this.points_per_sweep_);      builder.append(", ");

      builder.append("scan_height=");
      builder.append(this.scan_height_);
      builder.append("}");
      return builder.toString();
   }
}
