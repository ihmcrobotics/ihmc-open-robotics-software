package unitree_go_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class LidarState extends Packet<LidarState> implements Settable<LidarState>, EpsilonComparable<LidarState>
{
   public double stamp_;
   public java.lang.StringBuilder firmware_version_;
   public java.lang.StringBuilder software_version_;
   public java.lang.StringBuilder sdk_version_;
   public float sys_rotation_speed_;
   public float com_rotation_speed_;
   public byte error_state_;
   public float cloud_frequency_;
   public float cloud_packet_loss_rate_;
   public long cloud_size_;
   public long cloud_scan_num_;
   public float imu_frequency_;
   public float imu_packet_loss_rate_;
   public float[] imu_rpy_;
   public double serial_recv_stamp_;
   public long serial_buffer_size_;
   public long serial_buffer_read_;

   public LidarState()
   {
      firmware_version_ = new java.lang.StringBuilder(255);
      software_version_ = new java.lang.StringBuilder(255);
      sdk_version_ = new java.lang.StringBuilder(255);
      imu_rpy_ = new float[3];

   }

   public LidarState(LidarState other)
   {
      this();
      set(other);
   }

   public void set(LidarState other)
   {
      stamp_ = other.stamp_;

      firmware_version_.setLength(0);
      firmware_version_.append(other.firmware_version_);

      software_version_.setLength(0);
      software_version_.append(other.software_version_);

      sdk_version_.setLength(0);
      sdk_version_.append(other.sdk_version_);

      sys_rotation_speed_ = other.sys_rotation_speed_;

      com_rotation_speed_ = other.com_rotation_speed_;

      error_state_ = other.error_state_;

      cloud_frequency_ = other.cloud_frequency_;

      cloud_packet_loss_rate_ = other.cloud_packet_loss_rate_;

      cloud_size_ = other.cloud_size_;

      cloud_scan_num_ = other.cloud_scan_num_;

      imu_frequency_ = other.imu_frequency_;

      imu_packet_loss_rate_ = other.imu_packet_loss_rate_;

      for(int i1 = 0; i1 < imu_rpy_.length; ++i1)
      {
            imu_rpy_[i1] = other.imu_rpy_[i1];

      }

      serial_recv_stamp_ = other.serial_recv_stamp_;

      serial_buffer_size_ = other.serial_buffer_size_;

      serial_buffer_read_ = other.serial_buffer_read_;

   }

   public void setStamp(double stamp)
   {
      stamp_ = stamp;
   }
   public double getStamp()
   {
      return stamp_;
   }

   public void setFirmwareVersion(java.lang.String firmware_version)
   {
      firmware_version_.setLength(0);
      firmware_version_.append(firmware_version);
   }

   public java.lang.String getFirmwareVersionAsString()
   {
      return getFirmwareVersion().toString();
   }
   public java.lang.StringBuilder getFirmwareVersion()
   {
      return firmware_version_;
   }

   public void setSoftwareVersion(java.lang.String software_version)
   {
      software_version_.setLength(0);
      software_version_.append(software_version);
   }

   public java.lang.String getSoftwareVersionAsString()
   {
      return getSoftwareVersion().toString();
   }
   public java.lang.StringBuilder getSoftwareVersion()
   {
      return software_version_;
   }

   public void setSdkVersion(java.lang.String sdk_version)
   {
      sdk_version_.setLength(0);
      sdk_version_.append(sdk_version);
   }

   public java.lang.String getSdkVersionAsString()
   {
      return getSdkVersion().toString();
   }
   public java.lang.StringBuilder getSdkVersion()
   {
      return sdk_version_;
   }

   public void setSysRotationSpeed(float sys_rotation_speed)
   {
      sys_rotation_speed_ = sys_rotation_speed;
   }
   public float getSysRotationSpeed()
   {
      return sys_rotation_speed_;
   }

   public void setComRotationSpeed(float com_rotation_speed)
   {
      com_rotation_speed_ = com_rotation_speed;
   }
   public float getComRotationSpeed()
   {
      return com_rotation_speed_;
   }

   public void setErrorState(byte error_state)
   {
      error_state_ = error_state;
   }
   public byte getErrorState()
   {
      return error_state_;
   }

   public void setCloudFrequency(float cloud_frequency)
   {
      cloud_frequency_ = cloud_frequency;
   }
   public float getCloudFrequency()
   {
      return cloud_frequency_;
   }

   public void setCloudPacketLossRate(float cloud_packet_loss_rate)
   {
      cloud_packet_loss_rate_ = cloud_packet_loss_rate;
   }
   public float getCloudPacketLossRate()
   {
      return cloud_packet_loss_rate_;
   }

   public void setCloudSize(long cloud_size)
   {
      cloud_size_ = cloud_size;
   }
   public long getCloudSize()
   {
      return cloud_size_;
   }

   public void setCloudScanNum(long cloud_scan_num)
   {
      cloud_scan_num_ = cloud_scan_num;
   }
   public long getCloudScanNum()
   {
      return cloud_scan_num_;
   }

   public void setImuFrequency(float imu_frequency)
   {
      imu_frequency_ = imu_frequency;
   }
   public float getImuFrequency()
   {
      return imu_frequency_;
   }

   public void setImuPacketLossRate(float imu_packet_loss_rate)
   {
      imu_packet_loss_rate_ = imu_packet_loss_rate;
   }
   public float getImuPacketLossRate()
   {
      return imu_packet_loss_rate_;
   }


   public float[] getImuRpy()
   {
      return imu_rpy_;
   }

   public void setSerialRecvStamp(double serial_recv_stamp)
   {
      serial_recv_stamp_ = serial_recv_stamp;
   }
   public double getSerialRecvStamp()
   {
      return serial_recv_stamp_;
   }

   public void setSerialBufferSize(long serial_buffer_size)
   {
      serial_buffer_size_ = serial_buffer_size;
   }
   public long getSerialBufferSize()
   {
      return serial_buffer_size_;
   }

   public void setSerialBufferRead(long serial_buffer_read)
   {
      serial_buffer_read_ = serial_buffer_read;
   }
   public long getSerialBufferRead()
   {
      return serial_buffer_read_;
   }


   public static Supplier<LidarStatePubSubType> getPubSubType()
   {
      return LidarStatePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return LidarStatePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(LidarState other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.stamp_, other.stamp_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.firmware_version_, other.firmware_version_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.software_version_, other.software_version_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.sdk_version_, other.sdk_version_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sys_rotation_speed_, other.sys_rotation_speed_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.com_rotation_speed_, other.com_rotation_speed_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.error_state_, other.error_state_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.cloud_frequency_, other.cloud_frequency_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.cloud_packet_loss_rate_, other.cloud_packet_loss_rate_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.cloud_size_, other.cloud_size_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.cloud_scan_num_, other.cloud_scan_num_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.imu_frequency_, other.imu_frequency_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.imu_packet_loss_rate_, other.imu_packet_loss_rate_, epsilon)) return false;

      for(int i3 = 0; i3 < imu_rpy_.length; ++i3)
      {
                if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.imu_rpy_[i3], other.imu_rpy_[i3], epsilon)) return false;
      }

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.serial_recv_stamp_, other.serial_recv_stamp_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.serial_buffer_size_, other.serial_buffer_size_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.serial_buffer_read_, other.serial_buffer_read_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof LidarState)) return false;

      LidarState otherMyClass = (LidarState) other;

      if(this.stamp_ != otherMyClass.stamp_) return false;

      if (!us.ihmc.idl.IDLTools.equals(this.firmware_version_, otherMyClass.firmware_version_)) return false;

      if (!us.ihmc.idl.IDLTools.equals(this.software_version_, otherMyClass.software_version_)) return false;

      if (!us.ihmc.idl.IDLTools.equals(this.sdk_version_, otherMyClass.sdk_version_)) return false;

      if(this.sys_rotation_speed_ != otherMyClass.sys_rotation_speed_) return false;

      if(this.com_rotation_speed_ != otherMyClass.com_rotation_speed_) return false;

      if(this.error_state_ != otherMyClass.error_state_) return false;

      if(this.cloud_frequency_ != otherMyClass.cloud_frequency_) return false;

      if(this.cloud_packet_loss_rate_ != otherMyClass.cloud_packet_loss_rate_) return false;

      if(this.cloud_size_ != otherMyClass.cloud_size_) return false;

      if(this.cloud_scan_num_ != otherMyClass.cloud_scan_num_) return false;

      if(this.imu_frequency_ != otherMyClass.imu_frequency_) return false;

      if(this.imu_packet_loss_rate_ != otherMyClass.imu_packet_loss_rate_) return false;

      for(int i5 = 0; i5 < imu_rpy_.length; ++i5)
      {
                if(this.imu_rpy_[i5] != otherMyClass.imu_rpy_[i5]) return false;

      }
      if(this.serial_recv_stamp_ != otherMyClass.serial_recv_stamp_) return false;

      if(this.serial_buffer_size_ != otherMyClass.serial_buffer_size_) return false;

      if(this.serial_buffer_read_ != otherMyClass.serial_buffer_read_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("LidarState {");
      builder.append("stamp=");
      builder.append(this.stamp_);      builder.append(", ");
      builder.append("firmware_version=");
      builder.append(this.firmware_version_);      builder.append(", ");
      builder.append("software_version=");
      builder.append(this.software_version_);      builder.append(", ");
      builder.append("sdk_version=");
      builder.append(this.sdk_version_);      builder.append(", ");
      builder.append("sys_rotation_speed=");
      builder.append(this.sys_rotation_speed_);      builder.append(", ");
      builder.append("com_rotation_speed=");
      builder.append(this.com_rotation_speed_);      builder.append(", ");
      builder.append("error_state=");
      builder.append(this.error_state_);      builder.append(", ");
      builder.append("cloud_frequency=");
      builder.append(this.cloud_frequency_);      builder.append(", ");
      builder.append("cloud_packet_loss_rate=");
      builder.append(this.cloud_packet_loss_rate_);      builder.append(", ");
      builder.append("cloud_size=");
      builder.append(this.cloud_size_);      builder.append(", ");
      builder.append("cloud_scan_num=");
      builder.append(this.cloud_scan_num_);      builder.append(", ");
      builder.append("imu_frequency=");
      builder.append(this.imu_frequency_);      builder.append(", ");
      builder.append("imu_packet_loss_rate=");
      builder.append(this.imu_packet_loss_rate_);      builder.append(", ");
      builder.append("imu_rpy=");
      builder.append(java.util.Arrays.toString(this.imu_rpy_));      builder.append(", ");
      builder.append("serial_recv_stamp=");
      builder.append(this.serial_recv_stamp_);      builder.append(", ");
      builder.append("serial_buffer_size=");
      builder.append(this.serial_buffer_size_);      builder.append(", ");
      builder.append("serial_buffer_read=");
      builder.append(this.serial_buffer_read_);
      builder.append("}");
      return builder.toString();
   }
}
