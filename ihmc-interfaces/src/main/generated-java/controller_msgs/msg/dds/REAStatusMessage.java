package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC robot environment awareness module (REA).
       * It is destined to gather complementary information related to the current state REA is in.
       */
public class REAStatusMessage extends Packet<REAStatusMessage> implements Settable<REAStatusMessage>, EpsilonComparable<REAStatusMessage>
{

   /**
            * Informs if REA is currently collecting sensor data and estimating planar regions.
            * Note that if both, is_using_lidar and is_using_stereo_vision are false, REA is running but not collecting data.
            */
   public boolean is_running_;

   /**
            * Informs whether REA is collecting LIDAR data or not.
            */
   public boolean is_using_lidar_;

   /**
            * Informs whether REA is collecting pointcloud data from a stereo-camera.
            */
   public boolean is_using_stereo_vision_;

   /**
            * Informs whether REA's internal data has just been cleared.
            */
   public boolean has_cleared_;

   /**
            * Provides the current set of filter parameters used in REA.
            */
   public controller_msgs.msg.dds.REASensorDataFilterParametersMessage current_sensor_filter_parameters_;

   public REAStatusMessage()
   {





      current_sensor_filter_parameters_ = new controller_msgs.msg.dds.REASensorDataFilterParametersMessage();

   }

   public REAStatusMessage(REAStatusMessage other)
   {
      this();
      set(other);
   }

   public void set(REAStatusMessage other)
   {

      is_running_ = other.is_running_;


      is_using_lidar_ = other.is_using_lidar_;


      is_using_stereo_vision_ = other.is_using_stereo_vision_;


      has_cleared_ = other.has_cleared_;


      controller_msgs.msg.dds.REASensorDataFilterParametersMessagePubSubType.staticCopy(other.current_sensor_filter_parameters_, current_sensor_filter_parameters_);
   }


   /**
            * Informs if REA is currently collecting sensor data and estimating planar regions.
            * Note that if both, is_using_lidar and is_using_stereo_vision are false, REA is running but not collecting data.
            */
   public void setIsRunning(boolean is_running)
   {
      is_running_ = is_running;
   }
   /**
            * Informs if REA is currently collecting sensor data and estimating planar regions.
            * Note that if both, is_using_lidar and is_using_stereo_vision are false, REA is running but not collecting data.
            */
   public boolean getIsRunning()
   {
      return is_running_;
   }


   /**
            * Informs whether REA is collecting LIDAR data or not.
            */
   public void setIsUsingLidar(boolean is_using_lidar)
   {
      is_using_lidar_ = is_using_lidar;
   }
   /**
            * Informs whether REA is collecting LIDAR data or not.
            */
   public boolean getIsUsingLidar()
   {
      return is_using_lidar_;
   }


   /**
            * Informs whether REA is collecting pointcloud data from a stereo-camera.
            */
   public void setIsUsingStereoVision(boolean is_using_stereo_vision)
   {
      is_using_stereo_vision_ = is_using_stereo_vision;
   }
   /**
            * Informs whether REA is collecting pointcloud data from a stereo-camera.
            */
   public boolean getIsUsingStereoVision()
   {
      return is_using_stereo_vision_;
   }


   /**
            * Informs whether REA's internal data has just been cleared.
            */
   public void setHasCleared(boolean has_cleared)
   {
      has_cleared_ = has_cleared;
   }
   /**
            * Informs whether REA's internal data has just been cleared.
            */
   public boolean getHasCleared()
   {
      return has_cleared_;
   }



   /**
            * Provides the current set of filter parameters used in REA.
            */
   public controller_msgs.msg.dds.REASensorDataFilterParametersMessage getCurrentSensorFilterParameters()
   {
      return current_sensor_filter_parameters_;
   }


   public static Supplier<REAStatusMessagePubSubType> getPubSubType()
   {
      return REAStatusMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return REAStatusMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(REAStatusMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.is_running_, other.is_running_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.is_using_lidar_, other.is_using_lidar_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.is_using_stereo_vision_, other.is_using_stereo_vision_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.has_cleared_, other.has_cleared_, epsilon)) return false;


      if (!this.current_sensor_filter_parameters_.epsilonEquals(other.current_sensor_filter_parameters_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof REAStatusMessage)) return false;

      REAStatusMessage otherMyClass = (REAStatusMessage) other;


      if(this.is_running_ != otherMyClass.is_running_) return false;


      if(this.is_using_lidar_ != otherMyClass.is_using_lidar_) return false;


      if(this.is_using_stereo_vision_ != otherMyClass.is_using_stereo_vision_) return false;


      if(this.has_cleared_ != otherMyClass.has_cleared_) return false;


      if (!this.current_sensor_filter_parameters_.equals(otherMyClass.current_sensor_filter_parameters_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("REAStatusMessage {");

      builder.append("is_running=");
      builder.append(this.is_running_);      builder.append(", ");

      builder.append("is_using_lidar=");
      builder.append(this.is_using_lidar_);      builder.append(", ");

      builder.append("is_using_stereo_vision=");
      builder.append(this.is_using_stereo_vision_);      builder.append(", ");

      builder.append("has_cleared=");
      builder.append(this.has_cleared_);      builder.append(", ");

      builder.append("current_sensor_filter_parameters=");
      builder.append(this.current_sensor_filter_parameters_);
      builder.append("}");
      return builder.toString();
   }
}
