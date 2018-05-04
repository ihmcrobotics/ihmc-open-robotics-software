package us.ihmc.robotDataLogger;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class LoggerConfiguration extends Packet<LoggerConfiguration> implements Settable<LoggerConfiguration>, EpsilonComparable<LoggerConfiguration>
{
   public java.lang.StringBuilder camerasToCapture_;
   public boolean publicBroadcast_;

   public LoggerConfiguration()
   {
      camerasToCapture_ = new java.lang.StringBuilder(255);
   }

   public LoggerConfiguration(LoggerConfiguration other)
   {
      this();
      set(other);
   }

   public void set(LoggerConfiguration other)
   {
      camerasToCapture_.setLength(0);
      camerasToCapture_.append(other.camerasToCapture_);

      publicBroadcast_ = other.publicBroadcast_;

   }

   public void setCamerasToCapture(java.lang.String camerasToCapture)
   {
      camerasToCapture_.setLength(0);
      camerasToCapture_.append(camerasToCapture);
   }

   public java.lang.String getCamerasToCaptureAsString()
   {
      return getCamerasToCapture().toString();
   }
   public java.lang.StringBuilder getCamerasToCapture()
   {
      return camerasToCapture_;
   }

   public void setPublicBroadcast(boolean publicBroadcast)
   {
      publicBroadcast_ = publicBroadcast;
   }
   public boolean getPublicBroadcast()
   {
      return publicBroadcast_;
   }


   public static Supplier<LoggerConfigurationPubSubType> getPubSubType()
   {
      return LoggerConfigurationPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return LoggerConfigurationPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(LoggerConfiguration other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.camerasToCapture_, other.camerasToCapture_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.publicBroadcast_, other.publicBroadcast_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof LoggerConfiguration)) return false;

      LoggerConfiguration otherMyClass = (LoggerConfiguration) other;

      if (!us.ihmc.idl.IDLTools.equals(this.camerasToCapture_, otherMyClass.camerasToCapture_)) return false;

      if(this.publicBroadcast_ != otherMyClass.publicBroadcast_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("LoggerConfiguration {");
      builder.append("camerasToCapture=");
      builder.append(this.camerasToCapture_);      builder.append(", ");
      builder.append("publicBroadcast=");
      builder.append(this.publicBroadcast_);
      builder.append("}");
      return builder.toString();
   }
}
