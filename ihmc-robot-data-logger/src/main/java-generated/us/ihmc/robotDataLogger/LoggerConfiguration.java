package us.ihmc.robotDataLogger;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Settable;

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
      set(other);
   }

   public void set(LoggerConfiguration other)
   {
      camerasToCapture_.setLength(0);
      camerasToCapture_.append(other.camerasToCapture_);

      publicBroadcast_ = other.publicBroadcast_;
   }

   public java.lang.String getCamerasToCaptureAsString()
   {
      return getCamerasToCapture().toString();
   }

   public java.lang.StringBuilder getCamerasToCapture()
   {
      return camerasToCapture_;
   }

   public void setCamerasToCapture(java.lang.String camerasToCapture)
   {
      camerasToCapture_.setLength(0);
      camerasToCapture_.append(camerasToCapture);
   }

   public boolean getPublicBroadcast()
   {
      return publicBroadcast_;
   }

   public void setPublicBroadcast(boolean publicBroadcast)
   {
      publicBroadcast_ = publicBroadcast;
   }

   @Override
   public boolean epsilonEquals(LoggerConfiguration other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.camerasToCapture_, other.camerasToCapture_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.publicBroadcast_, other.publicBroadcast_, epsilon))
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
      if (!(other instanceof LoggerConfiguration))
         return false;

      LoggerConfiguration otherMyClass = (LoggerConfiguration) other;

      if (!us.ihmc.idl.IDLTools.equals(this.camerasToCapture_, otherMyClass.camerasToCapture_))
         return false;

      if (this.publicBroadcast_ != otherMyClass.publicBroadcast_)
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("LoggerConfiguration {");
      builder.append("camerasToCapture=");
      builder.append(this.camerasToCapture_);

      builder.append(", ");
      builder.append("publicBroadcast=");
      builder.append(this.publicBroadcast_);

      builder.append("}");
      return builder.toString();
   }
}