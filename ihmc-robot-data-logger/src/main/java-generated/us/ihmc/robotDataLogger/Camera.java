package us.ihmc.robotDataLogger;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;

public class Camera extends Packet<Camera> implements Settable<Camera>, EpsilonComparable<Camera>
{
   // Camera definition
   public java.lang.StringBuilder name_;
   // Human readable camera name
   public boolean interlaced_;
   // Is the input interlaced
   public java.lang.StringBuilder videoFile_;
   // Video file
   public java.lang.StringBuilder timestampFile_;

   public Camera()
   {
      name_ = new java.lang.StringBuilder(255);
      videoFile_ = new java.lang.StringBuilder(255);
      timestampFile_ = new java.lang.StringBuilder(255);
   }

   public Camera(Camera other)
   {
      this();
      set(other);
   }

   public void set(Camera other)
   {
      name_.setLength(0);
      name_.append(other.name_);

      interlaced_ = other.interlaced_;

      videoFile_.setLength(0);
      videoFile_.append(other.videoFile_);

      timestampFile_.setLength(0);
      timestampFile_.append(other.timestampFile_);

   }

   // Camera definition
   public void setName(java.lang.String name)
   {
      name_.setLength(0);
      name_.append(name);
   }

   // Camera definition
   public java.lang.String getNameAsString()
   {
      return getName().toString();
   }
   // Camera definition
   public java.lang.StringBuilder getName()
   {
      return name_;
   }

   // Human readable camera name
   public void setInterlaced(boolean interlaced)
   {
      interlaced_ = interlaced;
   }
   // Human readable camera name
   public boolean getInterlaced()
   {
      return interlaced_;
   }

   // Is the input interlaced
   public void setVideoFile(java.lang.String videoFile)
   {
      videoFile_.setLength(0);
      videoFile_.append(videoFile);
   }

   // Is the input interlaced
   public java.lang.String getVideoFileAsString()
   {
      return getVideoFile().toString();
   }
   // Is the input interlaced
   public java.lang.StringBuilder getVideoFile()
   {
      return videoFile_;
   }

   // Video file
   public void setTimestampFile(java.lang.String timestampFile)
   {
      timestampFile_.setLength(0);
      timestampFile_.append(timestampFile);
   }

   // Video file
   public java.lang.String getTimestampFileAsString()
   {
      return getTimestampFile().toString();
   }
   // Video file
   public java.lang.StringBuilder getTimestampFile()
   {
      return timestampFile_;
   }


   @Override
   public boolean epsilonEquals(Camera other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.name_, other.name_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.interlaced_, other.interlaced_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.videoFile_, other.videoFile_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.timestampFile_, other.timestampFile_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof Camera)) return false;

      Camera otherMyClass = (Camera) other;

      if (!us.ihmc.idl.IDLTools.equals(this.name_, otherMyClass.name_)) return false;

      if(this.interlaced_ != otherMyClass.interlaced_) return false;

      if (!us.ihmc.idl.IDLTools.equals(this.videoFile_, otherMyClass.videoFile_)) return false;

      if (!us.ihmc.idl.IDLTools.equals(this.timestampFile_, otherMyClass.timestampFile_)) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("Camera {");
      builder.append("name=");
      builder.append(this.name_);      builder.append(", ");
      builder.append("interlaced=");
      builder.append(this.interlaced_);      builder.append(", ");
      builder.append("videoFile=");
      builder.append(this.videoFile_);      builder.append(", ");
      builder.append("timestampFile=");
      builder.append(this.timestampFile_);
      builder.append("}");
      return builder.toString();
   }
}
