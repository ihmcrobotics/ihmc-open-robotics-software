package us.ihmc.robotDataLogger;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;

public class LogProperties extends Packet<LogProperties> implements Settable<LogProperties>, EpsilonComparable<LogProperties>
{
   public java.lang.StringBuilder version_;
   // Version of the properties
   public java.lang.StringBuilder name_;
   // Name of this log
   public us.ihmc.robotDataLogger.Variables variables_;
   public us.ihmc.robotDataLogger.Model model_;
   public java.lang.StringBuilder timestamp_;
   // When was this log taken
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.robotDataLogger.Camera>  cameras_;
   // Backward compatibility options
   public us.ihmc.robotDataLogger.Video video_;

   public LogProperties()
   {
      version_ = new java.lang.StringBuilder(255);
      name_ = new java.lang.StringBuilder(255);
      variables_ = new us.ihmc.robotDataLogger.Variables();
      model_ = new us.ihmc.robotDataLogger.Model();
      timestamp_ = new java.lang.StringBuilder(255);
      cameras_ = new us.ihmc.idl.IDLSequence.Object<us.ihmc.robotDataLogger.Camera> (255, us.ihmc.robotDataLogger.Camera.class, new us.ihmc.robotDataLogger.CameraPubSubType());
      video_ = new us.ihmc.robotDataLogger.Video();

   }

   public LogProperties(LogProperties other)
   {
      this();
      set(other);
   }

   public void set(LogProperties other)
   {
      version_.setLength(0);
      version_.append(other.version_);

      name_.setLength(0);
      name_.append(other.name_);

      us.ihmc.robotDataLogger.VariablesPubSubType.staticCopy(other.variables_, variables_);
      us.ihmc.robotDataLogger.ModelPubSubType.staticCopy(other.model_, model_);
      timestamp_.setLength(0);
      timestamp_.append(other.timestamp_);

      cameras_.set(other.cameras_);
      us.ihmc.robotDataLogger.VideoPubSubType.staticCopy(other.video_, video_);
   }

   public void setVersion(java.lang.String version)
   {
      version_.setLength(0);
      version_.append(version);
   }

   public java.lang.String getVersionAsString()
   {
      return getVersion().toString();
   }
   public java.lang.StringBuilder getVersion()
   {
      return version_;
   }

   // Version of the properties
   public void setName(java.lang.String name)
   {
      name_.setLength(0);
      name_.append(name);
   }

   // Version of the properties
   public java.lang.String getNameAsString()
   {
      return getName().toString();
   }
   // Version of the properties
   public java.lang.StringBuilder getName()
   {
      return name_;
   }


   // Name of this log
   public us.ihmc.robotDataLogger.Variables getVariables()
   {
      return variables_;
   }


   public us.ihmc.robotDataLogger.Model getModel()
   {
      return model_;
   }

   public void setTimestamp(java.lang.String timestamp)
   {
      timestamp_.setLength(0);
      timestamp_.append(timestamp);
   }

   public java.lang.String getTimestampAsString()
   {
      return getTimestamp().toString();
   }
   public java.lang.StringBuilder getTimestamp()
   {
      return timestamp_;
   }


   // When was this log taken
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.robotDataLogger.Camera>  getCameras()
   {
      return cameras_;
   }


   // Backward compatibility options
   public us.ihmc.robotDataLogger.Video getVideo()
   {
      return video_;
   }


   @Override
   public boolean epsilonEquals(LogProperties other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.version_, other.version_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.name_, other.name_, epsilon)) return false;

      if (!this.variables_.epsilonEquals(other.variables_, epsilon)) return false;
      if (!this.model_.epsilonEquals(other.model_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.timestamp_, other.timestamp_, epsilon)) return false;

      if (this.cameras_.size() != other.cameras_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.cameras_.size(); i++)
         {  if (!this.cameras_.get(i).epsilonEquals(other.cameras_.get(i), epsilon)) return false; }
      }

      if (!this.video_.epsilonEquals(other.video_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof LogProperties)) return false;

      LogProperties otherMyClass = (LogProperties) other;

      if (!us.ihmc.idl.IDLTools.equals(this.version_, otherMyClass.version_)) return false;

      if (!us.ihmc.idl.IDLTools.equals(this.name_, otherMyClass.name_)) return false;

      if (!this.variables_.equals(otherMyClass.variables_)) return false;
      if (!this.model_.equals(otherMyClass.model_)) return false;
      if (!us.ihmc.idl.IDLTools.equals(this.timestamp_, otherMyClass.timestamp_)) return false;

      if (!this.cameras_.equals(otherMyClass.cameras_)) return false;
      if (!this.video_.equals(otherMyClass.video_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("LogProperties {");
      builder.append("version=");
      builder.append(this.version_);      builder.append(", ");
      builder.append("name=");
      builder.append(this.name_);      builder.append(", ");
      builder.append("variables=");
      builder.append(this.variables_);      builder.append(", ");
      builder.append("model=");
      builder.append(this.model_);      builder.append(", ");
      builder.append("timestamp=");
      builder.append(this.timestamp_);      builder.append(", ");
      builder.append("cameras=");
      builder.append(this.cameras_);      builder.append(", ");
      builder.append("video=");
      builder.append(this.video_);
      builder.append("}");
      return builder.toString();
   }
}
