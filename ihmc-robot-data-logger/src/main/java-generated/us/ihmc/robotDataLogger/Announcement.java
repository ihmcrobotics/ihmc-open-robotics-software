package us.ihmc.robotDataLogger;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class Announcement extends Packet<Announcement> implements Settable<Announcement>, EpsilonComparable<Announcement>
{
   public java.lang.StringBuilder identifier_;
   public java.lang.StringBuilder name_;
   public java.lang.StringBuilder hostName_;
   public java.lang.StringBuilder reconnectKey_;
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.robotDataLogger.CameraAnnouncement>  cameras_;
   public us.ihmc.robotDataLogger.ModelFileDescription modelFileDescription_;
   public boolean log_;

   public Announcement()
   {
      identifier_ = new java.lang.StringBuilder(255);
      name_ = new java.lang.StringBuilder(255);
      hostName_ = new java.lang.StringBuilder(255);
      reconnectKey_ = new java.lang.StringBuilder(255);
      cameras_ = new us.ihmc.idl.IDLSequence.Object<us.ihmc.robotDataLogger.CameraAnnouncement> (127, us.ihmc.robotDataLogger.CameraAnnouncement.class, new us.ihmc.robotDataLogger.CameraAnnouncementPubSubType());
      modelFileDescription_ = new us.ihmc.robotDataLogger.ModelFileDescription();

   }

   public Announcement(Announcement other)
   {
      this();
      set(other);
   }

   public void set(Announcement other)
   {
      identifier_.setLength(0);
      identifier_.append(other.identifier_);

      name_.setLength(0);
      name_.append(other.name_);

      hostName_.setLength(0);
      hostName_.append(other.hostName_);

      reconnectKey_.setLength(0);
      reconnectKey_.append(other.reconnectKey_);

      cameras_.set(other.cameras_);
      us.ihmc.robotDataLogger.ModelFileDescriptionPubSubType.staticCopy(other.modelFileDescription_, modelFileDescription_);
      log_ = other.log_;

   }

   public void setIdentifier(java.lang.String identifier)
   {
      identifier_.setLength(0);
      identifier_.append(identifier);
   }

   public java.lang.String getIdentifierAsString()
   {
      return getIdentifier().toString();
   }
   public java.lang.StringBuilder getIdentifier()
   {
      return identifier_;
   }

   public void setName(java.lang.String name)
   {
      name_.setLength(0);
      name_.append(name);
   }

   public java.lang.String getNameAsString()
   {
      return getName().toString();
   }
   public java.lang.StringBuilder getName()
   {
      return name_;
   }

   public void setHostName(java.lang.String hostName)
   {
      hostName_.setLength(0);
      hostName_.append(hostName);
   }

   public java.lang.String getHostNameAsString()
   {
      return getHostName().toString();
   }
   public java.lang.StringBuilder getHostName()
   {
      return hostName_;
   }

   public void setReconnectKey(java.lang.String reconnectKey)
   {
      reconnectKey_.setLength(0);
      reconnectKey_.append(reconnectKey);
   }

   public java.lang.String getReconnectKeyAsString()
   {
      return getReconnectKey().toString();
   }
   public java.lang.StringBuilder getReconnectKey()
   {
      return reconnectKey_;
   }


   public us.ihmc.idl.IDLSequence.Object<us.ihmc.robotDataLogger.CameraAnnouncement>  getCameras()
   {
      return cameras_;
   }


   public us.ihmc.robotDataLogger.ModelFileDescription getModelFileDescription()
   {
      return modelFileDescription_;
   }

   public void setLog(boolean log)
   {
      log_ = log;
   }
   public boolean getLog()
   {
      return log_;
   }


   public static Supplier<AnnouncementPubSubType> getPubSubType()
   {
      return AnnouncementPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return AnnouncementPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(Announcement other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.identifier_, other.identifier_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.name_, other.name_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.hostName_, other.hostName_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.reconnectKey_, other.reconnectKey_, epsilon)) return false;

      if (this.cameras_.size() != other.cameras_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.cameras_.size(); i++)
         {  if (!this.cameras_.get(i).epsilonEquals(other.cameras_.get(i), epsilon)) return false; }
      }

      if (!this.modelFileDescription_.epsilonEquals(other.modelFileDescription_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.log_, other.log_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof Announcement)) return false;

      Announcement otherMyClass = (Announcement) other;

      if (!us.ihmc.idl.IDLTools.equals(this.identifier_, otherMyClass.identifier_)) return false;

      if (!us.ihmc.idl.IDLTools.equals(this.name_, otherMyClass.name_)) return false;

      if (!us.ihmc.idl.IDLTools.equals(this.hostName_, otherMyClass.hostName_)) return false;

      if (!us.ihmc.idl.IDLTools.equals(this.reconnectKey_, otherMyClass.reconnectKey_)) return false;

      if (!this.cameras_.equals(otherMyClass.cameras_)) return false;
      if (!this.modelFileDescription_.equals(otherMyClass.modelFileDescription_)) return false;
      if(this.log_ != otherMyClass.log_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("Announcement {");
      builder.append("identifier=");
      builder.append(this.identifier_);      builder.append(", ");
      builder.append("name=");
      builder.append(this.name_);      builder.append(", ");
      builder.append("hostName=");
      builder.append(this.hostName_);      builder.append(", ");
      builder.append("reconnectKey=");
      builder.append(this.reconnectKey_);      builder.append(", ");
      builder.append("cameras=");
      builder.append(this.cameras_);      builder.append(", ");
      builder.append("modelFileDescription=");
      builder.append(this.modelFileDescription_);      builder.append(", ");
      builder.append("log=");
      builder.append(this.log_);
      builder.append("}");
      return builder.toString();
   }
}
