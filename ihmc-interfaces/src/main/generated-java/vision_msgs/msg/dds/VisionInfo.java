package vision_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * Provides meta-information about a visual pipeline.
       * 
       * This message serves a similar purpose to sensor_msgs/CameraInfo, but instead
       * of being tied to hardware, it represents information about a specific
       * computer vision pipeline. This information stays constant (or relatively
       * constant) over time, and so it is wasteful to send it with each individual
       * result. By listening to these messages, subscribers will receive
       * the context in which published vision messages are to be interpreted.
       * Each vision pipeline should publish its VisionInfo messages to its own topic,
       * in a manner similar to CameraInfo.
       */
public class VisionInfo extends Packet<VisionInfo> implements Settable<VisionInfo>, EpsilonComparable<VisionInfo>
{
   /**
            * Used for sequencing
            */
   public std_msgs.msg.dds.Header header_;
   /**
            * Name of the vision pipeline. This should be a value that is meaningful to an
            * outside user.
            */
   public java.lang.StringBuilder method_;
   /**
            * Location where the metadata database is stored. The recommended location is
            * as an XML string on the ROS parameter server, but the exact implementation
            * and information is left up to the user.
            * The database should store information attached to class ids. Each
            * class id should map to an atomic, visually recognizable element. This
            * definition is intentionally vague to allow extreme flexibility. The
            * elements could be classes in a pixel segmentation algorithm, object classes
            * in a detector, different people's faces in a face detection algorithm, etc.
            * Vision pipelines report results in terms of numeric IDs, which map into
            * this  database.
            * The information stored in this database is, again, left up to the user. The
            * database could be as simple as a map from ID to class name, or it could
            * include information such as object meshes or colors to use for
            * visualization.
            */
   public java.lang.StringBuilder database_location_;
   /**
            * Metadata database version. This counter is incremented
            * each time the pipeline begins using a new version of the database (useful
            * in the case of online training or user modifications).
            * The counter value can be monitored by listeners to ensure that the pipeline
            * and the listener are using the same metadata.
            */
   public int database_version_;

   public VisionInfo()
   {
      header_ = new std_msgs.msg.dds.Header();
      method_ = new java.lang.StringBuilder(255);
      database_location_ = new java.lang.StringBuilder(255);
   }

   public VisionInfo(VisionInfo other)
   {
      this();
      set(other);
   }

   public void set(VisionInfo other)
   {
      std_msgs.msg.dds.HeaderPubSubType.staticCopy(other.header_, header_);
      method_.setLength(0);
      method_.append(other.method_);

      database_location_.setLength(0);
      database_location_.append(other.database_location_);

      database_version_ = other.database_version_;

   }


   /**
            * Used for sequencing
            */
   public std_msgs.msg.dds.Header getHeader()
   {
      return header_;
   }

   /**
            * Name of the vision pipeline. This should be a value that is meaningful to an
            * outside user.
            */
   public void setMethod(java.lang.String method)
   {
      method_.setLength(0);
      method_.append(method);
   }

   /**
            * Name of the vision pipeline. This should be a value that is meaningful to an
            * outside user.
            */
   public java.lang.String getMethodAsString()
   {
      return getMethod().toString();
   }
   /**
            * Name of the vision pipeline. This should be a value that is meaningful to an
            * outside user.
            */
   public java.lang.StringBuilder getMethod()
   {
      return method_;
   }

   /**
            * Location where the metadata database is stored. The recommended location is
            * as an XML string on the ROS parameter server, but the exact implementation
            * and information is left up to the user.
            * The database should store information attached to class ids. Each
            * class id should map to an atomic, visually recognizable element. This
            * definition is intentionally vague to allow extreme flexibility. The
            * elements could be classes in a pixel segmentation algorithm, object classes
            * in a detector, different people's faces in a face detection algorithm, etc.
            * Vision pipelines report results in terms of numeric IDs, which map into
            * this  database.
            * The information stored in this database is, again, left up to the user. The
            * database could be as simple as a map from ID to class name, or it could
            * include information such as object meshes or colors to use for
            * visualization.
            */
   public void setDatabaseLocation(java.lang.String database_location)
   {
      database_location_.setLength(0);
      database_location_.append(database_location);
   }

   /**
            * Location where the metadata database is stored. The recommended location is
            * as an XML string on the ROS parameter server, but the exact implementation
            * and information is left up to the user.
            * The database should store information attached to class ids. Each
            * class id should map to an atomic, visually recognizable element. This
            * definition is intentionally vague to allow extreme flexibility. The
            * elements could be classes in a pixel segmentation algorithm, object classes
            * in a detector, different people's faces in a face detection algorithm, etc.
            * Vision pipelines report results in terms of numeric IDs, which map into
            * this  database.
            * The information stored in this database is, again, left up to the user. The
            * database could be as simple as a map from ID to class name, or it could
            * include information such as object meshes or colors to use for
            * visualization.
            */
   public java.lang.String getDatabaseLocationAsString()
   {
      return getDatabaseLocation().toString();
   }
   /**
            * Location where the metadata database is stored. The recommended location is
            * as an XML string on the ROS parameter server, but the exact implementation
            * and information is left up to the user.
            * The database should store information attached to class ids. Each
            * class id should map to an atomic, visually recognizable element. This
            * definition is intentionally vague to allow extreme flexibility. The
            * elements could be classes in a pixel segmentation algorithm, object classes
            * in a detector, different people's faces in a face detection algorithm, etc.
            * Vision pipelines report results in terms of numeric IDs, which map into
            * this  database.
            * The information stored in this database is, again, left up to the user. The
            * database could be as simple as a map from ID to class name, or it could
            * include information such as object meshes or colors to use for
            * visualization.
            */
   public java.lang.StringBuilder getDatabaseLocation()
   {
      return database_location_;
   }

   /**
            * Metadata database version. This counter is incremented
            * each time the pipeline begins using a new version of the database (useful
            * in the case of online training or user modifications).
            * The counter value can be monitored by listeners to ensure that the pipeline
            * and the listener are using the same metadata.
            */
   public void setDatabaseVersion(int database_version)
   {
      database_version_ = database_version;
   }
   /**
            * Metadata database version. This counter is incremented
            * each time the pipeline begins using a new version of the database (useful
            * in the case of online training or user modifications).
            * The counter value can be monitored by listeners to ensure that the pipeline
            * and the listener are using the same metadata.
            */
   public int getDatabaseVersion()
   {
      return database_version_;
   }


   public static Supplier<VisionInfoPubSubType> getPubSubType()
   {
      return VisionInfoPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return VisionInfoPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(VisionInfo other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.header_.epsilonEquals(other.header_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.method_, other.method_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.database_location_, other.database_location_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.database_version_, other.database_version_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof VisionInfo)) return false;

      VisionInfo otherMyClass = (VisionInfo) other;

      if (!this.header_.equals(otherMyClass.header_)) return false;
      if (!us.ihmc.idl.IDLTools.equals(this.method_, otherMyClass.method_)) return false;

      if (!us.ihmc.idl.IDLTools.equals(this.database_location_, otherMyClass.database_location_)) return false;

      if(this.database_version_ != otherMyClass.database_version_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("VisionInfo {");
      builder.append("header=");
      builder.append(this.header_);      builder.append(", ");
      builder.append("method=");
      builder.append(this.method_);      builder.append(", ");
      builder.append("database_location=");
      builder.append(this.database_location_);      builder.append(", ");
      builder.append("database_version=");
      builder.append(this.database_version_);
      builder.append("}");
      return builder.toString();
   }
}
