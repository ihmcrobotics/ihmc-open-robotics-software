package mission_control_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class SystemAvailableMessage extends Packet<SystemAvailableMessage> implements Settable<SystemAvailableMessage>, EpsilonComparable<SystemAvailableMessage>
{
   /**
            * The hostname of the system
            * Equivalent of running `hostname` at the shell
            */
   public java.lang.StringBuilder hostname_;
   /**
            * A random ID for the topic name for messages from this machine
            * We do not use the hostname because we don't want to assume the hostname is normalized
            * to the format of topic names (i.e. hostnames might contain spaces, end with numbers, etc)
            * Generated in ihmc-high-level-behaviors/src/mission-control/java/us/ihmc/missionControl/MissionControlDaemon.java
            */
   public java.lang.StringBuilder instance_id_;

   public SystemAvailableMessage()
   {
      hostname_ = new java.lang.StringBuilder(255);
      instance_id_ = new java.lang.StringBuilder(255);
   }

   public SystemAvailableMessage(SystemAvailableMessage other)
   {
      this();
      set(other);
   }

   public void set(SystemAvailableMessage other)
   {
      hostname_.setLength(0);
      hostname_.append(other.hostname_);

      instance_id_.setLength(0);
      instance_id_.append(other.instance_id_);

   }

   /**
            * The hostname of the system
            * Equivalent of running `hostname` at the shell
            */
   public void setHostname(java.lang.String hostname)
   {
      hostname_.setLength(0);
      hostname_.append(hostname);
   }

   /**
            * The hostname of the system
            * Equivalent of running `hostname` at the shell
            */
   public java.lang.String getHostnameAsString()
   {
      return getHostname().toString();
   }
   /**
            * The hostname of the system
            * Equivalent of running `hostname` at the shell
            */
   public java.lang.StringBuilder getHostname()
   {
      return hostname_;
   }

   /**
            * A random ID for the topic name for messages from this machine
            * We do not use the hostname because we don't want to assume the hostname is normalized
            * to the format of topic names (i.e. hostnames might contain spaces, end with numbers, etc)
            * Generated in ihmc-high-level-behaviors/src/mission-control/java/us/ihmc/missionControl/MissionControlDaemon.java
            */
   public void setInstanceId(java.lang.String instance_id)
   {
      instance_id_.setLength(0);
      instance_id_.append(instance_id);
   }

   /**
            * A random ID for the topic name for messages from this machine
            * We do not use the hostname because we don't want to assume the hostname is normalized
            * to the format of topic names (i.e. hostnames might contain spaces, end with numbers, etc)
            * Generated in ihmc-high-level-behaviors/src/mission-control/java/us/ihmc/missionControl/MissionControlDaemon.java
            */
   public java.lang.String getInstanceIdAsString()
   {
      return getInstanceId().toString();
   }
   /**
            * A random ID for the topic name for messages from this machine
            * We do not use the hostname because we don't want to assume the hostname is normalized
            * to the format of topic names (i.e. hostnames might contain spaces, end with numbers, etc)
            * Generated in ihmc-high-level-behaviors/src/mission-control/java/us/ihmc/missionControl/MissionControlDaemon.java
            */
   public java.lang.StringBuilder getInstanceId()
   {
      return instance_id_;
   }


   public static Supplier<SystemAvailableMessagePubSubType> getPubSubType()
   {
      return SystemAvailableMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return SystemAvailableMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(SystemAvailableMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.hostname_, other.hostname_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.instance_id_, other.instance_id_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof SystemAvailableMessage)) return false;

      SystemAvailableMessage otherMyClass = (SystemAvailableMessage) other;

      if (!us.ihmc.idl.IDLTools.equals(this.hostname_, otherMyClass.hostname_)) return false;

      if (!us.ihmc.idl.IDLTools.equals(this.instance_id_, otherMyClass.instance_id_)) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("SystemAvailableMessage {");
      builder.append("hostname=");
      builder.append(this.hostname_);      builder.append(", ");
      builder.append("instance_id=");
      builder.append(this.instance_id_);
      builder.append("}");
      return builder.toString();
   }
}
