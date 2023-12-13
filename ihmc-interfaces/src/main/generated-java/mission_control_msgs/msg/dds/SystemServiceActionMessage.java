package mission_control_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class SystemServiceActionMessage extends Packet<SystemServiceActionMessage> implements Settable<SystemServiceActionMessage>, EpsilonComparable<SystemServiceActionMessage>
{
   /**
            * The name of the systemd service
            */
   public java.lang.StringBuilder service_name_;
   /**
            * The action to perform in the systemctl command
            * Valid values: [start, stop, restart, kill]
            */
   public java.lang.StringBuilder systemd_action_;

   public SystemServiceActionMessage()
   {
      service_name_ = new java.lang.StringBuilder(255);
      systemd_action_ = new java.lang.StringBuilder(255);
   }

   public SystemServiceActionMessage(SystemServiceActionMessage other)
   {
      this();
      set(other);
   }

   public void set(SystemServiceActionMessage other)
   {
      service_name_.setLength(0);
      service_name_.append(other.service_name_);

      systemd_action_.setLength(0);
      systemd_action_.append(other.systemd_action_);

   }

   /**
            * The name of the systemd service
            */
   public void setServiceName(java.lang.String service_name)
   {
      service_name_.setLength(0);
      service_name_.append(service_name);
   }

   /**
            * The name of the systemd service
            */
   public java.lang.String getServiceNameAsString()
   {
      return getServiceName().toString();
   }
   /**
            * The name of the systemd service
            */
   public java.lang.StringBuilder getServiceName()
   {
      return service_name_;
   }

   /**
            * The action to perform in the systemctl command
            * Valid values: [start, stop, restart, kill]
            */
   public void setSystemdAction(java.lang.String systemd_action)
   {
      systemd_action_.setLength(0);
      systemd_action_.append(systemd_action);
   }

   /**
            * The action to perform in the systemctl command
            * Valid values: [start, stop, restart, kill]
            */
   public java.lang.String getSystemdActionAsString()
   {
      return getSystemdAction().toString();
   }
   /**
            * The action to perform in the systemctl command
            * Valid values: [start, stop, restart, kill]
            */
   public java.lang.StringBuilder getSystemdAction()
   {
      return systemd_action_;
   }


   public static Supplier<SystemServiceActionMessagePubSubType> getPubSubType()
   {
      return SystemServiceActionMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return SystemServiceActionMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(SystemServiceActionMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.service_name_, other.service_name_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.systemd_action_, other.systemd_action_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof SystemServiceActionMessage)) return false;

      SystemServiceActionMessage otherMyClass = (SystemServiceActionMessage) other;

      if (!us.ihmc.idl.IDLTools.equals(this.service_name_, otherMyClass.service_name_)) return false;

      if (!us.ihmc.idl.IDLTools.equals(this.systemd_action_, otherMyClass.systemd_action_)) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("SystemServiceActionMessage {");
      builder.append("service_name=");
      builder.append(this.service_name_);      builder.append(", ");
      builder.append("systemd_action=");
      builder.append(this.systemd_action_);
      builder.append("}");
      return builder.toString();
   }
}
