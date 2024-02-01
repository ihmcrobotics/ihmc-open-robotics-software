package mission_control_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class SystemResourceUsageMessage extends Packet<SystemResourceUsageMessage> implements Settable<SystemResourceUsageMessage>, EpsilonComparable<SystemResourceUsageMessage>
{
   public java.lang.StringBuilder uptime_;
   /**
            * ======== System memory statistics ========
            */
   public float memory_used_;
   public float memory_total_;
   /**
            * ======== CPU statistics ========
            */
   public int cpu_count_;
   /**
            * [n] : usage for nth CPU
            */
   public us.ihmc.idl.IDLSequence.Float  cpu_usages_;
   /**
            * [n] : temperature for nth CPU
            */
   public us.ihmc.idl.IDLSequence.Float  cpu_temps_;
   /**
            * ======== Network statistics ========
            */
   public int iface_count_;
   /**
            * [n] : name for nth iface
            */
   public us.ihmc.idl.IDLSequence.StringBuilderHolder  iface_names_;
   /**
            * [n] : current rx (kbps) for nth iface
            */
   public us.ihmc.idl.IDLSequence.Float  iface_rx_kbps_;
   /**
            * [n] : current tx (kbps) for nth iface
            */
   public us.ihmc.idl.IDLSequence.Float  iface_tx_kbps_;
   /**
            * ======== NVIDIA GPU statistics ========
            */
   public int nvidia_gpu_count_;
   /**
            * [n] : model for nth GPU
            */
   public us.ihmc.idl.IDLSequence.StringBuilderHolder  nvidia_gpu_models_;
   /**
            * [n] : memory used for nth GPU
            */
   public us.ihmc.idl.IDLSequence.Float  nvidia_gpu_memory_used_;
   /**
            * [n] : total memory for nth GPU
            */
   public us.ihmc.idl.IDLSequence.Float  nvidia_gpu_memory_total_;
   /**
            * [n] : utilization for nth GPU
            */
   public us.ihmc.idl.IDLSequence.Float  nvidia_gpu_utilization_;
   /**
            * [n] : die temperature for nth GPU
            */
   public us.ihmc.idl.IDLSequence.Float  nvidia_gpu_temps_;

   public SystemResourceUsageMessage()
   {
      uptime_ = new java.lang.StringBuilder(255);
      cpu_usages_ = new us.ihmc.idl.IDLSequence.Float (100, "type_5");

      cpu_temps_ = new us.ihmc.idl.IDLSequence.Float (100, "type_5");

      iface_names_ = new us.ihmc.idl.IDLSequence.StringBuilderHolder (100, "type_d");
      iface_rx_kbps_ = new us.ihmc.idl.IDLSequence.Float (100, "type_5");

      iface_tx_kbps_ = new us.ihmc.idl.IDLSequence.Float (100, "type_5");

      nvidia_gpu_models_ = new us.ihmc.idl.IDLSequence.StringBuilderHolder (100, "type_d");
      nvidia_gpu_memory_used_ = new us.ihmc.idl.IDLSequence.Float (100, "type_5");

      nvidia_gpu_memory_total_ = new us.ihmc.idl.IDLSequence.Float (100, "type_5");

      nvidia_gpu_utilization_ = new us.ihmc.idl.IDLSequence.Float (100, "type_5");

      nvidia_gpu_temps_ = new us.ihmc.idl.IDLSequence.Float (100, "type_5");

   }

   public SystemResourceUsageMessage(SystemResourceUsageMessage other)
   {
      this();
      set(other);
   }

   public void set(SystemResourceUsageMessage other)
   {
      uptime_.setLength(0);
      uptime_.append(other.uptime_);

      memory_used_ = other.memory_used_;

      memory_total_ = other.memory_total_;

      cpu_count_ = other.cpu_count_;

      cpu_usages_.set(other.cpu_usages_);
      cpu_temps_.set(other.cpu_temps_);
      iface_count_ = other.iface_count_;

      iface_names_.set(other.iface_names_);
      iface_rx_kbps_.set(other.iface_rx_kbps_);
      iface_tx_kbps_.set(other.iface_tx_kbps_);
      nvidia_gpu_count_ = other.nvidia_gpu_count_;

      nvidia_gpu_models_.set(other.nvidia_gpu_models_);
      nvidia_gpu_memory_used_.set(other.nvidia_gpu_memory_used_);
      nvidia_gpu_memory_total_.set(other.nvidia_gpu_memory_total_);
      nvidia_gpu_utilization_.set(other.nvidia_gpu_utilization_);
      nvidia_gpu_temps_.set(other.nvidia_gpu_temps_);
   }

   public void setUptime(java.lang.String uptime)
   {
      uptime_.setLength(0);
      uptime_.append(uptime);
   }

   public java.lang.String getUptimeAsString()
   {
      return getUptime().toString();
   }
   public java.lang.StringBuilder getUptime()
   {
      return uptime_;
   }

   /**
            * ======== System memory statistics ========
            */
   public void setMemoryUsed(float memory_used)
   {
      memory_used_ = memory_used;
   }
   /**
            * ======== System memory statistics ========
            */
   public float getMemoryUsed()
   {
      return memory_used_;
   }

   public void setMemoryTotal(float memory_total)
   {
      memory_total_ = memory_total;
   }
   public float getMemoryTotal()
   {
      return memory_total_;
   }

   /**
            * ======== CPU statistics ========
            */
   public void setCpuCount(int cpu_count)
   {
      cpu_count_ = cpu_count;
   }
   /**
            * ======== CPU statistics ========
            */
   public int getCpuCount()
   {
      return cpu_count_;
   }


   /**
            * [n] : usage for nth CPU
            */
   public us.ihmc.idl.IDLSequence.Float  getCpuUsages()
   {
      return cpu_usages_;
   }


   /**
            * [n] : temperature for nth CPU
            */
   public us.ihmc.idl.IDLSequence.Float  getCpuTemps()
   {
      return cpu_temps_;
   }

   /**
            * ======== Network statistics ========
            */
   public void setIfaceCount(int iface_count)
   {
      iface_count_ = iface_count;
   }
   /**
            * ======== Network statistics ========
            */
   public int getIfaceCount()
   {
      return iface_count_;
   }


   /**
            * [n] : name for nth iface
            */
   public us.ihmc.idl.IDLSequence.StringBuilderHolder  getIfaceNames()
   {
      return iface_names_;
   }


   /**
            * [n] : current rx (kbps) for nth iface
            */
   public us.ihmc.idl.IDLSequence.Float  getIfaceRxKbps()
   {
      return iface_rx_kbps_;
   }


   /**
            * [n] : current tx (kbps) for nth iface
            */
   public us.ihmc.idl.IDLSequence.Float  getIfaceTxKbps()
   {
      return iface_tx_kbps_;
   }

   /**
            * ======== NVIDIA GPU statistics ========
            */
   public void setNvidiaGpuCount(int nvidia_gpu_count)
   {
      nvidia_gpu_count_ = nvidia_gpu_count;
   }
   /**
            * ======== NVIDIA GPU statistics ========
            */
   public int getNvidiaGpuCount()
   {
      return nvidia_gpu_count_;
   }


   /**
            * [n] : model for nth GPU
            */
   public us.ihmc.idl.IDLSequence.StringBuilderHolder  getNvidiaGpuModels()
   {
      return nvidia_gpu_models_;
   }


   /**
            * [n] : memory used for nth GPU
            */
   public us.ihmc.idl.IDLSequence.Float  getNvidiaGpuMemoryUsed()
   {
      return nvidia_gpu_memory_used_;
   }


   /**
            * [n] : total memory for nth GPU
            */
   public us.ihmc.idl.IDLSequence.Float  getNvidiaGpuMemoryTotal()
   {
      return nvidia_gpu_memory_total_;
   }


   /**
            * [n] : utilization for nth GPU
            */
   public us.ihmc.idl.IDLSequence.Float  getNvidiaGpuUtilization()
   {
      return nvidia_gpu_utilization_;
   }


   /**
            * [n] : die temperature for nth GPU
            */
   public us.ihmc.idl.IDLSequence.Float  getNvidiaGpuTemps()
   {
      return nvidia_gpu_temps_;
   }


   public static Supplier<SystemResourceUsageMessagePubSubType> getPubSubType()
   {
      return SystemResourceUsageMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return SystemResourceUsageMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(SystemResourceUsageMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.uptime_, other.uptime_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.memory_used_, other.memory_used_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.memory_total_, other.memory_total_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.cpu_count_, other.cpu_count_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsFloatSequence(this.cpu_usages_, other.cpu_usages_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsFloatSequence(this.cpu_temps_, other.cpu_temps_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.iface_count_, other.iface_count_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilderSequence(this.iface_names_, other.iface_names_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsFloatSequence(this.iface_rx_kbps_, other.iface_rx_kbps_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsFloatSequence(this.iface_tx_kbps_, other.iface_tx_kbps_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.nvidia_gpu_count_, other.nvidia_gpu_count_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilderSequence(this.nvidia_gpu_models_, other.nvidia_gpu_models_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsFloatSequence(this.nvidia_gpu_memory_used_, other.nvidia_gpu_memory_used_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsFloatSequence(this.nvidia_gpu_memory_total_, other.nvidia_gpu_memory_total_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsFloatSequence(this.nvidia_gpu_utilization_, other.nvidia_gpu_utilization_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsFloatSequence(this.nvidia_gpu_temps_, other.nvidia_gpu_temps_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof SystemResourceUsageMessage)) return false;

      SystemResourceUsageMessage otherMyClass = (SystemResourceUsageMessage) other;

      if (!us.ihmc.idl.IDLTools.equals(this.uptime_, otherMyClass.uptime_)) return false;

      if(this.memory_used_ != otherMyClass.memory_used_) return false;

      if(this.memory_total_ != otherMyClass.memory_total_) return false;

      if(this.cpu_count_ != otherMyClass.cpu_count_) return false;

      if (!this.cpu_usages_.equals(otherMyClass.cpu_usages_)) return false;
      if (!this.cpu_temps_.equals(otherMyClass.cpu_temps_)) return false;
      if(this.iface_count_ != otherMyClass.iface_count_) return false;

      if (!this.iface_names_.equals(otherMyClass.iface_names_)) return false;
      if (!this.iface_rx_kbps_.equals(otherMyClass.iface_rx_kbps_)) return false;
      if (!this.iface_tx_kbps_.equals(otherMyClass.iface_tx_kbps_)) return false;
      if(this.nvidia_gpu_count_ != otherMyClass.nvidia_gpu_count_) return false;

      if (!this.nvidia_gpu_models_.equals(otherMyClass.nvidia_gpu_models_)) return false;
      if (!this.nvidia_gpu_memory_used_.equals(otherMyClass.nvidia_gpu_memory_used_)) return false;
      if (!this.nvidia_gpu_memory_total_.equals(otherMyClass.nvidia_gpu_memory_total_)) return false;
      if (!this.nvidia_gpu_utilization_.equals(otherMyClass.nvidia_gpu_utilization_)) return false;
      if (!this.nvidia_gpu_temps_.equals(otherMyClass.nvidia_gpu_temps_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("SystemResourceUsageMessage {");
      builder.append("uptime=");
      builder.append(this.uptime_);      builder.append(", ");
      builder.append("memory_used=");
      builder.append(this.memory_used_);      builder.append(", ");
      builder.append("memory_total=");
      builder.append(this.memory_total_);      builder.append(", ");
      builder.append("cpu_count=");
      builder.append(this.cpu_count_);      builder.append(", ");
      builder.append("cpu_usages=");
      builder.append(this.cpu_usages_);      builder.append(", ");
      builder.append("cpu_temps=");
      builder.append(this.cpu_temps_);      builder.append(", ");
      builder.append("iface_count=");
      builder.append(this.iface_count_);      builder.append(", ");
      builder.append("iface_names=");
      builder.append(this.iface_names_);      builder.append(", ");
      builder.append("iface_rx_kbps=");
      builder.append(this.iface_rx_kbps_);      builder.append(", ");
      builder.append("iface_tx_kbps=");
      builder.append(this.iface_tx_kbps_);      builder.append(", ");
      builder.append("nvidia_gpu_count=");
      builder.append(this.nvidia_gpu_count_);      builder.append(", ");
      builder.append("nvidia_gpu_models=");
      builder.append(this.nvidia_gpu_models_);      builder.append(", ");
      builder.append("nvidia_gpu_memory_used=");
      builder.append(this.nvidia_gpu_memory_used_);      builder.append(", ");
      builder.append("nvidia_gpu_memory_total=");
      builder.append(this.nvidia_gpu_memory_total_);      builder.append(", ");
      builder.append("nvidia_gpu_utilization=");
      builder.append(this.nvidia_gpu_utilization_);      builder.append(", ");
      builder.append("nvidia_gpu_temps=");
      builder.append(this.nvidia_gpu_temps_);
      builder.append("}");
      return builder.toString();
   }
}
