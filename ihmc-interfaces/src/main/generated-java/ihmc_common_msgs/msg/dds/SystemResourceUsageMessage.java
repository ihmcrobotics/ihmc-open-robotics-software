package ihmc_common_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This is part of the IHMC Common message package.
       */
public class SystemResourceUsageMessage extends Packet<SystemResourceUsageMessage> implements Settable<SystemResourceUsageMessage>, EpsilonComparable<SystemResourceUsageMessage>
{
   /**
            * ======== System memory statistics ========
            */
   public float memory_used_;
   public float total_memory_;
   /**
            * ======== CPU statistics ========
            */
   public int cpu_count_;
   /**
            * [n] : usage for nth CPU
            */
   public us.ihmc.idl.IDLSequence.Float  cpu_usages_;
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
            * [n] : memory used for nth GPU
            */
   public us.ihmc.idl.IDLSequence.Float  nvidia_gpu_memory_used_;
   /**
            * [n] : total memory for nth GPU
            */
   public us.ihmc.idl.IDLSequence.Float  nvidia_gpu_total_memory_;
   /**
            * [n] : utilization for nth GPU
            */
   public us.ihmc.idl.IDLSequence.Float  nvidia_gpu_utilization_;

   public SystemResourceUsageMessage()
   {
      cpu_usages_ = new us.ihmc.idl.IDLSequence.Float (100, "type_5");

      iface_names_ = new us.ihmc.idl.IDLSequence.StringBuilderHolder (100, "type_d");
      iface_rx_kbps_ = new us.ihmc.idl.IDLSequence.Float (100, "type_5");

      iface_tx_kbps_ = new us.ihmc.idl.IDLSequence.Float (100, "type_5");

      nvidia_gpu_memory_used_ = new us.ihmc.idl.IDLSequence.Float (100, "type_5");

      nvidia_gpu_total_memory_ = new us.ihmc.idl.IDLSequence.Float (100, "type_5");

      nvidia_gpu_utilization_ = new us.ihmc.idl.IDLSequence.Float (100, "type_5");

   }

   public SystemResourceUsageMessage(SystemResourceUsageMessage other)
   {
      this();
      set(other);
   }

   public void set(SystemResourceUsageMessage other)
   {
      memory_used_ = other.memory_used_;

      total_memory_ = other.total_memory_;

      cpu_count_ = other.cpu_count_;

      cpu_usages_.set(other.cpu_usages_);
      iface_count_ = other.iface_count_;

      iface_names_.set(other.iface_names_);
      iface_rx_kbps_.set(other.iface_rx_kbps_);
      iface_tx_kbps_.set(other.iface_tx_kbps_);
      nvidia_gpu_count_ = other.nvidia_gpu_count_;

      nvidia_gpu_memory_used_.set(other.nvidia_gpu_memory_used_);
      nvidia_gpu_total_memory_.set(other.nvidia_gpu_total_memory_);
      nvidia_gpu_utilization_.set(other.nvidia_gpu_utilization_);
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

   public void setTotalMemory(float total_memory)
   {
      total_memory_ = total_memory;
   }
   public float getTotalMemory()
   {
      return total_memory_;
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
            * [n] : memory used for nth GPU
            */
   public us.ihmc.idl.IDLSequence.Float  getNvidiaGpuMemoryUsed()
   {
      return nvidia_gpu_memory_used_;
   }


   /**
            * [n] : total memory for nth GPU
            */
   public us.ihmc.idl.IDLSequence.Float  getNvidiaGpuTotalMemory()
   {
      return nvidia_gpu_total_memory_;
   }


   /**
            * [n] : utilization for nth GPU
            */
   public us.ihmc.idl.IDLSequence.Float  getNvidiaGpuUtilization()
   {
      return nvidia_gpu_utilization_;
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

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.memory_used_, other.memory_used_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.total_memory_, other.total_memory_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.cpu_count_, other.cpu_count_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsFloatSequence(this.cpu_usages_, other.cpu_usages_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.iface_count_, other.iface_count_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilderSequence(this.iface_names_, other.iface_names_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsFloatSequence(this.iface_rx_kbps_, other.iface_rx_kbps_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsFloatSequence(this.iface_tx_kbps_, other.iface_tx_kbps_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.nvidia_gpu_count_, other.nvidia_gpu_count_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsFloatSequence(this.nvidia_gpu_memory_used_, other.nvidia_gpu_memory_used_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsFloatSequence(this.nvidia_gpu_total_memory_, other.nvidia_gpu_total_memory_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsFloatSequence(this.nvidia_gpu_utilization_, other.nvidia_gpu_utilization_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof SystemResourceUsageMessage)) return false;

      SystemResourceUsageMessage otherMyClass = (SystemResourceUsageMessage) other;

      if(this.memory_used_ != otherMyClass.memory_used_) return false;

      if(this.total_memory_ != otherMyClass.total_memory_) return false;

      if(this.cpu_count_ != otherMyClass.cpu_count_) return false;

      if (!this.cpu_usages_.equals(otherMyClass.cpu_usages_)) return false;
      if(this.iface_count_ != otherMyClass.iface_count_) return false;

      if (!this.iface_names_.equals(otherMyClass.iface_names_)) return false;
      if (!this.iface_rx_kbps_.equals(otherMyClass.iface_rx_kbps_)) return false;
      if (!this.iface_tx_kbps_.equals(otherMyClass.iface_tx_kbps_)) return false;
      if(this.nvidia_gpu_count_ != otherMyClass.nvidia_gpu_count_) return false;

      if (!this.nvidia_gpu_memory_used_.equals(otherMyClass.nvidia_gpu_memory_used_)) return false;
      if (!this.nvidia_gpu_total_memory_.equals(otherMyClass.nvidia_gpu_total_memory_)) return false;
      if (!this.nvidia_gpu_utilization_.equals(otherMyClass.nvidia_gpu_utilization_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("SystemResourceUsageMessage {");
      builder.append("memory_used=");
      builder.append(this.memory_used_);      builder.append(", ");
      builder.append("total_memory=");
      builder.append(this.total_memory_);      builder.append(", ");
      builder.append("cpu_count=");
      builder.append(this.cpu_count_);      builder.append(", ");
      builder.append("cpu_usages=");
      builder.append(this.cpu_usages_);      builder.append(", ");
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
      builder.append("nvidia_gpu_memory_used=");
      builder.append(this.nvidia_gpu_memory_used_);      builder.append(", ");
      builder.append("nvidia_gpu_total_memory=");
      builder.append(this.nvidia_gpu_total_memory_);      builder.append(", ");
      builder.append("nvidia_gpu_utilization=");
      builder.append(this.nvidia_gpu_utilization_);
      builder.append("}");
      return builder.toString();
   }
}
