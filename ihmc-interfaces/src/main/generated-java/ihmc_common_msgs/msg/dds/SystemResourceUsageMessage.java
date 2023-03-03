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
   public float net_bytes_in_;
   public float net_bytes_out_;
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
      net_bytes_in_ = other.net_bytes_in_;

      net_bytes_out_ = other.net_bytes_out_;

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
   public void setNetBytesIn(float net_bytes_in)
   {
      net_bytes_in_ = net_bytes_in;
   }
   /**
            * ======== Network statistics ========
            */
   public float getNetBytesIn()
   {
      return net_bytes_in_;
   }

   public void setNetBytesOut(float net_bytes_out)
   {
      net_bytes_out_ = net_bytes_out;
   }
   public float getNetBytesOut()
   {
      return net_bytes_out_;
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

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.net_bytes_in_, other.net_bytes_in_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.net_bytes_out_, other.net_bytes_out_, epsilon)) return false;

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
      if(this.net_bytes_in_ != otherMyClass.net_bytes_in_) return false;

      if(this.net_bytes_out_ != otherMyClass.net_bytes_out_) return false;

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
      builder.append("net_bytes_in=");
      builder.append(this.net_bytes_in_);      builder.append(", ");
      builder.append("net_bytes_out=");
      builder.append(this.net_bytes_out_);      builder.append(", ");
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
