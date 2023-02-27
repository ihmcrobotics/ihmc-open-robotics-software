package us.ihmc.missionControl;

import com.sun.management.OperatingSystemMXBean;
import us.ihmc.commons.FormattingTools;

import java.lang.management.ManagementFactory;

public class MXBeanResourceMonitor
{

   private final OperatingSystemMXBean platformMXBean;
   private double systemLoadAverage;
   private double processCpuLoad;
   private long processCpuTime;
   private double systemCpuLoad;
   private long committedVirtualMemorySize;
   private long freePhysicalMemorySize;
   private double totalRAMGB;
   private double committedRAMGB;
   private double freeRAMGB;
   private double usedRAMGB;
   private String ramUsageString;
   private long freeSwapSpaceSize;
   private long totalSwapSpaceSize;
   private int availableProcessors;
   private String systemCPUPercentageString;

   public MXBeanResourceMonitor()
   {
      platformMXBean = ManagementFactory.getPlatformMXBean(OperatingSystemMXBean.class);
   }

   public void update()
   {
      systemLoadAverage = platformMXBean.getSystemLoadAverage();
      processCpuLoad = platformMXBean.getProcessCpuLoad();
      processCpuTime = platformMXBean.getProcessCpuTime();
      systemCpuLoad = platformMXBean.getSystemCpuLoad();
      committedVirtualMemorySize = platformMXBean.getCommittedVirtualMemorySize();
      freePhysicalMemorySize = platformMXBean.getFreePhysicalMemorySize();
      totalRAMGB = platformMXBean.getTotalPhysicalMemorySize() / 1000000000.0;
      committedRAMGB = platformMXBean.getCommittedVirtualMemorySize() / 1000000000.0;
      freeRAMGB = platformMXBean.getFreePhysicalMemorySize() / 1000000000.0;
      usedRAMGB = totalRAMGB - freeRAMGB;
      ramUsageString = FormattingTools.getFormattedDecimal1D(committedRAMGB) + " / " + FormattingTools.getFormattedDecimal1D(totalRAMGB) + " GB";
      freeSwapSpaceSize = platformMXBean.getFreeSwapSpaceSize();
      totalSwapSpaceSize = platformMXBean.getTotalSwapSpaceSize();
      availableProcessors = platformMXBean.getAvailableProcessors();

      systemCPUPercentageString = FormattingTools.getFormattedDecimal1D(systemCpuLoad) + " %";
   }

   public String getRamUsageString()
   {
      return ramUsageString;
   }

   public String getSystemCPUPercentageString()
   {
      return systemCPUPercentageString;
   }
}
