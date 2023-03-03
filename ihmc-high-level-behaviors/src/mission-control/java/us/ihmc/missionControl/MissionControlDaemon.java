package us.ihmc.missionControl;

import ihmc_common_msgs.msg.dds.SystemResourceUsageMessage;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.tools.processManagement.ProcessTools;
import us.ihmc.tools.thread.ExceptionHandlingThreadScheduler;

import java.util.ArrayList;
import java.util.Map;
import java.util.concurrent.ScheduledFuture;

public class MissionControlDaemon
{
   private final LinuxResourceMonitor resourceMonitor;
   private final SysstatNetworkMonitor networkMonitor;
   private final NVIDIAGPUUsageMonitor gpuUsageMonitor;

   private final ROS2Node ros2Node;
   private final IHMCROS2Publisher<SystemResourceUsageMessage> systemResourceUsagePublisher;

   private final ExceptionHandlingThreadScheduler updateThreadScheduler;
   private final ScheduledFuture<?> scheduledFuture;

   public MissionControlDaemon()
   {

      resourceMonitor = new LinuxResourceMonitor();
      networkMonitor = new SysstatNetworkMonitor();
      gpuUsageMonitor = new NVIDIAGPUUsageMonitor();

      networkMonitor.start();

      ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "mission_control_daemon");
      systemResourceUsagePublisher = ROS2Tools.createPublisher(ros2Node, ROS2Tools.SYSTEM_RESOURCE_USAGE);

      updateThreadScheduler = new ExceptionHandlingThreadScheduler("MissionControlUpdate", DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
      scheduledFuture = updateThreadScheduler.schedule(this::update, 0.2);

      Runtime.getRuntime().addShutdownHook(new Thread(this::destroy, "Shutdown"));
   }

   private void update()
   {
      if (ros2Node == null)
         return;

      resourceMonitor.update();

      SystemResourceUsageMessage message = new SystemResourceUsageMessage();

      // Set memory statistics
      {
         message.setMemoryUsed(resourceMonitor.getUsedRAMGiB());
         message.setTotalMemory(resourceMonitor.getTotalRAMGiB());
      }

      // Set CPU statistics
      {
         ArrayList<CPUCoreTracker> cpuCoreTrackers = resourceMonitor.getCpuCoreTrackers();
         int cpuCount = cpuCoreTrackers.size();
         message.setCpuCount(cpuCount);
         cpuCoreTrackers.forEach(cpuCoreTracker -> message.getCpuUsages().add(cpuCoreTracker.getPercentUsage()));
      }

      // Set network statistics
      {
         Map<String, Float> ifaceRxKbps = networkMonitor.getIfaceRxKbps();
         Map<String, Float> ifaceTxKbps = networkMonitor.getIfaceTxKbps();
         int ifaceCount = ifaceRxKbps.size();
         message.setIfaceCount(ifaceCount);

         for (Map.Entry<String, Float> entry : ifaceRxKbps.entrySet())
         {
            message.getIfaceNames().add(entry.getKey());
            message.getIfaceRxKbps().add(entry.getValue());
         }

         int index = 0;
         for (Map.Entry<String, Float> entry : ifaceTxKbps.entrySet())
         {
            if (message.getIfaceNames().get(index++).toString().equals(entry.getKey()))
               message.getIfaceTxKbps().add(entry.getValue());
         }
      }

      // Set GPU statistics
      {
         // TODO: support multiple GPUs
         if (gpuUsageMonitor.getHasNvidiaGPU())
         {
            message.setNvidiaGpuCount(1);
            message.getNvidiaGpuUtilization().add(gpuUsageMonitor.getGPUUsage());
            message.getNvidiaGpuMemoryUsed().add(gpuUsageMonitor.getGPUUsedMemoryMB());
            message.getNvidiaGpuTotalMemory().add(gpuUsageMonitor.getGPUTotalMemoryMB());
         }
      }

      // Publish the message
      systemResourceUsagePublisher.publish(message);
   }

   private String getStatus(String serviceName)
   {
      String statusOutput = ProcessTools.execSimpleCommand("systemctl status " + serviceName);
      String[] lines = statusOutput.split("\\R");
      if (lines.length < 3)
      {
         LogTools.error("Got: {}", statusOutput);
         return "error parsing systemd status";
      }
      else
      {
         return lines[2].trim();
      }
   }

   private void destroy()
   {
      networkMonitor.stop();
      scheduledFuture.cancel(false);
      systemResourceUsagePublisher.destroy();
      ros2Node.destroy();
   }

   public static void main(String[] args)
   {
      new MissionControlDaemon();

      try
      {
         Thread.sleep(10000000);
      }
      catch (InterruptedException e)
      {
         throw new RuntimeException(e);
      }
   }
}
