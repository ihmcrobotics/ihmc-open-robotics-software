package us.ihmc.missionControl;

import ihmc_common_msgs.msg.dds.SystemResourceUsageMessage;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.tools.processManagement.ProcessTools;
import us.ihmc.tools.thread.ExceptionHandlingThreadScheduler;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Map;
import java.util.concurrent.ScheduledFuture;

/**
 * Requires sysstat
 *    sudo apt install sysstat
 *    sudo systemctl enable sysstat
 *    sudo systemctl start sysstat
 */
public class MissionControlDaemon
{
   private final LinuxResourceMonitor resourceMonitor;
   private final SysstatNetworkMonitor networkMonitor;
   private final NVIDIAGPUMonitor gpuUsageMonitor;

   private final ROS2Node ros2Node;
   private final IHMCROS2Publisher<SystemResourceUsageMessage> systemResourceUsagePublisher;

   private final ExceptionHandlingThreadScheduler updateThreadScheduler;
   private final ScheduledFuture<?> scheduledFuture;

   public MissionControlDaemon()
   {
      resourceMonitor = new LinuxResourceMonitor();
      networkMonitor = new SysstatNetworkMonitor();
      gpuUsageMonitor = new NVIDIAGPUMonitor();

      networkMonitor.start();

      ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "mission_control_daemon");
      systemResourceUsagePublisher = ROS2Tools.createPublisher(ros2Node, ROS2Tools.SYSTEM_RESOURCE_USAGE);

      updateThreadScheduler = new ExceptionHandlingThreadScheduler("MissionControlUpdate", DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
      scheduledFuture = updateThreadScheduler.schedule(this::update, 0.25);

      Runtime.getRuntime().addShutdownHook(new Thread(this::destroy, "Shutdown"));
   }

   private void update()
   {
      if (ros2Node == null)
         return;

      SystemResourceUsageMessage message = new SystemResourceUsageMessage();

      resourceMonitor.update();

      // Set memory statistics
      {
         message.setMemoryUsed(resourceMonitor.getUsedRAMGiB());
         message.setMemoryTotal(resourceMonitor.getTotalRAMGiB());
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
            message.getNvidiaGpuUtilization().add(gpuUsageMonitor.getGpuUsage());
            message.getNvidiaGpuMemoryUsed().add(gpuUsageMonitor.getMemoryUsed());
            message.getNvidiaGpuTotalMemory().add(gpuUsageMonitor.getMemoryTotal());
         }
      }

      System.out.println("publishing...");

      // Publish the message
      systemResourceUsagePublisher.publish(message);
   }

   private String getStatus(String serviceName)
   {
      String statusOutput = null;
      try
      {
         statusOutput = ProcessTools.execSimpleCommand("systemctl status " + serviceName);
      }
      catch (IOException | InterruptedException ignored)
      {
      }
      if (statusOutput == null) return null;
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

   private static volatile boolean running = true;

   public static void main(String[] args)
   {
      new MissionControlDaemon();

      Runtime.getRuntime().addShutdownHook(new Thread(() ->
      {
         running = false;
         Runtime.getRuntime().halt(0); // Set exit code to 0
      }));

      while (running)
      {
         ThreadTools.sleep(1000);
      }
   }
}
