package us.ihmc.missionControl;

import mission_control_msgs.msg.dds.SystemAvailableMessage;
import mission_control_msgs.msg.dds.SystemResourceUsageMessage;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.log.LogTools;
import us.ihmc.missionControl.resourceMonitor.NVIDIAGPUMonitor;
import us.ihmc.missionControl.resourceMonitor.SysstatNetworkMonitor;
import us.ihmc.missionControl.systemd.SystemdServiceMonitor;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.tools.processManagement.ProcessTools;
import us.ihmc.tools.thread.ExceptionHandlingThreadScheduler;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.UUID;

public class MissionControlDaemon
{
   private final String hostname;
   private final String instanceId;

   private final LinuxResourceMonitor resourceMonitor;
   private SysstatNetworkMonitor networkMonitor; // Optional - requires sysstat
   private NVIDIAGPUMonitor nvidiaGPUMonitor; // Optional - requires an NVIDIA GPU
   private final List<SystemdServiceMonitor> serviceMonitors = new ArrayList<>();

   private final ROS2Node ros2Node;
   private final IHMCROS2Publisher<SystemAvailableMessage> systemAvailablePublisher;
   private final IHMCROS2Publisher<SystemResourceUsageMessage> systemResourceUsagePublisher;
   private final List<ExceptionHandlingThreadScheduler> schedulers = new ArrayList<>();

   public MissionControlDaemon()
   {
      hostname = ProcessTools.execSimpleCommandSafe("hostname");
      instanceId = UUID.randomUUID().toString().substring(0, 5);

      resourceMonitor = new LinuxResourceMonitor();

      if (MissionControlTools.sysstatAvailable())
         networkMonitor = new SysstatNetworkMonitor();
      else
         LogTools.info("Not using sysstat network monitor");

      if (MissionControlTools.nvidiaGPUAvailable())
         nvidiaGPUMonitor = new NVIDIAGPUMonitor();
      else
         LogTools.info("Not using NVIDIA GPU monitor");

      ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "mission_control_daemon_" + instanceId);
      systemAvailablePublisher = ROS2Tools.createPublisher(ros2Node, ROS2Tools.SYSTEM_AVAILABLE);
      systemResourceUsagePublisher = ROS2Tools.createPublisher(ros2Node, ROS2Tools.getSystemResourceUsageTopic(instanceId));

      ExceptionHandlingThreadScheduler systemAvailablePublisherScheduler = new ExceptionHandlingThreadScheduler("SystemAvailablePublisherScheduler");
      ExceptionHandlingThreadScheduler systemResourceUsagePublisherScheduler = new ExceptionHandlingThreadScheduler("SystemResourceUsagePublisherScheduler");

      schedulers.add(systemAvailablePublisherScheduler);
      schedulers.add(systemResourceUsagePublisherScheduler);

      systemAvailablePublisherScheduler.schedule(this::publishAvailable, 1.0);
      systemResourceUsagePublisherScheduler.schedule(this::publishResourceUsage, 0.25);

      MissionControlTools.findSystemdServiceNames().forEach(service -> serviceMonitors.add(new SystemdServiceMonitor(service)));

      Runtime.getRuntime().addShutdownHook(new Thread(this::destroy, "Shutdown"));
   }

   private void publishAvailable()
   {
      SystemAvailableMessage message = new SystemAvailableMessage();
      message.setHostname(hostname);
      message.setInstanceId(instanceId);
      message.setEpochTime(System.currentTimeMillis());
      systemAvailablePublisher.publish(message);
   }

   private void publishResourceUsage()
   {
      SystemResourceUsageMessage message = new SystemResourceUsageMessage();

      if (resourceMonitor != null)
      {
         resourceMonitor.update();
         message.setMemoryUsed(resourceMonitor.getUsedRAMGiB());
         message.setMemoryTotal(resourceMonitor.getTotalRAMGiB());
         ArrayList<CPUCoreTracker> cpuCoreTrackers = resourceMonitor.getCpuCoreTrackers();
         int cpuCount = cpuCoreTrackers.size();
         message.setCpuCount(cpuCount);
         cpuCoreTrackers.forEach(cpuCoreTracker -> message.getCpuUsages().add(cpuCoreTracker.getPercentUsage()));
      }

      if (networkMonitor != null)
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

      if (nvidiaGPUMonitor != null)
      {
         message.setNvidiaGpuCount(1); // TODO: support multiple GPUs
         message.getNvidiaGpuUtilization().add(nvidiaGPUMonitor.getGpuUsage());
         message.getNvidiaGpuMemoryUsed().add(nvidiaGPUMonitor.getMemoryUsed());
         message.getNvidiaGpuTotalMemory().add(nvidiaGPUMonitor.getMemoryTotal());
      }

      // Publish the message
      systemResourceUsagePublisher.publish(message);
   }

   private void destroy()
   {
      networkMonitor.stop();
      systemAvailablePublisher.destroy();
      systemResourceUsagePublisher.destroy();
      schedulers.forEach(ExceptionHandlingThreadScheduler::shutdown);
      serviceMonitors.forEach(SystemdServiceMonitor::destroy);
      ros2Node.destroy();
   }

   private static volatile boolean running = true;

   public static void main(String[] args)
   {
      if (!System.getProperty("os.name").toLowerCase().contains("linux"))
      {
         LogTools.warn("This program is only supported on Linux");
         return;
      }

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
