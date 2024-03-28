package us.ihmc.missionControl;

import mission_control_msgs.msg.dds.SystemAvailableMessage;
import mission_control_msgs.msg.dds.SystemResourceUsageMessage;
import mission_control_msgs.msg.dds.SystemServiceActionMessage;
import mission_control_msgs.msg.dds.SystemServiceLogRefreshMessage;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.MissionControlAPI;
import us.ihmc.ros2.ROS2PublisherBasics;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.log.LogTools;
import us.ihmc.missionControl.resourceMonitor.FreeMemoryMonitor;
import us.ihmc.missionControl.resourceMonitor.NVIDIAGPUMonitor;
import us.ihmc.missionControl.resourceMonitor.SysstatNetworkMonitor;
import us.ihmc.missionControl.resourceMonitor.UptimeMonitor;
import us.ihmc.missionControl.resourceMonitor.cpu.CPUCoreTracker;
import us.ihmc.missionControl.resourceMonitor.cpu.LmSensorsMonitor;
import us.ihmc.missionControl.resourceMonitor.cpu.ProcStatCPUMonitor;
import us.ihmc.missionControl.systemd.SystemdServiceMonitor;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.tools.processManagement.ProcessTools;
import us.ihmc.tools.thread.ExceptionHandlingThreadScheduler;

import java.util.*;

public class MissionControlDaemon
{
   /**
    * The hostname of the system
    */
   private final String hostname;
   /**
    * A unique ID for the instance of this daemon
    */
   private final UUID instanceId;

   private final ProcStatCPUMonitor cpuMonitor;
   private final FreeMemoryMonitor memoryMonitor;
   private final UptimeMonitor uptimeMonitor;
   private LmSensorsMonitor sensorsMonitor; // Optional - requires lm_sensors
   private SysstatNetworkMonitor networkMonitor; // Optional - requires sysstat
   private NVIDIAGPUMonitor nvidiaGPUMonitor; // Optional - requires an NVIDIA GPU
   private final Map<String, SystemdServiceMonitor> serviceMonitors = new HashMap<>();

   private final ROS2Node ros2Node;
   private final ROS2PublisherBasics<SystemAvailableMessage> systemAvailablePublisher;
   private final ROS2PublisherBasics<SystemResourceUsageMessage> systemResourceUsagePublisher;
   private final List<ExceptionHandlingThreadScheduler> schedulers = new ArrayList<>();

   public MissionControlDaemon()
   {
      hostname = ProcessTools.execSimpleCommandSafe("hostname");
      instanceId = UUID.randomUUID();

      cpuMonitor = new ProcStatCPUMonitor();
      cpuMonitor.start();

      memoryMonitor = new FreeMemoryMonitor();
      memoryMonitor.start();

      uptimeMonitor = new UptimeMonitor();
      uptimeMonitor.start();

      if (MissionControlTools.lmSensorsAvailable())
      {
         sensorsMonitor = new LmSensorsMonitor();
         sensorsMonitor.start();
      }
      else
      {
         LogTools.info("Not using lm_sensors monitor");
      }

      if (MissionControlTools.sysstatAvailable())
      {
         networkMonitor = new SysstatNetworkMonitor();
         networkMonitor.start();
      }
      else
      {
         LogTools.info("Not using sysstat network monitor");
      }

      if (MissionControlTools.nvidiaGPUAvailable())
      {
         nvidiaGPUMonitor = new NVIDIAGPUMonitor();
         nvidiaGPUMonitor.start();
      }
      else
      {
         LogTools.info("Not using NVIDIA GPU monitor");
      }

      String ros2NodeName = "mission_control_daemon_" + instanceId.toString().replace("-", ""); // ROS2 node names cannot have dashes
      ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, ros2NodeName);
      systemAvailablePublisher = ros2Node.createPublisher(MissionControlAPI.SYSTEM_AVAILABLE);
      systemResourceUsagePublisher = ros2Node.createPublisher(MissionControlAPI.getSystemResourceUsageTopic(instanceId));

      ExceptionHandlingThreadScheduler systemAvailablePublisherScheduler = new ExceptionHandlingThreadScheduler("SystemAvailablePublisherScheduler");
      ExceptionHandlingThreadScheduler systemResourceUsagePublisherScheduler = new ExceptionHandlingThreadScheduler("SystemResourceUsagePublisherScheduler");

      schedulers.add(systemAvailablePublisherScheduler);
      schedulers.add(systemResourceUsagePublisherScheduler);

      systemAvailablePublisherScheduler.schedule(this::publishAvailable, 1.0);
      systemResourceUsagePublisherScheduler.schedule(this::publishResourceUsage, 0.1);

      ROS2Tools.createCallbackSubscription(ros2Node, MissionControlAPI.getSystemServiceLogRefreshTopic(instanceId), subscriber ->
      {
         handleServiceLogRefreshMessage(subscriber.takeNextData());
      });
      ROS2Tools.createCallbackSubscription(ros2Node, MissionControlAPI.getSystemServiceActionTopic(instanceId), subscriber ->
      {
         SystemServiceActionMessage message = subscriber.takeNextData();
         LogTools.info("Received service action message " + message);
         handleServiceActionMessage(message);
      });
      ROS2Tools.createCallbackSubscription(ros2Node, MissionControlAPI.getSystemRebootTopic(instanceId), subscriber ->
      {
         ProcessTools.execSimpleCommandSafe("sudo reboot");
      });

      MissionControlTools.findSystemdServiceNames().forEach(service ->
      {
         LogTools.info("Watching systemd service: " + service);
         serviceMonitors.put(service, new SystemdServiceMonitor(instanceId, service, ros2Node));
      });
   }

   private void handleServiceLogRefreshMessage(SystemServiceLogRefreshMessage message)
   {
      String serviceName = message.getServiceNameAsString();
      SystemdServiceMonitor serviceMonitor = serviceMonitors.get(serviceName);

      if (serviceMonitor == null)
         return;

      List<String> logHistoryComplete = new ArrayList<>(serviceMonitor.getReader().getLogHistory());

      for (List<String> logHistorySplit : MissionControlTools.splitLogLines(logHistoryComplete, SystemdServiceMonitor.MAX_LOG_LINE_MESSAGE_LENGTH))
         serviceMonitor.publishStatus(logHistorySplit, true);
   }

   private void handleServiceActionMessage(SystemServiceActionMessage message)
   {
      String serviceName = message.getServiceNameAsString();
      SystemdServiceMonitor serviceMonitor = serviceMonitors.get(serviceName);

      if (serviceMonitor == null)
         return;

      String action = message.getSystemdActionAsString();

      switch (action)
      {
         case "start" -> serviceMonitor.start();
         case "stop" -> serviceMonitor.stop();
         case "restart" -> serviceMonitor.restart();
         case "kill" -> serviceMonitor.kill();
      }
   }

   private void publishAvailable()
   {
      SystemAvailableMessage message = new SystemAvailableMessage();
      message.setHostname(hostname);
      message.setInstanceId(instanceId.toString());
      systemAvailablePublisher.publish(message);
   }

   private void publishResourceUsage()
   {
      SystemResourceUsageMessage message = new SystemResourceUsageMessage();

      message.setUptime(uptimeMonitor.getUptime());

      message.setMemoryUsed(memoryMonitor.getMemoryUsedGiB());
      message.setMemoryTotal(memoryMonitor.getMemoryTotalGiB());
      Map<Integer, CPUCoreTracker> cpuCoreTrackers = cpuMonitor.getCpuCoreTrackers();
      int cpuCount = cpuCoreTrackers.size();
      message.setCpuCount(cpuCount);
      cpuCoreTrackers.values().forEach(cpuCoreTracker -> message.getCpuUsages().add(cpuCoreTracker.getPercentUsage()));

      if (sensorsMonitor != null)
      {
         // Do not assume there are the same amount of CPUs as the CPU Monitor
         Map<Integer, Integer> cpuTemps = sensorsMonitor.getCpuTemps();
         for (int cpu : cpuTemps.keySet())
         {
            int temp = cpuTemps.get(cpu);
            message.getCpuTemps().add(temp);
         }
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
         message.getNvidiaGpuModels().add(nvidiaGPUMonitor.getGpuModel());
         message.getNvidiaGpuUtilization().add(nvidiaGPUMonitor.getGpuUsage());
         message.getNvidiaGpuMemoryUsed().add(nvidiaGPUMonitor.getMemoryUsed());
         message.getNvidiaGpuMemoryTotal().add(nvidiaGPUMonitor.getMemoryTotal());
         message.getNvidiaGpuTemps().add(nvidiaGPUMonitor.getTemperature());
      }

      // Publish the message
      systemResourceUsagePublisher.publish(message);
   }

   private void destroy()
   {
      if (networkMonitor != null)
         networkMonitor.stop();
      systemAvailablePublisher.remove();
      systemResourceUsagePublisher.remove();
      schedulers.forEach(ExceptionHandlingThreadScheduler::shutdown);
      serviceMonitors.values().forEach(SystemdServiceMonitor::destroy);
      ros2Node.destroy();
   }

   public static void main(String[] args)
   {
      if (!System.getProperty("os.name").toLowerCase().contains("linux"))
      {
         LogTools.warn("This program is only supported on Linux");
         return;
      }

      LogTools.info("Starting Mission Control Daemon...");

      MissionControlDaemon daemon = new MissionControlDaemon();

      Runtime.getRuntime().addShutdownHook(new Thread(() ->
      {
         daemon.destroy();
         Runtime.getRuntime().halt(0); // Set exit code to 0
      }, MissionControlDaemon.class.getName() + "-Shutdown"));

      ThreadTools.sleepForever();
   }
}
