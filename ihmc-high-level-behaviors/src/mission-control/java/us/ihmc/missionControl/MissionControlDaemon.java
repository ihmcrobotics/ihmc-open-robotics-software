package us.ihmc.missionControl;

import mission_control_msgs.msg.dds.SystemAvailableMessage;
import mission_control_msgs.msg.dds.SystemResourceUsageMessage;
import mission_control_msgs.msg.dds.SystemServiceActionMessage;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.log.LogTools;
import us.ihmc.missionControl.resourceMonitor.FreeMemoryMonitor;
import us.ihmc.missionControl.resourceMonitor.NVIDIAGPUMonitor;
import us.ihmc.missionControl.resourceMonitor.SysstatNetworkMonitor;
import us.ihmc.missionControl.resourceMonitor.cpu.CPUCoreTracker;
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
   private SysstatNetworkMonitor networkMonitor; // Optional - requires sysstat
   private NVIDIAGPUMonitor nvidiaGPUMonitor; // Optional - requires an NVIDIA GPU
   private final Map<String, SystemdServiceMonitor> serviceMonitors = new HashMap<>();

   private final ROS2Node ros2Node;
   private final IHMCROS2Publisher<SystemAvailableMessage> systemAvailablePublisher;
   private final IHMCROS2Publisher<SystemResourceUsageMessage> systemResourceUsagePublisher;
   private final List<ExceptionHandlingThreadScheduler> schedulers = new ArrayList<>();

   public MissionControlDaemon()
   {
      hostname = ProcessTools.execSimpleCommandSafe("hostname");
      instanceId = UUID.randomUUID();

      cpuMonitor = new ProcStatCPUMonitor();
      cpuMonitor.start();

      memoryMonitor = new FreeMemoryMonitor();
      memoryMonitor.start();

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
      systemAvailablePublisher = ROS2Tools.createPublisher(ros2Node, ROS2Tools.SYSTEM_AVAILABLE);
      systemResourceUsagePublisher = ROS2Tools.createPublisher(ros2Node, ROS2Tools.getSystemResourceUsageTopic(instanceId));

      ExceptionHandlingThreadScheduler systemAvailablePublisherScheduler = new ExceptionHandlingThreadScheduler("SystemAvailablePublisherScheduler");
      ExceptionHandlingThreadScheduler systemResourceUsagePublisherScheduler = new ExceptionHandlingThreadScheduler("SystemResourceUsagePublisherScheduler");

      schedulers.add(systemAvailablePublisherScheduler);
      schedulers.add(systemResourceUsagePublisherScheduler);

      systemAvailablePublisherScheduler.schedule(this::publishAvailable, 1.0);
      systemResourceUsagePublisherScheduler.schedule(this::publishResourceUsage, 0.25);

      ROS2Tools.createCallbackSubscription(ros2Node, ROS2Tools.getSystemServiceActionTopic(instanceId), subscriber ->
      {
         SystemServiceActionMessage message = subscriber.takeNextData();
         LogTools.info("Received service action message " + message);
         handleServiceActionMessage(message);
      });

      MissionControlTools.findSystemdServiceNames().forEach(service ->
      {
         LogTools.info("Watching systemd service: " + service);
         serviceMonitors.put(service, new SystemdServiceMonitor(instanceId, service, ros2Node));
      });

      Runtime.getRuntime().addShutdownHook(new Thread(this::destroy, "Shutdown"));
   }

   private void handleServiceActionMessage(SystemServiceActionMessage message)
   {
      String serviceName = message.getServiceNameAsString();
      SystemdServiceMonitor serviceMonitor = serviceMonitors.get(serviceName);

      if (serviceMonitor == null)
         return;

      String action = message.getSystemdActionAsString();

      if (action.equals("start"))
      {
         serviceMonitor.start();
      }
      else if (action.equals("stop"))
      {
         serviceMonitor.stop();
      }
      else if (action.equals("restart"))
      {
         serviceMonitor.restart();
      }
   }

   private void publishAvailable()
   {
      SystemAvailableMessage message = new SystemAvailableMessage();
      message.setHostname(hostname);
      message.setInstanceId(instanceId.toString());
      message.setEpochTime(System.currentTimeMillis());
      systemAvailablePublisher.publish(message);
   }

   private void publishResourceUsage()
   {
      SystemResourceUsageMessage message = new SystemResourceUsageMessage();

      message.setMemoryUsed(memoryMonitor.getMemoryUsedGiB());
      message.setMemoryTotal(memoryMonitor.getMemoryTotalGiB());
      Map<Integer, CPUCoreTracker> cpuCoreTrackers = cpuMonitor.getCpuCoreTrackers();
      int cpuCount = cpuCoreTrackers.size();
      message.setCpuCount(cpuCount);
      cpuCoreTrackers.values().forEach(cpuCoreTracker -> message.getCpuUsages().add(cpuCoreTracker.getPercentUsage()));

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
      serviceMonitors.values().forEach(SystemdServiceMonitor::destroy);
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
