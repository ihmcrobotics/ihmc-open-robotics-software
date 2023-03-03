package us.ihmc.missionControl;

import ihmc_common_msgs.msg.dds.SystemResourceUsageMessage;
import org.apache.commons.lang3.tuple.MutablePair;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.log.LogTools;
import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.messager.kryo.KryoMessager;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.tools.processManagement.ProcessTools;
import us.ihmc.tools.thread.ExceptionHandlingThreadScheduler;

import java.util.ArrayList;
import java.util.TreeSet;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.ScheduledFuture;

import static us.ihmc.missionControl.MissionControlDaemon.API.*;

public class MissionControlDaemon
{
   private final LinuxResourceMonitor linuxResourceMonitor = new LinuxResourceMonitor();
   private final SysstatNetworkMonitor nloadNetworkMonitor = new SysstatNetworkMonitor();
   private final ChronyStatusMonitor chronyStatusMonitor = new ChronyStatusMonitor();
   private final GPUUsageMonitor gpuUsageMonitor = new GPUUsageMonitor();

   private final KryoMessager kryoMessagerServer;
   private final ExceptionHandlingThreadScheduler updateThreadScheduler;
   private final ScheduledFuture<?> scheduledFuture;
   private final TreeSet<String> servicesToTrack = new TreeSet<>();
   private boolean firstConnectedTick = true;
   private final ConcurrentLinkedQueue<String> servicesToTrackQueue = new ConcurrentLinkedQueue<>();

   private final ROS2Node ros2Node;
   private final IHMCROS2Publisher<SystemResourceUsageMessage> systemResourceUsagePublisher;

   public MissionControlDaemon()
   {
      kryoMessagerServer = KryoMessager.createServer(API.create(), PORT, "mission_control_service", 100);
      kryoMessagerServer.startMessagerAsyncronously();
      updateThreadScheduler = new ExceptionHandlingThreadScheduler("MissionControlUpdate", DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
      scheduledFuture = updateThreadScheduler.schedule(this::update, 0.2);

      servicesToTrack.add("mission-control-2");

      nloadNetworkMonitor.start();
      chronyStatusMonitor.start();

      ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "mission-control");
      systemResourceUsagePublisher = ROS2Tools.createPublisher(ros2Node, ROS2Tools.SYSTEM_RESOURCE_USAGE);

      Runtime.getRuntime().addShutdownHook(new Thread(this::destroy, "Shutdown"));
   }

   private void update()
   {
      if (ros2Node == null) return;

      linuxResourceMonitor.update();

      SystemResourceUsageMessage systemResourceUsageMessage = new SystemResourceUsageMessage();

      // Set memory statistics
      systemResourceUsageMessage.setMemoryUsed(linuxResourceMonitor.getUsedRAMGiB());
      systemResourceUsageMessage.setTotalMemory(linuxResourceMonitor.getTotalRAMGiB());

      // Set CPU statistics
      ArrayList<CPUCoreTracker> cpuCoreTrackers = linuxResourceMonitor.getCpuCoreTrackers();
      int cpuCount = cpuCoreTrackers.size();
      systemResourceUsageMessage.setCpuCount(cpuCount);
      for (int i = 0; i < cpuCount; i++)
      {
         float cpuUsage = systemResourceUsageMessage.getCpuUsages().get(i);
         systemResourceUsageMessage.getCpuUsages().set(i, cpuUsage);
      }

      // Set network statistics


      // Set GPU statistics


      if (kryoMessagerServer.isMessagerOpen())
      {
         if (firstConnectedTick)
         {
            firstConnectedTick = false;
            kryoMessagerServer.addTopicListener(ServiceNameToTrack, servicesToTrackQueue::add);
         }

         while (!servicesToTrackQueue.isEmpty())
         {
            servicesToTrack.add(servicesToTrackQueue.poll());
         }

         ArrayList<String> statuses = new ArrayList<>();
         for (String serviceName : servicesToTrack)
         {
            statuses.add(serviceName + ":" + getStatus(serviceName));
         }
         kryoMessagerServer.submitMessage(ServiceStatuses, statuses);

//         kryoMessagerServer.submitMessage(RAMUsage, MutablePair.of(linuxResourceMonitor.getUsedRAMGiB(), linuxResourceMonitor.getTotalRAMGiB()));
         ArrayList<Double> cpuUsages = new ArrayList<>();
         for (CPUCoreTracker cpuCoreTracker : linuxResourceMonitor.getCpuCoreTrackers())
         {
            cpuUsages.add(cpuCoreTracker.getPercentUsage());
         }
         kryoMessagerServer.submitMessage(API.CPUUsages, cpuUsages);

//         kryoMessagerServer.submitMessage(API.NetworkUsage,
//                                          MutablePair.of(nloadNetworkMonitor.getKilobytesPerSecondSent(),
//                                                         nloadNetworkMonitor.getKilobytesPerSecondReceived()));

         kryoMessagerServer.submitMessage(ChronySelectedServerStatus, chronyStatusMonitor.getChronycBestSourceStatus());

         if (gpuUsageMonitor.getHasNvidiaGPU())
         {
            kryoMessagerServer.submitMessage(GPUUsage, gpuUsageMonitor.getGPUUsage());
         }
      }
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
      scheduledFuture.cancel(false);
      updateThreadScheduler.shutdown();
//      nloadNetworkMonitor.stop();
      chronyStatusMonitor.stop();
      ros2Node.destroy();
      ExceptionTools.handle(kryoMessagerServer::closeMessager, DefaultExceptionHandler.PRINT_MESSAGE);
   }

   public static class API
   {
      public static final int PORT = 2151;

      private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();
      private static final MessagerAPIFactory.Category RootCategory = apiFactory.createRootCategory("Root");
      private static final MessagerAPIFactory.CategoryTheme MissionControlServiceTheme = apiFactory.createCategoryTheme("MissionControlService");

      public static final MessagerAPIFactory.Topic<MutablePair<Double, Double>> RAMUsage = topic("RAMUsage");
      public static final MessagerAPIFactory.Topic<ArrayList<Double>> CPUUsages = topic("CPUUsages");
      public static final MessagerAPIFactory.Topic<MutablePair<Double, Double>> NetworkUsage = topic("NetworkUsage");
      public static final MessagerAPIFactory.Topic<String> ServiceNameToTrack = topic("ServiceNameToTrack");
      public static final MessagerAPIFactory.Topic<ArrayList<String>> ServiceStatuses = topic("ServiceStatuses");
      public static final MessagerAPIFactory.Topic<String> ChronySelectedServerStatus = topic("ChronySelectedServerStatus");
      public static final MessagerAPIFactory.Topic<Double> GPUUsage = topic("GPUUsage");
      private static <T> MessagerAPIFactory.Topic<T> topic(String name)
      {
         return RootCategory.child(MissionControlServiceTheme).topic(apiFactory.createTypedTopicTheme(name));
      }

      public static synchronized MessagerAPIFactory.MessagerAPI create()
      {
         return apiFactory.getAPIAndCloseFactory();
      }
   }

   public static void main(String[] args)
   {
      new MissionControlDaemon();
   }
}
