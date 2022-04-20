package us.ihmc.missionControl;

import org.apache.commons.lang3.tuple.MutablePair;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.messager.kryo.KryoMessager;
import us.ihmc.tools.processManagement.ProcessTools;
import us.ihmc.tools.thread.ExceptionHandlingThreadScheduler;

import java.util.ArrayList;
import java.util.TreeSet;
import java.util.concurrent.ConcurrentLinkedQueue;

import static us.ihmc.missionControl.MissionControlService.API.*;

public class MissionControlService
{
   private final LinuxResourceMonitor linuxResourceMonitor = new LinuxResourceMonitor();
   private final NethogsNetworkMonitor nethogsNetworkMonitor = new NethogsNetworkMonitor();

   private final KryoMessager kryoMessagerServer;
   private final TreeSet<String> servicesToTrack = new TreeSet<>();
   private boolean firstConnectedTick = true;
   private final ConcurrentLinkedQueue<String> servicesToTrackQueue = new ConcurrentLinkedQueue<>();

   public MissionControlService()
   {
      kryoMessagerServer = KryoMessager.createServer(API.create(), PORT, "mission_control_service", 100);
      kryoMessagerServer.startMessagerAsyncronously();
      ExceptionHandlingThreadScheduler updateThreadScheduler = new ExceptionHandlingThreadScheduler("MissionControlUpdate",
                                                                                                    DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
      updateThreadScheduler.schedule(this::update, 0.2);

      servicesToTrack.add("mission-control-2");

      nethogsNetworkMonitor.start();
   }

   private void update()
   {
      linuxResourceMonitor.update();

//      LogTools.info("RAM: " + FormattingTools.getFormattedDecimal1D(linuxResourceMonitor.getUsedRAMGiB()) + " / " + FormattingTools.getFormattedDecimal1D(linuxResourceMonitor.getTotalRAMGiB()) + " GiB");
//      ArrayList<CPUCoreTracker> cpuCoreTrackers = linuxResourceMonitor.getCpuCoreTrackers();
//      for (int i = 0; i < cpuCoreTrackers.size(); i++)
//      {
//         CPUCoreTracker cpuCoreTracker = cpuCoreTrackers.get(i);
//         LogTools.info("CPU Core {}: {} %", i, FormattingTools.getFormattedDecimal1D(cpuCoreTracker.getPercentUsage()));
//      }

//      ProcessTools.execSimpleCommand()


      if (kryoMessagerServer.isMessagerOpen())
      {
         if (firstConnectedTick)
         {
            firstConnectedTick = false;
            kryoMessagerServer.registerTopicListener(ServiceNameToTrack, servicesToTrackQueue::add);
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

         kryoMessagerServer.submitMessage(RAMUsage, MutablePair.of(linuxResourceMonitor.getUsedRAMGiB(), linuxResourceMonitor.getTotalRAMGiB()));
         ArrayList<Double> cpuUsages = new ArrayList<>();
         for (CPUCoreTracker cpuCoreTracker : linuxResourceMonitor.getCpuCoreTrackers())
         {
            cpuUsages.add(cpuCoreTracker.getPercentUsage());
         }
         kryoMessagerServer.submitMessage(API.CPUUsages, cpuUsages);

         kryoMessagerServer.submitMessage(API.NetworkUsage,
                                          MutablePair.of(nethogsNetworkMonitor.getKilobytesPerSecondSent(),
                                                         nethogsNetworkMonitor.getKilobytesPerSecondReceived()));
      }
   }

   private String getStatus(String serviceName)
   {
      String statusOutput = ProcessTools.execSimpleCommand("systemctl status " + serviceName);
      String[] lines = statusOutput.split("\\R");
      return lines[2].trim();
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
      new MissionControlService();
   }
}
