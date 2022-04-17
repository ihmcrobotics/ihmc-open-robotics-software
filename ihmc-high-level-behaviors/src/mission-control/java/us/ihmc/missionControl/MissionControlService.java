package us.ihmc.missionControl;

import org.apache.commons.lang3.tuple.MutablePair;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.messager.kryo.KryoMessager;
import us.ihmc.tools.thread.ExceptionHandlingThreadScheduler;

import java.util.ArrayList;

import static us.ihmc.missionControl.MissionControlService.API.PORT;
import static us.ihmc.missionControl.MissionControlService.API.RAMUsage;

public class MissionControlService
{
   private final LinuxResourceMonitor linuxResourceMonitor = new LinuxResourceMonitor();

//   public static final ROS2Topic<Float64MultiArray> RAM_USAGE_TOPIC
//         = new ROS2Topic<>().withType(Float64MultiArray.class).withModule("missioncontrol").withSuffix("ramusage");
   private final KryoMessager kryoMessagerServer;

   //   public static final Topic

   public MissionControlService()
   {
      kryoMessagerServer = KryoMessager.createServer(API.create(), PORT, "mission_control_service", 100);
      kryoMessagerServer.startMessagerAsyncronously();
      ExceptionHandlingThreadScheduler updateThreadScheduler = new ExceptionHandlingThreadScheduler("MissionControlUpdate",
                                                                                                    DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
      updateThreadScheduler.schedule(this::update, 0.2);

//      ROS2Node ros2Node = new ROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "mission_control_service");

      //      new KryoMessager()
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

      if (kryoMessagerServer.isMessagerOpen())
      {
         kryoMessagerServer.submitMessage(RAMUsage, MutablePair.of(linuxResourceMonitor.getUsedRAMGiB(), linuxResourceMonitor.getTotalRAMGiB()));
         ArrayList<Double> cpuUsages = new ArrayList<>();
         for (CPUCoreTracker cpuCoreTracker : linuxResourceMonitor.getCpuCoreTrackers())
         {
            cpuUsages.add(cpuCoreTracker.getPercentUsage());
         }
         kryoMessagerServer.submitMessage(API.CPUUsages, cpuUsages);
      }
   }

   public static class API
   {
      public static final int PORT = 2151;

      private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();
      private static final MessagerAPIFactory.Category RootCategory = apiFactory.createRootCategory("Root");
      private static final MessagerAPIFactory.CategoryTheme MissionControlServiceTheme = apiFactory.createCategoryTheme("MissionControlService");

      public static final MessagerAPIFactory.Topic<MutablePair<Double, Double>> RAMUsage = topic("RAMUsage");
      public static final MessagerAPIFactory.Topic<ArrayList<Double>> CPUUsages = topic("CPUUsages");

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
