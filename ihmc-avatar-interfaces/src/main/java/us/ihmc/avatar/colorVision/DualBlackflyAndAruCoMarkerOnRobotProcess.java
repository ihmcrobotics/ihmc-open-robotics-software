package us.ihmc.avatar.colorVision;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoTools;
import us.ihmc.perception.spinnaker.SpinnakerSystemManager;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.thread.Activator;
import us.ihmc.tools.thread.Throttler;

/** To run this you have to download the Spinnaker SDK, move it to the robot computer, then run
 *  the install script inside of it.
 */
public class DualBlackflyAndAruCoMarkerOnRobotProcess
{
   private static final String LEFT_SERIAL_NUMBER = System.getProperty("blackfly.left.serial.number", "00000000");
   private static final String RIGHT_SERIAL_NUMBER = System.getProperty("blackfly.right.serial.number", "00000000");
   public static final double MAX_PERIOD = UnitConversions.hertzToSeconds(30.0);

   private final Activator nativesLoadedActivator;
   private SpinnakerSystemManager spinnakerSystemManager;
   private final ROS2Helper ros2Helper;
   private final RealtimeROS2Node realtimeROS2Node;
   private boolean nodeSpun = false;
   private final SideDependentList<DualBlackflyCamera> blackflies = new SideDependentList<>();
   private final Throttler throttler = new Throttler();
   private volatile boolean running = true;

   public DualBlackflyAndAruCoMarkerOnRobotProcess()
   {
      nativesLoadedActivator = BytedecoTools.loadOpenCVNativesOnAThread();

      if (!LEFT_SERIAL_NUMBER.equals("00000000"))
      {
         LogTools.info("Adding Blackfly LEFT with serial number: {}", LEFT_SERIAL_NUMBER);
         blackflies.put(RobotSide.LEFT, new DualBlackflyCamera(LEFT_SERIAL_NUMBER));
      }
      if (!RIGHT_SERIAL_NUMBER.equals("00000000"))
      {
         LogTools.info("Adding Blackfly RIGHT with serial number: {}", RIGHT_SERIAL_NUMBER);
         blackflies.put(RobotSide.RIGHT, new DualBlackflyCamera(RIGHT_SERIAL_NUMBER));
      }

      ROS2Node ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "blackfly_node");
      ros2Helper = new ROS2Helper(ros2Node);

      realtimeROS2Node = ROS2Tools.createRealtimeROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "videopub");

      Runtime.getRuntime().addShutdownHook(new Thread(this::destroy, "DualBlackflyShutdown"));
      ThreadTools.startAThread(this::update, "DualBlackflyNode");
   }

   private void update()
   {
      while (running)
      {
         throttler.waitAndRun(MAX_PERIOD);

         if (nativesLoadedActivator.poll())
         {
            if (nativesLoadedActivator.isNewlyActivated())
            {
               spinnakerSystemManager = new SpinnakerSystemManager();
               for (RobotSide side : blackflies.sides())
               {
                  DualBlackflyCamera blackfly = blackflies.get(side);
                  blackfly.create(spinnakerSystemManager.createBlackfly(blackfly.getSerialNumber()), side, ros2Helper, realtimeROS2Node);
               }
            }

            for (RobotSide side : blackflies.sides())
            {
               blackflies.get(side).update();
            }

            if (!nodeSpun)
            {
               boolean allInitialized = true;
               for (RobotSide side : blackflies.sides())
               {
                  allInitialized &= blackflies.get(side).getRos2VideoPublisher() != null;
               }

               if (allInitialized)
               {
                  nodeSpun = true;
                  LogTools.info("Spinning Realtime ROS 2 node");
                  realtimeROS2Node.spin();
               }
            }
         }
      }
   }

   private void destroy()
   {
      running = false;
      for (RobotSide side : blackflies.sides())
      {
         blackflies.get(side).destroy();
      }
      spinnakerSystemManager.destroy();
   }

   public static void main(String[] args)
   {
//      SpinnakerTools.printAllConnectedDevicesInformation();
      new DualBlackflyAndAruCoMarkerOnRobotProcess();
   }
}
