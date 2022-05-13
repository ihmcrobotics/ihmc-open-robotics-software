package us.ihmc.avatar.colorVision;

import std_msgs.msg.dds.Empty;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.perception.BytedecoTools;
import us.ihmc.perception.spinnaker.SpinnakerHardwareManager;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.thread.Activator;
import us.ihmc.tools.thread.PausablePeriodicThread;
import us.ihmc.utilities.ros.ROS1Helper;
import us.ihmc.utilities.ros.RosTools;

/** To run this you have to download the Spinnaker SDK, move it to the robot computer, then run
 *  the install script inside of it.
 */
public class DualBlackflyAndAruCoMarkerOnRobotProcess
{
   private static final String LEFT_SERIAL_NUMBER = System.getProperty("blackfly.left.serial.number", "00000000");
   private static final String RIGHT_SERIAL_NUMBER = System.getProperty("blackfly.right.serial.number", "00000000");

   private final PausablePeriodicThread thread;
   private final Activator nativesLoadedActivator;
   private final ROS1Helper ros1Helper;
   private final ROS2Helper ros2Helper;
   private final TypedNotification<Empty> reconnectROS1Notification = new TypedNotification<>();
   private SideDependentList<DualBlackflyCamera> blackflies = new SideDependentList<>();

   private SpinnakerHardwareManager spinnakerHardwareManager;

   public DualBlackflyAndAruCoMarkerOnRobotProcess()
   {
      nativesLoadedActivator = BytedecoTools.loadOpenCVNativesOnAThread();

      blackflies.put(RobotSide.LEFT, new DualBlackflyCamera(LEFT_SERIAL_NUMBER));
//      blackflies.put(RobotSide.RIGHT, new DualBlackflyCamera(RIGHT_SERIAL_NUMBER));

      ros1Helper = new ROS1Helper("blackfly_node");

      ROS2Node ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "blackfly_node");
      ros2Helper = new ROS2Helper(ros2Node);
      ros2Helper.subscribeViaCallback(DualBlackflyComms.RECONNECT_ROS1_NODE, reconnectROS1Notification::set);

      thread = new PausablePeriodicThread("DualBlackflyNode", UnitConversions.hertzToSeconds(31.0), false, this::update);
      Runtime.getRuntime().addShutdownHook(new Thread(this::destroy, "DualBlackflyShutdown"));
      thread.start();
   }

   private void update()
   {
      if (nativesLoadedActivator.poll())
      {
         if (nativesLoadedActivator.isNewlyActivated())
         {
            spinnakerHardwareManager = new SpinnakerHardwareManager();
            for (RobotSide side : blackflies.sides())
            {
               DualBlackflyCamera blackfly = blackflies.get(side);
               blackfly.create(spinnakerHardwareManager.buildBlackfly(blackfly.getSerialNumber()),
                               side,
                               ros1Helper,
                               RosTools.BLACKFLY_VIDEO_TOPICS.get(side),
                               ros2Helper);
            }
         }

         if (reconnectROS1Notification.poll())
         {
            ros1Helper.reconnectEverything();
         }

         for (RobotSide side : blackflies.sides())
         {
            blackflies.get(side).update();
         }
      }
   }

   private void destroy()
   {
      for (RobotSide side : blackflies.sides())
      {
         blackflies.get(side).destroy();
      }
      spinnakerHardwareManager.destroy();
   }

   public static void main(String[] args)
   {
//      SpinnakerTools.printAllConnectedDevicesInformation();
      new DualBlackflyAndAruCoMarkerOnRobotProcess();
   }
}
