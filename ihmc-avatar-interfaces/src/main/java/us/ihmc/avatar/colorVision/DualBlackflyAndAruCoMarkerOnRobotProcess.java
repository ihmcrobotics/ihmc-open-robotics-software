package us.ihmc.avatar.colorVision;

import controller_msgs.msg.dds.RigidBodyTransformMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCROS2Input;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoTools;
import us.ihmc.perception.OpenCVArUcoMarker;
import us.ihmc.perception.spinnaker.SpinnakerSystemManager;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.thread.Activator;
import us.ihmc.tools.thread.Throttler;

import java.util.List;

/** To run this you have to download the Spinnaker SDK, move it to the robot computer, then run
 *  the install script inside of it.
 */
public class DualBlackflyAndAruCoMarkerOnRobotProcess
{
   private static final String LEFT_SERIAL_NUMBER = System.getProperty("blackfly.left.serial.number", "00000000");
   private static final String RIGHT_SERIAL_NUMBER = System.getProperty("blackfly.right.serial.number", "00000000");
   public static final double MAX_PERIOD = UnitConversions.hertzToSeconds(20.0);

   private final Activator nativesLoadedActivator;
   private final ROS2SyncedRobotModel syncedRobot;
   private SpinnakerSystemManager spinnakerSystemManager;
   private final ROS2Helper ros2Helper;
   private final RealtimeROS2Node realtimeROS2Node;
   private boolean nodeSpun = false;
   private final SideDependentList<DualBlackflyCamera> blackflies = new SideDependentList<>();
   private final Throttler throttler = new Throttler();
   private volatile boolean running = true;
   private final List<OpenCVArUcoMarker> arUcoMarkersToTrack;
   private final RigidBodyTransform objectDetectionCameraTransform;
   private final IHMCROS2Input<RigidBodyTransformMessage> frameUpdateSubscription;

   public DualBlackflyAndAruCoMarkerOnRobotProcess(DRCRobotModel robotModel, List<OpenCVArUcoMarker> arUcoMarkersToTrack)
   {
      this.arUcoMarkersToTrack = arUcoMarkersToTrack;
      nativesLoadedActivator = BytedecoTools.loadOpenCVNativesOnAThread();

      ROS2Node ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "blackfly_node");
      ros2Helper = new ROS2Helper(ros2Node);

      syncedRobot = new ROS2SyncedRobotModel(robotModel, ros2Node);
      // Helpful to view relative sensor transforms when robot controller is not running
      syncedRobot.initializeToDefaultRobotInitialSetup(0.0, 0.0, 0.0, 0.0);

      if (!LEFT_SERIAL_NUMBER.equals("00000000"))
      {
         LogTools.info("Adding Blackfly LEFT with serial number: {}", LEFT_SERIAL_NUMBER);
         blackflies.put(RobotSide.LEFT, new DualBlackflyCamera(LEFT_SERIAL_NUMBER, syncedRobot));
      }
      if (!RIGHT_SERIAL_NUMBER.equals("00000000"))
      {
         LogTools.info("Adding Blackfly RIGHT with serial number: {}", RIGHT_SERIAL_NUMBER);
         blackflies.put(RobotSide.RIGHT, new DualBlackflyCamera(RIGHT_SERIAL_NUMBER, syncedRobot));
      }

      realtimeROS2Node = ROS2Tools.createRealtimeROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "videopub");

      objectDetectionCameraTransform = robotModel.getSensorInformation().getObjectDetectionCameraTransform();
      frameUpdateSubscription = ros2Helper.subscribe(ROS2Tools.OBJECT_DETECTION_FRAME_UPDATE);

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
                  blackfly.create(spinnakerSystemManager.createBlackfly(blackfly.getSerialNumber()), side, ros2Helper, realtimeROS2Node, arUcoMarkersToTrack);
               }
            }

            if (frameUpdateSubscription.getMessageNotification().poll())
            {
               MessageTools.toEuclid(frameUpdateSubscription.getMessageNotification().read(), objectDetectionCameraTransform);
            }

            syncedRobot.update();

            for (RobotSide side : blackflies.sides())
            {
               blackflies.get(side).update();
            }

            if (!nodeSpun)
            {
               boolean allInitialized = true;
               for (RobotSide side : blackflies.sides())
               {
                  allInitialized &= blackflies.get(side).getRos2ImagePublisher() != null;
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
      // This sleep is to let the above thread complete.
      // Typically, frames are captured in 35 ms or so, so we should be giving it
      // plenty of time here to stop.
      // Remember, this destroy is getting called on a new thread created by
      // the user doing a Ctrl+C.
      ThreadTools.sleep(250);

      for (RobotSide side : blackflies.sides())
      {
         blackflies.get(side).destroy();
      }
      // This sleep is because we just asked the Blackflies to stop aquiring images.
      // I have no idea how long it would normally take.
      ThreadTools.sleep(100);

      // This releases all the Spinnaker resources
      spinnakerSystemManager.destroy();
   }

   public static void main(String[] args)
   {
//      SpinnakerTools.printAllConnectedDevicesInformation();
   }
}
