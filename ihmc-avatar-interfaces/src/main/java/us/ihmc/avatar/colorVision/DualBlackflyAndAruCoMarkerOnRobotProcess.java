package us.ihmc.avatar.colorVision;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.log.LogTools;
import us.ihmc.perception.sceneGraph.ros2.ROS2SceneGraph;
import us.ihmc.perception.sensorHead.BlackflyLensProperties;
import us.ihmc.perception.spinnaker.SpinnakerBlackfly;
import us.ihmc.perception.spinnaker.SpinnakerBlackflyManager;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.tools.thread.Throttler;

/**
 * To run this you have to download the Spinnaker SDK, move it to the robot computer, then run
 * the install script inside of it.
 */
public class DualBlackflyAndAruCoMarkerOnRobotProcess
{
   private static final String LEFT_SERIAL_NUMBER = System.getProperty("blackfly.left.serial.number", "00000000");
   private static final String RIGHT_SERIAL_NUMBER = System.getProperty("blackfly.right.serial.number", "00000000");

   private final DRCRobotModel robotModel;
   private final ROS2SceneGraph sceneGraph;
   private final BlackflyLensProperties blackflyLensProperties;

   private final ROS2Node ros2Node;
   private final ROS2SyncedRobotModel syncedRobot;
   private final Thread syncedRobotUpdateThread;

   private final SpinnakerBlackflyManager spinnakerBlackflyManager = new SpinnakerBlackflyManager();
   private final SideDependentList<DualBlackflyCamera> dualBlackflyCameras = new SideDependentList<>();

   private volatile boolean running = true;

   public DualBlackflyAndAruCoMarkerOnRobotProcess(DRCRobotModel robotModel, BlackflyLensProperties blackflyLensProperties)
   {
      this.robotModel = robotModel;
      this.blackflyLensProperties = blackflyLensProperties;

      ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "blackfly_node");
      syncedRobot = new ROS2SyncedRobotModel(robotModel, ros2Node);
      // Helpful to view relative sensor transforms when robot controller is not running
      syncedRobot.initializeToDefaultRobotInitialSetup(0.0, 0.0, 0.0, 0.0);
      sceneGraph = new ROS2SceneGraph(new ROS2Helper(ros2Node));

      if (!LEFT_SERIAL_NUMBER.equals("00000000"))
      {
         LogTools.info("Adding Blackfly left with serial number: {}", LEFT_SERIAL_NUMBER);
         SpinnakerBlackfly spinnakerBlackfly = spinnakerBlackflyManager.createSpinnakerBlackfly(LEFT_SERIAL_NUMBER);
         DualBlackflyCamera dualBlackflyCamera = createDualBlackflyCamera(RobotSide.LEFT, spinnakerBlackfly);
         dualBlackflyCameras.set(RobotSide.LEFT, dualBlackflyCamera);
      }
      else
      {
         LogTools.warn("No serial number for left Blackfly specified. The sensor will not be available.");
      }

      if (!RIGHT_SERIAL_NUMBER.equals("00000000"))
      {
         LogTools.info("Adding Blackfly right with serial number: {}", RIGHT_SERIAL_NUMBER);
         SpinnakerBlackfly spinnakerBlackfly = spinnakerBlackflyManager.createSpinnakerBlackfly(RIGHT_SERIAL_NUMBER);
         DualBlackflyCamera dualBlackflyCamera = createDualBlackflyCamera(RobotSide.RIGHT, spinnakerBlackfly);
         dualBlackflyCameras.set(RobotSide.RIGHT, dualBlackflyCamera);
      }
      else
      {
         LogTools.warn("No serial number for right Blackfly specified. The sensor will not be available.");
      }

      // We update the synced robot model either at 20 hz or the rate of the Blackfly that is reading images the fastest
      syncedRobotUpdateThread = new Thread(() ->
      {
         // Represents the frequency at which images are read from the fastest Blackfly
         double highestFrequency = 0;

         Throttler throttler = new Throttler();

         while (running)
         {
            for (DualBlackflyCamera dualBlackflyCamera : dualBlackflyCameras.values())
            {
               // min of 20 hz
               highestFrequency = Math.max(20, Math.max(highestFrequency, dualBlackflyCamera.getReadFrequency()));
            }

            throttler.setFrequency(highestFrequency);
            throttler.waitAndRun();

            syncedRobot.update();
         }
      }, "ROS2SyncedRobotModel-update-thread");

      syncedRobotUpdateThread.start();

      Runtime.getRuntime().addShutdownHook(new Thread(this::destroy, getClass().getName() + "-Shutdown"));
   }

   private DualBlackflyCamera createDualBlackflyCamera(RobotSide side, SpinnakerBlackfly spinnakerBlackfly)
   {
      return new DualBlackflyCamera(robotModel,
                                    side,
                                    syncedRobot::getReferenceFrames,
                                    ros2Node,
                                    spinnakerBlackfly,
                                    blackflyLensProperties,
                                    sceneGraph);
   }

   private void destroy()
   {
      running = false;

      try
      {
         syncedRobotUpdateThread.join();
      }
      catch (InterruptedException e)
      {
         LogTools.error(e);
      }

      System.out.println("Destroying dual blackfly processes");

      for (DualBlackflyCamera dualBlackflyCamera : dualBlackflyCameras)
      {
         if (dualBlackflyCamera != null)
            dualBlackflyCamera.destroy();
      }

      spinnakerBlackflyManager.destroy();

      ros2Node.destroy();
   }
}
