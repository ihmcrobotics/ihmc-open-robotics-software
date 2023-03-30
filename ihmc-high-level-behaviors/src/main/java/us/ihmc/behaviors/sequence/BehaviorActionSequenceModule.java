package us.ihmc.behaviors.sequence;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.perception.ArUcoObjectsPerceptionManager;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.tools.thread.Throttler;

/**
 * The entry point and initial setup of the behavior sequence on-robot process.
 */
public class BehaviorActionSequenceModule
{
   private final ROS2SyncedRobotModel syncedRobot;
   private volatile boolean running = true;
   private final Throttler throttler = new Throttler();
   private final double PERIOD = Conversions.hertzToSeconds(30.0);
   private final ArUcoObjectsPerceptionManager arUcoObjectsPerceptionManager;
   private final BehaviorActionSequence sequence;

   public BehaviorActionSequenceModule(DRCRobotModel robotModel)
   {
      ROS2ControllerHelper ros2 = new ROS2ControllerHelper(PubSubImplementation.FAST_RTPS, "behavior_action_sequence", robotModel);

      syncedRobot = new ROS2SyncedRobotModel(robotModel, ros2.getROS2NodeInterface());
      syncedRobot.initializeToDefaultRobotInitialSetup(0.0, 0.0, 0.0, 0.0);

      arUcoObjectsPerceptionManager = new ArUcoObjectsPerceptionManager(syncedRobot.getReferenceFrames().getObjectDetectionCameraFrame());

      ReferenceFrameLibrary referenceFrameLibrary = new ReferenceFrameLibrary();
      referenceFrameLibrary.add(arUcoObjectsPerceptionManager.getPullDoorManager().getDoorPanel().getObjectFrame());
      referenceFrameLibrary.add(arUcoObjectsPerceptionManager.getPullDoorManager().getDoorFrame().getObjectFrame());
      referenceFrameLibrary.add(arUcoObjectsPerceptionManager.getPushDoorManager().getDoorPanel().getObjectFrame());
      referenceFrameLibrary.add(arUcoObjectsPerceptionManager.getPushDoorManager().getDoorFrame().getObjectFrame());

      sequence = new BehaviorActionSequence(robotModel, ros2, referenceFrameLibrary);

      Runtime.getRuntime().addShutdownHook(new Thread(this::destroy, "Shutdown"));
      ThreadTools.startAThread(this::actionThread, "ActionThread");
   }

   private void actionThread()
   {
      while (running)
      {
         throttler.waitAndRun(PERIOD);

         syncedRobot.update();
         arUcoObjectsPerceptionManager.update();
         sequence.update();
      }
   }

   private void destroy()
   {
      running = false;
   }
}
