package us.ihmc.behaviors.sequence;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.perception.BehaviorSequencePerceptionManager;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.tools.thread.Throttler;

public class BehaviorActionSequenceModule
{
   private final ROS2SyncedRobotModel syncedRobot;
   private volatile boolean running = true;
   private final Throttler throttler = new Throttler();
   private final double PERIOD = Conversions.hertzToSeconds(30.0);
   private final BehaviorSequencePerceptionManager behaviorSequencePerceptionManager;
   private final BehaviorActionSequence sequence;

   public BehaviorActionSequenceModule(DRCRobotModel robotModel)
   {
      ROS2ControllerHelper ros2 = new ROS2ControllerHelper(PubSubImplementation.FAST_RTPS, "behavior_action_sequence", robotModel);

      syncedRobot = new ROS2SyncedRobotModel(robotModel, ros2.getROS2NodeInterface());
      syncedRobot.initializeToDefaultRobotInitialSetup(0.0, 0.0, 0.0, 0.0);

      behaviorSequencePerceptionManager = new BehaviorSequencePerceptionManager(syncedRobot.getReferenceFrames().getObjectDetectionCameraFrame());

      ReferenceFrameLibrary referenceFrameLibrary = new ReferenceFrameLibrary();
      referenceFrameLibrary.add(behaviorSequencePerceptionManager.getPullDoorManager().getDoorPanel().getObjectFrame());
      referenceFrameLibrary.add(behaviorSequencePerceptionManager.getPullDoorManager().getDoorFrame().getObjectFrame());
      referenceFrameLibrary.add(behaviorSequencePerceptionManager.getPushDoorManager().getDoorPanel().getObjectFrame());
      referenceFrameLibrary.add(behaviorSequencePerceptionManager.getPushDoorManager().getDoorFrame().getObjectFrame());

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
         behaviorSequencePerceptionManager.update();
         sequence.update();
      }
   }

   private void destroy()
   {
      running = false;
   }
}
