package us.ihmc.behaviors.sequence;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.perception.sceneGraph.PredefinedSceneNodeLibrary;
import us.ihmc.perception.sceneGraph.ROS2DetectableSceneNodesSubscription;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.tools.thread.Throttler;

/**
 * The entry point and initial setup of the behavior sequence on-robot process.
 */
public class BehaviorActionSequenceModule
{
   private final ROS2SyncedRobotModel syncedRobot;
   private final ROS2DetectableSceneNodesSubscription detectableSceneNodesSubscription;
   private volatile boolean running = true;
   private final Throttler throttler = new Throttler();
   private final double PERIOD = Conversions.hertzToSeconds(30.0);
   private final BehaviorActionSequence sequence;

   public BehaviorActionSequenceModule(DRCRobotModel robotModel, PredefinedSceneNodeLibrary predefinedSceneNodeLibrary)
   {
      ROS2ControllerHelper ros2 = new ROS2ControllerHelper(PubSubImplementation.FAST_RTPS, "behavior_action_sequence", robotModel);

      syncedRobot = new ROS2SyncedRobotModel(robotModel, ros2.getROS2NodeInterface());
      syncedRobot.initializeToDefaultRobotInitialSetup(0.0, 0.0, 0.0, 0.0);


      detectableSceneNodesSubscription = new ROS2DetectableSceneNodesSubscription(predefinedSceneNodeLibrary.getDetectableSceneNodes(), ros2);

      sequence = new BehaviorActionSequence(robotModel, ros2, predefinedSceneNodeLibrary);

      Runtime.getRuntime().addShutdownHook(new Thread(this::destroy, "Shutdown"));
      ThreadTools.startAThread(this::actionThread, "ActionThread");
   }

   private void actionThread()
   {
      while (running)
      {
         throttler.waitAndRun(PERIOD);

         syncedRobot.update();
         detectableSceneNodesSubscription.update();
         sequence.update();
      }
   }

   private void destroy()
   {
      running = false;
   }
}
